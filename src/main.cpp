#include <TaskScheduler.h>
#include <LittleFS.h>
#include <webServer.h>
#include <updater.h>
#include <fetch.h>
#include <configManager.h>
#include <dashboard.h>

#include <ESP8266mDNS.h>

#include "Shelly.h"
#include "Logger.h"
#include "Utils.h"
#include "Network.h"
#include "Dimmer.h"
#include "libPID.h"

typedef enum
{
  MNG_INITIALIZING = 0,
  MNG_DIMMER_OFF,
  MNG_DIMMER_ON
} MngState_t;

/** TaskSheduler definitions */

void BackgroundCbk(void);
void ManagerCbk(void);
void GetShellyPowerCbk(void);
void PidFilterCbk(void);

/** Save callback */
void SaveConfigCallback();

/** Timer ISR callback */
IRAM_ATTR void Dimmer_ISR();

Scheduler m_runnerP0;
Scheduler m_runnerP1;

/** Background task execution */
Task m_taskManager(50 * TASK_MILLISECOND, TASK_FOREVER, &ManagerCbk, &m_runnerP0);
Task m_taskBackground(25 * TASK_MILLISECOND, TASK_FOREVER, &BackgroundCbk, &m_runnerP0);

/** High prio tasks execution */
/** First get Shelly power, then filter it with PID*/
Task m_taskGetPower(POWER_GRID_MEASURE_PERIOD_MS *TASK_MILLISECOND, TASK_FOREVER, &GetShellyPowerCbk, &m_runnerP1);
Task m_taskPidFilter((POWER_GRID_MEASURE_PERIOD_MS / NB_PID_SAMPLES) * TASK_MILLISECOND, TASK_FOREVER, &PidFilterCbk, &m_runnerP1);

/** Shelly http requests */
WiFiClient espClient;
Shelly m_shelly(espClient);

/** Dimmer */
/** 3 channels, grid frequency, PID sample time, Measure period of sensor*/
Dimmer m_dimmer(1, POWER_GRID_FREQUENCY_HZ, POWER_GRID_MEASURE_PERIOD_MS / NB_PID_SAMPLES, POWER_GRID_MEASURE_PERIOD_MS * 2);

/** UDP over wifi */
WiFiUDP m_udp;

static MngState_t m_appliState = MNG_INITIALIZING;

void setup()
{
  // Logger init
  Logger::init();
  Logger::setLogLevel(LogLevel::DEBUG);

  // Dimmer init
  m_dimmer.mapChannelToPin(0, SSR1_Pin);
  // m_dimmer.mapChannelToPin(1, SSR2_Pin);
  // m_dimmer.mapChannelToPin(2, SSR3_Pin);
  m_dimmer.setChannelPower(0, 750);
  // m_dimmer.setChannelPower(1, 1000);
  // m_dimmer.setChannelPower(2, 1000);
  //  Todo : compute it based on channels power
  m_dimmer.setMaxOutputPower(750);
  m_dimmer.turnOff();

  // Check ESP flash size
  Utils::checkEspFlash();

  // Filesystem init
  LittleFS.begin();

  // Init configuration manager (data stored in EEPROM)
  configManager.begin();
  configManager.setConfigSaveCallback(SaveConfigCallback);

  // Apply PID parameters from config
  m_dimmer.setPidParameters(configManager.data.Kp, configManager.data.Ki, configManager.data.Kd);

  // Setup teleplotUdp
  Logger::setupTeleplotUdp(&m_udp, "192.168.1.96", 47269);

  // Network initialization
  Network::begin();
  // Once time is given by NTP, sync time to logger
  Logger::syncTime();

  // Web server init
  GUI.begin();

  // Dashboard init
  dash.data.simuSolarPower = 0;
  dash.data.simuHeaterEnabled = false;
  dash.data.teleplotEnabled = false;
  dash.begin(250);

  // Shelly init
  /* TODO : could be host name shellyem-c45bbee1d9b1 ?*/
  m_shelly.setIpAddress("192.168.1.58");

  // Hardware timer init for dimmer control (at 50Hz, *4 to drive SSR at right time)
  Utils::initHwTimer(POWER_GRID_FREQUENCY_HZ * 4, Dimmer_ISR);

  // Sheduler tasks setup and recursive start
  m_runnerP0.setHighPriorityScheduler(&m_runnerP1);
  m_runnerP0.enableAll(true);
}

void loop()
{
  // tasks execution
  m_runnerP0.execute();
}

/** TaskScheduler callback tasks */
void BackgroundCbk(void)
{
  if (!m_runnerP0.isOverrun())
  {
    // Prior to some freeze loop action, update ManageCbk

    // Wifi, mDNS
    Network::loop();

    // OTA updater
    updater.loop();

    // Config manager storage in EEPROM
    configManager.loop();

    // Dashboard update
    dash.loop();
  }
}

void ManagerCbk(void)
{
  if (!m_runnerP0.isOverrun())
  {
    float_t Kp, Ki, Kd;

    // Enable or disable teleplot over UDP
    Logger::enableTeleplotUdp(dash.data.teleplotEnabled);

    // Enable or disable heater retroaction simulation
    m_dimmer.setSimuEnabled(dash.data.simuHeaterEnabled);

    // Check if PID parameters have changed from config manager
    m_dimmer.getPidParameters(Kp, Ki, Kd);
    if ((configManager.data.Kp != Kp) ||
        (configManager.data.Ki != Ki) ||
        (configManager.data.Kd != Kd))
    {
      Logger::log(LogLevel::INFO, "ManagerCbk: PID parameters changed Kp=%.3f Ki=%.3f Kd=%.3f",
                  configManager.data.Kp, configManager.data.Ki, configManager.data.Kd);

      // Update dimmer PID parameters
      m_dimmer.setPidParameters(configManager.data.Kp,
                                configManager.data.Ki,
                                configManager.data.Kd);
    }

    switch (m_appliState)
    {
    case MNG_INITIALIZING:
    case MNG_DIMMER_OFF:
      if ((updater.getStatusEnum() != FSUpdaterStatus::FLASHING) &&
          (Network::isWifiMngCaptivePortal() == false))
      {
        m_appliState = MNG_DIMMER_ON;
        m_dimmer.turnOn();
        Utils::enableHwTimer();

        Logger::log(LogLevel::DEBUG, "Go state MNG_DIMMER_ON");
      }
      break;

    case MNG_DIMMER_ON:
      if ((updater.getStatusEnum() == FSUpdaterStatus::FLASHING) ||
          (Network::isWifiMngCaptivePortal() == true))
      {
        m_appliState = MNG_DIMMER_OFF;
        Utils::disableHwTimer();
        m_dimmer.turnOff();

        Logger::log(LogLevel::DEBUG, "Go state MNG_DIMMER_OFF");
      }
      break;

    default:
      break;
    }
  }
}

/*Called every POWER_GRID_MEASURE_PERIOD_MS/NB_PID_SAMPLES  = 250 ms*/
void PidFilterCbk(void)
{
  float_t power = 0.0f;
  float_t gridPower = 0.0f;

  gridPower = m_shelly.getActivePower();

  // Simulate solar production
  if (dash.data.simuSolarPower > 0)
  {
    gridPower -= dash.data.simuSolarPower;
  }

  // Update the power routed based on Shelly grid active power measure
  power = m_dimmer.update(gridPower);

  // Update graphic on dashboard
  dash.data.powerRouted = power;
}

void GetShellyPowerCbk(void)
{
  float power = 0.0f;
  bool success = false;

  if (!m_runnerP1.isOverrun())
  {
    if (m_appliState == MNG_DIMMER_ON)
    {
      success = m_shelly.updateMeasures();
      if (success)
      {
        power = m_shelly.getActivePower();
        // Update graphic on dashboard
        dash.data.gridPower = power;
      }
    }
  }
}

/** configManager save callback  */

void SaveConfigCallback()
{
  Logger::log(LogLevel::INFO, "Configuration saved in EEPROM");
}

/** Dimmer ISR */
IRAM_ATTR void Dimmer_ISR()
{
  m_dimmer.updateChannelsOutput();
}

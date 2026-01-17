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

/******************************************************************************
 * @defgroup Enumerations Declaration
 *************************************************************************@{ */
typedef enum
{
  MNG_INITIALIZING = 0,
  MNG_DIMMER_OFF,
  MNG_DIMMER_ON
} MngState_t;

/** @} */

/******************************************************************************
 * @defgroup Private Functions Declaration
 *************************************************************************@{ */

/** TaskSheduler prototype definitions */
static void backgroundCbk(void);
static void managerCbk(void);
static void readShellyPowerCbk(void);
static void pidFilterCbk(void);

/** Save callback when value stored in EEPROM */
static void saveConfigCallback();

/** Setup dimmer channels according to power settings */
static void setupDimmerChannels(uint16_t powerSSR1, uint16_t powerSSR2, uint16_t powerSSR3);

/** Callback used by coap */
static void coapResponseCallback(CoapPacket &packet, IPAddress ip, int port);

/** Timer ISR callback */
static IRAM_ATTR void dimmer_ISR();

/** @} */

/******************************************************************************
 * @defgroup Private Variables
 *************************************************************************@{ */
Scheduler m_runnerP0;
Scheduler m_runnerP1;

/** Background task execution */
Task m_taskManager(50 * TASK_MILLISECOND, TASK_FOREVER, &managerCbk, &m_runnerP0);
Task m_taskBackground(10 * TASK_MILLISECOND, TASK_FOREVER, &backgroundCbk, &m_runnerP0);

/** High prio tasks execution */
/** First get Shelly power, then filter it with PID */
Task m_taskGetPower(PID_SAMPLE_PERIOD_MS *TASK_MILLISECOND, TASK_FOREVER, &readShellyPowerCbk, &m_runnerP1);
Task m_taskPidFilter(DIMMER_OUTPUT_PERIOD_MS *TASK_MILLISECOND, TASK_FOREVER, &pidFilterCbk, &m_runnerP1);

/** Used for shelly and teleplot */
WiFiClient m_espClient;
WiFiUDP m_udp;

/** Shelly power reading and averaging  */
Shelly m_shelly(m_espClient, m_udp);

/** Dimmer */
/** grid frequency, PID sample time, Measure period of sensor*/
Dimmer m_dimmer(POWER_GRID_FREQUENCY_HZ, POWER_GRID_MEASURE_PERIOD_MS, PID_SAMPLE_PERIOD_MS, DIMMER_OUTPUT_PERIOD_MS);

/** Application state */
static MngState_t m_appliState = MNG_INITIALIZING;

/** SSR power settings */
static uint16_t m_powerSSR1 = 0;
static uint16_t m_powerSSR2 = 0;
static uint16_t m_powerSSR3 = 0;

/** @} */

/******************************************************************************
 * @defgroup Public Functions Implementation
 *************************************************************************@{ */

/**
 * @brief setup function called once at startup of the program
 */
void setup()
{
  /* Initialize outputs SSR pins to low*/
  digitalWrite(SSR1_Pin, LOGICAL_LOW);
  digitalWrite(SSR2_Pin, LOGICAL_LOW);
  digitalWrite(SSR3_Pin, LOGICAL_LOW);

  // Filesystem init
  LittleFS.begin();

  // Logger init
  Logger::init();
  Logger::setLogLevel(LogLevel::DEBUG);

  // Check ESP flash size
  Utils::checkEspFlash();

  // Init configuration manager (data stored in EEPROM)
  configManager.begin();
  configManager.setConfigSaveCallback(saveConfigCallback);

  // Apply initial dimmer configuration from config manager
  setupDimmerChannels(configManager.data.SSR1,
                      configManager.data.SSR2,
                      configManager.data.SSR3);
  // Apply PID parameters from config
  m_dimmer.setPidParameters(configManager.data.Kp, configManager.data.Ki, configManager.data.Kd);
  m_dimmer.turnOff();

  // Network initialization
  Network::begin();
  Logger::syncTime();
  // IP is DEV PC
  Logger::setupTeleplotUdp(&m_udp, "192.168.1.96", 47269);

  // Web server init
  GUI.begin();

  // Dashboard init
  dash.data.simuSolarPower = 0;
  dash.data.simuHeaterEnabled = false;
  dash.data.teleplotEnabled = false;
  dash.begin(250);

  // Shelly init
  /* TODO : could be host name shellyem-c45bbee1d9b1 resolved by mdns ?*/
  m_shelly.setup("192.168.1.58");
  if (m_shelly.isCoIotEnabled())
  {
    m_shelly.setupCoIoT(coapResponseCallback);
  }

  // Hardware timer init for dimmer control (at 50Hz, *4 to drive SSR at right time)
  Utils::initHwTimer(POWER_GRID_FREQUENCY_HZ * 4, dimmer_ISR);

  // Sheduler tasks setup and recursive start
  m_runnerP0.setHighPriorityScheduler(&m_runnerP1);
  m_runnerP0.enableAll(true);
  if (m_shelly.isCoIotEnabled())
  {
    // Delay the pid task execution since CoIoT reques in background task
    m_taskPidFilter.delay(50 * TASK_MILLISECOND);
  }
}

/**
 * @brief main loop function called repeatedly after setup()
 */
void loop()
{
  // tasks execution
  m_runnerP0.execute();
}

/** @} */

/******************************************************************************
 * @defgroup  Private Functions Implementation
 *************************************************************************@{ */

/**
 * @brief Background task called by TaskScheduler regularly to handle non time-critical functions
 */
static void backgroundCbk(void)
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

    // Shelly measurement
    m_shelly.loop();

    // Dashboard update
    dash.loop();
  }
}

/**
 * @brief Manager task called by TaskScheduler regularly to handle application state machine
 *
 */
static void managerCbk(void)
{
  float_t Kp, Ki, Kd;

  if (!m_runnerP0.isOverrun())
  {
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
      Logger::log(LogLevel::INFO, "backgroundCbk: PID parameters changed Kp=%.3f Ki=%.3f Kd=%.3f",
                  configManager.data.Kp, configManager.data.Ki, configManager.data.Kd);

      // Update dimmer PID parameters
      m_dimmer.setPidParameters(configManager.data.Kp,
                                configManager.data.Ki,
                                configManager.data.Kd);
    }

    /** Check if the channel setting has changed */
    if ((configManager.data.SSR1 != m_powerSSR1) ||
        (configManager.data.SSR2 != m_powerSSR2) ||
        (configManager.data.SSR3 != m_powerSSR3))
    {
      Logger::log(LogLevel::INFO, "backgroundCbk: SSR power settings changed SSR1=%dW SSR2=%dW SSR3=%dW",
                  configManager.data.SSR1,
                  configManager.data.SSR2,
                  configManager.data.SSR3);

      // Reconfigure dimmer channels
      setupDimmerChannels(configManager.data.SSR1,
                          configManager.data.SSR2,
                          configManager.data.SSR3);
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

/**
 * @brief Pid filter task called by TaskScheduler regularly to compute the power to be routed
 *        Called every DIMMER_OUTPUT_PERIOD_MS  = 250 ms
 */
static void pidFilterCbk(void)
{
  float_t power = 0.0f;
  float_t gridPower = 0.0f;
  bool newMeasure = false;

  if (!m_runnerP1.isOverrun())
  {
    gridPower = m_shelly.getActivePower();

    // Simulate solar production
    if (dash.data.simuSolarPower > 0)
    {
      gridPower -= dash.data.simuSolarPower;
    }

    // Update the power routed based on Shelly grid active power measure
    newMeasure = m_shelly.isNewMeasure();
    power = m_dimmer.update(gridPower, newMeasure);

    if (newMeasure)
    {
      // Update graphic on dashboard
      dash.data.powerRouted = power;
    }
  }
}

/**
 * @brief Shelly power read task called by TaskScheduler regularly to get the power from Shelly device
 *
 */
static void readShellyPowerCbk(void)
{
  float power = 0.0f;
  bool success = false;

  if (!m_runnerP1.isOverrun())
  {
    if (m_appliState == MNG_DIMMER_ON)
    {
      success = m_shelly.updateMeasures();
      if (success && (m_shelly.isCoIotEnabled() == false))
      {
        power = m_shelly.getActivePower();
        // Update graphic on dashboard
        dash.data.gridPower = power;
      }
    }
  }
}
/**
 * @brief Save configuration callback called when configuration is stored in EEPROM
 */
static void saveConfigCallback()
{
  Logger::log(LogLevel::INFO, "Configuration saved in EEPROM");
}

/**
 * @brief Coap response callback
 *
 */
static void coapResponseCallback(CoapPacket &packet, IPAddress ip, int port)
{
  float power = 0.0f;

  m_shelly.coapResponseCallbackImpl(packet, ip, port);

  if (m_shelly.getErrorStatus() == false)
  {
    power = m_shelly.getActivePower();

    // Update graphic on dashboard
    dash.data.gridPower = power;
  }
}

/**
 * @brief Setup dimmer channels according to power settings
 *
 * @param powerSSR1 Load power on SSR1
 * @param powerSSR2 Load power on SSR2
 * @param powerSSR3 Load power on SSR3
 */
static void setupDimmerChannels(uint16_t powerSSR1, uint16_t powerSSR2, uint16_t powerSSR3)
{
  uint8_t nbChannels = 0;
  uint16_t maxPower = 0;
  bool ssr1Used = false;
  bool ssr2Used = false;

  /** Initialize all SSR pins to LOW */
  if (m_powerSSR1 > 0)
  {
    digitalWrite(SSR1_Pin, LOGICAL_LOW);
  }
  if (m_powerSSR2 > 0)
  {
    digitalWrite(SSR2_Pin, LOGICAL_LOW);
  }
  if (m_powerSSR3 > 0)
  {
    digitalWrite(SSR3_Pin, LOGICAL_LOW);
  }

  /** Setup new power values */
  m_powerSSR1 = powerSSR1;
  m_powerSSR2 = powerSSR2;
  m_powerSSR3 = powerSSR3;

  if (m_powerSSR1 > 0)
  {
    nbChannels++;
  }
  if (m_powerSSR2 > 0)
  {
    nbChannels++;
  }
  if (m_powerSSR3 > 0)
  {
    nbChannels++;
  }
  m_dimmer.setNbChannels(nbChannels);

  for (uint8_t i = 0; i < nbChannels; i++)
  {
    if (m_powerSSR1 > 0 && !ssr1Used)
    {
      ssr1Used = true;
      maxPower += m_powerSSR1;
      m_dimmer.mapChannelToPin(i, SSR1_Pin);
      m_dimmer.setChannelPower(i, m_powerSSR1);
    }
    else if (m_powerSSR2 > 0 && !ssr2Used)
    {
      ssr2Used = true;
      maxPower += m_powerSSR2;
      m_dimmer.mapChannelToPin(i, SSR2_Pin);
      m_dimmer.setChannelPower(i, m_powerSSR2);
    }
    else if (m_powerSSR3 > 0)
    {
      maxPower += m_powerSSR3;
      m_dimmer.mapChannelToPin(i, SSR3_Pin);
      m_dimmer.setChannelPower(i, m_powerSSR3);
    }
  }
  m_dimmer.setMaxOutputPower(maxPower);
}

/**
 * @brief ISR for dimmer control
 */
static IRAM_ATTR void dimmer_ISR()
{
  m_dimmer.updateChannelsOutput();
}

/** @} */

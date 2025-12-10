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
void SaveCallback();

/** Timer ISR callback */
IRAM_ATTR void Dimmer_ISR();

   

Scheduler m_runnerP0;
Scheduler m_runnerP1;

/** Background task execution */
Task m_taskManager(   50 * TASK_MILLISECOND, TASK_FOREVER,  &ManagerCbk,    &m_runnerP0);
Task m_taskBackground(25 * TASK_MILLISECOND, TASK_FOREVER,  &BackgroundCbk, &m_runnerP0);

/** High prio tasks execution */
/** First get Shelly power, then filter it with PID*/
Task m_taskGetPower(POWER_GRID_MEASURE_PERIOD_MS * TASK_MILLISECOND, TASK_FOREVER, &GetShellyPowerCbk, &m_runnerP1);
Task m_taskPidFilter((POWER_GRID_MEASURE_PERIOD_MS/NB_PID_SAMPLES) * TASK_MILLISECOND, TASK_FOREVER, &PidFilterCbk, &m_runnerP1);



/** Shelly http requests */
WiFiClient espClient;
Shelly m_shelly(espClient);

/** Dimmer */
/** 3 channels, grid frequency, PID sample time, Measure period (TODO)*/
Dimmer m_dimmer(3, POWER_GRID_FREQUENCY_HZ, POWER_GRID_MEASURE_PERIOD_MS/NB_PID_SAMPLES, POWER_GRID_MEASURE_PERIOD_MS * 2); 

/** UDP over wifi */
WiFiUDP m_udp;

static MngState_t m_appliState = MNG_INITIALIZING;

void setup()
{
    Serial.begin(115200);

    //Logger init
    Logger::init();
    Logger::setLogLevel(LogLevel::DEBUG);
    
    // Check ESP flash size
    Utils::checkEspFlash();

    // Filesystem init
    LittleFS.begin();
   
    // Init configuration manager (data stored in EEPROM)
    configManager.begin();
    configManager.setConfigSaveCallback(SaveCallback);

    // Setup teleplotUdp
    Logger::setupTeleplotUdp(&m_udp, "192.168.1.96", 47269);

    // Network initialization
    Network::begin();
    // Once time is given by NTP, sync time to logger
    Logger::syncTime();
    // Once Wifi is ready set teleplot udp enable
    Logger::enableTeleplotUdp(true);
  
    // Web server init
    GUI.begin();

    // Dashboard refresh
    dash.begin(500);

    // Shelly init
    /* TODO : could be host name shellyem-c45bbee1d9b1 ?*/
    m_shelly.setIpAddress("192.168.1.58");
  
    // Dimmer init
    m_dimmer.mapChannelToPin(0, SSR1_Pin);
    m_dimmer.mapChannelToPin(1, SSR2_Pin);
    m_dimmer.mapChannelToPin(2, SSR3_Pin);
    m_dimmer.turnOff();

    // Hardware timer init for dimmer control (at 50Hz, *4 to drive SSR at right time)
    Utils::initHwTimer(POWER_GRID_FREQUENCY_HZ*4, Dimmer_ISR);

     // Sheduler tasks setup and recursive start
    m_runnerP0.setHighPriorityScheduler(&m_runnerP1); 
    m_runnerP0.enableAll(true);

}

void loop()
{   
    //tasks execution
    m_runnerP0.execute();
}


/** TaskScheduler callback tasks */
void BackgroundCbk(void)
{
  if  (!m_runnerP0.isOverrun())
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
  if  (!m_runnerP0.isOverrun())
  {
    switch (m_appliState)
    {
      case MNG_INITIALIZING :
      case MNG_DIMMER_OFF :
        if ((updater.getStatusEnum() != FSUpdaterStatus::FLASHING) &&
           (Network::isWifiMngCaptivePortal() == false) )
        {
          m_appliState  = MNG_DIMMER_ON;
          m_dimmer.turnOn();
          Utils::enableHwTimer();

          Logger::log(LogLevel::DEBUG, "Go state MNG_DIMMER_ON");
        }
        break;

      case MNG_DIMMER_ON :
        if ((updater.getStatusEnum() == FSUpdaterStatus::FLASHING) || 
           (Network::isWifiMngCaptivePortal() == true) )
        {
          m_appliState  = MNG_DIMMER_OFF;
          Utils::disableHwTimer();
          m_dimmer.turnOff();

          Logger::log(LogLevel::DEBUG, "Go state MNG_DIMMER_OFF");
        }
        break;

      default :
        break;
    }
  }
}

/*Called every POWER_GRID_MEASURE_PERIOD_MS/NB_PID_SAMPLES  = 100 ms*/
void PidFilterCbk(void)
{
  m_dimmer.update(m_shelly.getActivePower());
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
      }
    
      // Update graphic on dashboard
      dash.data.gridPower = power;
    }
  }
}


/** configManager save callback  */

void SaveCallback() 
{
    Logger::log(LogLevel::INFO, "Configuration saved in EEPROM"); 
}

/** Dimmer ISR */
IRAM_ATTR void Dimmer_ISR()
{
  m_dimmer.updateChannelsOutput();
}


#include <Arduino.h>
#include <TaskScheduler.h>
#include <LittleFS.h>

#include <WiFiManager.h>
#include <webServer.h>
#include <updater.h>
#include <fetch.h>
#include <configManager.h>
#include <timeSync.h>

#include "Shelly.h"
#include "Logger.h"
#include "Utils.h"


void checkEspFlash();

/** TaskSheduler definitions */

void GetShellyPowerCbk(void);
void BackgroundCbk(void);

Scheduler m_runnerP0;
Scheduler m_runnerP1;

/* Background task execution */
Task m_taskBackground(100* TASK_MILLISECOND, TASK_FOREVER,  &BackgroundCbk, &m_runnerP0);

/* High prio tasks execution */
Task m_taskGetPower(500* TASK_MILLISECOND, TASK_FOREVER, &GetShellyPowerCbk, &m_runnerP1);


/** Shelly http requests */
WiFiClient espClient;
Shelly m_shelly(espClient);




void setup()
{
    Serial.begin(115200);

    Utils::checkEspFlash();

    LittleFS.begin();
    GUI.begin();
    configManager.begin();
    WiFiManager.begin(configManager.data.projectName);
    timeSync.begin();

    //Logger init
    Logger::init();
    Logger::setLogLevel(LogLevel::DEBUG);

    // Shelly init
    m_shelly.setIpAddress("192.168.1.58");
  
    // Sheduler tasks setup and recursive start
    m_runnerP0.setHighPriorityScheduler(&m_runnerP1); 
    m_runnerP0.enableAll(true);
  
}

void loop()
{
    //software interrupts
    WiFiManager.loop();
    updater.loop();

    //tasks execution
    m_runnerP0.execute();
}

//***********************************
//************* Callback tasks
//***********************************

void BackgroundCbk(void)
{

}

void GetShellyPowerCbk(void)
{
  float power = 0.0f;

  // skip if an update is in progress, not in captive portal and no task overrun
  if ( (!m_runnerP1.isOverrun()) &&
       (updater.getStatus() != 254) &&
       (WiFiManager.isCaptivePortal() == false) )
  {
    m_shelly.updateMeasures();
    power = m_shelly.getActivePower();

    Logger::log(LogLevel::DEBUG, "Shelly: Active Power = %f W", power);
  }
}




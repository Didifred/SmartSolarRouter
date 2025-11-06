
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <timeSync.h>
#include "Logger.h"
#include "Network.h"


String Network::m_deviceHostname;

/** Public methods */

void Network::begin(void)
{
    m_deviceHostname = "solar-router-" + WiFi.macAddress().substring(12,14) + WiFi.macAddress().substring(15,17);
    const char* hostname_c_str = m_deviceHostname.c_str();

    // WiFi Manager init
    Logger::log(LogLevel::INFO, "Hostname: %s ", hostname_c_str);
    WiFiManager.begin(hostname_c_str);

    // NTP time sync init
    timeSync.begin();

    // mDNS init works out of the box for Ubuntu, Windows/Android may need additional steps ...
    mdnsBegin(hostname_c_str);
}

void Network::loop(void)
{
    WiFiManager.loop();
    MDNS.update();
}

bool Network::isWifiMngCaptivePortal(void)
{
    return WiFiManager.isCaptivePortal();
}

String& Network::getDeviceHostname(void)
{
    return (m_deviceHostname);
}

/** Private methods */
bool Network::mdnsBegin(const char* hostname)
{
    bool success = MDNS.begin(hostname);

    if (success)
    {
        Logger::log(LogLevel::INFO, "mDNS responder started with domain name: %s.local", hostname);

        MDNS.addService("http", "tcp", 80);
        MDNS.addServiceTxt("http", "tcp", "name", hostname);
        MDNS.addServiceTxt("http", "tcp", "fonction", "dimmer");
    }
    else
    {
        Logger::log(LogLevel::ERROR, "Error setting up mDNS responder!");
    }

    return success;
}



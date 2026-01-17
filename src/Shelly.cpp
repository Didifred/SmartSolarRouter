/**
 * @file Shelly.cpp
 * @brief Implementation of the Shelly class for interfacing with Shelly energy meter devices over HTTP.
 *
 * This file provides methods to set the IP address of the Shelly device, update its settings,
 * retrieve measurement data (such as active power), and handle HTTP communication and JSON parsing.
 *
 * Dependencies:
 * - Arduino core
 * - ESP8266WiFi and ESP8266HTTPClient libraries for network communication
 * - ArduinoJson for JSON parsing
 * - Logger for logging/debugging
 *
 * @author
 * @date
 */

#include <ArduinoJson.h>
#include "Shelly.h"
#include "Logger.h"

#define COAP_PORT 5683
#define COIOT_SHELLY_ACTIVE_POWER_ID 4105

/** \public interfaces */

Shelly::Shelly(WiFiClient &wifiClient, WiFiUDP &udp) : m_espClient(wifiClient), m_udp(udp)
{
  /* could be also hostname "shellyem-xxxxxxx" */
  m_IP = "0.0.0.0";
  m_activePower = 0.0f;
  m_error = false;
  m_newMeasure = false;
  m_coap = nullptr;
  m_type = ShellyType::UNKNOWN;
  m_coIotEnabled = false;
  m_nbChannels = 0;
}

void Shelly::setup(const String &ip)
{
  m_IP = ip;

  updateSettings();
}

ShellyType Shelly::getShellyType(void)
{
  return (m_type);
}

bool Shelly::isCoIotEnabled(void)
{
  return (m_coIotEnabled);
}

void Shelly::setupCoIoT(CoapCallback callbackResponse)
{
  m_coap = new Coap(m_udp, 256);

  if (m_coap != nullptr)
  {
    m_coap->response(callbackResponse);
    m_coap->start();
  }
}

bool Shelly::updateMeasures(void)
{
  bool success;

  if (m_coIotEnabled)
  {
    success = updateMeasuresCoIot();
  }
  else
  {
    success = updateMeasuresHttp();
  }

  return (success);
}

bool Shelly::updateMeasuresCoIot(void)
{
  IPAddress ip;
  uint16_t packetId;
  bool success = false;

  success = ip.fromString(m_IP);

  if ((m_coap != nullptr) && success)
  {
    success = false;

    packetId = m_coap->get(ip, COAP_PORT, "cit/s");

    if (packetId > 0)
    {
      success = true;
      Logger::log(LogLevel::TRACE, "Shelly : CoIot request, packetId=%i", packetId);
    }
  }

  if (success == false)
  {
    Logger::log(LogLevel::ERROR, "Shelly: CoIot request cit/s failed, IP=%s !", m_IP.c_str());
  }

  return success;
}

bool Shelly::updateMeasuresHttp(void)
{
  String requestStr = "http://" + m_IP + "/emeter/0";
  int httpCodeStatus;
  bool success = false;

  success = m_httpClient.begin(m_espClient, requestStr);

  if (success)
  {
    httpCodeStatus = m_httpClient.GET();

    if (httpCodeStatus == HTTP_CODE_OK)
    {
      JsonDocument doc;
      String payload = m_httpClient.getString();
      m_httpClient.end();

      DeserializationError error = deserializeJson(doc, payload);

      if (error == DeserializationError::Ok)
      {
        m_activePower = doc["power"].as<float>();

        if (m_error)
        {
          Logger::log(LogLevel::INFO, "Shelly: back OK, active Power = %f W", m_activePower);
        }
        else
        {
          Logger::log(LogLevel::TRACE, "Shelly: active Power = %f W", m_activePower);
        }
        success = true;
        m_newMeasure = true;
      }
      else
      {
        Logger::log(LogLevel::ERROR, "Shelly: JSON deserialization failed with error %s", error.c_str());
      }
    }
    else
    {
      m_httpClient.end();
      Logger::log(LogLevel::ERROR, "Shelly: HTTP request %s failed with code '%s'",
                  requestStr.c_str(),
                  m_httpClient.errorToString(httpCodeStatus).c_str());
    }
  }
  else
  {
    m_httpClient.end();
    Logger::log(LogLevel::ERROR, "Shelly: HTTP url parsing %s failed", requestStr.c_str());
  }

  m_error = !(success);

  return success;
}

float Shelly::getActivePower() const
{
  return m_activePower;
}

bool Shelly::isNewMeasure()
{
  bool newMeasure = m_newMeasure;

  if (m_newMeasure)
  {
    m_newMeasure = false;
  }

  return newMeasure;
}

bool Shelly::getErrorStatus()
{
  return m_error;
}

void Shelly::loop()
{
  if ((m_coap != nullptr) && (m_type == ShellyType::SHEM_GEN1))
  {
    m_coap->loop();
  }
}

void Shelly::coapResponseCallbackImpl(CoapPacket &packet, IPAddress ip, int port)
{
  char jsonBuffer[packet.payloadlen + 1];
  JsonDocument doc;
  bool success = false;

  memcpy(jsonBuffer, packet.payload, packet.payloadlen);
  jsonBuffer[packet.payloadlen] = '\0';

  // Parse JSON using ArduinoJson
  DeserializationError error = deserializeJson(doc, jsonBuffer);

  if (error == DeserializationError::Ok)
  {
    if ((doc["G"].is<JsonArray>()))
    {
      JsonArray G = doc["G"].as<JsonArray>();

      // Iterate through all measurements
      for (JsonArray item : G)
      {
        if (item.size() >= 3)
        {
          uint8_t channel = item[0];
          uint16_t code = item[1];

          // Process only channel 0
          if ((channel == 0) && (code == COIOT_SHELLY_ACTIVE_POWER_ID))
          {
            m_activePower = item[2];

            if (m_error)
            {
              Logger::log(LogLevel::INFO, "Shelly: back OK, active Power = %f W", m_activePower);
            }
            else
            {
              Logger::log(LogLevel::TRACE, "Shelly: active Power = %f W", m_activePower);
            }
            success = true;
            m_newMeasure = true;
            break;
          }
        }
      }

      if (success == false)
      {
        error = DeserializationError::IncompleteInput;
      }
    }
    else
    {
      error = DeserializationError::InvalidInput;
    }
  }

  if (success == false)
  {
    Logger::log(LogLevel::ERROR, "Shelly: JSON deserialization failed with error %s", error.c_str());
  }

  m_error = !(success);
}

/** \private interfaces */

/**
 * @brief  Request to Shelly the settings information in order to recognize version of it
 *
 * @return true if success
 */
bool Shelly::updateSettings(void)
{
  String requestStr = "http://" + m_IP + "/settings";
  bool success = false;
  int httpCodeStatus;

  success = m_httpClient.begin(m_espClient, requestStr);

  if (success)
  {
    httpCodeStatus = m_httpClient.GET();

    if (httpCodeStatus == (int)HTTP_CODE_OK)
    {
      JsonDocument doc;
      String payload = m_httpClient.getString();
      m_httpClient.end();

      Logger::log(LogLevel::TRACE, "Shelly: raw json settings = %s ", payload.c_str());
      DeserializationError error = deserializeJson(doc, payload);

      if (error == DeserializationError::Ok)
      {
        // Extract Device Information
        const char *deviceType = doc["device"]["type"];
        const char *hostname = doc["device"]["hostname"];

        // Extract Firmware Information
        const char *firmware = doc["fw"];

        // Extract CoIoT Information
        bool coiotEnabled = doc["coiot"]["enabled"];
        int coiotUpdatePeriod = doc["coiot"]["update_period"];

        Logger::log(LogLevel::INFO, "Shelly: deviceType= %s, hostname= %s, firmware=%s, coiot=%i (period = %i)",
                    deviceType, hostname, firmware, coiotEnabled, coiotUpdatePeriod);

        if (strcmp(deviceType, "SHEM") == 0)
        {
          Logger::log(LogLevel::INFO, "Shelly: SHEM_GEN1 detected");
          m_type = ShellyType::SHEM_GEN1;
          m_coIotEnabled = coiotEnabled;
          m_nbChannels = 2;
        }
        else if (strcmp(deviceType, "SHEM-3") == 0)
        {
          Logger::log(LogLevel::WARNING, "Shelly: SHEM_TRI_GEN1 detected : not yet implemented");
          m_type = ShellyType::SHEM_TRI_GEN1;
          m_coIotEnabled = coiotEnabled;
          m_nbChannels = 3;
        }
        else if (strstr(hostname, "shellyproem50-") ||
                 strstr(hostname, "shellypro1em50-"))
        {
          Logger::log(LogLevel::WARNING, "Shelly: SHEM_PRO_GEN2 detected : not yet implemented");
          m_type = ShellyType::SHEM_PRO_GEN2;
          m_coIotEnabled = false;
          m_nbChannels = 2;
        }
        else if (strstr(hostname, "shellypro3em-"))
        {
          Logger::log(LogLevel::WARNING, "Shelly: SHEM_PRO_GEN2 detected : not yet implemented");
          m_type = ShellyType::SHEM_TRI_PRO_GEN2;
          m_coIotEnabled = false;
          m_nbChannels = 3;
        }
        else
        {
          /* No yet managed or not an EM Shelly !*/
          Logger::log(LogLevel::ERROR, "Shelly: not a Shelly EM device !");
          m_type = ShellyType::UNKNOWN;
          m_coIotEnabled = false;
        }
      }
      else
      {
        Logger::log(LogLevel::ERROR, "Shelly: JSON deserialization failed: %s", error.c_str());
      }
    }
    else
    {
      m_httpClient.end();
      Logger::log(LogLevel::ERROR, "Shelly: HTTP request %s failed with code '%s'",
                  requestStr.c_str(),
                  m_httpClient.errorToString(httpCodeStatus).c_str());
    }
  }
  else
  {
    m_httpClient.end();
    Logger::log(LogLevel::ERROR, "Shelly: HTTP url parsing %s failed", requestStr.c_str());
  }

  if (m_coIotEnabled)
  {
    Logger::log(LogLevel::INFO, "Shelly: power read protocol used is coIot");
  }
  else
  {
    Logger::log(LogLevel::INFO, "Shelly: power read protocol used is HTTP");
  }

  return success;
}

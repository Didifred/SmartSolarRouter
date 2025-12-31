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
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "Shelly.h"
#include "Logger.h"

/** \public interfaces */

Shelly::Shelly(WiFiClient &wifiClient) : m_espClient(wifiClient)
{
  /* could be also hostname "shellyem-xxxxxxx" */
  m_IP = "0.0.0.0";
  m_activePower = 0.0f;
  m_error = false;
}

void Shelly::setIpAddress(const String &ip)
{
  m_IP = ip;
}

void Shelly::updateSettings(void)
{
  int httpCodeStatus;

  m_httpClient.begin(m_espClient, "http://" + m_IP + "/settings");

  httpCodeStatus = m_httpClient.GET();

  if (httpCodeStatus == HTTP_CODE_OK)
  {
    /* TODO */
    /* shall be done to detect kind of Shelly : Tri etc */
    /* to inform about hostname of shelly in console*/
  }

  m_httpClient.end();
}

bool Shelly::updateMeasures(void)
{
  int httpCodeStatus;
  bool success = false;

  m_httpClient.begin(m_espClient, "http://" + m_IP + "/emeter/0");
  httpCodeStatus = m_httpClient.GET();

  if (httpCodeStatus == HTTP_CODE_OK)
  {
    String payload = m_httpClient.getString();
    m_httpClient.end();

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!doc.is<JsonVariant>())
    {
      error = DeserializationError::InvalidInput;
    }

    if (error == DeserializationError::Ok)
    {
      m_activePower = doc["power"].as<float>();

      if (m_error)
      {
        Logger::log(LogLevel::INFO, "Shelly: back OK, active Power = %f W", m_activePower);
      }
      success = true;
    }
    else
    {
      Logger::log(LogLevel::ERROR, "Shelly: JSON deserialization failed: %s", error.c_str());
    }
  }
  else
  {
    m_httpClient.end();
    Logger::log(LogLevel::ERROR, "Shelly: HTTP request failed with code '%s'", m_httpClient.errorToString(httpCodeStatus).c_str());
  }

  m_error = !(success);

  return success;
}

float Shelly::getActivePower() const
{
  return m_activePower;
}

/** \private interfaces */

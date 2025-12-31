#ifndef SHELLY_H
#define SHELLY_H

//***********************************
//************* LIBRAIRIES ESP
//***********************************
// #include <Arduino.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>

class Shelly
{
public:
  /**
   * @brief Constructs a Shelly object with the specified WiFi client.
   * @param wifiClient Reference to a WiFiClient object used for network communication.
   */
  Shelly(WiFiClient &wifiClient);

  /**
   * @brief Sets the IP address or Hostname for the device.
   * @param ip The IP address or Hostname to set, provided as a String.
   */
  void setIpAddress(const String &ip);

  /**
   * @brief Request to Shelly the settings information in order to recognize version of it
   */
  void updateSettings(void);

  /**
   * @brief Updates the measured power value from the Shelly device via HTTP request.
   * This function sends an HTTP GET request to the Shelly device to retrieve the current power measurement.
   * @return true if the power measurement was successfully updated; false otherwise.
   */
  bool updateMeasures(void);

  /**
   * @brief Retrieves the latest active power measurement.
   * @return The active power value in watts.
   */
  float getActivePower() const;

private:
  float m_activePower;
  bool m_error;
  String m_IP;
  HTTPClient m_httpClient;
  WiFiClient &m_espClient;
};

#endif // SHELLY_H

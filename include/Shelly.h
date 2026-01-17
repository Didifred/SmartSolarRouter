#ifndef SHELLY_H
#define SHELLY_H

#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>

enum class ShellyType
{
  SHEM_GEN1 = 0,
  SHEM_TRI_GEN1,
  SHEM_PRO_GEN2,
  SHEM_TRI_PRO_GEN2,
  UNKNOWN
};

class Shelly
{
public:
  /**
   * @brief Constructs a Shelly object with the specified WiFi client.
   * @param wifiClient Reference to a WiFiClient object used for network communication.
   */
  Shelly(WiFiClient &wifiClient, WiFiUDP &udp);

  /**
   * @brief Setup shelly device
   * @param ip Ip address provided as a string
   */
  void setup(const String &ip);

  /**
   * Setup CoIot handling
   */
  void setupCoIoT(CoapCallback callback);

  /**
   * @brief Is CoIot protocol is enabled ?
   *
   * @return true
   * @return false
   */
  bool isCoIotEnabled(void);

  /**
   * @brief Coap callback implementation
   */
  void coapResponseCallbackImpl(CoapPacket &packet, IPAddress ip, int port);

  /**
   * @brief Get the Shelly Type enum
   *
   * @return ShellyType
   */
  ShellyType getShellyType(void);

  /**
   * @brief Update measures
   */
  bool updateMeasures(void);

  /**
   * @brief Updates the measured power value from the Shelly device via HTTP request.
   * This function sends an HTTP GET request to the Shelly device to retrieve the current power measurement.
   * @return true if the power measurement was successfully updated; false otherwise.
   */
  bool updateMeasuresHttp(void);

  /**
   * @brief Updates the measured power value from the Shelly device via CoIOT request.
   * @return true if the power measurement was successfully updated; false otherwise.
   */
  bool updateMeasuresCoIot(void);

  /**
   * @brief Retrieves the latest active power measurement.
   * @return The active power value in watts.
   */
  float getActivePower() const;

  /**
   * @brief Checks if a new measurement is available.
   * @return true if a new measurement is available; false otherwise.
   */
  bool isNewMeasure();

  /**
   * @brief  Check power reading on shelly is Ok
   * @return true if error on reading power
   */
  bool getErrorStatus();

  /**
   * @brief Loop to be called inside main loop
   *
   */
  void loop();

private:
  bool updateSettings(void);

  ShellyType m_type;
  bool m_coIotEnabled;
  uint8_t m_nbChannels;
  float m_activePower;
  bool m_error;
  String m_IP;
  HTTPClient m_httpClient;
  WiFiClient &m_espClient;
  WiFiUDP &m_udp;
  Coap *m_coap;
  bool m_newMeasure;
};

#endif // SHELLY_H

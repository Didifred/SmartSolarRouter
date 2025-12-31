#ifndef NETWORK_H
#define NETWORK_H

#include <WString.h>

class Network
{

  /**
   * @class Network
   * @brief Static class related to networking.
   */

public:
  Network() = delete; // Prevent instantiation of this class
  ~Network() = delete;

  /**
   * @brief Initialize network stuff
   *        To be called in setup()
   */
  static void begin(void);

  /**
   * @brief To be called in the main loop
   */
  static void loop(void);

  /**
   * @brief return captive portal state
   * @return true if in captive portal
   */
  static bool isWifiMngCaptivePortal(void);

  /**
   * @brief Get the device hostname based on MAC address
   * @return Device hostname as String
   */
  static String &getDeviceHostname(void);

private:
  /**
   * @brief Initialize mdns hello servives.
   * @param domainName The domain name to use for mDNS.
   * @return true if mdns started successfully, false otherwise.
   */
  static bool mdnsBegin(const char *hostname);

  static String m_deviceHostname;
};

#endif /* NETWORK_H */

/**
 * @file Logger.cpp
 * @brief Implementation of the Logger class for logging messages with different log levels and timestamps.
 *
 * This file provides the implementation for the Logger class, which supports logging messages
 * to the serial output with various log levels (NONE, DEBUG, INFO, WARNING, ERROR). It formats
 * log messages with a timestamp (based on system uptime) and the log level. The logger supports
 * both String and printf-style formatted messages. The log level can be set to control the verbosity
 * of the output.
 *
 * Public Interfaces:
 * - void Logger::init(void): Initializes the logger and sets the default log level.
 * - void Logger::log(LogLevel logLevel, const String& message): Logs a message with a specified log level.
 * - void Logger::log(LogLevel logLevel, const char* message, ...): Logs a formatted message with a specified log level.
 * - void Logger::setLogLevel(LogLevel level): Sets the current log level for filtering messages.
 *
 * Private Interfaces:
 * - String Logger::getTimestamp(): Returns a formatted timestamp string based on system uptime.
 * - String Logger::logLevelToString(LogLevel logLevel): Converts a log level enum to its string representation.
 *
 * Dependencies:
 * - Logger.h: Logger class definition.
 *
 * Usage:
 * 1. Call Logger::init() to initialize the logger.
 * 2. Use Logger::log() methods to log messages at various log levels.
 * 3. Optionally, adjust the log level using Logger::setLogLevel().
 */

#include <HardwareSerial.h>
#include <stdio.h>
#include "Logger.h"

/** \public interfaces */

bool Logger::m_initDone = false;
LogLevel Logger::m_logLevel = LogLevel::NONE;
bool Logger::m_teleplotUdpOn = false;
WiFiUDP *Logger::m_udp = nullptr;
const char *Logger::m_hostIp = "";
uint16_t Logger::m_udpPort = 0;
uint64_t Logger::m_baseTimeEpochMs = 0;
uint32_t Logger::m_baseMillis = 0;
String *Logger::m_logBuffer = nullptr;

void Logger::init(void)
{
  // Serial init
  Serial.begin(115200);
  Serial.println("");

  m_initDone = true;
}

void Logger::syncTime(void)
{
  time_t now;

  time(&now);
  m_baseTimeEpochMs = now * 1000;
  m_baseMillis = (uint32_t)millis();
}

void Logger::setupTeleplotUdp(WiFiUDP *udp, const char *hostIp, const uint16_t port)
{
  m_udp = udp;
  m_hostIp = hostIp;
  m_udpPort = port;
}

void Logger::log(const LogLevel logLevel, const String &message)
{
  if (logLevel >= m_logLevel)
  {
    String logMessage = " [" + logLevelToString(logLevel) + "] " + message;

    output(logMessage);
  }
}

void Logger::log(const LogLevel logLevel, const char *message, ...)
{
  // Nothing to log if loglevel below current one */
  if (logLevel >= m_logLevel)
  {
    va_list args;
    char stackBuffer[256]; // Stack buffer for small messages
    int sizeNeeded = 0;
    String logMessage;

    // First, try to format the message into the stack buffer
    va_start(args, message);
    sizeNeeded = vsnprintf(stackBuffer, sizeof(stackBuffer), message, args);
    va_end(args);

    if ((sizeNeeded < (int)sizeof(stackBuffer)) && (sizeNeeded >= 0))
    {
      // Fits in stack buffer
      logMessage = " [" + logLevelToString(logLevel) + "] " + String(stackBuffer);
    }
    else if (sizeNeeded >= 0)
    {
      // Allocate enough space on the heap
      char *heapBuffer = new char[sizeNeeded + 1];
      va_start(args, message);
      vsnprintf(heapBuffer, sizeNeeded + 1, message, args);
      va_end(args);

      logMessage = " [" + logLevelToString(logLevel) + "] " + String(heapBuffer);
      delete[] heapBuffer;
    }
    else
    {
      // vsnprintf error
      logMessage = " [" + logLevelToString(logLevel) + "] " + String("Log formatting error");
    }

    output(logMessage);
  }
}

void Logger::setLogLevel(const LogLevel level)
{
  if (level != m_logLevel)
  {
    log(LogLevel::INFO, "Logger is set to " + logLevelToString(level) + " level");
    m_logLevel = level;
  }
}

void Logger::enableTeleplotUdp(const bool enable)
{
  m_teleplotUdpOn = enable;
}

void Logger::plot(const String &varname, const String &value,
                  std::optional<String> unit)
{

  if (m_teleplotUdpOn && (m_udp != nullptr))
  {
    uint64_t epochMs;
    StringType valueType = getStringType(value);
    String flag;

    // Send to Teleplot server
    // See plot format in https://github.com/nesnes/teleplot/blob/main/README.md#telemetry-format
    getTimestampMs(&epochMs);
    if (valueType == StringType::IS_TEXT)
    {
      flag.concat("|t");
    }

    m_udp->beginPacket(m_hostIp, m_udpPort);
    if (unit.has_value())
    {
      m_udp->printf("%s:%llu:%s§%s%s\n", varname.c_str(), epochMs, value.c_str(), unit.value().c_str(), flag.c_str());
    }
    else
    {
      m_udp->printf("%s:%llu:%s%s\n", varname.c_str(), epochMs, value.c_str(), flag.c_str());
    }
    m_udp->endPacket();
  }
}

/** \private interfaces */

/**
 * @brief Get timestamp
 *
 * @param epochMs Pointer to receive epoch time in milliseconds
 * @return Time stamp in HH:MM:SS.mmm format
 */
String Logger::getTimestampMs(uint64_t *epochMs)
{
  String timestamp;
  char buffer[13]; // "HH:MM:SS.mmm\0" = 13 chars
  time_t now;
  struct tm *local;
  uint16_t milliseconds;

  // Get current epoch time in milliseconds
  *epochMs = m_baseTimeEpochMs + (uint32_t)(millis() - m_baseMillis);

  // Convert epoch time in seconds to local time structure
  now = *epochMs / 1000;
  local = localtime(&now);
  // Get milliseconds part
  milliseconds = (*epochMs) % 1000;

  // Format timestamp as HH:MM:SS.mmm
  sprintf(buffer, "%02i:%02i:%02i.%03i",
          local->tm_hour,
          local->tm_min,
          local->tm_sec,
          milliseconds);
  timestamp.concat(buffer);

  return timestamp;
}

/**
 * @brief Stringtify logLevel
 *
 * @param logLevel  logLevel among debug, info, error
 * @return String representing logLevel
 */
String Logger::logLevelToString(const LogLevel logLevel)
{
  String s;

  switch (logLevel)
  {
  case LogLevel::NONE:
    s = "NONE";
    break;

  case LogLevel::DEBUG:
    s = "DEBUG";
    break;

  case LogLevel::INFO:
    s = "INFO";
    break;

  case LogLevel::WARNING:
    s = "WARNING";
    break;

  case LogLevel::ERROR:
    s = "ERROR";
    break;

  default:
    s = "UNKNOWN";
    break;
  }

  return s;
}

/**
 * @brief Output a log on Serial and Teleplot Udp if on
 * @param logMsg Message to printed on outputs
 */
void Logger::output(const String &logMessage)
{
  String traceLog;
  uint64_t epochMs;
  // Timestamp in HH:MM:SS.mmm
  String timestamp = getTimestampMs(&epochMs);

  traceLog = timestamp + logMessage;

  if (m_initDone)
  {
    if (m_logBuffer != nullptr)
    {
      // Flush buffered logs first
      Serial.print(*m_logBuffer);
      delete m_logBuffer;
      m_logBuffer = nullptr;
    }
    Serial.println(traceLog);
  }
  else
  {
    // Serial not initialized yet
    if (m_logBuffer == nullptr)
    {
      m_logBuffer = new String();
    }

    if (m_logBuffer != nullptr)
    {
      m_logBuffer->concat(traceLog + "\n");
    }
  }

  // + write in file in the future

  if (m_teleplotUdpOn)
  {
    if (m_udp != nullptr)
    {
      // Send to Teleplot server
      m_udp->beginPacket(m_hostIp, m_udpPort);
      m_udp->printf(">%llu:%s\n", epochMs, logMessage.c_str());
      m_udp->endPacket();
    }
  }
}

/**
 * @brief Get the String Type object
 *
 * @param str
 * @return StringType enum { IS_TEXT, IS_EMPTY, IS_INTEGER, IS_FLOAT}
 */
StringType Logger::getStringType(const String &str)
{
  StringType result = StringType::IS_TEXT; // Default to text

  if (str.length() == 0)
  {
    result = StringType::IS_EMPTY;
  }
  else
  {
    bool hasDecimal = false;
    bool hasDigit = false;
    bool isValid = true;
    uint32_t startIndex = 0;

    // Check for sign
    if ((str[0] == '-') || (str[0] == '+'))
    {
      if (str.length() == 1)
      {
        isValid = false;
      }
      else
      {
        startIndex = 1;
      }
    }

    // Validate characters if still valid
    if (isValid)
    {
      for (uint32_t i = startIndex; i < str.length(); i++)
      {
        if (isdigit(str[i]))
        {
          hasDigit = true;
        }
        else if (str[i] == '.')
        {
          if (hasDecimal)
          {
            isValid = false; // Multiple dots
            break;
          }
          hasDecimal = true;
        }
        else
        {
          isValid = false; // Contains non-numeric char
          break;
        }
      }
    }

    // Determine final type
    if (isValid && hasDigit)
    {
      result = hasDecimal ? StringType::IS_FLOAT : StringType::IS_INTEGER;
    }
  }

  return result;
}

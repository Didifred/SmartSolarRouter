
#ifndef LOGGER_H
#define LOGGER_H

#include <WString.h>
#include <WiFiUdp.h>

enum class LogLevel
{
    NONE = 0,
    DEBUG ,
    INFO ,
    WARNING ,
    ERROR 
};

enum class StringType
{
    IS_EMPTY = 0,
    IS_TEXT,
    IS_INTEGER,
    IS_FLOAT
};

class Logger
/**
 * @class Logger
 * @brief Static utility class for logging messages with different log levels.
 *
 * The Logger class provides static methods to initialize the logging system,
 * log messages at various severity levels, and configure the minimum log level.
 * Instantiation of this class is prevented.
 *
 * Usage:
 *   Logger::init();
 *   Logger::setLogLevel(LogLevel::INFO);
 *   Logger::log(LogLevel::ERROR, "An error occurred");
 *
 * Private helper methods are used for timestamp generation and log level string conversion.
 */
{
public:
    Logger() = delete; // Prevent instantiation of this class
    ~Logger() = delete;


    /**
     * @brief Initializes the Logger module.
     * 
     * @param udp
     * @param hostIp
     */
    static void init(void);

    /**
     * @brief Setup teleplot over udp
     * 
     * @param udp  Udp socket
     * @param hostIp host ip
     * @param port udp port
     */
    static void setupTeleplotUdp(WiFiUDP* udp, const char* hostIp, const uint16_t port);

    /**
     * @brief Sync the time for logger module
     * 
     * @param epoch 
     */
    static void syncTime();

    /**
     * @brief Logs a message with the specified log level.
     * 
     * @param logLevel The severity level of the log message.
     * @param message The message to be logged.
     */
    static void log(const LogLevel logLevel, const String &message);

    /**
     * @brief Logs a formatted message with the specified log level.
     * This method supports printf-style formatting.
     * 
     * @param logLevel The severity level of the log message.
     * @param message The format string for the log message.
     * @param ... Additional arguments for formatting in C-printf style.
     */
    static void log(const LogLevel logLevel, const char* message, ...);

    /**
     * @brief Sets the minimum log level for messages to be logged.
     * Messages with a severity level below this level will be ignored.
     * 
     * @param level The minimum log level to set.
     */
    static void setLogLevel(const LogLevel level);

    /**
     * @brief Enable or disable teleplot over UDP
     * 
     * @param enable 
     */
    static void enableTeleplotUdp(const bool enable);

    /**
     * @brief Plot the variable time based
     * 
     * @param varname The name of the variable
     * @param value The value in string
     * @param unit An optional string for unit
     */
    static void plot(const String& varname, const String& value, 
                     std::optional<String> unit = std::nullopt);


private:
    static String getTimestampMs(uint64_t* epochMs = nullptr);
    static String logLevelToString(const LogLevel logLevel);
    static void output(const String& logMessage);
    static StringType getStringType(const String& str);

    static LogLevel m_logLevel;
    static bool m_teleplotUdpOn;
    static WiFiUDP* m_udp;
    static const char* m_hostIp;
    static uint16_t m_udpPort;
    static uint64_t m_baseTimeEpochMs;
    static uint32_t m_baseMillis;
};

#endif // LOGGER_H
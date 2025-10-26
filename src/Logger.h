
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

enum class LogLevel
{
    NONE = 0,
    DEBUG ,
    INFO ,
    WARNING ,
    ERROR 
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
     */
    static void init(void);


    /**
     * @brief Logs a message with the specified log level.
     * 
     * @param logLevel The severity level of the log message.
     * @param message The message to be logged.
     */
    static void log(LogLevel logLevel, const String &message);

    /**
     * @brief Logs a formatted message with the specified log level.
     * This method supports printf-style formatting.
     * 
     * @param logLevel The severity level of the log message.
     * @param message The format string for the log message.
     * @param ... Additional arguments for formatting in C-printf style.
     */
    static void log(LogLevel logLevel, const char* message, ...);

    /**
     * @brief Sets the minimum log level for messages to be logged.
     * Messages with a severity level below this level will be ignored.
     * 
     * @param level The minimum log level to set.
     */
    static void setLogLevel(LogLevel level);

private:
    static String getTimestamp();
    static String logLevelToString(LogLevel logLevel);

    static LogLevel m_logLevel;
};

#endif // LOGGER_H
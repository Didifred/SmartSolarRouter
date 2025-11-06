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
 * - uptime.h: Provides system uptime information for timestamping log messages.
 *
 * Usage:
 * 1. Call Logger::init() to initialize the logger.
 * 2. Use Logger::log() methods to log messages at various log levels.
 * 3. Optionally, adjust the log level using Logger::setLogLevel().
 */
#include <HardwareSerial.h>
#include <stdio.h>
#include "Logger.h"
#include "uptime.h"

/** \public interfaces */

LogLevel Logger::m_logLevel = LogLevel::NONE;

void Logger::init(void)
{
    m_logLevel = LogLevel::DEBUG;
}

void Logger::log(LogLevel logLevel, const String& message)
{
    if (logLevel >= m_logLevel)
    {
        String logMessage = getTimestamp() + " [" + logLevelToString(logLevel) + "] " + message;
        Serial.println(logMessage);
    }
}

void Logger::log(LogLevel logLevel, const char* message, ...)
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
            logMessage = getTimestamp() + " [" + logLevelToString(logLevel) + "] " + String(stackBuffer);
        }
        else if (sizeNeeded >= 0)
        {
            // Allocate enough space on the heap
            char* heapBuffer = new char[sizeNeeded + 1];
            va_start(args, message);
            vsnprintf(heapBuffer, sizeNeeded + 1, message, args);
            va_end(args);

            logMessage = getTimestamp() + " [" + logLevelToString(logLevel) + "] " + String(heapBuffer);
            delete[] heapBuffer;
        }
        else
        {
            // vsnprintf error
            logMessage = getTimestamp() + " [" + logLevelToString(logLevel) + "] " + String("Log formatting error");
        }
        
        Serial.println(logMessage);
        // + write to file 
    }
}

void Logger::setLogLevel(LogLevel level)
{
    m_logLevel = level;
}

/** \private interfaces */
String Logger::getTimestamp()
{
  String timestamp;

  uptime::calculateUptime();
  timestamp = String(uptime::getHours()) +":"+
              String(uptime::getMinutes()) +":"+
              String(uptime::getSeconds()) +"."+
              String(uptime::getMilliseconds());

   return timestamp;
}

String Logger::logLevelToString(LogLevel logLevel)
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
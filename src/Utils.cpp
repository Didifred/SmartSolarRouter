
#include <Arduino.h>
#include "Utils.h"
#include "Logger.h"

void Utils::checkEspFlash()
{
  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Logger::log(LogLevel::INFO, "Flash real id:   %08X", ESP.getFlashChipId());
  Logger::log(LogLevel::INFO, "Flash real size: %u bytes\n", realSize);

  Logger::log(LogLevel::INFO,"Flash ide  size: %u bytes", ideSize);
  Logger::log(LogLevel::INFO,"Flash ide speed: %u Hz", ESP.getFlashChipSpeed());
  Logger::log(LogLevel::INFO, "Flash ide mode:  %s", (ideMode == FM_QIO ? "QIO" : 
                                                        ideMode == FM_QOUT ? "QOUT" : 
                                                        ideMode == FM_DIO  ? "DIO" :
                                                        ideMode == FM_DOUT ? "DOUT" : "UNKNOWN") );

  if (ideSize != realSize) 
  {
    Logger::log(LogLevel::WARNING, "Flash Chip configuration wrong!");
  } 
  else 
  {
    Logger::log(LogLevel::INFO, "Flash Chip configuration ok.");
  }

}
#include <Esp.h>
#include "Utils.h"
#include "Logger.h"

#define TIMER1_FREQ_HZ 80000000UL

void Utils::checkEspFlash()
{
  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Logger::log(LogLevel::INFO, "Flash real id:   %08X", ESP.getFlashChipId());
  Logger::log(LogLevel::INFO, "Flash real size: %u bytes\n", realSize);

  Logger::log(LogLevel::INFO, "Flash ide  size: %u bytes", ideSize);
  Logger::log(LogLevel::INFO, "Flash ide speed: %u Hz", ESP.getFlashChipSpeed());
  Logger::log(LogLevel::INFO, "Flash ide mode:  %s", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT"
                                                                              : ideMode == FM_DIO    ? "DIO"
                                                                              : ideMode == FM_DOUT   ? "DOUT"
                                                                                                     : "UNKNOWN"));

  if (ideSize != realSize)
  {
    Logger::log(LogLevel::WARNING, "Flash Chip configuration wrong!");
  }
  else
  {
    Logger::log(LogLevel::INFO, "Flash Chip configuration ok.");
  }
}

void Utils::initHwTimer(uint8_t frequency, timercallback userFunc)
{
  uint32_t periodTicks = (TIMER1_FREQ_HZ / (frequency));
  float periodMs = ((float)periodTicks / (TIMER1_FREQ_HZ / 1000));

  timer1_attachInterrupt(userFunc);
  timer1_write(periodTicks);

  Logger::log(LogLevel::INFO, "Init ISR timer1 with period : %f ms", periodMs);
}

void Utils::disableHwTimer(void)
{
  timer1_disable();
}

void Utils::enableHwTimer(void)
{
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
}

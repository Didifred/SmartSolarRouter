#include <Arduino.h>
#include "Dimmer.h"
#include "Logger.h"
#include "hwConfig.h"


Dimmer::Dimmer(uint8_t nbChannels)
{
  
}


void Dimmer::update(void)
{
  static uint8_t count = 0;
  static uint16_t channel1 = 0;
  static uint16_t channel2 = 0;
  static uint16_t channel3 = 0;

  count++;

  if (count % 2 == 0)
  {
    channel1 = !channel1;
    digitalWrite(SSR1_Pin, channel1);
  }

  if (count % 4 == 0)
  {
    channel2 = !channel2;
    digitalWrite(SSR2_Pin, channel2);
  } 

  if (count % 8 == 0)
  {
    channel3 = !channel3;
    digitalWrite(SSR3_Pin, channel3);
  } 

}

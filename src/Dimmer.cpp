#include <Arduino.h>
#include <dashboard.h>
#include "Dimmer.h"
#include "Logger.h"
#include "libPID.h"


Dimmer::Dimmer(uint8_t nbChannels, uint8_t gridFrequency) : m_pid(1.0f, 0.0f, 0.0f, PidDirection::DIRECT)
{
  if (nbChannels > MAX_DIMMER_CHANNELS)
  {
    Logger::log(LogLevel::WARNING, "Dimmer: nbChannels (%d) exceeds MAX_DIMMER_CHANNELS (%d). \
                                    Limiting to MAX_DIMMER_CHANNELS.", nbChannels, MAX_DIMMER_CHANNELS);
    nbChannels = MAX_DIMMER_CHANNELS;
  }
  m_nbChannels = nbChannels;
  m_gridFrequency = gridFrequency;
  m_ssrPinStates = new bool[nbChannels];
  m_pinMapping = new uint8_t[nbChannels];
  m_state = false;

  for (uint8_t i = 0; i < nbChannels; i++)
  {
    m_ssrPinStates[i] = false;
    m_pinMapping[i] = 0;
  }

  // TODO setup it dynamically
  m_pid.SetOutputLimits(0, 3000);

  m_pid.SetSampleTime(1000/gridFrequency);
}

Dimmer::~Dimmer()
{
  delete[] m_ssrPinStates;
  delete[] m_pinMapping;
} 

void Dimmer::mapChannelToPin(uint8_t channel, uint8_t pin)
{
  if (channel < m_nbChannels)
  {
    m_pinMapping[channel] = pin;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  else
  {
    Logger::log(LogLevel::ERROR, "Dimmer: mapChannelPin: channel (%d) exceeds number of channels (%d).",
                channel, m_nbChannels);
  }
}

void Dimmer::turnOff(void)
{
  m_state = false;
  for (uint8_t i = 0; i < m_nbChannels; i++)
  {
    m_ssrPinStates[i] = false;
    digitalWrite(m_pinMapping[i], LOW);
  }
}

void Dimmer::turnOn(void)
{
  m_state = true;
}

void Dimmer::update(float_t gridPower)
{ 
  static float_t HeaterPower = 0.0f;

  // Simulate a solar production of 1500 W 
  gridPower -= 1500;
  gridPower += HeaterPower;

  HeaterPower = m_pid.Compute(gridPower, -50);

  // Update graphic on dashboard
  dash.data.powerRouted = HeaterPower;

}

IRAM_ATTR void Dimmer::updateChannelsOutput(void)
{
  /* Todo handle a simple circular buffer to pass commands*/
  static uint8_t count = 0;

  if (m_state != false)
  {
    count++;

    if (count % 2 == 0)
    {
      m_ssrPinStates[0] = !m_ssrPinStates[0];
      digitalWrite(m_pinMapping[0], m_ssrPinStates[0]);
    }

    if (count % 4 == 0)
    {
      m_ssrPinStates[1] = !m_ssrPinStates[1];
      digitalWrite(m_pinMapping[1], m_ssrPinStates[1]);
    } 

    if (count % 8 == 0)
    {
      m_ssrPinStates[2] = !m_ssrPinStates[2];
      digitalWrite(m_pinMapping[2], m_ssrPinStates[2]);
    } 
  }
  else
  {
    for (uint8_t i = 0; i < m_nbChannels; i++)
    {
      m_ssrPinStates[i] = false;
      digitalWrite(m_pinMapping[i], LOW);
    }
  }
}

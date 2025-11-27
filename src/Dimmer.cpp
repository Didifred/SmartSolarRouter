#include <Arduino.h>
#include <dashboard.h>
#include "Dimmer.h"
#include "Logger.h"
#include "libPID.h"


Dimmer::Dimmer(uint8_t nbChannels, uint8_t gridFrequency, uint16_t sampleTime) : m_pid(0.4f, 1.5f, 0.0f, PidDirection::DIRECT)
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

  // TODO setup it with configuration values in EEPROM
  m_pid.SetOutputLimits(0, 3000);
  m_pid.SetSampleTime(sampleTime);

  // Initialize dashboard PID values for simulation
  dash.data.Kp = m_pid.GetKp();
  dash.data.Ki = m_pid.GetKi();
  dash.data.Kd = m_pid.GetKd();
  dash.data.solarPowerSimul = 1500.0f;
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
  static float_t  HeaterPower = 0.0f;
  static float_t  InstantHeaterPower[NB_PID_SAMPLES] = {0.0f};
  static uint8_t  DelayMeasureCount = 0;
  static uint16_t InputCount = 0;

  // Simulate a solar production of 1500 W during first 10s
  if (InputCount < (10000 / m_pid.GetSampleTime()))
  {
    gridPower -= dash.data.solarPowerSimul;
  }
  InputCount++;

  if (InputCount >  (20000 / m_pid.GetSampleTime()))
  {
    InputCount = 0;
  }
  
  // Simulation retroaction 500ms
  if (DelayMeasureCount >= NB_PID_SAMPLES)
  {
    HeaterPower = 0;
    for (uint8_t i = 0; i < NB_PID_SAMPLES; i++)
    {
      HeaterPower += InstantHeaterPower[i];
    }
    HeaterPower /= NB_PID_SAMPLES;
    DelayMeasureCount = 0;
  }
  gridPower += HeaterPower;

  
  // Check if parameters changes
  if (m_pid.GetKp() != dash.data.Kp ||
      m_pid.GetKi() != dash.data.Ki ||
      m_pid.GetKd() != dash.data.Kd )
  {
    Logger::log(LogLevel::INFO, "Dimmer: PID parameters updated Kp=%.3f Ki=%.3f Kd=%.3f",
                dash.data.Kp, dash.data.Ki, dash.data.Kd);

    m_pid.SetTunings(dash.data.Kp, dash.data.Ki, dash.data.Kd);

    m_pid.Initialize(gridPower, HeaterPower);
  }

  // PID compute
  InstantHeaterPower[DelayMeasureCount] = m_pid.Compute(gridPower, -50);


  Logger::plot("Dimmer.GridPower", String(gridPower), "W");
  Logger::plot("Dimmer.HeaterPower", String(InstantHeaterPower[DelayMeasureCount]), "W");

  DelayMeasureCount++;
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

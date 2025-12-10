#include <Arduino.h>
#include <dashboard.h>
#include "Dimmer.h"
#include "Logger.h"
#include "libPID.h"


Dimmer::Dimmer(uint8_t nbChannels, uint8_t gridFrequency, uint16_t samplePeriod, uint16_t measurePeriod) : 
        m_pid(0.25f, 0.9f, 0.0f, PidProportionalOption::ON_ERROR, PidDirection::DIRECT)
{
  if (nbChannels > MAX_DIMMER_CHANNELS)
  {
    Logger::log(LogLevel::WARNING, "Dimmer: nbChannels (%d) exceeds MAX_DIMMER_CHANNELS (%d). \
                                    Limiting to MAX_DIMMER_CHANNELS.", nbChannels, MAX_DIMMER_CHANNELS);
    nbChannels = MAX_DIMMER_CHANNELS;
  }
  m_nbChannels = nbChannels;
  m_gridFrequency = gridFrequency;
  m_measurePeriod = measurePeriod;
  m_ssrPinStates = new bool[nbChannels];
  m_pinMapping = new uint8_t[nbChannels];
  m_outputs_array_size = measurePeriod / samplePeriod;
  m_outputs = new float_t[m_outputs_array_size];
  m_outputs_index = 0;
  m_previousGridPower = 0.0f;
  m_state = false;
  m_simuOn = true;

  for (uint8_t i = 0; i < nbChannels; i++)
  {
    m_ssrPinStates[i] = false;
    m_pinMapping[i] = 0;
  }

  initOutputsArray(0.0f);

  // TODO setup it with configuration values in EEPROM / add an API in order to set it from main
  m_pid.SetOutputLimits(0, 2400);
  m_pid.SetSampleTime(samplePeriod);

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
  delete[] m_outputs;
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

float_t Dimmer::getOutputsAverage(void)
{
  float_t avg = 0.0f;

  for (uint8_t i = 0; i < m_outputs_array_size; i++)
  {
    avg += m_outputs[i];
  }
  avg /= m_outputs_array_size;

  return avg;
} 

float_t Dimmer::simulate(float_t gridPower, float_t outputsAvg)
{
  static uint16_t m_simulCount = 0;
  
  // Simulate a solar production during first 20s and then no solar production
  if (m_simulCount < (20000 / m_pid.GetSampleTime()))
  {
    gridPower -= dash.data.solarPowerSimul;
  }
  m_simulCount++;

  if (m_simulCount > (40000 / m_pid.GetSampleTime()))
  {
    m_simulCount = 0;
  }

  // Simulation of shelly measure (retroaction)
  if (gridPower < 50.0f)
  {
    gridPower += outputsAvg;
  }

  return gridPower;
}

void Dimmer::update(float_t gridPower)
{ 
  float_t avg = 0.0f;

  Logger::plot("Dimmer.GridPowerIn", String(gridPower), "W");

  avg = getOutputsAverage();

  if (m_simuOn == false)
  {
    if (gridPower != m_previousGridPower)
    {
      m_previousGridPower = gridPower;
      m_lastAvgOutput = avg;
    }
    else
    {
      // Estimate gridPower based on last average output variation
      gridPower += (avg - m_lastAvgOutput);
    }
  }
  else
  {
    gridPower = simulate(gridPower, avg);
  } 
  
  Logger::plot("Dimmer.GridPowerEstimated", String(gridPower), "W");
  
  // Check if parameters changes
  if (m_pid.GetKp() != dash.data.Kp ||
      m_pid.GetKi() != dash.data.Ki ||
      m_pid.GetKd() != dash.data.Kd )
  {
    Logger::log(LogLevel::INFO, "Dimmer: PID parameters updated Kp=%.3f Ki=%.3f Kd=%.3f",
                dash.data.Kp, dash.data.Ki, dash.data.Kd);

    m_pid.SetTunings(dash.data.Kp, dash.data.Ki, dash.data.Kd);
    m_pid.Initialize(gridPower, avg);
  }

  // PID compute
  if (gridPower < 50.0f)
  {
    m_outputs[m_outputs_index] = m_pid.Compute(gridPower, -50.0f);
    /*The first time initialize all avg array ?*/
  }
  else
  {
    m_pid.Initialize(gridPower, 0.0f);
    initOutputsArray(0.0f);
  }
  Logger::plot("Dimmer.InstantHeaterPower", String(m_outputs[m_outputs_index]), "W");

  // Post 4 commands at t=0, +250ms, +500ms, +750ms

  m_outputs_index = (m_outputs_index +1) % m_outputs_array_size;
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

/**  Private */

/**
 * @brief Initialize the outputs array to zero
 * 
 */
void Dimmer::initOutputsArray(float_t value)
{
  for (uint8_t i = 0; i < m_outputs_array_size; i++)
  {
    m_outputs[i] = value;
  } 
}
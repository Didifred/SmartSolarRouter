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
  m_samplePeriod = samplePeriod;
  m_ssrTimer = new uint16_t[nbChannels];
  m_pinMapping = new uint8_t[nbChannels];
  m_channelPower = new uint16_t[nbChannels];

  m_outputsArraySize = measurePeriod / samplePeriod;
  m_outputs = new float_t[m_outputsArraySize];
  m_outputError = 0.0f;

  m_outputsIndex = 0;
  m_previousGridPower = 0.0f;
  m_state = false;
  m_simuOn = true;
  m_paramsChanged = false;

  for (uint8_t i = 0; i < nbChannels; i++)
  {
    m_ssrTimer[i] = 0;
    m_pinMapping[i] = 0;
  }

  initOutputsArray(0.0f);

  m_pid.SetSampleTime(samplePeriod);

  // Initialize dashboard PID values for simulation
  dash.data.solarPowerSimul = 1500.0f;
  dash.data.teleplotEnabled = false;
}

Dimmer::~Dimmer()
{
  delete[] m_ssrTimer;
  delete[] m_pinMapping;
  delete[] m_outputs;
} 

bool Dimmer::mapChannelToPin(uint8_t channel, uint8_t pin)
{
  bool success = false;

  if (channel < m_nbChannels)
  {
    m_pinMapping[channel] = pin;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOGICAL_LOW);

    success = true;
  }
 
  return success;
}

bool Dimmer::setChannelPower(uint8_t channel, uint16_t power)
{
  bool success = false;

  if (channel < m_nbChannels)
  {
    m_channelPower[channel] = power;

    success = true;
  }

  return success;
}

void Dimmer::setMaxOutputPower(uint16_t max)
{
  m_pid.SetOutputLimits(0, max);
}

void Dimmer::turnOff(void)
{
  m_state = false;
  for (uint8_t i = 0; i < m_nbChannels; i++)
  {
    m_ssrTimer[i] = 0;
    digitalWrite(m_pinMapping[i], LOGICAL_LOW);
  }
}

void Dimmer::turnOn(void)
{
  m_state = true;
}

void Dimmer::getPidParameters(float_t& Kp, float_t& Ki, float_t& Kd)
{
  Kp = m_pid.GetKp();
  Ki = m_pid.GetKi();
  Kd = m_pid.GetKd();
} 

void Dimmer::setPidParameters(float_t Kp, float_t Ki, float_t Kd)
{
  m_pid.SetParameters(Kp, Ki, Kd);

  m_paramsChanged = true;
} 

float_t Dimmer::update(float_t gridPower)
{ 
  float_t avg = 0.0f;
  float_t instantPower = 0.0f;
  float_t instantPowerAdj = 0.0f;
  uint16_t ssrTimers[m_nbChannels];

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
    gridPower = simulateSolarProduction(gridPower);
    gridPower = simulateRetroaction(gridPower, avg);
  } 
  
  Logger::plot("Dimmer.GridPowerEstimated", String(gridPower), "W");
  
  /** Check if PID parameters changes */
  if (m_paramsChanged == true)
  {
    m_pid.Initialize(gridPower, avg);
    m_paramsChanged = false;
  }

  /** PID compute */
  instantPower = m_pid.Compute(gridPower, -50.0f);
  m_outputs[m_outputsIndex] = instantPower;
  Logger::plot("Dimmer.InstantHeaterPower", String(instantPower), "W");

  /** Compensate the output from previous calculation */
  instantPowerAdj = instantPower + m_outputError;

  /** Compute timers values for SSRs based on instantPower */
  uint16_t nbHalfPeriodsPerSample = (m_samplePeriod / (1000/(m_gridFrequency*2))) ;
  for( uint8_t j=0; j < m_nbChannels; j++)
  {
    if (instantPowerAdj >= m_channelPower[j])
    {
      /** Full power for this channel */
      ssrTimers[j] = nbHalfPeriodsPerSample*2;
      instantPowerAdj -= m_channelPower[j];
    }
    else
    {
      float_t pulseWidth = (instantPowerAdj / m_channelPower[j]) * nbHalfPeriodsPerSample;
      uint16_t pulseWidthInt = roundf(pulseWidth);

      /** Error due to quantization */
      m_outputError = (((pulseWidth - (float_t)pulseWidthInt))*m_channelPower[j])/nbHalfPeriodsPerSample;
      Logger::plot("Dimmer.OutputError", String(m_outputError), "W");

      if (pulseWidthInt == 0)
      {
        /** No power for this channel */
        ssrTimers[j] = 0;
      }
      else
      {
        if (pulseWidthInt % 2 == 0)
        {
          /** Full sinus counts*/
          ssrTimers[j] = pulseWidthInt*2 - 1;
        }
        else
        {
          /** Half sinus counts*/
          ssrTimers[j] = pulseWidthInt*2;
        }
      }

      /** No power for the remaining channels */
      for (uint8_t i=j+1;  i< m_nbChannels; i++)
      {
        ssrTimers[i] = 0;
      }
      break;
    }

    Logger::plot("Dimmer.SsrTimers[" + String(j) + "]", String(ssrTimers[j]), "count");
  }
  

  /** Copy the timers to the class members read in time ISR */
  noInterrupts();
  for (uint8_t i = 0; i < m_nbChannels; i++)
  {
    m_ssrTimer[i] = ssrTimers[i];
  }
  interrupts();

  m_outputsIndex = (m_outputsIndex +1) % m_outputsArraySize;

  return instantPower;
}



IRAM_ATTR void Dimmer::updateChannelsOutput(void)
{
  if (m_state != false)
  {
    for (uint8_t i = 0; i < m_nbChannels; i++)
    {
      if (m_ssrTimer[i] > 0)
      {
        digitalWrite(m_pinMapping[i], LOGICAL_HIGH);
        m_ssrTimer[i]--;
      }
      else
      {
        digitalWrite(m_pinMapping[i], LOGICAL_LOW);
      }
    }
  }
  else
  {
    for (uint8_t i = 0; i < m_nbChannels; i++)
    {
      digitalWrite(m_pinMapping[i], LOGICAL_LOW);
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
  for (uint8_t i = 0; i < m_outputsArraySize; i++)
  {
    m_outputs[i] = value;
  } 
}

float_t Dimmer::getOutputsAverage(void)
{
  float_t avg = 0.0f;

  for (uint8_t i = 0; i < m_outputsArraySize; i++)
  {
    avg += m_outputs[i];
  }
  avg /= m_outputsArraySize;

  return avg;
} 


float_t Dimmer::simulateSolarProduction(float_t gridPower)
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

  return gridPower;
}

float_t Dimmer::simulateRetroaction(float_t gridPower, float_t outputsAvg)
{
  // Simulation of shelly measure (retroaction)
  gridPower += outputsAvg;

  return gridPower;
}
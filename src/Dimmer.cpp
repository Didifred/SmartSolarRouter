#include <Arduino.h>
#include <dashboard.h>
#include "Dimmer.h"
#include "Logger.h"
#include "libPID.h"

Dimmer::Dimmer(uint8_t gridFrequency, uint16_t measurePeriod, uint16_t pidPeriod, uint16_t outputPeriod) : m_pid(0.4f, 1.0f, 0.0f, PidProportionalOption::ON_ERROR, PidDirection::DIRECT)
{
  m_gridFrequency = gridFrequency;
  m_measurePeriod = measurePeriod;
  m_pidPeriod = pidPeriod;
  m_outputPeriod = outputPeriod;
  m_nbChannels = 0;

  m_filterSize = measurePeriod / pidPeriod;
  m_nbHalfSinPerOutputPeriod = (m_outputPeriod / (1000 / (m_gridFrequency * 2)));

  m_outputError = 0.0f;
  m_filterIndex = 0;
  m_previousGridPower = 0.0f;
  m_state = false;
  m_simuOn = false;
  m_paramsChanged = false;
  m_ssrCycleTimer = 0;
  m_ssrNewValues = false;
  m_lastAvgOutput = 0;

  m_ssrTimer = nullptr;
  m_nextSsrTimer = nullptr;
  m_pinMapping = nullptr;
  m_channelPower = nullptr;

  m_outputs = new float_t[m_filterSize];
  initOutputsArray(0.0f);

  m_inputs = new float_t[m_filterSize];
  initInputsArray(0.0f);

  m_pid.SetSampleTime(pidPeriod);

  Logger::log(LogLevel::DEBUG, "Dimmer: gridFrequency=%i, measurePeriod=%i, pidPeriod=%i, m_nbHalfSinPerOutputPeriod=%i, filterSize=%i",
              m_gridFrequency, m_measurePeriod, m_pidPeriod,
              m_nbHalfSinPerOutputPeriod, m_filterSize);
}

Dimmer::~Dimmer()
{
  if (m_ssrTimer != nullptr)
  {
    delete[] m_ssrTimer;
  }
  if (m_nextSsrTimer != nullptr)
  {
    delete[] m_nextSsrTimer;
  }
  if (m_pinMapping != nullptr)
  {
    delete[] m_pinMapping;
  }
  if (m_channelPower != nullptr)
  {
    delete[] m_channelPower;
  }
  if (m_outputs != nullptr)
  {
    delete[] m_outputs;
  }
}

void Dimmer::setNbChannels(uint8_t nbChannels)
{
  if ((nbChannels <= MAX_DIMMER_CHANNELS) && (nbChannels > 0))
  {
    if (nbChannels != m_nbChannels)
    {
      if (m_ssrTimer != nullptr)
      {
        delete[] m_ssrTimer;
      }
      if (m_nextSsrTimer != nullptr)
      {
        delete[] m_nextSsrTimer;
      }
      if (m_pinMapping != nullptr)
      {
        delete[] m_pinMapping;
      }
      if (m_channelPower != nullptr)
      {
        delete[] m_channelPower;
      }

      m_nbChannels = nbChannels;
      m_ssrTimer = new uint16_t[nbChannels];
      m_nextSsrTimer = new uint16_t[nbChannels];
      m_pinMapping = new uint8_t[nbChannels];
      m_channelPower = new uint16_t[nbChannels];

      for (uint8_t i = 0; i < nbChannels; i++)
      {
        m_ssrTimer[i] = 0;
        m_nextSsrTimer[i] = 0;
        m_pinMapping[i] = 0;
        m_channelPower[i] = 0;
      }
    }
  }
  else
  {
    Logger::log(LogLevel::ERROR, "Invalid number of channels (%d), must be greater than zero and less than or equal to MAX_DIMMER_CHANNELS (%d).",
                m_nbChannels);
  }
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
  m_ssrCycleTimer = 0;
}

void Dimmer::setSimuEnabled(bool enabled)
{
  m_simuOn = enabled;
}

void Dimmer::getPidParameters(float_t &Kp, float_t &Ki, float_t &Kd)
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

float_t Dimmer::update(float_t gridPower, bool newMeasure)
{
  float_t avg = 0.0f;
  float_t outPower = 0.0f;
  float_t outPowerAdj = 0.0f;
  uint16_t ssrTimers[m_nbChannels];

  if (newMeasure == true)
  {
    if (m_simuOn == true)
    {
      // Simulate retroaction
      if (gridPower != m_previousGridPower)
      {
        avg = getOutputsAverage();
        m_lastAvgOutput = avg;

        m_previousGridPower = gridPower;
      }

      gridPower += m_lastAvgOutput;
    }

    Logger::plot("Dimmer.GridPowerIn", String(gridPower), "W");
    m_inputs[m_filterIndex] = gridPower;

    /**< Filter the input grid power */
    gridPower = getInputsAverage();
    Logger::plot("Dimmer.GridPowerEstimated", String(gridPower), "W");

    Logger::log(LogLevel::TRACE, "Dimmer update : %f W", gridPower);

    /**< Check if PID parameters changes */
    if (m_paramsChanged == true)
    {
      m_pid.Initialize(gridPower, avg);
      m_paramsChanged = false;
    }

    /**< PID compute */
    outPower = m_pid.Compute(gridPower, -50.0f);
    m_outputs[m_filterIndex] = outPower;
    Logger::plot("Dimmer.OutPower", String(outPower), "W");

    m_filterIndex = (m_filterIndex + 1) % m_filterSize;
  }
  else
  {
    /**< Hold last output value */
    outPower = m_outputs[m_filterIndex];
  }

  /** Compensate the output from previous calculation */
  // outPowerAdj = outPower + m_outputError;
  outPowerAdj = outPower;

  /** Compute timers values for SSRs based on outPower */
  for (uint8_t j = 0; j < m_nbChannels; j++)
  {
    if (outPowerAdj >= m_channelPower[j])
    {
      /** Full power for this channel */
      ssrTimers[j] = m_nbHalfSinPerOutputPeriod * 2;
      outPowerAdj -= m_channelPower[j];
    }
    else
    {
      float_t pulseWidth = (outPowerAdj / m_channelPower[j]) * m_nbHalfSinPerOutputPeriod;
      uint16_t pulseWidthInt = roundf(pulseWidth);

      /** Error due to quantization */
      m_outputError = (((pulseWidth - (float_t)pulseWidthInt)) * m_channelPower[j]) / m_nbHalfSinPerOutputPeriod;
      // Logger::plot("Dimmer.OutputError", String(m_outputError), "W");

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
          ssrTimers[j] = (pulseWidthInt * 2) - 1;
        }
        else
        {
          /** Half sinus counts*/
          ssrTimers[j] = pulseWidthInt * 2;
        }
      }

      /** No power for the remaining channels */
      for (uint8_t i = j + 1; i < m_nbChannels; i++)
      {
        ssrTimers[i] = 0;
      }
      break;
    }
  }
  // Logger::plot("Dimmer.SsrTimers_0", String(ssrTimers[0]), "ticks");

  /** Copy the timers to the class members read in time ISR */
  noInterrupts();
  for (uint8_t i = 0; i < m_nbChannels; i++)
  {
    m_nextSsrTimer[i] = ssrTimers[i];
  }
  m_ssrNewValues = true;
  interrupts();

  return outPower;
}

IRAM_ATTR void Dimmer::updateChannelsOutput(void)
{
  if (m_state != false)
  {
    if ((m_ssrNewValues == true) && (m_ssrCycleTimer == 0))
    {
      m_ssrCycleTimer = m_nbHalfSinPerOutputPeriod * 2;
      m_ssrNewValues = false;

      // Copy the new timers values
      for (uint8_t i = 0; i < m_nbChannels; i++)
      {
        m_ssrTimer[i] = m_nextSsrTimer[i];
      }
    }
    else
    {
      if (m_ssrCycleTimer > 0)
      {
        m_ssrCycleTimer--;
      }
    }

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
  for (uint8_t i = 0; i < m_filterSize; i++)
  {
    m_outputs[i] = value;
  }
}

/**
 * @brief Initialize the inputs array to zero
 *
 */
void Dimmer::initInputsArray(float_t value)
{
  for (uint8_t i = 0; i < m_filterSize; i++)
  {
    m_inputs[i] = value;
  }
}

float_t Dimmer::getInputsAverage(void)
{
  float_t avg = 0.0f;

  for (uint8_t i = 0; i < m_filterSize; i++)
  {
    avg += m_inputs[i];
  }
  avg /= m_filterSize;

  return avg;
}

float_t Dimmer::getOutputsAverage(void)
{
  float_t avg = 0.0f;

  for (uint8_t i = 0; i < m_filterSize; i++)
  {
    avg += m_outputs[i];
  }
  avg /= m_filterSize;

  return avg;
}

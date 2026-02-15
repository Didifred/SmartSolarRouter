#include <Arduino.h>
#include <dashboard.h>
#include <cfloat>
#include "Dimmer.h"
#include "Logger.h"
#include "libPID.h"

Dimmer::Dimmer(uint8_t gridFrequency,
               uint16_t measurePeriod,
               uint16_t pidPeriod,
               uint16_t outputPeriod) : m_gridFrequency(gridFrequency),
                                        m_measurePeriod(measurePeriod),
                                        m_pidPeriod(pidPeriod),
                                        m_outputPeriod(outputPeriod),
                                        m_nbHalfSinPerOutputPeriod(outputPeriod / (1000 / (gridFrequency * 2))),
                                        m_pid(0.2f, 0.25f, 0.0f, PidProportionalOption::ON_ERROR, PidDirection::DIRECT)

{
  m_nbChannels = 0;
  m_gridPowerFiltered = 0.0f;
  m_previousGridPower = 0.0f;
  m_previousOutput = 0.0f;
  m_outputError = 0.0f;
  m_state = false;
  m_simuOn = false;
  m_paramsChanged = false;
  m_ssrCycleTimer = 0;
  m_ssrNewValues = false;

  m_pinMapping = nullptr;
  m_channelPower = nullptr;

  m_filterIndex = 0;
  m_filterSize = 2;
  m_outputs = new float_t[m_filterSize];
  initArray(m_outputs, m_filterSize, 0.0f);

  m_pid.SetSampleTime(pidPeriod);

  Logger::log(LogLevel::DEBUG, "Dimmer: gridFrequency=%i, measurePeriod=%i, pidPeriod=%i, m_nbHalfSinPerOutputPeriod=%i",
              m_gridFrequency, m_measurePeriod, m_pidPeriod,
              m_nbHalfSinPerOutputPeriod);
}

Dimmer::~Dimmer()
{
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
      m_ssrPulses.resize(nbChannels);
      m_nextSsrPulses.resize(nbChannels);

      if (m_pinMapping != nullptr)
      {
        delete[] m_pinMapping;
      }
      if (m_channelPower != nullptr)
      {
        delete[] m_channelPower;
      }

      m_nbChannels = nbChannels;
      m_pinMapping = new uint8_t[nbChannels];
      m_channelPower = new uint16_t[nbChannels];

      for (uint8_t i = 0; i < nbChannels; i++)
      {
        m_ssrPulses[i].clearAll();
        m_nextSsrPulses[i].clearAll();
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
  m_ssrCycleTimer = 0;
  for (uint8_t i = 0; i < m_nbChannels; i++)
  {
    m_ssrPulses[i].clearAll();
    digitalWrite(m_pinMapping[i], LOGICAL_LOW);
  }
}

void Dimmer::turnOn(void)
{
  m_state = true;
  m_ssrCycleTimer = ((m_nbHalfSinPerOutputPeriod * 2) - 1);
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
  float_t change = 0.0f;
  float_t outPower = 0.0f;
  float_t k = 1.0f;
  float_t avg = 0.0f;

  std::vector<Bitset<DIMMER_ISR_NB_TICKS>> ssrPulses(m_nbChannels);

  /** newMeasure argument is new as protocol, consider for pid only a change */
  if ((newMeasure == true) && (gridPower == m_previousGridPower))
  {
    newMeasure = false;
  }

  if (newMeasure == true)
  {
    if (m_simuOn == true)
    {
      // Simulate retroaction
      avg = getAverage(m_outputs, m_filterSize);

      gridPower += avg;
    }
    Logger::plot("Dimmer.GridPowerIn", String(gridPower), "W");
    Logger::log(LogLevel::DEBUG, "Pid update with new measure: gridPower=%.2f W", gridPower);

    /** Filter the grid power oscillation due to Shelly sampling measurement (40ms -1xx ms) and noise */
    change = std::abs(gridPower - m_gridPowerFiltered);
    if (change < 100.0f)
    {
      k = 0.2f;
    }
    else if (change < 200.0f)
    {
      k = 0.2f + 0.7 * ((change - 100.0f) / 100.0f); // between 0.2 and 0.9
    }
    else
    {
      k = 0.9f;
    }
    m_gridPowerFiltered = (gridPower * k) + (m_gridPowerFiltered * (1.0f - k));
    Logger::plot("Dimmer.GridPowerFlt", String(m_gridPowerFiltered), "W");

    /**< Check if PID parameters changes */
    if (m_paramsChanged == true)
    {
      m_pid.Initialize(m_gridPowerFiltered, m_previousOutput);
      m_paramsChanged = false;
    }

    /**< PID compute */
    outPower = m_pid.Compute(m_gridPowerFiltered, -50.0f);
    Logger::plot("Dimmer.OutPowerPID", String(outPower), "W");

    /**< Store gridPower and outpower */
    m_previousGridPower = gridPower;
    m_previousOutput = outPower;

    m_outputs[m_filterIndex] = outPower;

    m_filterIndex = (m_filterIndex + 1) % m_filterSize;
  }
  else
  {
    /**< Restore last output value */
    outPower = m_previousOutput;
  }

  computeNextSsrPulses(outPower);

  return outPower;
}

void Dimmer::uniformize(Bitset<DIMMER_ISR_NB_TICKS> &bitset, uint16_t numerator, uint16_t denominator)
{
  uint16_t nbPulses = numerator / 2; /** Minimal pulse widh is 2 half period */
  uint16_t onWidthCount = numerator;
  uint16_t offWidth = denominator - numerator;
  float_t spaceOff = ((float_t)offWidth) / nbPulses;
  float_t sumOff = 0.0f;
  uint16_t intSpaceOff = (2 * denominator) - 2; /** Applied when numerator = 1*/
  uint16_t iBit = 0;

  for (uint16_t iPulses = 0; iPulses < nbPulses; iPulses++)
  {
    /** Add Pulse */
    bitset.set(iBit++);
    bitset.set(iBit++);
    bitset.set(iBit++);
    sumOff += spaceOff;
    onWidthCount -= 2;

    if ((sumOff < 1.0f))
    {
      bitset.set(iBit++);
    }
    else
    {
      bitset.clear(iBit++);
      intSpaceOff = (uint16_t)sumOff;
      sumOff -= intSpaceOff;
      for (uint16_t iOff = 0; iOff < intSpaceOff; iOff++)
      {
        bitset.clear(iBit++);
        bitset.clear(iBit++);
      }
    }
  }

  /** Last pulse (rest of nbPulses) is added at the end of the loop,
   * so we add here the odd pulse if exists*/
  if (onWidthCount == 1)
  {
    iBit = ((2 * denominator) - 2) - intSpaceOff;
    bitset.set(iBit++);
    bitset.set(iBit++);
  }
}

void Dimmer::computeNextSsrPulses(float_t outPower)
{
  std::vector<Bitset<DIMMER_ISR_NB_TICKS>> ssrPulses(m_nbChannels);

  /** Compute timers values for SSRs based on outPower */
  for (uint8_t i = 0; i < m_nbChannels; i++)
  {
    if (outPower >= m_channelPower[i])
    {
      /** Full power for this channel */
      ssrPulses[i].setAll();
      outPower -= m_channelPower[i];
    }
    else
    {
      uint8_t numerator = 0;
      float_t ratio = 0.0f;
      float_t quantum = m_channelPower[i] / DIMMER_FRACTION_DENOMINATOR;

      /** Get best fraction for the remaining power */
      ratio = outPower / m_channelPower[i];
      numerator = roundf((DIMMER_FRACTION_DENOMINATOR * outPower) / m_channelPower[i]);
      m_outputError += (ratio - ((float_t)numerator / DIMMER_FRACTION_DENOMINATOR)) * m_channelPower[i];

      /** Compensate base on previous error accumulation */
      if (m_outputError >= quantum)
      {
        if (numerator < DIMMER_FRACTION_DENOMINATOR)
        {
          numerator++;
          m_outputError -= quantum;
        }
      }
      else if (m_outputError <= -quantum)
      {
        if (numerator > 0)
        {
          numerator--;
          m_outputError += quantum;
        }
      }
      Logger::plot("Dimmer.OutputError", String(m_outputError), "W");

      if (numerator == DIMMER_FRACTION_DENOMINATOR)
      {
        /** Full power */
        ssrPulses[i].setAll();
      }
      else
      {
        /** Clear all bits */
        ssrPulses[i].clearAll();

        /** Uniformize the pulses */
        uniformize(ssrPulses[i], numerator, DIMMER_FRACTION_DENOMINATOR);
      }

      /** No power for the remaining channels */
      for (uint8_t j = i + 1; j < m_nbChannels; j++)
      {
        ssrPulses[j].clearAll();
      }
      break;
    } /* outPower < m_channelPower[j] */
  } /* Loop on all channels */

  Logger::log(LogLevel::TRACE, "Dimmer ssr[0] pulses %s", ssrPulses[0].toString().c_str());

  /** Copy the timers to the class members read in time ISR */
  noInterrupts();
  for (uint8_t i = 0; i < m_nbChannels; i++)
  {
    m_nextSsrPulses[i] = ssrPulses[i];
  }
  m_ssrNewValues = true;
  interrupts();
}

IRAM_ATTR void Dimmer::updateChannelsOutput(void)
{
  if (m_state != false)
  {
    if ((m_ssrNewValues == true) && (m_ssrCycleTimer == ((m_nbHalfSinPerOutputPeriod * 2) - 1)))
    {
      m_ssrCycleTimer = 0;
      m_ssrNewValues = false;

      // Copy the new timers values
      for (uint8_t i = 0; i < m_nbChannels; i++)
      {
        m_ssrPulses[i] = m_nextSsrPulses[i];
      }
    }
    else
    {
      if (m_ssrCycleTimer < ((m_nbHalfSinPerOutputPeriod * 2) - 1))
      {
        m_ssrCycleTimer++;
      }
    }

    for (uint8_t i = 0; i < m_nbChannels; i++)
    {
      if (m_ssrPulses[i].get(m_ssrCycleTimer) == true)
      {
        digitalWrite(m_pinMapping[i], LOGICAL_HIGH);
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
 * @brief Initialize the array to zero
 *
 */
void Dimmer::initArray(float_t *array, uint8_t size, float_t value)
{
  for (uint8_t i = 0; i < size; i++)
  {
    array[i] = value;
  }
}

/**
 * @brief Compute the average of the array
 *
 * @param array
 * @param size
 * @return float_t
 */
float_t Dimmer::getAverage(float_t *array, uint8_t size)
{
  float_t avg = 0.0f;

  for (uint8_t i = 0; i < size; i++)
  {
    avg += array[i];
  }
  avg /= size;

  return avg;
}

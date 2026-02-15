#ifndef DIMMER_H
#define DIMMER_H

#include <stdint.h>
#include "libPID.h"
#include "Utils.h"
#include "hwConfig.h"
#include "routerConfig.h"

class Dimmer
{
public:
  /**
   * @brief Create a dimmer object
   *
   * @param gridFrequency  Frequency of the grid
   * @param measurePeriod Period of the measure (ms)
   * @param pidPeriod Sampling period of pid (ms)
   * @param outPeriod Output period refresh (ms)
   *
   */
  Dimmer(uint8_t gridFrequency = POWER_GRID_FREQUENCY_HZ,
         uint16_t measurePeriod = POWER_GRID_MEASURE_PERIOD_MS,
         uint16_t pidPeriod = PID_SAMPLE_PERIOD_MS,
         uint16_t outputPeriod = DIMMER_OUTPUT_PERIOD_MS);
  ~Dimmer();

  /**
   * @brief Set the number of SSR channels of the dimmer
   * @param nbChannels
   */
  void setNbChannels(uint8_t nbChannels);

  /**
   * @brief Update the dimmer's digital outputs state.
   *
   * This function is placed in IRAM (IRAM_ATTR) so it can be executed
   * from interrupt context.
   *
   * @details
   * - Intended to be short and non-blocking; avoid long computations or delays.
   * - Safe to call from an ISR.
   */
  IRAM_ATTR void updateChannelsOutput(void);

  /**
   * @brief Set the Pid Parameters
   *
   * @param Kp    Proportional parameter
   * @param Ki    Integral parameter
   * @param Kd    Derivative parameter
   */
  void setPidParameters(float_t Kp, float_t Ki, float_t Kd);

  /**
   * @brief Get the Pid Parameters
   *
   * @param Kp  Proportional parameter
   * @param Ki  Integral parameter
   * @param Kd  Derivative parameter
   */
  void getPidParameters(float_t &Kp, float_t &Ki, float_t &Kd);

  /**
   * @brief Compute and update power to be applied on the channels
   * @param gridPower   Grid power measurement
   * @param newMeasure  Indicates if a new measurement is available
   * @return The computed output power in W
   */
  float_t update(float_t gridPower, bool newMeasure);

  /**
   * @brief Uniformize the SSR pulses for a given period
   *
   * @param bitset The bitset to fill with the pulse pattern
   * @param numerator Numerator of the fraction of output power to apply (between 0 and denominator)
   * @param denominator   Denominator of the fraction of output power to apply (between 1 and DIMMER_FRACTION_DENOMINATOR)
   */
  void uniformize(Bitset<DIMMER_ISR_NB_TICKS> &bitset, uint16_t numerator, uint16_t denominator);

  /**
   * @brief Compute the next SSR pulses pattern to apply based on the output power to apply on the load
   *
   * @param outPower
   */
  void computeNextSsrPulses(float_t outPower);

  /**
   * @brief Associate a logical dimmer channel with a physical MCU pin.
   *
   * Establishes which hardware pin will be driven when operations are
   * performed on the specified dimmer channel. After calling this function,
   * references to the channel should affect the configured pin.
   *
   * @param channel Logical channel identifier (0-based).
   * @param pin     Hardware pin number (board-specific numbering) to bind to the channel.
   * @return true if the mapping was successful; false if the channel index is out of range.
   */
  bool mapChannelToPin(uint8_t channel, uint8_t pin);

  /**
   * @brief Set the resistive power affectated to a channel
   *
   * @param channel Logical channel identifier (0-based).
   * @param power   Power in W
   * @return true if the power was set successfully; false if the channel index is out of range.
   */
  bool setChannelPower(uint8_t channel, uint16_t power);

  /**
   * @brief Set the maximum power driven by the dimmer in W
   *
   * @param max Maximum output power.
   */
  void setMaxOutputPower(uint16_t max);

  /**
   * @brief Turn the dimmer outputs off.
   */
  void turnOff(void);

  /**
   * @brief Turn the dimmer outputs on.
   */
  void turnOn(void);

  /**
   * @brief Set simulation of output retroaction enabled or disabled
   *
   * @param enable true to enable simulation, false to disable
   */
  void setSimuEnabled(bool enabled);

private:
  void initArray(float_t *array, uint8_t size, float_t value);
  float_t getAverage(float_t *array, uint8_t size);

  const uint8_t m_gridFrequency;             /** The frequency of the grid'power */
  const uint16_t m_measurePeriod;            /** The period of the grid measure */
  const uint16_t m_pidPeriod;                /** The sample period of PID  */
  const uint16_t m_outputPeriod;             /** The output period of the dimmer */
  const uint16_t m_nbHalfSinPerOutputPeriod; /** Number of half periods per output period */
  uint8_t m_nbChannels;                      /** Number of output channels */

  bool m_state;                                             /** Global state ON or OFF */
  bool m_paramsChanged;                                     /** Flag to indicate PID parameters have changed */
  bool m_simuOn;                                            /** Simulation of heater retroaction */
  std::vector<Bitset<DIMMER_ISR_NB_TICKS>> m_ssrPulses;     /** Vector of pulses to control ssr pins ON or OFF, size is m_nbChannels*/
  std::vector<Bitset<DIMMER_ISR_NB_TICKS>> m_nextSsrPulses; /** Next array of pulses to control ssr pins ON or OFF, size is m_nbChannels*/
  bool m_ssrNewValues;                                      /** Flag to indicate new SSR pulses values are available */
  uint16_t m_ssrCycleTimer;                                 /** Cycle time for SSR control in number of half periods */

  uint16_t *m_channelPower; /** Resistive power of each channels, size is m_nbChannels */
  uint8_t *m_pinMapping;    /** Pin affectation to channels, size is m_nbChannels */
  uint8_t m_filterSize;     /** Size is m_measurePeriod / m_pidPeriod */
  float_t *m_outputs;       /** Array of float_t, size is m_filterSize */
  uint8_t m_filterIndex;    /** Current index in m_outputs array*/

  float_t m_outputError; /** Retained error for decimation outputs calculation */

  float_t m_gridPowerFiltered; /** Filtered grid power used for PID input */
  float_t m_previousGridPower;
  float_t m_previousOutput;
  PID m_pid;
};

#endif /*DIMMER_H */

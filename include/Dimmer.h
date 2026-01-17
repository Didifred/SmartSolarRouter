#ifndef DIMMER_H
#define DIMMER_H

#include <stdint.h>
#include "libPID.h"
#include "hwConfig.h"
#include "routerConfig.h"

class Dimmer
{
public:
  Dimmer(uint8_t gridFrequency = POWER_GRID_FREQUENCY_HZ,
         uint16_t measurePeriod = 1000, uint16_t pidPeriod = 250, uint16_t outputPeriod = 250);
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
   */
  float_t update(float_t gridPower, bool newMeasure);

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
  void initOutputsArray(float_t value);
  float_t getOutputsAverage(void);

  void initInputsArray(float_t value);
  float_t getInputsAverage(void);

  uint8_t m_nbChannels;                /** Number of output channels */
  uint8_t m_gridFrequency;             /** The frequency of the grid'power */
  uint16_t m_measurePeriod;            /** The period of the grid measure */
  uint16_t m_pidPeriod;                /** The sample period of PID  */
  uint16_t m_outputPeriod;             /** The output period of the dimmer */
  uint16_t m_nbHalfSinPerOutputPeriod; /** Number of half periods per output period */

  bool m_state;             /** Global state ON or OFF */
  bool m_paramsChanged;     /** Flag to indicate PID parameters have changed */
  bool m_simuOn;            /** Simulation of heater retroaction */
  uint16_t *m_ssrTimer;     /** Array of timers to control ssr pins ON or OFF, size is m_nbChannels*/
  uint16_t *m_nextSsrTimer; /** Next array of timers to control ssr pins ON or OFF, size is m_nbChannels*/
  bool m_ssrNewValues;      /** Flag to indicate new SSR timers values are available */
  uint16_t m_ssrCycleTimer; /** Cycle time for SSR control in number of half periods */

  uint16_t *m_channelPower; /** Resistive power of each channels, size is m_nbChannels */
  uint8_t *m_pinMapping;    /** Pin affectation to channels, size is m_nbChannels */
  uint8_t m_filterSize;     /** Size is m_measurePeriod / m_pidPeriod */
  float_t *m_outputs;       /** Array of float_t, size is m_filterSize */
  float_t *m_inputs;        /** Array of float_t, size is m_filterSize */
  uint8_t m_filterIndex;    /** Current index in m_outputs array*/

  float_t m_outputError; /** Retained error for decimation outputs calculation */

  float_t m_previousGridPower;
  float_t m_lastAvgOutput;
  PID m_pid;
};

#endif /*DIMMER_H */

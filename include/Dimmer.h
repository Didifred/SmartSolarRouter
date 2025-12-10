#ifndef DIMMER_H
#define DIMMER_H

#include <stdint.h>
#include "libPID.h"
#include "hwConfig.h"
#include "routerConfig.h"


class Dimmer
{
public:
    Dimmer(uint8_t nbChannels = 3, uint8_t gridFrequency = 50, 
           uint16_t samplePeriod = 250, uint16_t measurePeriod = 1000);
    ~Dimmer();

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
     * @brief Compute and update power to be applied on the channels
     * @param gridPower   Grid power measurement
     */
    void update(float_t gridPower);

    /**
     * @brief Associate a logical dimmer channel with a physical MCU pin.
     *
     * Establishes which hardware pin will be driven when operations are
     * performed on the specified dimmer channel. After calling this function,
     * references to the channel should affect the configured pin.
     *
     * @param channel Logical channel identifier (typically 0-based). 
     * @param pin     Hardware pin number (board-specific numbering) to bind to
     *                the channel.
     */
    void mapChannelToPin(uint8_t channel, uint8_t pin);

    /**
     * @brief Turn the dimmer outputs off.
     */
    void turnOff(void);

    /**
     * @brief Turn the dimmer outputs off.
     */
    void turnOn(void);

private:

    void initOutputsArray(float_t value);

    float_t simulate(float gridPower, float_t outputsAvg);
    float_t getOutputsAverage(void);

    uint8_t m_nbChannels;
    uint8_t m_gridFrequency;
    uint16_t m_measurePeriod;
    bool m_state; /** ON or OFF */
    bool* m_ssrPinStates; /** Array of boolean, size is m_nbChannels*/
    bool m_simuOn;
    uint8_t *m_pinMapping; /** Pin affectation to channels */
    float_t*  m_outputs; /** Array of float_t, size is m_outputs_array_size */
    uint8_t m_outputs_array_size; /** m_measurePeriod / samplePeriod */
    uint8_t m_outputs_index;
    float_t m_previousGridPower;
    float_t m_lastAvgOutput;
    PID m_pid;
    
};


#endif /*DIMMER_H */
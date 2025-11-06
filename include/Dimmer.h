#ifndef DIMMER_H
#define DIMMER_H

#include <stdint.h>
#include "hwConfig.h"


class Dimmer
{
public:
    Dimmer(uint8_t nbChannels = 3, uint8_t gridFrequency = 50);
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

    uint8_t m_nbChannels;
    uint8_t m_gridFrequency;
    bool m_state;
    bool* m_ssrPinStates;
    uint8_t *m_pinMapping;
};


#endif /*DIMMER_H */
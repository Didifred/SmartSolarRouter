
#ifndef UTILS_H
#define UTILS_H

class Utils


/**
 * @brief Enables the hardware timer interrupts.
 * 
 * This function resumes the hardware timer interrupts that were previously
 * disabled using disableHwTimer().
 */
{

/**
 * @class Utils
 * @brief Utility class providing various helper functions.
 */

public:
    Utils() = delete; // Prevent instantiation of this class
    ~Utils() = delete;
    
    /**
     * @brief Initializes the Logger module.
     */
    static void checkEspFlash(void);

    /**
     * @brief Initialize a hardware timer to call a user function at a specified frequency.
     * @param gridFrequency The frequency in Hz.
     * @param userFunc The user function to be called by the timer interrupt.
     */
    static void initHwTimer(uint8_t frequency, timercallback userFunc);

    /**
     * @brief Disables the hardware timer interrupts.
    */
    static void disableHwTimer(void);

     /**
     * @brief Enable the hardware timer interrupts.
    */
    static void enableHwTimer(void);

};

#endif /* UTILS_H */
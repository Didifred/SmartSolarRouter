
#ifndef UTILS_H
#define UTILS_H

class Utils
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

};

#endif /* UTILS_H */
#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include <stdint.h>
#include <pins_arduino.h>

/** d1_mini esp8286 board */
#define SSR1_Pin  D5    // use SSR1 output - IO14
#define SSR2_Pin  D1    // use SSR2 output - IO5 
#define SSR3_Pin  D2    // use SSR3 output - IO4 (or D0 - IO16) )

/** Maximum of channels to be allocated to one dimmer instance  */ 
#define MAX_DIMMER_CHANNELS 3

/** Logical levels for SSRs */
#define LOGICAL_HIGH    HIGH
#define LOGICAL_LOW     LOW

#endif // HW_CONFIG_H */
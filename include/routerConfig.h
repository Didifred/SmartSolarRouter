#ifndef ROUTER_CONFIG_H
#define ROUTER_CONFIG_H


/** Power grid measure period in ms */
#define POWER_GRID_MEASURE_PERIOD_MS   500

/** Nb pid samples per measure (must be greater than 1) */
#define NB_PID_SAMPLES                 2

/** Power grid frequency */
#define POWER_GRID_FREQUENCY_HZ        50

/** Dimmer frequency (x4  to be able to drive half-period times)*/
#define DIMMER_ISR_FREQUENCY           POWER_GRID_FREQUENCY_HZ*4



#endif // ROUTER_CONFIG_H */
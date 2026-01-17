#ifndef ROUTER_CONFIG_H
#define ROUTER_CONFIG_H

/** Power grid measure period in ms */
#define POWER_GRID_MEASURE_PERIOD_MS 1000

/** PID sample period in ms */
#define PID_SAMPLE_PERIOD_MS 250

/** Dimmer output period in ms */
#define DIMMER_OUTPUT_PERIOD_MS 250

/** Power grid frequency */
#define POWER_GRID_FREQUENCY_HZ 50

/** Dimmer frequency (x4  to be able to drive half-period times)*/
#define DIMMER_ISR_FREQUENCY POWER_GRID_FREQUENCY_HZ * 4

#endif // ROUTER_CONFIG_H */

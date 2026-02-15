#ifndef ROUTER_CONFIG_H
#define ROUTER_CONFIG_H

/** Power grid measure period in ms */
#define POWER_GRID_MEASURE_PERIOD_MS 1000

/** PID sample period in ms */
#define PID_SAMPLE_PERIOD_MS 1000

/** Dimmer output period in ms */
#define DIMMER_OUTPUT_PERIOD_MS 120

/** Power grid frequency */
#define POWER_GRID_FREQUENCY_HZ 50

/** Dimmer frequency (x4  to be able to drive half-period times)*/
#define DIMMER_ISR_FREQUENCY (POWER_GRID_FREQUENCY_HZ * 4)

/** Number of ticks for the output period */
#define DIMMER_ISR_NB_TICKS ((DIMMER_ISR_FREQUENCY * DIMMER_OUTPUT_PERIOD_MS) / 1000)

/** Denominator fraction used */
#define DIMMER_FRACTION_DENOMINATOR (DIMMER_OUTPUT_PERIOD_MS / 10)

#endif // ROUTER_CONFIG_H */

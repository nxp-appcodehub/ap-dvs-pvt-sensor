#ifndef _APP_CONFIG_
#define _APP_CONFIG_

#include "FreeRTOSConfig.h"
#include "fsl_pca9420.h"
/**
 * Sets the trim value for the FRO.
 * Valid Values: 192000000U or 250000000U
 */
#define FRO_TRIM_FREQ_HZ           250000000U

/*
 * Set the min and max voltages according to the selected trim.
 */
#if FRO_TRIM_FREQ_HZ == 192000000U
#define MIN_VDDCORE                kPCA9420_Sw1OutVolt0V800
#define MAX_VDDCORE                kPCA9420_Sw1OutVolt0V900
#elif FRO_TRIM_FREQ_HZ == 250000000U
#define MIN_VDDCORE                kPCA9420_Sw1OutVolt0V850
#define MAX_VDDCORE                kPCA9420_Sw1OutVolt1V025
#else
#error "FRO_TRIM_FREQ_HZ must be 192MHz or 250MHz"
#endif

/**
 * - 0: Disables PVT task. Can be used to profile impact of PVT task on performance.
 * - 1: Enables the PVT task.
 */
#define ENABLE_PVT                 1

/**
 * How often for PVT task to check if VDDCORE can go lower.
 */
#define PVT_TASK_WAIT_MS           10000U

/**
 * Amount if time it takes the PMIC SW1_OUT to stabilize after decreasing it 1 step.
 */
#define PMIC_SETTLING_TIME_MS      5U

/**
 * Time to wait between enabling the PVT ring oscillator and reading the delay.
 */
#define PVT_RING_OSC_WAIT_TIME_MS  500U

/*
 * Amount of time to wait in between Coremark runs.
 * Allows idle task to activate low power mode (deep sleep).
 */
#define WORKLOAD_DELAY_MS          5000U

/**
 * Task priorities.
 * PVT Should be highest priority so it can increase VDDCORE ASAP if the PVT interrupt triggers.
 */
#define workload_task_PRIORITY     (configMAX_PRIORITIES - 2)
#define pvt_task_PRIORITY          (configMAX_PRIORITIES - 1)

#endif /* _APP_CONFIG_ */

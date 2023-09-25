/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PVT_H_
#define _PVT_H_

#include <stdint.h>
#include <stdbool.h>
#include "pvt_int.h"

/*! @file */

/*@{*/
/**
 * Macro to make library version information.
 */
#define PVT_LIB_MAKE_VERSION(major, minor, bugfix) (((major & 0xFF) << 16) | \
                                                    ((minor & 0xFF) << 8)  | \
                                                     (bugfix & 0xFF))

/*@}*/


/*@{*/
/**
 * PVT delay type.
 */
typedef uint8_t pvt_delay_t;
/*@}*/

#if defined(__cplusplus)
extern "C" {
#endif

/*@{*/
/**
 * @brief Returns the library version MAJOR.MINOR.BUGFIX.
 * @note Format:
 *             24    16    8      0
 *       [     |MAJOR|MINOR|BUGFIX]
 *       Example:0x010203 -> 1.2.3
 *
 * @code
 * // Use PVT_LIB_MAKE_VERSION to compare library versions.
 * if (PVT_GetLibVersion() < PVT_LIB_MAKE_VERSION(1,0,0)) {
 *     PRINTF("Library is not up to date.");
 * }
 * @endcode
 * @return Library version.
 */
uint32_t PVT_GetLibVersion(void);

/*@{*/
/**
 * @brief Initializes the PVT peripheral.
 * @note The complete initialization should look like below.
 *
 * @code
 * pvt_delay_t delay;
 * PVT_Init();
 * if (PVT_ReadDelayFromOTP(false, CLOCK_GetFreq(kCLOCK_CoreSysClk),  &delay) != 0) {
 *     read_pvt_delay_from_ring_osc();
 * }
 * EnableIRQ((IRQn_Type) PVT0_IRQn);
 * PVT_EnableAlertCount();
 * PVT_SetDelay(delay);
 * PVT_Start();
 * @endcode
 */
void PVT_Init(void);

/**
 * @brief De-initializes the PVT peripheral.
 */
void PVT_Deinit(void);

/**
 * @brief Enables PVT sensing.
 */
void PVT_Start(void);

/**
 * @brief Disables PVT sensing.
 */
void PVT_Stop(void);

/**
 * @brief Reads the PVT delay value from OTP.
 *
 * @param otp_initialized Specifies whether the OTP is already initialized or not.
 * false: The library will initialize the OTP, read the delay value, and then de-initialize the OTP.
 * true: OTP initialized by the calling app. Library will read the delay value from OTP and return.
 * @param core_freq_hz Core clock frequency in hertz.
 * @param fro_trim_freq_hz FRO trim frequency in hertz.
 * @param delay_value Pointer to a pvt_delay_t where the PVT delay value will be returned.
 * @retval 0 Successfully read the delay from OTP.
 * @retval 1 Failed to read delay from OTP.
 */
int32_t PVT_ReadDelayFromOTP(bool otp_initialized,
                             uint32_t core_freq_hz,
                             uint32_t fro_trim_freq_hz,
                             pvt_delay_t *delay_value);

/**
 * @brief Enables the PVT ring oscillator to calculate the optimal delay value.
 *
 * @note Must call PVT_ReadDelayFromRingOsc() exactly 500ms after enabling the PVT ring oscillator to get an
 * accurate delay value.
 *
 * @code
 * volatile bool ring_osc_ready;
 *
 * void ringo_wait_timer_cb(void) {
 *     ring_osc_ready = true;
 * }
 *
 * void read_pvt_delay_from_ring_osc() {
 *     pvt_delay_t delay;
 *     PVT_EnableRingOsc();
 *     ring_osc_ready = false;
 *     START_RINGO_WAIT_TIMER(PVT_RING_OSC_WAIT_TIME_MS);
 *     while (!ring_osc_ready);
 *     PVT_ReadDelayFromRingOsc(PVT_RING_OSC_WAIT_TIME_MS, &delay);
 *     PVT_DisableRingOsc();
 * }
 * @endcode
 */
void PVT_EnableRingOsc(void);

/**
 * @brief Disable the PVT ring oscillator.
 */
void PVT_DisableRingOsc(void);

/**
 * @brief Uses the PVT ring oscillator to calculate the optimal delay value.
 *
 * @note Must first enable the ring oscillator using PVT_EnableRingOsc().
 *
 * @code
 * volatile bool ring_osc_ready;
 *
 * void ringo_wait_timer_cb(void) {
 *     ring_osc_ready = true;
 * }
 *
 * void read_pvt_delay_from_ring_osc() {
 *     pvt_delay_t delay;
 *     PVT_EnableRingOsc();
 *     ring_osc_ready = false;
 *     START_RINGO_WAIT_TIMER(PVT_RING_OSC_WAIT_TIME_MS);
 *     while (!ring_osc_ready);
 *     PVT_ReadDelayFromRingOsc(PVT_RING_OSC_WAIT_TIME_MS, &delay);
 *     PVT_DisableRingOsc();
 * }
 * @endcode
 *
 * @param wait_time_ms Time in milliseconds since the ring oscillator was activated.
 * @param fro_trim_freq_hz FRO trim frequency in hertz.
 * @param delay_value Pointer to a pvt_delay_t where the PVT delay value will be placed.
 * @retval 0 Successfully read the delay from Ring Oscillator.
 * @retval 1 Failed to read delay from Ring Oscillator.
 */
int32_t PVT_ReadDelayFromRingOsc(uint32_t wait_time_ms,
                                 uint32_t fro_trim_freq_hz,
                                 pvt_delay_t *delay_value);

/**
 * @brief Reads the current PVT delay value.
 *
 * @param delay_value Pointer to a pvt_delay_t where the PVT delay value will be placed.
 * @retval 0 Successfully read the delay.
 * @retval 1 Failed to read delay because it has not been set.
 */
int32_t PVT_GetDelay(pvt_delay_t *delay_value);

/**
 * @brief Sets the delay value for the PVT.
 *
 * @note Must call PVT_Start() to start sensing.
 *
 * @param delay value read using PVT_ReadDelayFromOTP() or PVT_ReadDelayFromRingOsc().
 * @retval 0 Successfully sets the delay.
 * @retval 1 Failed to set delay. Invalid delay passed in.
 */
int32_t PVT_SetDelay(pvt_delay_t delay);

/**
 * @brief Enables the 20-bit PVT alert counter.
 *
 * Enables counting the amount of times the PVT interrupt has triggered.
 *
 * @note It is up to the user to clear the counter.
 */
void PVT_EnableAlertCount(void);

/**
 * @brief Disables the 20-bit PVT alert counter.
 */
void PVT_DisableAlertCount(void);

/**
 * @brief Reads the 20-bit PVT alert counter.
 *
 * @return 20-bit PVT alert counter value.
 */
uint32_t PVT_GetAlertCount(void);

/**
 * @brief Resets the 20-bit PVT alert counter.
 */
void PVT_ClearAlertCount(void);
/*@}*/

#if defined(__cplusplus)
}
#endif

#endif /* _PVT_H_ */

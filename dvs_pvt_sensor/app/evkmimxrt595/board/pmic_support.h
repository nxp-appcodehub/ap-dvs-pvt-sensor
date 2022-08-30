/*
 * Copyright 2019, 2022 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PMIC_SUPPORT_H_
#define _PMIC_SUPPORT_H_

#include "fsl_pca9420.h"
#include "fsl_power.h"

/*******************************************************************************
 * DEFINITION
 ******************************************************************************/
#define NUM_PMIC_MODES 4
/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/
bool BOARD_SetPmicVoltageForFreq(uint32_t cm33_clk_freq, uint32_t dsp_clk_freq);
void BOARD_InitPmicPins(void);
void BOARD_InitPmic(void);
void BOARD_SetPmicVoltageBeforeDeepSleep(void);
void BOARD_RestorePmicVoltageAfterDeepSleep(void);
void BOARD_SetPmicVoltageBeforeDeepPowerDown(void);
void BOARD_ConfigPmicModes(pca9420_modecfg_t *cfg, uint8_t num);
void BOARD_GetModeConfig(pca9420_mode_t mode, pca9420_modecfg_t *cfg);
void BOARD_SetModeConfig(pca9420_mode_t mode, pca9420_modecfg_t cfg);
pca9420_sw1_out_t BOARD_GetActiveVddcore(void);
void BOARD_SetActiveVddcore(pca9420_sw1_out_t volt);
#if defined(__cplusplus)
}
#endif /* __cplusplus*/

#endif /* _PMIC_SUPPORT_H_ */

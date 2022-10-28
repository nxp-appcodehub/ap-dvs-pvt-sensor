/*
 * Copyright 2019-2020, 2022 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "board.h"
#include "pmic_support.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PMIC_DECREASE_LVD_LEVEL_IF_HIGHER_THAN(currVolt, targetVolt) \
    do                                                               \
    {                                                                \
        if ((uint32_t)(currVolt) > (uint32_t)(targetVolt))           \
        {                                                            \
            POWER_SetLvdFallingTripVoltage(kLvdFallingTripVol_720);  \
        }                                                            \
    } while (0)

#define IOPCTL_PIO_ANAMUX_DI 0x00u        /*!<@brief Analog mux is disabled */
#define IOPCTL_PIO_FULLDRIVE_DI 0x00u     /*!<@brief Normal drive */
#define IOPCTL_PIO_FUNC0 0x00u            /*!<@brief Selects pin function 0 */
#define IOPCTL_PIO_INBUF_EN 0x40u         /*!<@brief Enables input buffer function */
#define IOPCTL_PIO_INV_DI 0x00u           /*!<@brief Input function is not inverted */
#define IOPCTL_PIO_PSEDRAIN_EN 0x0400u    /*!<@brief Pseudo Output Drain is enabled */
#define IOPCTL_PIO_PULLDOWN_EN 0x00u      /*!<@brief Enable pull-down function */
#define IOPCTL_PIO_PUPD_DI 0x00u          /*!<@brief Disable pull-up / pull-down function */
#define IOPCTL_PIO_SLEW_RATE_NORMAL 0x00u /*!<@brief Normal mode */
/*******************************************************************************
 * Variables
 ******************************************************************************/
static pca9420_handle_t pca9420Handle; 
static pca9420_modecfg_t pca9420CurrModeCfg[NUM_PMIC_MODES];
static pca9420_mode_t pca9420CurrMode;
static const pca9420_sw1_out_t pca9420VoltLevel[] = {kPCA9420_Sw1OutVolt1V100, kPCA9420_Sw1OutVolt1V000,
                                                     kPCA9420_Sw1OutVolt0V900, kPCA9420_Sw1OutVolt0V800,
                                                     kPCA9420_Sw1OutVolt0V700};
static bool pmicVoltChangedForDeepSleep;
static pca9420_sw1_out_t pmicVoltValueBeforeChange;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t BOARD_CalcVoltLevel(uint32_t cm33_clk_freq, uint32_t dsp_clk_freq)
{
    uint32_t i;
    uint32_t volt;
    uint32_t freq = MAX(cm33_clk_freq, dsp_clk_freq);

    for (i = 0U; i < POWER_FREQ_LEVELS_NUM; i++)
    {
        if (freq > powerFreqLevel[i])
        {
            break;
        }
    }

    if (i == 0U) /* Frequency exceed max supported */
    {
        volt = POWER_INVALID_VOLT_LEVEL;
    }
    else
    {
        volt = pca9420VoltLevel[i - 1U];
    }

    return volt;
}

void BOARD_InitPmicPins(void) {
    const uint32_t fc15_i2c_scl_config = (/* Pin is configured as I2C_SCL */
                                          IOPCTL_PIO_FUNC0 |
                                          /* Disable pull-up / pull-down function */
                                          IOPCTL_PIO_PUPD_DI |
                                          /* Enable pull-down function */
                                          IOPCTL_PIO_PULLDOWN_EN |
                                          /* Enables input buffer function */
                                          IOPCTL_PIO_INBUF_EN |
                                          /* Normal mode */
                                          IOPCTL_PIO_SLEW_RATE_NORMAL |
                                          /* Normal drive */
                                          IOPCTL_PIO_FULLDRIVE_DI |
                                          /* Analog mux is disabled */
                                          IOPCTL_PIO_ANAMUX_DI |
                                          /* Pseudo Output Drain is enabled */
                                          IOPCTL_PIO_PSEDRAIN_EN |
                                          /* Input function is not inverted */
                                          IOPCTL_PIO_INV_DI);
    /* FC15_SCL PIN (coords: K4) is configured as I2C SCL */
    IOPCTL->FC15_I2C_SCL = fc15_i2c_scl_config;

    const uint32_t fc15_i2c_sda_config = (/* Pin is configured as I2C_SDA */
                                          IOPCTL_PIO_FUNC0 |
                                          /* Disable pull-up / pull-down function */
                                          IOPCTL_PIO_PUPD_DI |
                                          /* Enable pull-down function */
                                          IOPCTL_PIO_PULLDOWN_EN |
                                          /* Enables input buffer function */
                                          IOPCTL_PIO_INBUF_EN |
                                          /* Normal mode */
                                          IOPCTL_PIO_SLEW_RATE_NORMAL |
                                          /* Normal drive */
                                          IOPCTL_PIO_FULLDRIVE_DI |
                                          /* Analog mux is disabled */
                                          IOPCTL_PIO_ANAMUX_DI |
                                          /* Pseudo Output Drain is enabled */
                                          IOPCTL_PIO_PSEDRAIN_EN |
                                          /* Input function is not inverted */
                                          IOPCTL_PIO_INV_DI);
    /* FC15_SDA PIN (coords: K6) is configured as I2C SDA */
    IOPCTL->FC15_I2C_SDA = fc15_i2c_sda_config;
}

void BOARD_InitPmic(void)
{
    pca9420_config_t pca9420Config;

    CLOCK_AttachClk(kFRO_DIV4_to_FLEXCOMM15);
    BOARD_PMIC_I2C_Init();
    PCA9420_GetDefaultConfig(&pca9420Config);
    pca9420Config.I2C_SendFunc    = BOARD_PMIC_I2C_Send;
    pca9420Config.I2C_ReceiveFunc = BOARD_PMIC_I2C_Receive;
    /* Disable active discharge prevent voltage spike when changing voltage */
    pca9420Config.disableSw1Bleed = true;
    PCA9420_Init(&pca9420Handle, &pca9420Config);
}

bool BOARD_SetPmicVoltageForFreq(uint32_t cm33_clk_freq, uint32_t dsp_clk_freq)
{
    power_lvd_falling_trip_vol_val_t lvdVolt;
    uint32_t volt;
    bool ret;

    PCA9420_GetCurrentMode(&pca9420Handle, &pca9420CurrMode);
    PCA9420_ReadModeConfigs(&pca9420Handle, pca9420CurrMode, &pca9420CurrModeCfg[pca9420CurrMode], 1);

    lvdVolt = POWER_GetLvdFallingTripVoltage();

    /* Enter FBB mode first */
    if (POWER_GetBodyBiasMode(kCfg_Run) != kPmu_Fbb)
    {
        POWER_EnterFbb();
    }

    volt = BOARD_CalcVoltLevel(cm33_clk_freq, dsp_clk_freq);
    ret  = volt != POWER_INVALID_VOLT_LEVEL;

    if (ret)
    {
        if (volt < kPCA9420_Sw1OutVolt0V800)
        {
            POWER_DisableLVD();
        }
        else
        {
            if (volt < kPCA9420_Sw1OutVolt0V900)
            {
                PMIC_DECREASE_LVD_LEVEL_IF_HIGHER_THAN(lvdVolt, kLvdFallingTripVol_795);
            }
            else if (volt < kPCA9420_Sw1OutVolt1V000)
            {
                PMIC_DECREASE_LVD_LEVEL_IF_HIGHER_THAN(lvdVolt, kLvdFallingTripVol_885);
            }
            else
            {
            }
        }

        /* Configure vddcore voltage value */
        pca9420CurrModeCfg[pca9420CurrMode].sw1OutVolt = (pca9420_sw1_out_t)volt;
        PCA9420_WriteModeConfigs(&pca9420Handle, pca9420CurrMode, &pca9420CurrModeCfg[pca9420CurrMode], 1);

        if (volt >= kPCA9420_Sw1OutVolt0V800)
        {
            POWER_RestoreLVD();
        }
    }

    return ret;
}

void BOARD_SetPmicVoltageBeforeDeepSleep(void)
{
    PCA9420_GetCurrentMode(&pca9420Handle, &pca9420CurrMode);
    PCA9420_ReadModeConfigs(&pca9420Handle, pca9420CurrMode, &pca9420CurrModeCfg[pca9420CurrMode], 1);

    if (pca9420CurrModeCfg[pca9420CurrMode].sw1OutVolt <= kPCA9420_Sw1OutVolt0V700)
    {
        pmicVoltValueBeforeChange   = pca9420CurrModeCfg[pca9420CurrMode].sw1OutVolt;
        pmicVoltChangedForDeepSleep = true;
        /* On resume from deep sleep with external PMIC, LVD is always used even if we have already disabled it.
         * Here we need to set up a safe threshold to avoid LVD reset and interrupt. */
        POWER_SetLvdFallingTripVoltage(kLvdFallingTripVol_720);
        pca9420CurrModeCfg[pca9420CurrMode].sw1OutVolt = kPCA9420_Sw1OutVolt0V750;
        PCA9420_WriteModeConfigs(&pca9420Handle, pca9420CurrMode, &pca9420CurrModeCfg[pca9420CurrMode], 1);
    }
    else
    {
    }
}

void BOARD_RestorePmicVoltageAfterDeepSleep(void)
{
    if (pmicVoltChangedForDeepSleep)
    {
        PCA9420_GetCurrentMode(&pca9420Handle, &pca9420CurrMode);
        PCA9420_ReadModeConfigs(&pca9420Handle, pca9420CurrMode, &pca9420CurrModeCfg[pca9420CurrMode], 1);
        pca9420CurrModeCfg[pca9420CurrMode].sw1OutVolt = pmicVoltValueBeforeChange;
        PCA9420_WriteModeConfigs(&pca9420Handle, pca9420CurrMode, &pca9420CurrModeCfg[pca9420CurrMode], 1);
        pmicVoltChangedForDeepSleep = false;
    }
    else
    {
    }
}

void BOARD_SetPmicVoltageBeforeDeepPowerDown(void)
{
    PCA9420_GetCurrentMode(&pca9420Handle, &pca9420CurrMode);
    PCA9420_ReadModeConfigs(&pca9420Handle, pca9420CurrMode, &pca9420CurrModeCfg[pca9420CurrMode], 1);

    /* Wakeup from deep power down is same as POR, and need VDDCORE >= 1.0V. Otherwise
       0.9V LVD reset value may cause wakeup failure. */
    if (pca9420CurrModeCfg[pca9420CurrMode].sw1OutVolt < kPCA9420_Sw1OutVolt1V000)
    {
        pca9420CurrModeCfg[pca9420CurrMode].sw1OutVolt = kPCA9420_Sw1OutVolt1V000;
        PCA9420_WriteModeConfigs(&pca9420Handle, pca9420CurrMode, &pca9420CurrModeCfg[pca9420CurrMode], 1);
    }
    else
    {
    }
}

void BOARD_ConfigPmicModes(pca9420_modecfg_t *cfg, uint8_t num) {
    uint8_t i;
    
    assert(num == NUM_PMIC_MODES);

    for (i = 0; i < num; i++)
    {
        PCA9420_GetDefaultModeConfig(&pca9420CurrModeCfg[i]);
    }

    /* Configuration PMIC mode to align with power lib like below:
     *  0b00    run mode, no special.
     *  0b01    deep sleep mode, vddcore 0.6V.
     *  0b10    deep powerdown mode, vddcore off.
     *  0b11    full deep powerdown mode vdd1v8 and vddcore off. */

    /* Mode 1: VDDCORE 0.6V. */
    pca9420CurrModeCfg[1].sw1OutVolt = kPCA9420_Sw1OutVolt0V600;
    /* Mode 2: VDDCORE off. */
    pca9420CurrModeCfg[2].enableSw1Out = false;

    /* Mode 3: VDDCORE, VDD1V8 and VDDIO off. */
    pca9420CurrModeCfg[3].enableSw1Out  = false;
    pca9420CurrModeCfg[3].enableSw2Out  = false;
    pca9420CurrModeCfg[3].enableLdo2Out = false;

    PCA9420_WriteModeConfigs(&pca9420Handle, kPCA9420_Mode0, &pca9420CurrModeCfg[0], ARRAY_SIZE(pca9420CurrModeCfg));
    
    for (i = 0; i < num; i++)
    {
        cfg[i] = pca9420CurrModeCfg[i];
    }
}

void BOARD_GetModeConfig(pca9420_mode_t mode, pca9420_modecfg_t *cfg) {
    *cfg = pca9420CurrModeCfg[mode];
}

void BOARD_SetModeConfig(pca9420_mode_t mode, pca9420_modecfg_t cfg) {
    pca9420CurrModeCfg[mode] = cfg;
    PCA9420_WriteModeConfigs(&pca9420Handle, mode, &pca9420CurrModeCfg[mode], 1);
}

pca9420_sw1_out_t BOARD_GetActiveVddcore(void) {
    pca9420_modecfg_t active_cfg;
    BOARD_GetModeConfig(kPCA9420_Mode0, &active_cfg);
    return active_cfg.sw1OutVolt;
}

void BOARD_SetActiveVddcore(pca9420_sw1_out_t volt) {
    pca9420_modecfg_t active_cfg;
    BOARD_GetModeConfig(kPCA9420_Mode0, &active_cfg);
    active_cfg.sw1OutVolt = volt;
    BOARD_SetModeConfig(kPCA9420_Mode0, active_cfg);
}

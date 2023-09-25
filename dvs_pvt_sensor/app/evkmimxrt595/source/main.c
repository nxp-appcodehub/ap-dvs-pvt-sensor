/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Drivers */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_pca9420.h"
#include "fsl_gpio.h"
#include "fsl_power.h"
#include "fsl_clock.h"
#include "fsl_utick.h"

/* Libraries */
#include "pvt.h"

/* Board */
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "pmic_support.h"
#include "app_config.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
 * Read the silicon ID.
 */
#define SILICONREV_ID_MINOR(x)     (x & SYSCTL0_SILICONREV_ID_MINOR_MASK)
#define SILICONREV_ID_MAJOR(x)     ((x & SYSCTL0_SILICONREV_ID_MAJOR_MASK) >> SYSCTL0_SILICONREV_ID_MAJOR_SHIFT)

/**
 * PMIC defines.
 */
#define PMIC_MIN_VOLT_MV           500
#define PMIC_STEP_SIZE_MV          25
#define CURRENT_VDDCORE_MV()       ((uint32_t) (PMIC_MIN_VOLT_MV + (BOARD_GetActiveVddcore() * PMIC_STEP_SIZE_MV)))

/*
 * GPIO to profile the PVT task.
 */
#define PROFILE_PVT_TASK_PORT      4U
#define PROFILE_PVT_TASK_PIN       28U
#define INIT_PVT_PROFILE_PIN()     {gpio_pin_config_t gpio_init = {kGPIO_DigitalOutput, 0};\
                                    GPIO_PortInit(GPIO, PROFILE_PVT_TASK_PORT);\
                                    GPIO_PinInit(GPIO, PROFILE_PVT_TASK_PORT, PROFILE_PVT_TASK_PIN, &gpio_init);}
#define SET_PVT_PROFILE_PIN()      (GPIO->SET[PROFILE_PVT_TASK_PORT] = 1U << PROFILE_PVT_TASK_PIN)
#define CLR_PVT_PROFILE_PIN()      (GPIO->CLR[PROFILE_PVT_TASK_PORT] = 1U << PROFILE_PVT_TASK_PIN)

/*
 * Timer to signal the PVT task to run.
 */
#define WAIT_TIMER                 UTICK0
#define WAIT_TIMER_CLK             kLPOSC_to_UTICK_CLK
#define INIT_WAIT_TIMER()          {CLOCK_AttachClk(WAIT_TIMER_CLK); \
                                    UTICK_Init(WAIT_TIMER);}
#define START_WAIT_TIMER(ms,cb)    (UTICK_SetTick(WAIT_TIMER, kUTICK_Onetime, ms*1000, cb))

#define APP_INFO                   "PVT Application Software Pack\r\n\n" \
                                   "This application uses the PMIC.\r\n" \
                                   "Make sure JS28 is set to [2:3] and JS25 is on.\r\n\n" \
                                   "You can monitor the following:\r\n" \
                                   "    JP27[2]: core frequency/200\r\n" \
                                   "    JS25[1]: VDDCORE\r\n" \
                                   "    J28[1] : PVT Task\r\n\n" \
                                   "PVT Lib Version = 0x%06x\r\n" \
                                   "SILICON_REV_ID = %X:%X\r\n" \
                                   "UUID = 0x%X_%X_%X_%X\r\n\n" \
                                   "\033[33;1m" \
                                   "NOTE: " \
                                   "\033[0m" \
                                   "Coremark is not optimized. Score should only be used to check " \
                                   "the impact of enabling the PVT task on app performance.\r\n\n", \
                                   PVT_GetLibVersion(), SILICONREV_ID_MAJOR(SYSCTL0->SILICONREV_ID), \
                                   SILICONREV_ID_MINOR(SYSCTL0->SILICONREV_ID), SYSCTL0->UUID[3], \
                                   SYSCTL0->UUID[2], SYSCTL0->UUID[1], SYSCTL0->UUID[0]
/*******************************************************************************
 * Variables
 ******************************************************************************/
TaskHandle_t pvt_task_handle;
SemaphoreHandle_t vddcore_mutex = NULL;
volatile bool ring_osc_ready = false;
uint32_t main_clk_freq_hz, min_vddcore, max_vddcore;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void PVT0_IRQHandler(void);
static void pvt_wait_timer_callback(void);
static void pvt_ring_osc_wait_timer_callback(void);
static void workload_task(void *pvParameters);
static void pvt_task(void *pvParameters);
static bool adjust_vddcore(void);
static void config_pvt(void);
static void read_pvt_delay_from_ring_osc(uint8_t *delay);
extern void coremark_main(void);
#if configUSE_TICKLESS_IDLE == 2
extern void tickless_timer_init();
#endif
/*******************************************************************************
 * Code
 ******************************************************************************/
/**
 * PVT0 interrupt handler that disables PVT sensing and signals/yields to the PVT task
 * to immediately increase VDDCORE.
 */
void PVT0_IRQHandler(void) {
    NVIC_ClearPendingIRQ((IRQn_Type) PVT0_IRQn);
    /* Disable the PVT sensor so it doesn't keep interrupting while we try to increase VDDCORE */
    PVT_Stop();
    /* Wake-up the PVT task to increase VDDCORE immediately */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(pvt_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * Timer callback to signals/yields the PVT task to adjust VDDCORE.
 */
static void pvt_wait_timer_callback(void) {
    /* Wake-up the PVT task to increase VDDCORE immediately */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(pvt_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * Timer callback to signal that the ring osc is ready to read pvt delay.
 */
static void pvt_ring_osc_wait_timer_callback(void) {
    ring_osc_ready = true;
}

int main(void) {
    pca9420_modecfg_t pmic_mode_cfg[NUM_PMIC_MODES];

    /* Initialize board hardware. */
    BOARD_InitPins();
    BOARD_BootClockFroRUN(FRO_TRIM_FREQ_HZ);
    BOARD_InitDebugConsole();

    PRINTF(APP_INFO);

    /* Configure PMIC */
    BOARD_InitPmicPins();
    BOARD_InitPmic();
    BOARD_ConfigPmicModes(pmic_mode_cfg, NUM_PMIC_MODES);
    BOARD_SetActiveVddcore(MAX_VDDCORE);
    vddcore_mutex = xSemaphoreCreateMutex();

#if configUSE_TICKLESS_IDLE == 2
    tickless_timer_init();
#endif

#if ENABLE_PVT
    /* Initialize UTICK to use for reading ring osc delay and PVT task */
    INIT_WAIT_TIMER();
    /* Start the pvt sensor */
    config_pvt();
    if (xTaskCreate(pvt_task, "PVT_task", configMINIMAL_STACK_SIZE + 100, NULL, pvt_task_PRIORITY, &pvt_task_handle) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }
#endif

    if (xTaskCreate(workload_task, "Workload_task", configMINIMAL_STACK_SIZE + 1000, NULL, workload_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1)
            ;
    }

    PRINTF("Press ENTER to start application.\r\n\n");
    GETCHAR();
    PRINTF("Starting Application. Coremark will print results every 10-20s.\r\n");

    vTaskStartScheduler();
    for (;;)
        ;
}

/**
 * Periodically runs Coremark and idles for WORKLOAD_DELAY_TICKS.
 */
static void workload_task(void *pvParameters) {
    for (;;) {
        coremark_main();
        PRINTF("--------------------------\r\n"  \
               "Optimized VDDCORE = %umV\r\n\n"  \
               "--------------------------\r\n", \
               CURRENT_VDDCORE_MV());

        /* Add a delay to let it go into idle mode */
        vTaskDelay(pdMS_TO_TICKS(WORKLOAD_DELAY_MS));
    }
}

/**
 * Finds the optimal VDDCORE for the application.
 * Runs when one of the following conditions is met:
 *     (a) First time running
 *     (b) PVT Interrupt triggered
 *     (c) VDDCORE > MIN_VDDCORE and WAIT_TIMER expired
 */
static void pvt_task(void *pvParameters) {
    bool at_min_vddcore = false;

    /* Add a pin toggle to profile task activity */
    INIT_PVT_PROFILE_PIN();

    for(;;) {
        /* Take the lock to prevent idle task from going into low power mode in the middle of adjusting VDDCORE */
        if (xSemaphoreTake(vddcore_mutex, portMAX_DELAY) == pdTRUE) {
            SET_PVT_PROFILE_PIN();
            at_min_vddcore = adjust_vddcore();
            CLR_PVT_PROFILE_PIN();
            xSemaphoreGive(vddcore_mutex);
        }

        if (!at_min_vddcore) {
            /* Only enable periodic timer if it's possible for VDDCORE to go lower */
            START_WAIT_TIMER(PVT_TASK_WAIT_MS, pvt_wait_timer_callback);
        }

        /* Wait for UTICK or PVT interrupt notification */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

/**
 * Increases VDDCORE if the PVT interrupt triggered, else it tries
 * to decrease VDDCORE as much as possible.
 *
 * Returns true if VDDCORE == MIN_VDDCORE, false otherwise.
 */
static bool adjust_vddcore(void) {
    pca9420_sw1_out_t cur_vddcore = BOARD_GetActiveVddcore();

    if (PVT_GetAlertCount() > 0) {
        /* PVT interrupt fired, so we increase VDDCORE */
        BOARD_SetActiveVddcore(++cur_vddcore);
        PVT_ClearAlertCount();
        PVT_Start();
        return false;
    }

    /* Try to decrease VDDCORE as much as possible until we reach MIN_VDDCORE or PVT interrupt fires */
    while (cur_vddcore > MIN_VDDCORE) {
        BOARD_SetActiveVddcore(--cur_vddcore);
        START_WAIT_TIMER(PMIC_SETTLING_TIME_MS, pvt_wait_timer_callback);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (PVT_GetAlertCount() > 0) {
            BOARD_SetActiveVddcore(++cur_vddcore);
            PVT_ClearAlertCount();
            PVT_Start();
            break;
        }
    }

    return (cur_vddcore <= MIN_VDDCORE);
}

/**
 * Enables the PVT ring oscillator, reads and sets the PVT delay,
 * enables the PVT alert counter, enables the PVT interrupt,
 * and finally starts the PVT sensing.
 * */
static void config_pvt(void) {
    pvt_delay_t delay;
    status_t status;
    bool read_from_otp = true;

    PVT_Init();

    /* Disable LVD interrupts  and set it to the lowest setting */
    PMC->CTRL = 0;
    PMC->LVDCORECTRL = 0;

    /* Set Active VDDCORE to max for current FRO trim */
    BOARD_SetActiveVddcore(MAX_VDDCORE);

    /* Try to read delay from OTP. If unsuccessful, try to read from ring oscillator */
    status = PVT_ReadDelayFromOTP(false, CLOCK_GetFreq(kCLOCK_CoreSysClk), FRO_TRIM_FREQ_HZ, &delay);
    PRINTF("OTP: %d\r\n", delay);
    status = kStatus_Fail;
    if (status != kStatus_Success) {
        /* VDDCORE must == 0.9V and and temp. == 25C before calling PVT_ReadDelayFromRingOsc */
        PRINTF("\033[35;1m" \
               "WARNING: " \
               "\033[0m" \
               "Reading delay from Ring Osc. Make sure the following are met:\r\n" \
                "    VDDCORE     == %dmV\r\n" \
                "    Temperature == 25C\r\n\n" \
                "Press ENTER to continue\r\n\n", PMIC_MIN_VOLT_MV + \
                ((uint32_t)MAX_VDDCORE * PMIC_STEP_SIZE_MV));
        GETCHAR();
        read_pvt_delay_from_ring_osc(&delay);
        read_from_otp = false;
    }

    PRINTF("%s%u\r\n\n", (read_from_otp == true) ? "OTP Delay = " : "Ring Osc. Delay = ", delay);
    GETCHAR();
    /* Enable interrupt */
    NVIC_ClearPendingIRQ((IRQn_Type) PVT0_IRQn);
    EnableIRQ((IRQn_Type) PVT0_IRQn);
    /* This is used to check if the PVT interrupt triggered when trying to optimize VDDCORE */
    PVT_EnableAlertCount();
    /* Set the delay and start sensing */
    PVT_SetDelay(delay);
    PVT_Start();
}

/**
 * Follows the specific sequence as defined in the pvt.h documentation to read the delay
 * from the ring oscillator.
 */
static void read_pvt_delay_from_ring_osc(uint8_t *delay) {
   status_t status;
   ring_osc_ready = false;
   PVT_EnableRingOsc();
   /* Use a timer to wait 500ms because it is more accurate */
   START_WAIT_TIMER(PVT_RING_OSC_WAIT_TIME_MS, pvt_ring_osc_wait_timer_callback);
   while (!ring_osc_ready);
   status = PVT_ReadDelayFromRingOsc(PVT_RING_OSC_WAIT_TIME_MS, FRO_TRIM_FREQ_HZ, delay);
   PVT_DisableRingOsc();
   if (status != kStatus_Success) {
       PRINTF("\033[31;1m" \
               "ERROR: " \
               "\033[0m" \
               "Unable to read delay from Ring Osc! Reset and try again.\r\n");
       while(1);
   }
}

/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2019, 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/* Compiler includes. */
#if defined(__ICCARM__)
#include <intrinsics.h>
#endif
#include <stdbool.h>
#include <limits.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pvt.h"

#if configUSE_TICKLESS_IDLE == 2
#include "fsl_device_registers.h"
#include "fsl_ostimer.h"
#include "fsl_power.h"
#endif

#define OSTIMER_CLOCK_HZ           CLK_RTC_32K_CLK
#define OSTIMER_CLOCKS_PER_TICK    (OSTIMER_CLOCK_HZ / configTICK_RATE_HZ)

#define APP_DEEPSLEEP_RUNCFG0 (SYSCTL0_PDRUNCFG0_RBBSRAM_PD_MASK | SYSCTL0_PDRUNCFG0_RBB_PD_MASK)
#define APP_DEEPSLEEP_RAM_APD 0x00FFF000U /* 0x80000 - 0x2FFFFF keep powered */
#define APP_DEEPSLEEP_RAM_PPD 0x0U
#define APP_EXCLUDE_FROM_DEEPSLEEP                                                                           \
    (((const uint32_t[]){APP_DEEPSLEEP_RUNCFG0,                                                              \
                         (SYSCTL0_PDSLEEPCFG1_FLEXSPI0_SRAM_APD_MASK | SYSCTL0_PDSLEEPCFG1_SRAM_SLEEP_MASK), \
                         APP_DEEPSLEEP_RAM_APD, APP_DEEPSLEEP_RAM_PPD}))
                                       
extern uint32_t SystemCoreClock; /* in Kinetis SDK, this contains the system core clock speed */
extern uint32_t gRamEnable;
extern SemaphoreHandle_t vddcore_mutex;

volatile uint32_t deep_sleep_block_mask = 0;

/*
 * The number of SysTick increments that make up one tick period.
 */
#if configUSE_TICKLESS_IDLE == 2
static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The maximum number of tick periods that can be suppressed.
 */
#if configUSE_TICKLESS_IDLE == 2
static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The flag of LPTIMER is occurs or not.
 */
#if configUSE_TICKLESS_IDLE == 2
static volatile bool ulLPTimerInterruptFired = false;
#endif /* configUSE_TICKLESS_IDLE */

#if configUSE_TICKLESS_IDLE == 2
static volatile uint64_t lastOstimerValue;

static void OSTimer_CaptureCallback(void)
{
    ulLPTimerInterruptFired = true;
}

static void OSTimer_SetMatchInterruptTime(uint32_t clocks)
{
    lastOstimerValue = OSTIMER_GetCurrentTimerValue(OSTIMER0);

    /* Set the match value with unit of ticks. */
    OSTIMER_SetMatchValue(OSTIMER0, lastOstimerValue + clocks, OSTimer_CaptureCallback);
}

static uint32_t OSTimer_GetElapsedTicks()
{
    uint64_t ostimerValue = OSTIMER_GetCurrentTimerValue(OSTIMER0);
    uint32_t ticks;

    ticks = (uint32_t)((ostimerValue - lastOstimerValue) / OSTIMER_CLOCKS_PER_TICK);
    return ticks;
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t ulReloadValue, ulCompleteTickPeriods;
    eSleepModeStatus eSleepStatus;

    /* Make sure the SysTick reload value does not overflow the counter. */
    if (xExpectedIdleTime > xMaximumPossibleSuppressedTicks)
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }
    if (xExpectedIdleTime == 0)
    {
        return;
    }

    /* Make sure pvt thread is not in the middle of adjusting voltage */
    if (xSemaphoreTake(vddcore_mutex, 0) == pdFALSE)
    {
        return;
    }

    /* Calculate the reload value required to wait xExpectedIdleTime
    tick periods.  -1 is used because this code will execute part way
    through one of the tick periods. */
    ulReloadValue = OSTIMER_CLOCKS_PER_TICK * (xExpectedIdleTime - 1UL);

    /* Stop the RTC and systick momentarily.  The time the RTC and systick is stopped for
    is accounted for as best it can be, but using the tickless mode will
    inevitably result in some tiny drift of the time maintained by the
    kernel with respect to calendar time. */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    __disable_irq();
    __DSB();
    __ISB();

    eSleepStatus = eTaskConfirmSleepModeStatus();
    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if (eSleepStatus == eAbortSleep)
    {
        /* Release the VDDCORE semaphore so PVT task can adjust the voltage if need be */
        xSemaphoreGive(vddcore_mutex);

        /* Restart from whatever is left in the count register to complete
        this tick period. */
        SysTick->LOAD = SysTick->VAL;

        /* Restart SysTick. */
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

        /* Reset the reload register to the value required for normal tick
        periods. */
        SysTick->LOAD = ulTimerCountsForOneTick - 1UL;

        /* Re-enable interrupts - see comments above __disable_irq()
        call above. */
        __enable_irq();
    }
    else
    {
        if (eSleepStatus != eNoTasksWaitingTimeout)
        {
            /* Set the new reload value. */
            OSTimer_SetMatchInterruptTime(ulReloadValue);

            EnableDeepSleepIRQ(OS_EVENT_IRQn);
        }

        /* Sleep until something happens. */
        if (deep_sleep_block_mask)
        {
            __DSB();
            __WFI();
            __ISB();
        }
        else
        {
            POWER_EnterDeepSleep(APP_EXCLUDE_FROM_DEEPSLEEP);
        }

        /* Release the VDDCORE semaphore so PVT task can adjust the voltage if need be */
        PVT_ClearAlertCount();
        xSemaphoreGive(vddcore_mutex);

        ulLPTimerInterruptFired = false;

        /* Re-enable interrupts - see comments above __disable_irq()
        call above. */
        __enable_irq();
        __NOP();
        if (ulLPTimerInterruptFired)
        {
            /* The tick interrupt handler will already have pended the tick
            processing in the kernel.  As the pending tick will be
            processed as soon as this function exits, the tick value
            maintained by the tick is stepped forward by one less than the
            time spent waiting. */
            ulCompleteTickPeriods   = xExpectedIdleTime - 1UL;
            ulLPTimerInterruptFired = false;
        }
        else
        {
            /* Something other than the tick interrupt ended the sleep.
            Work out how long the sleep lasted rounded to complete tick
            periods (not the ulReload value which accounted for part
            ticks). */
            ulCompleteTickPeriods = OSTimer_GetElapsedTicks();
        }

        /* Set SysTick->LOAD back to its standard value.
        The critical section is used to ensure the tick interrupt
        can only execute once in the case that the reload register is near
        zero. */
        portENTER_CRITICAL();
        {
            SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
            vTaskStepTick(ulCompleteTickPeriods);
            SysTick->LOAD = ulTimerCountsForOneTick - 1UL;
        }
        portEXIT_CRITICAL();
    }
}

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
void vPortSetupTimerInterrupt(void)
{
    /* Calculate the constants required to configure the tick interrupt. */
    ulTimerCountsForOneTick   = (configCPU_CLOCK_HZ / configTICK_RATE_HZ);
    xMaximumPossibleSuppressedTicks = ULONG_MAX;

    /* Configure SysTick to interrupt at the requested rate. */
    SysTick->LOAD = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
    SysTick->VAL  = 0UL;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}

void tickless_timer_init(void)
{
    /* Enable 32KHz Oscillator clock */
    CLOCK_EnableOsc32K(true);
    CLOCK_EnableClock(kCLOCK_Rtc);
    RTC->CTRL &= ~RTC_CTRL_RTC_OSC_PD_MASK;
    CLOCK_DisableClock(kCLOCK_Rtc);

    /* Initialize the OS timer */
    CLOCK_AttachClk(kOSC32K_to_OSTIMER_CLK);
    OSTIMER_Init(OSTIMER0);
}
#endif /* configUSE_TICKLESS_IDLE */


// Copyright 2018 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "main.h"
#include "PowerMode.h"
#include "stm32l4xx_hal.h"

#include <stdint.h>

volatile PowerMode power_mode{PowerMode::RUN};
PowerMode previous_mode{PowerMode::RUN};        // Always start in RUN mode.

/**
 * Handle state transitions here.  Valid transitions are from RUN to any mode,
 * LPRUN to any mode, SLEEP to LPRUN or STOP, and STOP to LPRUN or SLEEP.
 *
 * If moving from RUN to any SLEEP or STOP, we assume the upper layers have
 * already disconnected the connection, either bu
 *
 * @param ulExpectedIdleTime
 */
extern "C" void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
    switch (power_mode)
    {
    case PowerMode::RUN:
        break;
    case PowerMode::LPRUN:
        if (previous_mode != power_mode);
        break;
    case PowerMode::SLEEP:
        break;
    case PowerMode::STOP:
        break;
    }
}

extern "C" void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
    switch (power_mode)
    {
    case PowerMode::RUN:
        break;
    case PowerMode::LPRUN:
        break;
    case PowerMode::SLEEP:
        break;
    case PowerMode::STOP:
        break;
    }
}


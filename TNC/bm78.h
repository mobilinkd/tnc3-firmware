// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__BM78_H_
#define MOBILINKD__TNC__BM78_H_

#include <stm32l4xx_hal.h>

#ifdef __cplusplus
#include <cstdint>

extern "C" {
#else
#include <stdint.h>
#endif

/**
 * The BM78 module says that the module is ready about 475ms after start,
 * but that the proper thing to do is wait until P1_5 (BT_STATE2) is high
 * and P0_4 (BT_STATE1) is low.
 *
 * This waits for the module to reach this state.  If it fails to become
 * ready after 2 seconds, it enters the error handler.
 *
 * @note This does not work to detect whether the module is ready for
 *  EEPROM programming, only for transparent data mode.  The time required
 *  and state indication for EEPROM programming mode is not specified.
 */
void bm78_wait_until_ready(void);
void bm78_state_change(void);
int bm78_disable(void);
int bm78_enable(void);
int bm78_initialized(void);
int bm78_initialize(void);
void bm78_initialize_mac_address(void);

HAL_StatusTypeDef bm78_send(const char* data, uint16_t size, uint32_t timeout);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MOBILINKD__TNC__BM78_H_

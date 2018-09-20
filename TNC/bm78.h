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

void bm78_state_change(void);
int bm78_disable(void);
int bm78_enable(void);
int bm78_initialized(void);
int bm78_initialize(void);
HAL_StatusTypeDef bm78_send(const char* data, uint16_t size, uint32_t timeout);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MOBILINKD__TNC__BM78_H_

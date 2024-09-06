// Copyright 2017-2024 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void startIOEventTask(void const* argument);

extern osMessageQId ioEventQueueHandle;
#ifndef NUCLEOTNC
typedef enum ConnectionState {DISCONNECTED, USB_CONNECTED, BT_CONNECTED} ConnectionStateType;
extern volatile ConnectionStateType connectionState;
#endif

#ifdef __cplusplus
}

namespace mobilinkd { namespace tnc {

void print_startup_banner() __attribute__((noinline));

}} // mobilinkd::tnc

#endif

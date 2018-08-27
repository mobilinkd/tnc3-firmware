// Copyright 2017 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__GPIO_HPP_
#define MOBILINKD__TNC__GPIO_HPP_

#include "main.h"
#include "stm32l4xx_hal.h"
#include <cstdint>

namespace mobilinkd { namespace tnc {

template <uint32_t BANK, uint16_t PIN>
struct GPIO {
    static void on() {
        HAL_GPIO_WritePin(reinterpret_cast<GPIO_TypeDef*>(BANK), PIN, GPIO_PIN_SET);
    }
    static void off() {
        HAL_GPIO_WritePin(reinterpret_cast<GPIO_TypeDef*>(BANK), PIN, GPIO_PIN_RESET);
    }
    static void toggle() {
        HAL_GPIO_TogglePin(reinterpret_cast<GPIO_TypeDef*>(BANK), PIN);
    }
    static bool get() {
        return HAL_GPIO_ReadPin(reinterpret_cast<GPIO_TypeDef*>(BANK), PIN);
    }
#if 0
    static bool operator bool() {
        return HAL_GPIO_ReadPin(BANK, PIN);
    }
    static bool operator=(bool value) {
        return HAL_GPIO_WritePin(BANK, PIN, GPIO_PinState(value));
    }
#endif
};

namespace gpio {

typedef GPIO<(uint32_t)GPIOB_BASE,PTT_A_Pin> PTT_SIMPLEX;
typedef GPIO<(uint32_t)GPIOB_BASE,PTT_B_Pin> PTT_MULTIPLEX;
typedef GPIO<(uint32_t)GPIOH_BASE,USB_POWER_Pin> USB_POWER;
typedef GPIO<(uint32_t)GPIOH_BASE,SW_POWER_Pin> SW_POWER;
typedef GPIO<(uint32_t)GPIOB_BASE,BT_RESET_Pin> BT_RESET;
typedef GPIO<(uint32_t)GPIOB_BASE,BT_STATE1_Pin> BT_STATE1_IN;
typedef GPIO<(uint32_t)GPIOB_BASE,BT_STATE2_Pin> BT_STATE2_IN;
typedef GPIO<(uint32_t)GPIOB_BASE,BT_CMD_Pin> BT_CMD;
typedef GPIO<(uint32_t)GPIOC_BASE,BT_WAKE_Pin> BT_WAKE;
typedef GPIO<(uint32_t)GPIOA_BASE,BT_SLEEP_Pin> BT_SLEEP;
typedef GPIO<(uint32_t)GPIOB_BASE,USB_CE_Pin> USB_CE;
typedef GPIO<(uint32_t)GPIOB_BASE,VDD_EN_Pin> VDD_ENABLE;
typedef GPIO<(uint32_t)GPIOA_BASE,BAT_DIVIDER_Pin> BAT_DIVIDER;
typedef GPIO<(uint32_t)GPIOB_BASE,AUDIO_ATTEN_Pin> AUDIO_OUT_ATTEN;
typedef GPIO<(uint32_t)GPIOA_BASE,LED_TX_Pin> LED_TX;
typedef GPIO<(uint32_t)GPIOA_BASE,LED_RX_Pin> LED_RX;
typedef GPIO<(uint32_t)GPIOA_BASE,LED_BT_Pin> LED_BT;

// Special
typedef GPIO<GPIOA_BASE,GPIO_PIN_12> USB_DP;
typedef GPIO<GPIOA_BASE,GPIO_PIN_11> USB_DM;

}}} // mobilinkd::tnc::gpio

#endif // MOBILINKD__TNC__GPIO_HPP_

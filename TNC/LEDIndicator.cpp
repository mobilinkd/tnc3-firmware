// Copyright 2017-2024 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "LEDIndicator.h"
#include "main.h"

#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_tim.h>
#include <stm32l4xx_hal_tim_ex.h>
#include <cmsis_os.h>

#include <functional>
#include <atomic>
#include <array>
#include <tuple>

#include <stdint.h>

extern TIM_HandleTypeDef LED_PWM_TIMER_HANDLE;

namespace mobilinkd {
namespace tnc {

#ifndef NUCLEOTNC
    constexpr uint32_t BLUE_CHANNEL = TIM_CHANNEL_1;
    constexpr uint32_t GREEN_CHANNEL = TIM_CHANNEL_2;
    constexpr uint32_t RED_CHANNEL = TIM_CHANNEL_3;
#else
    constexpr uint32_t BLUE_CHANNEL = TIM_CHANNEL_3;   // YELLOW...
    constexpr uint32_t GREEN_CHANNEL = TIM_CHANNEL_2;
    constexpr uint32_t RED_CHANNEL = TIM_CHANNEL_1;
#endif

using LedAction = std::tuple<int16_t, int16_t, int16_t, int16_t>; // increment (RGB), count in 10ms units

constexpr std::array<LedAction, 2> TurningOnIndication {
    LedAction{12, 6, 0, 100},
    LedAction{0, 0, 0, 1000}
};

constexpr std::array<LedAction, 6> InitializingIndication {
    LedAction{12, 6, 0, 100},
    LedAction{-1200, -600, 0, 1},
    LedAction{24, 12, 0, 50},
    LedAction{-1200, -600, 0, 1},
    LedAction{48, 24, 0, 25},
    LedAction{0, 0, 0, 1000}
};

constexpr std::array<LedAction, 3> OVPErrorIndication {
    LedAction{400, 0, 0, 5},
    LedAction{-2000, 0, 0, 1},
    LedAction{0, 0, 0, 5}
};

constexpr std::array<LedAction, 11> BatteryLowIndication {
    LedAction{50, 12, 0, 40},
    LedAction{-2000, -480, 0, 1},
    LedAction{24, 6, 0, 48},
    LedAction{-1104, -288, 0, 1},
    LedAction{12, 3, 0, 58},
    LedAction{-696, -174, 0, 1},
    LedAction{8, 2, 0, 69},
    LedAction{-552, -138, 0, 1},
    LedAction{4, 1, 0, 83},
    LedAction{-332, -83, 0, 1},
    LedAction{0, 0, 0, -1},
};

constexpr std::array<LedAction, 3> TurningOffIndication {
    LedAction{2000, 800, 0, 1},
    LedAction{-10, -4, 0, 200},
    LedAction{0, 0, 0, -1},
};

/**
 * No connection shows a low, slow breathing. Each breath inhale takes
 * for 500ms, is held for 500ms, and exhaled in 500ms. This is repeated
 * every 10 seconds.  Maximum brightness is 20%.
 *
 * Each interrupt occurs at 10ms intervals.
 *
 * The sequence is:
 *  - ramp up 500ms(50)
 *  - hold 500ms (50)
 *  - ramp down 500ms (50)
 *  - wait 9000ms (850)
 *
 *
 */
constexpr std::array<LedAction, 4> NoConnectionIndication {
    LedAction{0, 0, 10, 65},
    LedAction{0, 0, 0, 20},
    LedAction{0, 0, -10, 65},
    LedAction{0, 0, 0, 850}
};

/**
 * Bluetooth connection shows a double blip. Each blip lasts for 200ms
 * and is separated by 200ms, and is repeated ever 5 seconds.
 *
 * Each interrupt occurs at 10ms intervals.
 *
 * The sequence is:
 *  - ramp up 100s(10)
 *  - ramp down 100ms (10)
 *  - wait 200ms (20)
 *  - ramp up 100ms (10)
 *  - ramp down 100ms (10)
 *  - wait 4400ms (440)
 *
 *
 */
constexpr std::array<LedAction, 6> BluetoothConnectionIndication {
    LedAction{0, 0, 200, 10},
    LedAction{0, 0, -200, 10},
    LedAction{0, 0, 0, 20},
    LedAction{0, 0, 200, 10},
    LedAction{0, 0, -200, 10},
    LedAction{0, 0, 0, 440}
};

/**
 * USB connection shows a triple blip. Each blip lasts for 200ms. The
 * first two are separated by 400ms.  The third comes 200ms later.  This
 * is repeated ever 5 seconds.
 *
 * Each interrupt occurs at 10ms intervals.
 *
 * The sequence is:
 *  - ramp up 100s(10)
 *  - ramp down 100ms (10)
 *  - wait 200ms (20)
 *  - ramp up 100s(10)
 *  - ramp down 100ms (10)
 *  - wait 400ms (20)
 *  - ramp up 100ms (10)
 *  - ramp down 100ms (10)
 *  - wait 3800ms (440)
 *
 *
 */
constexpr std::array<LedAction, 9> USBConnectionIndication {
    LedAction{0, 0, 200, 10},
    LedAction{0, 0, -200, 10},
    LedAction{0, 0, 0, 20},
    LedAction{0, 0, 200, 10},
    LedAction{0, 0, -200, 10},
    LedAction{0, 0, 0, 20},
    LedAction{0, 0, 200, 10},
    LedAction{0, 0, -200, 10},
    LedAction{0, 0, 0, 400}
};

constexpr std::array<LedAction, 3> VDDErrorIndication {
    LedAction{0, 0, 400, 5},
    LedAction{0, 0, -2000, 1},
    LedAction{0, 0, 0, 4},
};

class RGBIndicator {
    const LedAction* actions = nullptr;
    size_t action_steps = 0;
    size_t step = 0;
    bool red_state = false;
    bool green_state = false;
    bool blue_state = false;
    uint16_t red = 0;
    uint16_t green = 0;
    uint16_t blue = 0;
    int32_t time_step;

public:
    void reset() {
        HAL_TIM_Base_Stop_IT(&LED_PWM_TIMER_HANDLE);
        HAL_TIM_PWM_Stop(&LED_PWM_TIMER_HANDLE, RED_CHANNEL);
        HAL_TIM_PWM_Stop(&LED_PWM_TIMER_HANDLE, GREEN_CHANNEL);
        HAL_TIM_PWM_Stop(&LED_PWM_TIMER_HANDLE, BLUE_CHANNEL);
        step = 0;
        red_state = false;
        green_state = false;
        blue_state = false;
        red = 0;
        green = 0;
        blue = 0;
        time_step = 0;
    }

    template <size_t N>
    void start(const std::array<LedAction, N>& indication) {
        reset();
        actions = &indication.front();
        action_steps = N;
        HAL_TIM_Base_Start_IT(&LED_PWM_TIMER_HANDLE);
    }

    void interrupt_callback() {
        if (std::get<3>(actions[step]) == -1) {
            reset();
        }
        red += std::get<0>(actions[step]);
        green += std::get<1>(actions[step]);
        blue += std::get<2>(actions[step]);
        time_step += 1;
        if (time_step == std::get<3>(actions[step])) {
            step += 1;
            time_step = 0;
            if (step == action_steps) {
                step = 0;
            }
        }

        // CCR registers must match the TIM_CHANNEL used for each LED in Flash.
#ifndef NUCLEOTNC
        LED_PWM_TIMER_HANDLE.Instance->CCR1 = blue;
        LED_PWM_TIMER_HANDLE.Instance->CCR2 = green;
        LED_PWM_TIMER_HANDLE.Instance->CCR3 = red;
#else
        LED_PWM_TIMER_HANDLE.Instance->CCR1 = red;
        LED_PWM_TIMER_HANDLE.Instance->CCR2 = green;
        LED_PWM_TIMER_HANDLE.Instance->CCR3 = blue; // YELLOW
#endif

        if (red && !red_state) {
            red_state = true;
            HAL_TIM_PWM_Start(&LED_PWM_TIMER_HANDLE, RED_CHANNEL);
        }

        if (green && !green_state) {
            green_state = true;
            HAL_TIM_PWM_Start(&LED_PWM_TIMER_HANDLE, GREEN_CHANNEL);
        }

        if (blue && !blue_state) {
            blue_state = true;
            HAL_TIM_PWM_Start(&LED_PWM_TIMER_HANDLE, BLUE_CHANNEL);
        }
    }

    void set_red(uint16_t value) {
        red = value;
        if (!value) {
            HAL_TIM_PWM_Stop(&LED_PWM_TIMER_HANDLE, RED_CHANNEL);
            red_state = false;
        }
    }

    void set_green(uint16_t value) {
        green = value;
        if (!value) {
            HAL_TIM_PWM_Stop(&LED_PWM_TIMER_HANDLE, GREEN_CHANNEL);
            green_state = false;
        }
    }

    void set_blue(uint16_t value) {
        blue = value;
        if (!value) {
            HAL_TIM_PWM_Stop(&LED_PWM_TIMER_HANDLE, BLUE_CHANNEL);
            blue_state = false;
        }
    }
};

static RGBIndicator indicator;

}
} // mobilinkd::tnc

void LED_TIMER_PeriodElapsedCallback()
{
    mobilinkd::tnc::indicator.interrupt_callback();
}

void indicate_turning_on(void)
{
    mobilinkd::tnc::indicator.start(mobilinkd::tnc::TurningOnIndication);
}

void indicate_initializing(void)
{
    mobilinkd::tnc::indicator.start(mobilinkd::tnc::InitializingIndication);
}

void indicate_ovp_error(void)
{
    mobilinkd::tnc::indicator.start(mobilinkd::tnc::OVPErrorIndication);
}

void indicate_battery_low(void)
{
    mobilinkd::tnc::indicator.start(mobilinkd::tnc::BatteryLowIndication);
}

void indicate_turning_off()
{
    mobilinkd::tnc::indicator.start(mobilinkd::tnc::TurningOffIndication);
}

void reset_indicator()
{
    mobilinkd::tnc::indicator.reset();
}

void indicate_initializing_ble(void)
{
    tx_off();
}

void indicate_vdd_error(void)
{
    mobilinkd::tnc::indicator.start(mobilinkd::tnc::VDDErrorIndication);
}

void indicate_on()
{
    tx_off();
    rx_off();
}

void indicate_waiting_to_connect(void)
{
    mobilinkd::tnc::indicator.start(mobilinkd::tnc::NoConnectionIndication);
}

void indicate_connected_via_usb(void)
{
    mobilinkd::tnc::indicator.start(mobilinkd::tnc::USBConnectionIndication);
}

void indicate_connected_via_ble(void)
{
    mobilinkd::tnc::indicator.start(mobilinkd::tnc::BluetoothConnectionIndication);
}

void tx_on(void)
{
    mobilinkd::tnc::indicator.set_red(1000);
}

void tx_off(void)
{
    mobilinkd::tnc::indicator.set_red(0);
}

// DCD is active.
void rx_on()
{
    mobilinkd::tnc::indicator.set_green(1000);
}

// DCD is active.
void rx_off()
{
    mobilinkd::tnc::indicator.set_green(0);
}

// Copyright 2017-2024 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include <stdint.h>

/**
 * @file Power Control
 *
 * Power state management.
 *
 * An key design aspect of the TNC is that when VUSB is present, VDD is
 * automatically switched on via an OR gate. This is necessary because
 * the STM32's VDDUSB must be powered for USB BCD negotiation to work.
 *
 * TNC has a number of operational states:
 *
 * - SHUTDOWN_BAT -- Shutdown, no USB (most trivial state)
 * - SHUTDOWN_USB -- Shutdown, USB charger
 * - STOP_NO_POWER -- Stopped, USB host did not negotiate 500mA
 * - STOP_CHARGING -- Stopped, USB host negotiated 500mA
 * - STOP_NO_POWER_SUSPEND -- Stopped, USB host did not negotiate
 *       500mA, suspended
 * - STOP_CHARGING_SUSPEND -- Stopped, USB host negotiated 500mA,
 *         suspended
 * - DISCONNECTED -- Powered on, no USB, not connected.
 * - DISCONNECTED_NO_POWER -- Powered on, USB host did not negotiate
 *         500mA, not connected.
 * - DISCONNECTED_CHARGING -- Powered on, USB host negotiated 500mA, not
 *         connected.
 * - DISCONNECTED_CHARGE_ADAPTER -- Powered on, USB charging adapter, not
 *         connected.
 * - BT_CONNECTED -- Powered on, no USB, BT connected.
 * - BT_CONNECTED_NO_POWER -- Powered on, BT connected, USB host did not
 *         negotiate 500mA.
 *
 * In SHUTDOWN_BAT mode, the VDD domain is powered down, and the TNC is
 * configured to wake from button and VDD_SENSE going high. If the
 * TNC is configured for WAKE_FROM_VUSB, the TNC will remain awake after
 * VDD_SENSE goes high. Otherwise it will negotiate charging then go back
 * to a low-power state.
 *
 * In SHUTDOWN_USB mode, VDD_EN is low, the VDD domain is powered on only
 * via VUSB, and the TNC is configured to wake from button, OVP, and VDD_SENSE
 * going low (USB disconnect). If
 */

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t low_battery;
extern volatile uint32_t usb_resume;

void update_power_monitor_timer(void);

typedef enum {WAKE_UP, SHUTDOWN} WakeType;

typedef enum {
    WAKE_FROM_RUNNING,      // Restart while not asleep
    WAKE_FROM_OVP,          // OVP event
    WAKE_FROM_BUTTON,       // Power button event
    WAKE_FROM_VUSB,         // VDD present
    WAKE_FROM_HW_RESET,     // Hardware reset
    WAKE_FROM_SW_RESET,     // Software reset
    WAKE_FROM_BOR,          // Brown-out reset
    WAKE_FROM_RTC,          // Real-time clock alarm
    WAKE_FROM_UNKNOWN       // Unknown wake-up event
} WakeFromType;

typedef enum PowerState {
    POWER_STATE_UNKNOWN,        // Unknown connection state
    POWER_STATE_VBAT,           // Running from VBAT only
    POWER_STATE_VBUS,           // VBUS detected but has not enumerated
    POWER_STATE_VBUS_HOST,      // VBUS detected upstream host
    POWER_STATE_VBUS_ENUM,      // VBUS detected enumerated by host
    POWER_STATE_VBUS_CHARGER    // VBUS detected battery charger
} PowerStateType;

extern PowerStateType powerState;

void enable_vdd(void);
void disable_vdd(void);
int is_battery_low(void);

/**
 * The TNC enters a low-power state, drawing less than 5uA when charging
 * is disabled. The TNC enters this mode only when USB not connected to
 * a USB host.
 *
 * The TNC is configured to wake up from these sources:
 *
 *  - VUSB change
 *  - Power button
 *  - RTC Alarm
 *  - VBAT level below 3.4V (ADC)
 *
 * STOP2 is used when either VUSB is not present or when VUSB is present
 * and the TNC is connected to a charger, rather than to a USB host.
 */
void stop2(uint32_t low_power_state) __attribute__ ((noreturn));
void _configure_power_on_disconnect(void);

#ifdef __cplusplus
}

#include <atomic>

namespace mobilinkd { namespace tnc {

extern uint16_t VREFINT_MIN;
extern uint16_t VREFINT_MAX;

/**
 * The type of power on or off process to follow.
 *
 * - POWERON occurs when the RTC backup domain has been erased due to
 *   complete power loss.  This should only happen if the battery is
 *   completely drained or removed.
 * - NORMAL occurs when the TNC is powered off and the TNC is not on USB
 *   power.  Note that this is the state when the TNC was powered off,
 *   not the state when it is powered on.
 * - VUSB occurs when the TNC is powered off and the TNC is connected to
 *   USB power.
 */
enum PowerType {UNKNOWN, POWERON, NORMAL, VUSB, SAFE};

HAL_StatusTypeDef init_power_monitor();
HAL_StatusTypeDef start_power_monitor();
HAL_StatusTypeDef stop_power_monitor();
uint32_t get_bat_level();
uint32_t read_battery_level();

/**
 * Enable the analog domain (ADCs, OPAMPS, DAC in normal mode) and the
 * I2C interface. These are only needed when a connection has been
 * established.
 *
 * @pre ADC2, OPAMP1, DAC_CHANNEL_1 and I2C are in a de-initialized state.
 * @post ADC2, OPAMP1, DAC_CHANNEL_1 and I2C are initialized.
 */
void configure_power_on_connect();

/**
 * Disable the analog domain (ADCs, OPAMPS, DAC in low-power mode) and
 * I2C. This is to save power.
 *
 * @pre ADC2, OPAMP1, DAC_CHANNEL_1 and I2C are in an initialized state.
 * @post ADC2, OPAMP1, DAC_CHANNEL_1 and I2C are de-initialized.
 */
void configure_power_on_disconnect();

void configure_device_for_stop2(int8_t usb_connected);
void configure_gpio_wake_from_stop2(int8_t);
bool should_wake_from_stop2(int8_t usb_connected);
void configure_device_for_wake_from_stop2(bool was_usb_connected, bool is_usb_connected) __attribute__ ((noreturn));
void power_down_vdd_for_stop(int8_t usb_connected);

void initialize_audio();
void enable_interrupts();
void _enable_vdd();
void _disable_vdd();

}} // mobilinkd::tnc

#endif //__cplusplus

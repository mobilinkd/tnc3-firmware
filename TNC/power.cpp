// Copyright 2024 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"

#include "AudioLevel.hpp"
#include "AudioInput.hpp"
#include "LEDIndicator.h"
#include "Log.h"
#include "main.h"
#include "power.h"
#include "usb_device.h"

#include <stm32l4xx_hal.h>

#include <cmsis_os.h>

#include <atomic>

extern osMessageQId ioEventQueueHandle;
extern RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef BATTERY_ADC_HANDLE;
extern ADC_HandleTypeDef DEMODULATOR_ADC_HANDLE;
extern CRC_HandleTypeDef hcrc;
extern DAC_HandleTypeDef hdac1;
extern I2C_HandleTypeDef hi2c1;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern RNG_HandleTypeDef hrng;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart3;
extern PCD_HandleTypeDef hpcd_USB_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern IWDG_HandleTypeDef hiwdg;

PowerState powerState = POWER_STATE_UNKNOWN;

extern "C" void enable_vdd()
{
    mobilinkd::tnc::_enable_vdd();
}
extern "C" void disable_vdd()
{
    mobilinkd::tnc::_disable_vdd();
}

extern "C" void _configure_power_on_disconnect()
{
    mobilinkd::tnc::configure_power_on_disconnect();
}

extern "C" void stop2(uint32_t low_power_state)
{
    using namespace mobilinkd::tnc;

    vTaskSuspendAll();

    INFO("stop2");

    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, low_power_state | TNC_LOWPOWER_STOP2);
    HAL_PWR_DisableBkUpAccess();

    go_back_to_sleep = 0;
    GPIO_PinState usb_connected = GPIO_PIN_RESET;

    do {
        usb_connected = HAL_GPIO_ReadPin(USB_POWER_GPIO_Port, USB_POWER_Pin);
        if (!usb_connected) charging_enabled = 0;

        configure_device_for_stop2(usb_connected);

        __asm volatile ( "cpsid i" );
        __asm volatile ( "dsb" );
        __asm volatile ( "isb" );

        power_down_vdd_for_stop(usb_connected);
        configure_gpio_wake_from_stop2(usb_connected);
        HAL_PWREx_DisableLowPowerRunMode();    // Required to enter STOP2

        /* Set Stop mode 2 */
        MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP2);

        /* Set SLEEPDEEP bit of Cortex System Control Register */
        SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

        /* Request Wait For Event */
        __SEV();
        __WFE();
        __WFE();

        __asm volatile ( "nop" );
        __asm volatile ( "nop" );

        /* Reset SLEEPDEEP bit of Cortex System Control Register */
        CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

        __asm volatile ( "nop" );
        __asm volatile ( "nop" );
    } while (!should_wake_from_stop2(usb_connected));

    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, 0);
    HAL_PWR_DisableBkUpAccess();

    GPIO_PinState is_usb_connected = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
    configure_device_for_wake_from_stop2(usb_connected, is_usb_connected);
    // No return -- reset.
}

extern "C" void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    uint16_t vrefint_raw = BATTERY_ADC_HANDLE.Instance->JDR1;
    osMessagePut(ioEventQueueHandle, CMD_VREFINT_WATCHDOG | vrefint_raw, 0);
}

extern "C" void update_power_monitor_timer()
{
    uint32_t pclkFreq = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = (pclkFreq / 250'000U) - 1U;

    __HAL_TIM_SET_PRESCALER(&htim6, prescaler);
}

extern "C" int is_battery_low()
{
    // No need to check for low battery on startup is VBUS is present.
    if (HAL_GPIO_ReadPin(USB_POWER_GPIO_Port, USB_POWER_Pin) == GPIO_PIN_SET) {
        return 0;
    }

    uint32_t vbat = mobilinkd::tnc::get_bat_level();

    return vbat < 3400;
}

namespace mobilinkd { namespace tnc {


uint16_t VREFINT_MIN;
uint16_t VREFINT_MAX;


/**
 * This monitors VDDA by monitoring the ADC's reported value for VREFINT. As
 * VDDA increases, the reported VREFINT value decreases. As VDDA decreases,
 * the reported VREFINT value increases.
 * 
 * VDDA will increase when the audio input exceeds 3.6V. This happens because
 * the protection diodes on the audio input path dump the excess voltage to VDDA.
 * 
 * VDDA will decrease when the battery runs low. It will drop below 3.3V when
 * the battery drops below about 3.35V. There is a drop-out voltage of about
 * 20-50mV. The VReg has nominal 2.5% accuracy (3.218 - 3.383V). VREFINT has
 * a calibrated output within 1% over the nominal temperature range.
 * 
 * The ADC watchdog is configured to trigger outside the 3.18-3.42V range (about
 * 3.5%) and report it to the event loop.
 * 
 * At start-up, the stored VREFINT calibration is used to determine the
 * nominal min and max values for the ADC windowed watchdog. These values
 * are stored in VREFINT_MIN and VREFINT_MAX.
 * 
 * An interrupt is raised by the ADC watchdog when VREFINT exceeds this range,
 * indicating a problem with VDDA.
 */
HAL_StatusTypeDef init_power_monitor()
{
    ADC_ChannelConfTypeDef sConfig = {};
    ADC_InjectionConfTypeDef iConfig = {};
    ADC_AnalogWDGConfTypeDef wConfig = {};
    HAL_StatusTypeDef status;

    INFO("init power monitor");

    // ADC must be READY (initialized and not active).
    if ((BATTERY_ADC_HANDLE.State & HAL_ADC_STATE_READY) == 0) {
        ERROR("ADC not READY; State = %ld", BATTERY_ADC_HANDLE.State);
        CxxErrorHandler();
    }

    static constexpr int VDDA_MAX_MV = 3420;
    static constexpr int VDDA_MIN_MV = 3180;

    uint32_t vrefcal = ((uint16_t)*(VREFINT_CAL_ADDR)); // VREFINT at 3.0V, 30C
    uint32_t VREFINT_NOMINAL = (vrefcal * 30) / 33;     // VREFINT will measure lower at higher VDD.
    UNUSED(VREFINT_NOMINAL);

    VREFINT_MIN = (vrefcal * 3000) / VDDA_MAX_MV;     // VREFINT will measure lower at higher VDD.
    VREFINT_MAX = (vrefcal * 3000) / VDDA_MIN_MV;     // VREFINT will measure higher at lower VDD.

    INFO("VREFINT_NOM = %lu", VREFINT_NOMINAL);
    INFO("VREFINT_MIN = %hu (%dmV)", VREFINT_MIN, VDDA_MAX_MV);
    INFO("VREFINT_MAX = %hu (%dmV)", VREFINT_MAX, VDDA_MIN_MV);

    sConfig.Channel = DEMODULATOR_ADC_CHANNEL;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    status = HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig);
    if (status != HAL_OK) CxxErrorHandler2(status);

    iConfig.InjectedChannel = ADC_CHANNEL_VREFINT;
    iConfig.InjectedRank = ADC_INJECTED_RANK_1;
    iConfig.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    iConfig.InjectedSingleDiff = ADC_SINGLE_ENDED;
    iConfig.InjectedOffsetNumber = ADC_OFFSET_NONE;
    iConfig.InjectedOffset = 0;
    iConfig.InjectedNbrOfConversion = 1;
    iConfig.InjectedDiscontinuousConvMode = DISABLE;
    iConfig.AutoInjectedConv = ENABLE;
    iConfig.QueueInjectedContext = DISABLE;
    iConfig.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    iConfig.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
    iConfig.InjecOversamplingMode = DISABLE;
    status = HAL_ADCEx_InjectedConfigChannel(&BATTERY_ADC_HANDLE, &iConfig);
    if (status != HAL_OK) CxxErrorHandler2(status);

    wConfig.Channel = ADC_CHANNEL_VREFINT;
    wConfig.HighThreshold = VREFINT_MAX;
    wConfig.LowThreshold = VREFINT_MIN;
    wConfig.ITMode = ENABLE;
    wConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_INJEC;
    wConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
    status = HAL_ADC_AnalogWDGConfig(&BATTERY_ADC_HANDLE, &wConfig);
    if (status != HAL_OK) CxxErrorHandler2(status);

    return HAL_OK;
}

HAL_StatusTypeDef start_power_monitor()
{
    HAL_StatusTypeDef status;

    INFO("start power monitor");

    uint32_t prescaler = (HAL_RCC_GetPCLK1Freq() / 250'000U) - 1U;
    __HAL_TIM_SET_PRESCALER(&htim6, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim6, 2499); // 100sps

    status = HAL_ADC_Start(&BATTERY_ADC_HANDLE);
    if (status != HAL_OK) CxxErrorHandler2(status);

    status = HAL_TIM_Base_Start(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);

    return HAL_OK;
}

HAL_StatusTypeDef stop_power_monitor()
{
    HAL_StatusTypeDef status;

    INFO("stop power monitor");

    status = HAL_TIM_Base_Stop(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);

    status = HAL_ADC_Stop(&BATTERY_ADC_HANDLE);
    if (status != HAL_OK) CxxErrorHandler2(status);

    return HAL_OK;
}

uint32_t get_bat_level()
{
    // ADC must be READY (initialized and not active).
    if ((BATTERY_ADC_HANDLE.State & HAL_ADC_STATE_READY) == 0) {
        ERROR("ADC not READY; State = %ld", BATTERY_ADC_HANDLE.State);
        CxxErrorHandler();
    }

    // Can only be called during initialization with HSI Sysclock.
    if (HAL_RCC_GetPCLK1Freq() != 16'000'000) CxxErrorHandler();

    // Always oversampled.
    static constexpr uint32_t VMAX = 16383;

    ADC_ChannelConfTypeDef sConfig = {};
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(BAT_DIVIDER_GPIO_Port, BAT_DIVIDER_Pin, GPIO_PIN_RESET);
    DELAY(1); // Stabilize power.

    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    status = HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig);
    if (status != HAL_OK) {
        CxxErrorHandler2(status);
    }

    // The voltage divider for the battery monitor has an output impedeance
    // of 4k7 ohms. Per the datasheet, for an R[ain] of 3.9-10k on a slow
    // analog channel, a sampling time of 247.5 clks is needed.
    //
    // 247.5 sample time = 260 clks/sample, 16x oversampling = 4160 clks. At
    // 16MHz, that's ~3800 samples/sec For 8 samples, that will take ~2ms.
    uint32_t prescaler = (HAL_RCC_GetPCLK1Freq() / 250'000U) - 1U;
    __HAL_TIM_SET_PRESCALER(&htim6, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim6, 74); // 3333sps

    status = HAL_ADC_Start(&BATTERY_ADC_HANDLE);
    if (status != HAL_OK) CxxErrorHandler2(status);
    status = HAL_TIM_Base_Start(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);
    status = HAL_ADC_PollForConversion(&BATTERY_ADC_HANDLE, 5);
    if (status != HAL_OK) CxxErrorHandler2(status);

    uint32_t vrefint = HAL_ADC_GetValue(&BATTERY_ADC_HANDLE);
    vrefint >>= 2; // Oversampled.

    status = HAL_TIM_Base_Stop(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);
    status = HAL_ADC_Stop(&BATTERY_ADC_HANDLE);
    if (status != HAL_OK) CxxErrorHandler2(status);

    sConfig.Channel = BATTERY_ADC_CHANNEL;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    status = HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig);
    if (status != HAL_OK) {
        CxxErrorHandler2(status);
    }

    status = HAL_ADC_Start(&BATTERY_ADC_HANDLE);
    if (status != HAL_OK) CxxErrorHandler2(status);
    status = HAL_TIM_Base_Start(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);

    uint32_t bat = 0;
    for (size_t i = 8; i != 0; --i)
    {
        status = HAL_ADC_PollForConversion(&BATTERY_ADC_HANDLE, 5);
        if (status != HAL_OK) CxxErrorHandler2(status);
        bat += HAL_ADC_GetValue(&BATTERY_ADC_HANDLE);
    }

    status = HAL_TIM_Base_Stop(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);
    status = HAL_ADC_Stop(&BATTERY_ADC_HANDLE);
    if (status != HAL_OK) CxxErrorHandler2(status);

    bat >>= 3;

    HAL_GPIO_WritePin(BAT_DIVIDER_GPIO_Port, BAT_DIVIDER_Pin, GPIO_PIN_SET);

    sConfig.Channel = AUDIO_IN;
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    status = HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig);
    if (status != HAL_OK) {
        CxxErrorHandler2(status);
    }

    uint32_t vrefcal = ((uint16_t)*(VREFINT_CAL_ADDR));
    uint32_t VREFINT_NOMINAL = 30 * vrefcal / 33;
    uint32_t vdda = 3300 * VREFINT_NOMINAL / vrefint;

    INFO("Vrefint = %lu", vrefint);
    INFO("Vrefcal = %lu", VREFINT_NOMINAL);
    INFO("Vbat = %lu (raw)", bat);

    // Order of operations is important to avoid underflow.
    uint32_t vbat = (bat * 2 * vdda) / VMAX;
    uint32_t vref = vdda * vrefint / VMAX;
    UNUSED(vref);

    INFO("Vref = %lumV", vref);
    INFO("Vdda = %lumV", vdda);
    INFO("Vbat = %lumV", vbat);

    return vbat;
}

uint32_t read_battery_level()
{
    ADC_ChannelConfTypeDef sConfig = {};
    HAL_StatusTypeDef status;

    const uint32_t VMAX = BATTERY_ADC_HANDLE.Init.OversamplingMode == ENABLE ? 16383 : 4095;

    status = HAL_ADCEx_InjectedPollForConversion(&BATTERY_ADC_HANDLE, 5);
    if (status != HAL_OK) CxxErrorHandler2(status);
    uint32_t vrefint = BATTERY_ADC_HANDLE.Instance->JDR1;

    status = stop_power_monitor();
    if (status != HAL_OK) CxxErrorHandler2(status);

    HAL_GPIO_WritePin(BAT_DIVIDER_GPIO_Port, BAT_DIVIDER_Pin, GPIO_PIN_RESET);

    // The voltage divider for the battery monitor has an output impedeance
    // of 4k7 ohms. Per the datasheet, for an R[ain] of 3.9-10k on a slow
    // analog channel, a sampling time of 247.5 clks is needed. (We can get
    // by with 92.5 clks, with a slightly less accurate input.)
    //
    // 247.5 sample time = 260 clks/sample, 16x oversampling = 4160 clk + 37
    // clks needed for injected channel. At 2MHz, that's ~400 samples/sec.
    // For 8 samples, that will take 20ms.
    sConfig.Channel = BATTERY_ADC_CHANNEL;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    status = HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig);
    if (status != HAL_OK) {
        CxxErrorHandler2(status);
    }

    uint32_t prescaler = (HAL_RCC_GetPCLK1Freq() / 250'000U) - 1U;
    __HAL_TIM_SET_PRESCALER(&htim6, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim6, 624); // 400sps

    status = HAL_ADC_Start(&BATTERY_ADC_HANDLE);
    if (status != HAL_OK) CxxErrorHandler2(status);
    status = HAL_TIM_Base_Start(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);

    uint32_t bat = 0;
    uint32_t tickstart = HAL_GetTick();
    for (size_t i = 8; i != 0; --i)
    {
        while ((BATTERY_ADC_HANDLE.Instance->ISR & ADC_FLAG_EOC) == 0) {
            if (HAL_GetTick() - tickstart > 40) {
                CxxErrorHandler2(HAL_StatusTypeDef(i));
            }
        }
        bat += BATTERY_ADC_HANDLE.Instance->DR;
    }

    bat >>= 3;

    HAL_GPIO_WritePin(BAT_DIVIDER_GPIO_Port, BAT_DIVIDER_Pin, GPIO_PIN_SET);

    status = HAL_TIM_Base_Stop(&htim6);
    if (status != HAL_OK) CxxErrorHandler2(status);

    status = HAL_ADC_Stop(&BATTERY_ADC_HANDLE);
    if (status != HAL_OK) CxxErrorHandler2(status);

    sConfig.Channel = AUDIO_IN;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    status = HAL_ADC_ConfigChannel(&BATTERY_ADC_HANDLE, &sConfig);
    if (status != HAL_OK) {
        CxxErrorHandler2(status);
    }

    status = start_power_monitor();
    if (status != HAL_OK) CxxErrorHandler2(status);

    uint32_t vrefcal = ((uint16_t)*(VREFINT_CAL_ADDR));
    uint32_t VREFINT_NOMINAL = 30 * vrefcal / 33;
    uint32_t vdda = 3300 * VREFINT_NOMINAL / vrefint;

    INFO("Vrefint = %lu", vrefint);
    INFO("Vrefcal = %lu", VREFINT_NOMINAL);
    INFO("Vbat = %lu (raw)", bat);

    // Order of operations is important to avoid underflow.
    uint32_t vbat = (bat * 2 * vdda) / VMAX;
    uint32_t vref = vdda * vrefint / VMAX;
    UNUSED(vref);

    INFO("Vref = %lumV", vref);
    INFO("Vdda = %lumV", vdda);
    INFO("Vbat = %lumV", vbat);

    return vbat;
}

bool should_wake_from_stop2(int8_t usb_connected)
{
    bool result = false;

    uint32_t shutdown_reg = READ_REG(BKUP_TNC_LOWPOWER_STATE);
    uint32_t power_config_reg = READ_REG(BKUP_POWER_CONFIG);

    __asm volatile ( "cpsie i" );    // Enable interrupts.

    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_IWDG_Refresh(&hiwdg); // Refresh IWDG while checking wake-up event.

    HAL_Init();
    SystemClock_Config();
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    TPI->ACPR = 7;    // 16MHz SysClock -> 2MHz SWO.

    auto status = HAL_RTC_WaitForSynchro(&hrtc);
    if (status != HAL_OK) {
        ERROR("HAL_RTC_WaitForSynchro() = %d", status);
    }

    MX_TIM1_Init();    // Needed to re-init PWM timer properly.
    __HAL_TIM_SET_PRESCALER(&LED_PWM_TIMER_HANDLE, 15);

    __HAL_RCC_GPIOH_CLK_ENABLE();

    INFO("INPUT = 0x%04lx", GPIOH->IDR);

    if ((shutdown_reg & TNC_LOWPOWER_LOW_BAT) && !(USB_POWER_GPIO_Port->IDR & USB_POWER_Pin)) {
        // Low battery and no VUSB power; return immediately to stop mode.
        SysClock2();
        indicate_battery_low();
        while (SW_POWER_GPIO_Port->IDR & SW_POWER_Pin) {
            HAL_IWDG_Refresh(&hiwdg); // Refresh IWDG while button pressed.
        }
        return result;
    }

    uint32_t start = HAL_GetTick();

    if ((USB_POWER_GPIO_Port->IDR & USB_POWER_Pin) && !usb_connected) {
        // VUSB connect
        while (USB_POWER_GPIO_Port->IDR & USB_POWER_Pin) {
            if (HAL_GetTick() - start > 2000) {
                INFO("USB Connected");
                result = true;
                if (power_config_reg & POWER_CONFIG_WAKE_FROM_USB) {
                    go_back_to_sleep = 0;
                } else if (shutdown_reg & TNC_LOWPOWER_LOW_BAT) {
                    go_back_to_sleep = 0;
                } else {
                    go_back_to_sleep = 1;
                }
                break;
            }
        }
    } else if (!(USB_POWER_GPIO_Port->IDR & USB_POWER_Pin) && usb_connected) {    // VUSB disconnect
        SysClock2();
        while (!(USB_POWER_GPIO_Port->IDR & USB_POWER_Pin)) {
            if (HAL_GetTick() - start > 2000) {
                // Battery charging off.
                HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_SET);
                HAL_PWR_EnableBkUpAccess();
                WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_VBAT | TNC_LOWPOWER_STOP2);
                HAL_PWR_DisableBkUpAccess();

                INFO("USB Disconnected");
                break;
            }
        }
    } else if (SW_POWER_GPIO_Port->IDR & SW_POWER_Pin) {        // SW_POWER press
        SysClock2();
        INFO("power button");
        while (SW_POWER_GPIO_Port->IDR & SW_POWER_Pin) {
            if (HAL_GetTick() - start > 2000) {
                INFO("wake up");
                result = true;
                break;
            }
        }
    }

    return result;
}

/*
 * Put device in stop mode. The configuration will differ depending
 * on whether VUSB is present. If it is present, then VDD will be
 * present and the BT/BLE module must be held in reset  to lower
 * current consumption. Otherwise, these pins must be left floating
 * to minimize current consumption.
 *
 * VDD_EN must be pulled low.
 *
 * USB_CE must not be changed.
 *
 * GPIO configuration is retained in stop mode.
 * 
 * LSE is running.
 * 
 * SWD pins are active in DEBUG builds.
 */
void configure_device_for_stop2(int8_t usb_connected)
{
    go_back_to_sleep = 0;

    reset_indicator();

    HAL_OPAMP_DeInit(&hopamp1);
    HAL_TIM_PWM_DeInit(&htim1);
    HAL_TIM_Base_DeInit(&htim6);
    HAL_TIM_Base_DeInit(&htim7);
    HAL_I2C_DeInit(&hi2c1);
    HAL_ADC_DeInit(&DEMODULATOR_ADC_HANDLE);
    HAL_RNG_DeInit(&hrng);
    HAL_CRC_DeInit(&hcrc);
    HAL_DAC_DeInit(&hdac1);
    HAL_PWR_DisablePVD();
    HAL_UART_DeInit(&huart3);
    HAL_PCD_MspDeInit(&hpcd_USB_FS);
    HAL_ADCEx_EnterADCDeepPowerDownMode(&DEMODULATOR_ADC_HANDLE);

    // Ensure GPIO clocks are enabled.
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

#if defined(DEBUG)
    HAL_DBGMCU_EnableDBGStopMode();
    // SWD = PA13, PA14, PB3
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All^(GPIO_PIN_13|GPIO_PIN_14));                 // Except SWDIO & SWCLK.
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All^(GPIO_PIN_3|VDD_EN_Pin|USB_CE_Pin));        // Except SWO, VDD_EN & USB_CE.
#else
    HAL_DBGMCU_DisableDBGStopMode();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All^(VDD_EN_Pin|USB_CE_Pin));
#endif
    // LSE = PC14, PC15
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_All^(GPIO_PIN_14|GPIO_PIN_15));
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_All);

    SysClock2();

    if (usb_connected) {
        // With USB power BT must be kept off using BT_SLEEP.
        HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);
    } else {
        // Without USB power BT will be off.
        HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_SET);
        HAL_GPIO_DeInit(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin);
    }
}

void configure_device_for_wake_from_stop2(bool was_usb_connected, bool is_usb_connected)
{
    uint32_t reg = is_usb_connected ? TNC_LOWPOWER_VUSB : TNC_LOWPOWER_VBAT;
    if (go_back_to_sleep) {
        HAL_PWR_EnableBkUpAccess();
        WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_STOP2| reg | TNC_LOWPOWER_RECONFIG);
        HAL_PWR_DisableBkUpAccess();
    }

    HAL_NVIC_SystemReset();
    abort();
}

void configure_gpio_wake_from_stop2(int8_t usb_connected)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;

    // Reset wakeup pins
    HAL_NVIC_DisableIRQ(USB_POWER_EXTI_IRQn);
    HAL_NVIC_DisableIRQ(SW_POWER_EXTI_IRQn);
    HAL_GPIO_DeInit(USB_POWER_GPIO_Port, USB_POWER_Pin);
    HAL_GPIO_DeInit(SW_POWER_GPIO_Port, SW_POWER_Pin);

    // Wake up whenever there is a change in VUSB to handle connect/disconnect events.
    GPIO_InitStruct.Pin = USB_POWER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_POWER_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SW_POWER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW_POWER_GPIO_Port, &GPIO_InitStruct);

    HAL_PWREx_EnableInternalWakeUpLine();

    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
    // GPIOH is used for USB_POWER and SW_POWER
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
}

void power_down_vdd_for_stop(int8_t usb_connected)
{
    if (usb_connected) {
        HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_RESET);
        // Wait for VDD_SENSE to register VDD off.
        while (USB_POWER_GPIO_Port->IDR & USB_POWER_Pin) asm volatile("nop");
    }
}

void initialize_audio()
{
    audio::init_log_volume();
    audio::setAudioOutputLevel();
    audio::setAudioInputLevels();
}

void enable_interrupts()
{
    HAL_NVIC_ClearPendingIRQ(SW_POWER_EXTI_IRQn);
    HAL_NVIC_SetPriority(SW_POWER_EXTI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SW_POWER_EXTI_IRQn);

    // VUSB Sense
    HAL_NVIC_ClearPendingIRQ(USB_POWER_EXTI_IRQn);
    HAL_NVIC_SetPriority(USB_POWER_EXTI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_POWER_EXTI_IRQn);

    HAL_NVIC_ClearPendingIRQ(SW_BOOT_EXTI_IRQn);
    HAL_NVIC_SetPriority(SW_BOOT_EXTI_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(SW_BOOT_EXTI_IRQn);

    HAL_NVIC_ClearPendingIRQ(BT_STATE1_EXTI_IRQn);
    HAL_NVIC_SetPriority(BT_STATE1_EXTI_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(BT_STATE1_EXTI_IRQn);

    HAL_NVIC_ClearPendingIRQ(BT_STATE2_EXTI_IRQn);
    HAL_NVIC_SetPriority(BT_STATE2_EXTI_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(BT_STATE2_EXTI_IRQn);

    HAL_NVIC_ClearPendingIRQ(ADC1_IRQn);
    HAL_NVIC_SetPriority(ADC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);
}

void configure_power_on_connect()
{
    __HAL_RCC_TIM7_CLK_ENABLE();
    __HAL_RCC_CRC_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_RNG_CLK_ENABLE();
    __HAL_RCC_OPAMP_CLK_ENABLE();
    __HAL_RCC_DAC1_CLK_ENABLE();
}

/*
 * It is important to note here that the UART cannot be disabled when the
 * Bluetooth module is not connected. It must be properly configured for
 * the Bluetooth module connection management to work properly. If it is
 * disabled when a Bluetooth connection is established, the BT module will
 * not behave properly.
 */
void configure_power_on_disconnect()
{
    __HAL_RCC_TIM7_CLK_DISABLE();
    __HAL_RCC_CRC_CLK_DISABLE();
    __HAL_RCC_I2C1_CLK_DISABLE();
    __HAL_RCC_RNG_CLK_DISABLE();
    __HAL_RCC_OPAMP_CLK_DISABLE();
    __HAL_RCC_DAC1_CLK_DISABLE();
}

static int vdd_counter = 0;

void _enable_vdd()
{
    auto x = taskENTER_CRITICAL_FROM_ISR();

    if (!vdd_counter) {
        HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_SET);
    }
    vdd_counter++;

    taskEXIT_CRITICAL_FROM_ISR(x);
}

void _disable_vdd()
{
    auto x = taskENTER_CRITICAL_FROM_ISR();

    vdd_counter--;
    if (!vdd_counter) {
        HAL_GPIO_WritePin(VDD_EN_GPIO_Port, VDD_EN_Pin, GPIO_PIN_RESET);
    }

    taskEXIT_CRITICAL_FROM_ISR(x);
}

}} // mobilinkd::tnc

#pragma GCC diagnostic pop

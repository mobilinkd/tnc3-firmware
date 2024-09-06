// Copyright 2018-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "AudioLevel.hpp"
#include "Log.h"
#include "IOEventTask.h"
#include "PortInterface.h"
#include "PortInterface.hpp"
#include "main.h"
#include "AudioInput.hpp"
#include "HdlcFrame.hpp"
#include "Kiss.hpp"
#include "KissHardware.hpp"
#include "ModulatorTask.hpp"
#include "Modulator.hpp"
#include "UsbPort.hpp"
#include "SerialPort.hpp"
#include "NullPort.hpp"
#include "LEDIndicator.h"
#include "bm78.h"
#include "KissHardware.h"
#include "power.h"

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "cmsis_os.h"

extern osMessageQId hdlcOutputQueueHandle;
extern osThreadId modulatorTaskHandle;
extern osThreadId audioInputTaskHandle;

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

#ifdef STM32L4P5xx
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
#define HPCD hpcd_USB_OTG_FS
#else
extern PCD_HandleTypeDef hpcd_USB_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef FS_Desc;
#define HPCD hpcd_USB_FS
#endif

extern osTimerId usbShutdownTimerHandle;
extern osTimerId powerOffTimerHandle;
extern IWDG_HandleTypeDef hiwdg;

volatile ConnectionState connectionState = ConnectionState::DISCONNECTED;

/**
 * Update the SysClock depending on power and connection state.
 *
 * The SysClock must be >= 16MHz when USB is connected. This is an STM32 USB
 * hardware requirement. Some PCD functions will silently fail if the clock
 * is too slow. The main result is increased current consumption. this is a
 * problem during shutdown, where current can be 1mA rather than 5uA. To
 * simplify things, the SysClock48 is used if the clock is less than 48MHz.
 *
 * The SysClock will be set to the appropriate frequency for the demodulator
 * when the connection is established. This is always >= 48MHz. (Today it is
 * only 48MHz.)
 */
void updateSysClock()
{
    vTaskSuspendAll();

    if ((PowerState::POWER_STATE_VBAT == powerState)
        && (ConnectionState::DISCONNECTED == connectionState))
    {
        mobilinkd::tnc::configure_power_on_disconnect();
        SysClock2();
    } else {
        if (SystemCoreClock < 48000000) SysClock48();
    }

    xTaskResumeAll();
}

static PTT getPttStyle(const mobilinkd::tnc::kiss::Hardware& hardware)
{
    return hardware.options & KISS_OPTION_PTT_SIMPLEX ? PTT::SIMPLEX : PTT::MULTIPLEX;
}

static void stop() __attribute__ ((noreturn));

static void stop()
{
    INFO("STOP mode");

    bool battery_low = !!(READ_REG(BKUP_TNC_LOWPOWER_STATE) & TNC_LOWPOWER_LOW_BAT);

    HAL_NVIC_DisableIRQ(SW_POWER_EXTI_IRQn); // Disable SW_BUTTON and VUSB_SENSE.

    // The USB PCD is stopped to disconnect the 1.5k data line pull-up.
    // This is necessary to detect VUSB changes while asleep.
    if (POWER_STATE_VBUS_ENUM == powerState || POWER_STATE_VBUS_HOST == powerState) {
        HAL_PCD_Stop(&HPCD); // Disconnect; remove 1.5k pullup.
    }

    // Disable Bluetooth Module
    HAL_NVIC_DisableIRQ(BT_STATE1_EXTI_IRQn);
    HAL_NVIC_DisableIRQ(BT_STATE2_EXTI_IRQn);
    HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);
    mobilinkd::tnc::configure_power_on_disconnect();

    osDelay(100); // Allow VUSB_SENSE time to de-energize from 1.5k pullup leakage.
    if (!(USB_POWER_GPIO_Port->IDR & USB_POWER_Pin)) powerState = POWER_STATE_VBAT;

    HAL_PWR_EnableBkUpAccess();
    WRITE_REG(BKUP_POWER_CONFIG, (powerOnViaUSB() ? POWER_CONFIG_WAKE_FROM_USB : 0) | (powerOffViaUSB() ? POWER_CONFIG_SLEEP_ON_USB : 0));
    HAL_PWR_DisableBkUpAccess();

    ::stop2((powerState == POWER_STATE_VBAT ? TNC_LOWPOWER_VBAT : TNC_LOWPOWER_VUSB) |
            (battery_low ? TNC_LOWPOWER_LOW_BAT : 0));
    // No return...
}

static void onShutdownCommand(bool low_battery = false) __attribute__ ((noreturn));

static void onShutdownCommand(bool low_battery)
{
    // Need to reset the TNC if connected to shut down cleanly.
    if (connectionState != ConnectionState::DISCONNECTED) {
        HAL_PWR_EnableBkUpAccess();
        WRITE_REG(BKUP_TNC_LOWPOWER_STATE,
                (low_battery ? TNC_LOWPOWER_LOW_BAT : 0) |
                (powerState == POWER_STATE_VBAT ? TNC_LOWPOWER_VBAT : TNC_LOWPOWER_VUSB) |
                TNC_LOWPOWER_STOP2 | TNC_LOWPOWER_RECONFIG);
        HAL_PWR_DisableBkUpAccess();
        HAL_NVIC_SystemReset();
    }

    stop();
}

static void usb_device_init()
{
  // Set the MaxPower value to 500mA -- the amount of power needed to
  // charge the battery.
  uint16_t length;
  uint8_t* cfg = USBD_CDC.GetFSConfigDescriptor(&length);
  cfg[8] = 0xFA;

  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
}

/**
 * Connected to a normal USB host. Expect to enumerate after BCD discovery
 * is completed.
 * 
 * In an ideal world, charging would be delayed until the host enumerates
 * and accepts our current consumption needs. However, there are enough broken
 * USB charging devices in the world that are detected as a USB host even
 * though they have no way to enumerate.
 */
static void onUsbHostConnected(bool charging_port)
{
    UNUSED(charging_port);
    if (charging_port) {
        INFO("USB charging host port connected");
    } else {
        INFO("USB standard host port connected");
    }
    powerState = POWER_STATE_VBUS_HOST;
    HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_RESET);
    charging_enabled = 1;
}

static void onUsbHostEnumerated()
{
    INFO("USB host enumerated");
    if (ConnectionState::BT_CONNECTED == connectionState) {
        // Prevent USB serial connections from being established.
        HAL_PCD_Stop(&HPCD);
    } else {
        powerState = POWER_STATE_VBUS_ENUM;
    }
}

void onUsbChargerConnected()
{
    INFO("USB charger connected");
    powerState = POWER_STATE_VBUS_CHARGER;
    HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_RESET);
    charging_enabled = 1;
}

void onUsbDiscoveryComplete()
{
    INFO("USB discovery complete");
    if ((powerState != POWER_STATE_VBUS) && go_back_to_sleep) {
        osTimerStop(usbShutdownTimerHandle);
        osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 1);
    } else if (POWER_STATE_VBUS_HOST == powerState) {
        // Enumerate.
        HAL_PCD_Start(&HPCD);
    }
}

void onUsbDiscoveryError()
{
    // This happens when powering VUSB from a bench supply.
    osTimerStop(usbShutdownTimerHandle);
    HAL_PCDEx_DeActivateBCD(&HPCD); // May be skipped on BCD discovery error.
    if (HAL_GPIO_ReadPin(USB_POWER_GPIO_Port, USB_POWER_Pin) == GPIO_PIN_SET)
    {
        INFO("Not a recognized USB charging device");
        INFO("USB charging enabled");
        powerState = POWER_STATE_VBUS_CHARGER;
        HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_RESET);
        charging_enabled = 1;
    }
    if (go_back_to_sleep) {
        osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 1);
    }
}

/**
  * @brief  Send BCD message to user layer
  * @param  hpcd: PCD handle
  * @param  msg: LPM message
  * @retval None
  */
extern "C" void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg)
{
    UNUSED(hpcd);

    HAL_IWDG_Refresh(&hiwdg);

    switch(msg)
    {
    case PCD_BCD_CONTACT_DETECTION:
        break;
    case PCD_BCD_STD_DOWNSTREAM_PORT:
        // Only charge after negotiation
        onUsbHostConnected(false);
        break;
    case PCD_BCD_CHARGING_DOWNSTREAM_PORT:
        onUsbHostConnected(true);
        break;
    case PCD_BCD_DEDICATED_CHARGING_PORT:
        onUsbChargerConnected();
        break;
    case PCD_BCD_DISCOVERY_COMPLETED:
        onUsbDiscoveryComplete();
        break;
    case PCD_BCD_ERROR:
        onUsbDiscoveryError();
        break;
    default:
        break;
    }
}

/**
 * VUSB has been detected. Update the power state and increase system clock
 * if needed. Initialize the PCD and start battery charge detection.
 * 
 * @pre VUSB is present.
 * @pre PCD is stopped.
 * @post Battery charger detection is complete.
 * @post If standard downstream port is detected and not shutting down, the
 *  USB peripheral controller has been started, and USB enumeration is
 *  forthcoming.
 */
static void setUsbConnected()
{
    HAL_StatusTypeDef status;
    powerState = POWER_STATE_VBUS;
    updateSysClock();
    HAL_PCD_MspInit(&hpcd_USB_FS);
    status = HAL_PCDEx_ActivateBCD(&HPCD); // Must call before calling HAL_PCDEx_BCD_VBUSDetect.
    if (status != HAL_OK) CxxErrorHandler2(status);
    HAL_PCDEx_BCD_VBUSDetect(&HPCD);
}

static void indicate_connection_state(ConnectionState connectionState)
{
    switch (connectionState) {
    case ConnectionState::DISCONNECTED:
        indicate_waiting_to_connect();
        break;
    case ConnectionState::BT_CONNECTED:
        indicate_connected_via_ble();
        break;
    case ConnectionState::USB_CONNECTED:
        indicate_connected_via_usb();
        break;
    }
}

void startIOEventTask(void const*)
{
    using namespace mobilinkd::tnc;

    HAL_IWDG_Refresh(&hiwdg);

    init_ioport();
    initCDC();
    initSerial();

    INFO("USB_POWER_GPIO_Port = 0x%04lx", USB_POWER_GPIO_Port->IDR);
    powerState = (USB_POWER_GPIO_Port->IDR & USB_POWER_Pin) ? PowerState::POWER_STATE_VBUS : PowerState::POWER_STATE_VBAT;
    if (!go_back_to_sleep) {
        print_startup_banner();
    }

    auto& hardware = kiss::settings();

    if (reset_requested or !hardware.load() or !hardware.crc_ok())
    {
        if (reset_requested) {
            INFO("Hardware reset requested.");
        }

        hardware.init();
        hardware.store();
    }

    // This must be called to detect USB charging ports.
    // Sysclock must be >= 16MHz for USB. On HSI here.
    usb_device_init();
    HAL_PCD_MspDeInit(&hpcd_USB_FS);

    if (!go_back_to_sleep) {
        // Normal startup.
        hardware.debug();

        // On TNC3, power monitor must be started before initializing audio
        // because ADC is used.
        init_power_monitor();
        start_power_monitor();

        initialize_audio();
        setPtt(getPttStyle(hardware));
        indicate_waiting_to_connect();

        // Wait to start threads until after potential clock change for USB.
        // The modulator task cannot start until after EEPROM settings are
        // loaded or initialized.
        osThreadResume(modulatorTaskHandle);
        osThreadResume(audioInputTaskHandle);

        // Update system clock after modulator starts.
        osThreadYield();
        if (powerState == PowerState::POWER_STATE_VBUS) {
            setUsbConnected();
        } else {
            updateSysClock();
        }

        // Ensure nothing is connected at this point because TNC must get
        // BT device interrupts to configure connection properly.
        bm78_reset();
        bm78_wait_until_ready();
        __HAL_RCC_USART3_CLK_DISABLE(); // UART clock gated until connected.
    } else if (powerState == PowerState::POWER_STATE_VBUS) {
        // Powered off and USB insertion event.
        setUsbConnected();
        osTimerStart(usbShutdownTimerHandle, 2000);
    } else {
        // Powered off and USB disconnected or low-battery.
        osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 0);
    }

    enable_interrupts();
    mobilinkd::tnc::configure_power_on_disconnect();

    bool power_button_down = false;

    uint32_t vddErrorNotificationTick = 0U;
    uint32_t lowPowerNotificationTick = 0U;
    uint16_t lowerPowerNotificationCount = 0;
    const uint16_t LOW_POWER_LIMIT = 128;
    const uint32_t LOW_POWER_TIMEOUT = 500; // 500ms

    /* Infinite loop */
    for (;;)
    {
        osEvent evt = osMessageGet(ioEventQueueHandle, 100);
        if (hdlc::ioFramePool().size() != 0 && !getModulator().get_ptt()->state()) {
            // If the IO event loop is inactive or the frame pool is empty for
            // too long, the TNC is essentially non-functional. When  transmitting,
            // the modulator is responsible for updating the watchdog.
            HAL_IWDG_Refresh(&hiwdg); // Refresh IWDG in IO loop (primary refresh).
        }

        if (vddErrorNotificationTick) {
            if (HAL_GetTick() - vddErrorNotificationTick > 500) {
                vddErrorNotificationTick = 0;
                indicate_connection_state(connectionState);
            }
        }

        if (evt.status != osEventMessage)
            continue;

        uint32_t cmd = evt.value.v;
        if (cmd < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
        {
            uint16_t arg = cmd & 0xFFFF;
            cmd &= 0x07FF0000;
            switch (cmd) {
            case CMD_RESTORE_SYSCLK:
                updateSysClock();
                break;
            case CMD_VREFINT_WATCHDOG:
                if (arg < mobilinkd::tnc::VREFINT_MIN) {
                    ERROR("VDDA too high");
                    if (vddErrorNotificationTick == 0) indicate_vdd_error();
                    vddErrorNotificationTick = HAL_GetTick();
                } else if (arg > mobilinkd::tnc::VREFINT_MAX) {
                     if (powerState == PowerState::POWER_STATE_VBAT) {
                        auto const tick = HAL_GetTick();
                        if (tick - lowPowerNotificationTick > LOW_POWER_TIMEOUT) {
                            lowerPowerNotificationCount = 1;
                        } else {
                            lowerPowerNotificationCount += 1;
                        }
                        lowPowerNotificationTick = tick;
                        if (lowerPowerNotificationCount == LOW_POWER_LIMIT) {
                            stop_power_monitor();
                            onShutdownCommand(true);
                        }
                    } else {
                        ERROR("VDD low while on VUSB: %d", arg);
                        HAL_Delay(10);
                        // CxxErrorHandler();
                    }
                } else {
                    WARN("Spurious Analog Watchdog alert");
                    HAL_Delay(10);
                }
                __HAL_ADC_CLEAR_FLAG(&BATTERY_ADC_HANDLE, ADC_FLAG_AWD1);
                break;
            case CMD_USB_CDC_CONNECT:
                if ((connectionState == ConnectionState::DISCONNECTED) && openCDC())
                {
                    connectionState = ConnectionState::USB_CONNECTED;
                    // Disable Bluetooth Module
                    HAL_NVIC_DisableIRQ(BT_STATE1_EXTI_IRQn);
                    HAL_NVIC_DisableIRQ(BT_STATE2_EXTI_IRQn);
                    HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_RESET);

                    INFO("CDC Opened");
                    configure_power_on_connect();
                    getModulator().init(hardware);    // Need to re-init modulator after reconfig.
                    if (!power_button_down) indicate_connected_via_usb();
                    osMessagePut(audioInputQueueHandle,
                        audio::DEMODULATOR, osWaitForever);
                }
                break;
            case CMD_USB_CONNECTED:
                INFO("VBUS Detected");
                if (powerState != POWER_STATE_VBAT) {
                    ERROR("Duplicate event");
                    break;
                }
                setUsbConnected();
                break;
            case CMD_USB_RESUME:
                INFO("USB resume");
                if (POWER_STATE_VBUS_ENUM == powerState) {
                    if (charging_enabled) {
                        HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_RESET);
                    }
                }
                break;
            case CMD_USB_SUSPEND:
                INFO("USB suspend");
                // Suspend will be called before enumeration. Do not stop PCD
                // in POWER_STATE_VBUS_HOST otherwise enumeration will fail.
                // Normal case here after POWER_STATE_VBUS_ENUM is the USB
                // cable has been disconnected. In order to properly detect
                // VUSB loss, the 1.5k pull-up needs to be disconnected. The
                // down side is that USB suspend/resume when enumerated no
                // longer works properly.
                //
                // Maybe the best thing to do here if not connected via BT is
                // to just shut down.
                if (POWER_STATE_VBUS_ENUM == powerState) {
                    INFO("PCD Stop");
                    HAL_PCD_Stop(&HPCD); // Disconnect; remove 1.5k pull-up.
                    if (charging_enabled) {
                        HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_SET);
                    }
                    powerState = POWER_STATE_VBUS_HOST;
                }
                break;
            case CMD_USB_DISCONNECTED:
                INFO("VBUS Lost");
                if (powerState == POWER_STATE_VBAT) {
                    ERROR("Duplicate event");
                    break;
                }
                powerState = POWER_STATE_VBAT;
#ifdef STM32L433xx
                HAL_PCD_Stop(&hpcd_USB_FS);
#endif
                HAL_PCD_MspDeInit(&hpcd_USB_FS);
                HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_SET);
                charging_enabled = 0;

                if (powerOffViaUSB()) {
                    osMessagePut(ioEventQueueHandle, CMD_SHUTDOWN, 0);
                }

                if (connectionState == ConnectionState::DISCONNECTED) {
                    updateSysClock();
                    break;
                } else if (connectionState == ConnectionState::BT_CONNECTED) {
                    break;
                } // else connectionState == ConnectionState::USB_CONNECTED
            [[ fallthrough ]]; // when the CDC part was connected.
            case CMD_USB_CDC_DISCONNECT:
                INFO("CDC Disconnect");
                if (connectionState == ConnectionState::USB_CONNECTED) {
                    connectionState = ConnectionState::DISCONNECTED;
                    osMessagePut(audioInputQueueHandle, audio::IDLE, osWaitForever);
                    kiss::getAFSKTestTone().stop();
                    getModulator().abort();
                    openNull();
                    INFO("CDC Closed");

                    // Enable Bluetooth Module
                    HAL_GPIO_WritePin(BT_SLEEP_GPIO_Port, BT_SLEEP_Pin, GPIO_PIN_SET);
                    bm78_wait_until_ready();

                    HAL_NVIC_EnableIRQ(BT_STATE1_EXTI_IRQn);
                    HAL_NVIC_EnableIRQ(BT_STATE2_EXTI_IRQn);

                    configure_power_on_disconnect();
                    if (!power_button_down) indicate_waiting_to_connect();
                    updateSysClock();
                }
                break;
            case CMD_POWER_BUTTON_DOWN:
                INFO("Power Down");
                power_button_down = true;
                indicate_turning_off();
                if (auto result = osTimerStart(powerOffTimerHandle, 1890) == osOK) {
                    INFO("shutdown timer started");
                } else {
                    (void) result;
                    ERROR("shutdown timer start failed = %d", result);
                }
                break;
            case CMD_POWER_BUTTON_UP:
                INFO("Power Up");
                power_button_down = false;
                osTimerStop(powerOffTimerHandle);
                read_battery_level();
                indicate_connection_state(connectionState);
                break;
            case CMD_BOOT_BUTTON_DOWN:
                TNC_DEBUG("BOOT Down");
                // If the TNC is connected to a USB host, reboot.  The boot pin
                // is being held so it will boot into the bootloader.
                if ((POWER_STATE_VBUS_ENUM == powerState || POWER_STATE_VBUS_HOST == powerState) and getNullPort() == ioport)
                {
                    HAL_PWR_EnableBkUpAccess();
                    WRITE_REG(BKUP_TNC_LOWPOWER_STATE, TNC_LOWPOWER_DFU);
                    HAL_PWR_DisableBkUpAccess();
                    HAL_NVIC_SystemReset();
                }
                break;
            case CMD_BOOT_BUTTON_UP:
                TNC_DEBUG("BOOT Up");
                break;
            case CMD_BT_CONNECT:
                TNC_DEBUG("BT Connect");
                if (openSerial())
                {
                    connectionState = ConnectionState::BT_CONNECTED;
                    configure_power_on_connect();
                    if (POWER_STATE_VBUS_ENUM == powerState) {
                        powerState = POWER_STATE_VBUS_HOST;
                        HAL_PCD_Stop(&HPCD);
                    }
                    INFO("BT Opened");
                    if (!power_button_down) indicate_connected_via_ble();
                    getModulator().init(hardware);    // Need to re-init modulator after reconfig.
                    osMessagePut(audioInputQueueHandle,
                        audio::DEMODULATOR, osWaitForever);
                    osThreadYield();
                }
                break;
            case CMD_BT_DISCONNECT:
                INFO("BT Disconnect");
                __HAL_RCC_USART3_CLK_DISABLE();
                connectionState = ConnectionState::DISCONNECTED;
                openNull();
                if (POWER_STATE_VBUS_HOST == powerState) {
                    HAL_PCD_Start(&HPCD);
                }
                osMessagePut(audioInputQueueHandle, audio::IDLE,
                    osWaitForever);
                kiss::getAFSKTestTone().stop();
                getModulator().abort();
                INFO("BT Closed");
                updateSysClock();
                configure_power_on_disconnect();
                if (!power_button_down) indicate_waiting_to_connect();
                break;
            case CMD_SET_PTT_SIMPLEX:
                getModulator().set_ptt(&simplexPtt);
                break;
            case CMD_SET_PTT_MULTIPLEX:
                getModulator().set_ptt(&multiplexPtt);
                break;
            case CMD_SHUTDOWN:
                onShutdownCommand();
                break;
            case CMD_USB_CHARGE_ENABLE:
                CxxErrorHandler();
                break;
            case CMD_USB_DISCOVERY_COMPLETE:
                CxxErrorHandler();
                break;
            case CMD_USB_CHARGER_CONNECTED:
                CxxErrorHandler();
                break;
            case CMD_USB_HOST_CONNECTED:
                CxxErrorHandler();
                break;
            case CMD_USB_HOST_ENUMERATED:
                onUsbHostEnumerated();
                break;
           case CMD_USB_DISCOVERY_ERROR:
                CxxErrorHandler();
                break;
            case CMD_BT_DEEP_SLEEP:
                INFO("BT deep sleep");
                break;
            case CMD_BT_ACCESS:
                INFO("BT access enabled");
                break;
            case CMD_BT_TX:
                INFO("BT transmit");
                break;
            case CMD_BT_IDLE:
                INFO("BT idle");
                break;
            default:
                WARN("unknown command = %04x", static_cast<unsigned int>(cmd));
                break;
            }
            continue;
        }

        using hdlc::IoFrame;

        auto frame = static_cast<IoFrame*>(evt.value.p);

        if (frame->source() & IoFrame::RF_DATA)
        {
            TNC_DEBUG("RF frame");
            frame->source(frame->source() & 0x70);
            if (!ioport->write(frame, frame->size() + 100))
            {
                ERROR("Timed out sending frame");
                // The frame has been passed to the write() call.  It owns it now.
                // hdlc::release(frame);
            }
        }
        else
        {
            TNC_DEBUG("Serial frame");
            if ((frame->type() & 0x0F) == IoFrame::DATA)
            {
                kiss::getAFSKTestTone().stop();
                if (osMessagePut(hdlcOutputQueueHandle,
                    reinterpret_cast<uint32_t>(frame),
                    osWaitForever) != osOK)
                {
                    ERROR("Failed to write frame to TX queue");
                    hdlc::release(frame);
                }
            }
            else
            {
                kiss::handle_frame(frame->type(), frame);
            }
        }
    }
}

namespace mobilinkd {
namespace tnc {

void print_startup_banner()
{
#ifdef KISS_LOGGING
    uint32_t* uid = (uint32_t*) UID_BASE;  // STM32L4xx (same for 476 and 432)

    INFO("%s version %s", mobilinkd::tnc::kiss::HARDWARE_VERSION,
        mobilinkd::tnc::kiss::FIRMWARE_VERSION);
    INFO("CPU core clock: %luHz", SystemCoreClock);
    INFO("    Device UID: %08lX %08lX %08lX", uid[0], uid[1], uid[2]);
    INFO("   MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
        mac_address[0], mac_address[1], mac_address[2],
        mac_address[3], mac_address[4], mac_address[5])

    uint8_t* version_ptr = (uint8_t*) 0x1FFF6FF2;

    int version = *version_ptr;

    INFO("Bootloader version: 0x%02X", version);
#endif
}

}
} // mobilinkd::tnc

// Copyright 2018 Rob Riggs <rob@mobilinkd.com>
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
#include "UsbPort.hpp"
#include "SerialPort.hpp"
#include "NullPort.hpp"
#include "LEDIndicator.h"

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "cmsis_os.h"

extern osMessageQId hdlcOutputQueueHandle;
extern PCD_HandleTypeDef hpcd_USB_FS;

extern "C" void shutdown(void);
extern "C" void TNC_Error_Handler(int dev, int err);

void startIOEventTask(void const*)
{
  mobilinkd::tnc::print_startup_banner();

  auto& hardware = mobilinkd::tnc::kiss::settings();
  if (reset_requested) hardware.init();
  else {} // hardware.load();
  hardware.debug();
  strcpy((char*)hardware.mycall, "WX9O");
  hardware.update_crc();
  mobilinkd::tnc::audio::setAudioOutputLevel();
  mobilinkd::tnc::audio::setAudioInputLevels();

  if (reset_requested) HAL_GPIO_WritePin(BT_CMD_GPIO_Port, BT_CMD_Pin, GPIO_PIN_SET);

  osDelay(100);
  HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET);
  osDelay(100);

  if (reset_requested) {
      HAL_GPIO_WritePin(BT_CMD_GPIO_Port, BT_CMD_Pin, GPIO_PIN_RESET);
  }

  indicate_on();

  if (HAL_GPIO_ReadPin(BT_STATE_GPIO_Port, BT_STATE_Pin) == GPIO_PIN_SET)
  {
    DEBUG("BT Connected at start");
    openSerial();
    INFO("BT Opened");
    indicate_connected_via_ble();
  } else {
    indicate_waiting_to_connect();
  }

  /* Infinite loop */
  for(;;)
  {
    osEvent evt = osMessageGet(ioEventQueueHandle, osWaitForever);
    if (evt.status != osEventMessage) continue;

    uint32_t cmd = evt.value.v;
    if (cmd < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
    {
      switch (cmd) {
      case CMD_USB_CDC_CONNECT:
        if (openCDC()) {
          // Disable Bluetooth Module
          HAL_NVIC_DisableIRQ(EXTI1_IRQn);
          HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_RESET);

          INFO("CDC Opened");
          indicate_connected_via_usb();
          osMessagePut(audioInputQueueHandle,
            mobilinkd::tnc::audio::DEMODULATOR, osWaitForever);
        }
        break;
      case CMD_USB_DISCONNECTED:
        INFO("VBUS Lost");
        HAL_PCD_MspDeInit(&hpcd_USB_FS);
        HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_SET);
        if (mobilinkd::tnc::ioport != mobilinkd::tnc::getUsbPort())
          break;
        // fall-through
      case CMD_USB_CDC_DISCONNECT:
        closeCDC();
        // Enable Bluetooth Module
        HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET);
        osDelay(1000);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);

        INFO("CDC Closed");
        indicate_waiting_to_connect();
        osMessagePut(audioInputQueueHandle,
          mobilinkd::tnc::audio::IDLE, osWaitForever);
        break;
      case CMD_POWER_BUTTON_DOWN:
        INFO("Power Down");
        osMessagePut(audioInputQueueHandle,
          mobilinkd::tnc::audio::IDLE, osWaitForever);
        break;
      case CMD_POWER_BUTTON_UP:
        DEBUG("Power Up");
        shutdown();
        break;
      case CMD_BOOT_BUTTON_DOWN:
        DEBUG("BOOT Down");
        break;
      case CMD_BOOT_BUTTON_UP:
        DEBUG("BOOT Up");
        osMessagePut(audioInputQueueHandle,
          mobilinkd::tnc::audio::AUTO_ADJUST_INPUT_LEVEL, osWaitForever);
        if (mobilinkd::tnc::ioport != mobilinkd::tnc::getNullPort()) {
          osMessagePut(audioInputQueueHandle,
            mobilinkd::tnc::audio::DEMODULATOR, osWaitForever);
        } else {
          osMessagePut(audioInputQueueHandle,
            mobilinkd::tnc::audio::IDLE, osWaitForever);
        }
        break;
      case CMD_BT_CONNECT:
        DEBUG("BT Connect");
        if (openSerial()) {
          osMessagePut(audioInputQueueHandle,
            mobilinkd::tnc::audio::DEMODULATOR, osWaitForever);
          INFO("BT Opened");
          indicate_connected_via_ble();
          HAL_PCD_EP_SetStall(&hpcd_USB_FS, CDC_CMD_EP);
        }
        break;
      case CMD_BT_DISCONNECT:
        INFO("BT Disconnect");
        closeSerial();
        indicate_waiting_to_connect();
        HAL_PCD_EP_ClrStall(&hpcd_USB_FS, CDC_CMD_EP);
        osMessagePut(audioInputQueueHandle,
          mobilinkd::tnc::audio::IDLE, osWaitForever);
        INFO("BT Closed");
        break;
      case CMD_SET_PTT_SIMPLEX:
        getModulator().set_ptt(&simplexPtt);
        break;
      case CMD_SET_PTT_MULTIPLEX:
        getModulator().set_ptt(&multiplexPtt);
        break;
      case CMD_SHUTDOWN:
        shutdown();
        break;
      case CMD_USB_CONNECTED:
        INFO("VBUS Detected");
        HAL_PCD_MspInit(&hpcd_USB_FS);
        HAL_PCDEx_BCD_VBUSDetect(&hpcd_USB_FS);
        break;
      case CMD_USB_CHARGE_ENABLE:
        HAL_GPIO_WritePin(USB_CE_GPIO_Port, USB_CE_Pin, GPIO_PIN_RESET);
        break;
      case CMD_USB_DISCOVERY_COMPLETE:
        USBD_Start(&hUsbDeviceFS);
        initCDC();
        break;
      case CMD_USB_DISCOVERY_ERROR:
        TNC_Error_Handler(6, 0);
        break;
      default:
        WARN("unknown command = %04x", static_cast<unsigned int>(cmd));
        break;
      }
      continue;
    }

    using mobilinkd::tnc::hdlc::IoFrame;

    auto frame = static_cast<IoFrame*>(evt.value.p);

    switch (frame->source()) {
    case IoFrame::RF_DATA:
      DEBUG("RF frame");
      if (!mobilinkd::tnc::ioport->write(frame, 100)) {
        ERROR("Timed out sending frame");
        mobilinkd::tnc::hdlc::release(frame);
      }
      break;
    case IoFrame::SERIAL_DATA:
      DEBUG("Serial frame");
      if ((frame->type() & 0x0F) == IoFrame::DATA) {
        if (osMessagePut(hdlcOutputQueueHandle, reinterpret_cast<uint32_t>(frame),
          osWaitForever) != osOK) {
          ERROR("Failed to write frame to TX queue");
          mobilinkd::tnc::hdlc::release(frame);
        }
      } else {
        mobilinkd::tnc::kiss::handle_frame(frame->type(), frame);
      }
      break;
    case IoFrame::DIGI_DATA:
      DEBUG("Digi frame");
      if (osMessagePut(hdlcOutputQueueHandle, reinterpret_cast<uint32_t>(frame),
        osWaitForever) != osOK) {
        mobilinkd::tnc::hdlc::release(frame);
      }
      break;
    case IoFrame::FRAME_RETURN:
      mobilinkd::tnc::hdlc::release(frame);
      break;
    }
  }
}

void startCdcBlinker(void const*)
{
  for (;;) {
    osDelay(4500);
  }
}

extern osThreadId cdcBlinkerHandle;

namespace mobilinkd { namespace tnc {

void print_startup_banner()
{
    uint32_t* uid = (uint32_t*)0x1FFF7590;  // STM32L4xx (same for 476 and 432)

    INFO("%s version %s", mobilinkd::tnc::kiss::HARDWARE_VERSION, mobilinkd::tnc::kiss::FIRMWARE_VERSION);
    INFO("CPU core clock: %luHz", SystemCoreClock);
    INFO(" Serial number: %08lX %08lX %08lX", uid[0], uid[1], uid[2]);

    uint8_t* version_ptr = (uint8_t*)0x1FFF6FF2;

    int version = *version_ptr;

    INFO("Bootloader version: 0x%02X", version);
}

void start_cdc_blink()
{
  osThreadResume(cdcBlinkerHandle);
}

void stop_cdc_blink()
{
  osThreadSuspend(cdcBlinkerHandle);
}

}} // mobilinkd::tnc

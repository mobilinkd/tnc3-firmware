// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "bm78.h"
#include "GPIO.hpp"
#include "Log.h"
#include "main.h"

#include "stm32l4xx_hal.h"

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <array>

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart3;

namespace mobilinkd { namespace tnc { namespace bm78 {

/**
 * The BM78 module is a dual-mode BT3.0 & BLE5.0 UART modules.  It supports
 * transparent data transfer in one or both modes.  The module documentation
 * only clearly describes how to configure the modules using a (horrible)
 * Windows application.  Microchip publishes a library for use with their
 * PIC chips that we use as a guide for implementing our own configuration
 * code.
 *
 * The module must be booted into EEPROM programming mode in order to make
 * changes.  This mode uses 115200 baud with no hardware flow control
 *
 * We program the module for the following features:
 *
 *  - The module is set for 115200 baud with hardware flow control.
 *  - The name is changed to TNC3.
 *  - The BT3.0 pairing PIN is set to 1234
 *  - The BLE5.0 pairing PIN is set to "625653" (MBLNKD on a phone pad).
 *  - The module power setting is set as low as possible for BLE.
 */

const uint32_t BT_INIT_MAGIC = 0xc0a2;

void bm78_reset()
{
  // Must use HAL_Delay() here as osDelay() may not be available.
  mobilinkd::tnc::gpio::BT_RESET::off();
  HAL_Delay(1200);
  mobilinkd::tnc::gpio::BT_RESET::on();
  HAL_Delay(800);

}

void exit_command_mode()
{
    gpio::BT_CMD::on();
    bm78_reset();
    INFO("BM78 in PASSTHROUGH mode");
}

HAL_StatusTypeDef read_response(uint8_t* buffer, uint16_t size, uint32_t timeout)
{
    memset(buffer, 0, size);

    auto result = HAL_UART_Receive(&huart3, buffer, size - 1, timeout);

    if (result == HAL_TIMEOUT) result = HAL_OK;

    return result;
}

bool enter_program_mode()
{
    // Ensure we start out disconnected.
    gpio::BT_CMD::off();
    HAL_Delay(10);
    gpio::BT_RESET::off();
    HAL_Delay(10);      // Spec says minimum 63ns
    gpio::BT_RESET::on();
    HAL_Delay(200);     // I could not find timing specifications for this.

    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.BaudRate = 115200;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        CxxErrorHandler();
    }

    // Read (cmd = 0x29) all 1K bytes, starting at address 0, 128 bytes at a time.
    uint8_t cmd[] = {0x01, 0x29, 0xfc, 0x03, 0x00, 0x00, 0x80};

    constexpr const uint16_t BLOCK_SIZE = 128;

    for (uint16_t addr = 0; addr != 0x500; addr += BLOCK_SIZE)
    {
        cmd[5] = addr & 0xFF;
        cmd[4] = (addr >> 8) & 0xFF;
        if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
        {
            ERROR("Read EEPROM transmit failed");
            return false;
        }

        uint8_t buffer[BLOCK_SIZE + 10];

        if (HAL_UART_Receive(&huart3, buffer, BLOCK_SIZE + 10, 1000) != HAL_OK)
        {
            ERROR("Read EEPROM receive failed");
            return false;
        }

        for (size_t i = 0; i != BLOCK_SIZE; i += 16) {
            printf("%04X: ", addr + i);
            for (size_t j = 0; j != 16; j++) {
                printf("%02X ", buffer[i + j + 10]);
            }
            printf("\r\n");
            HAL_Delay(10);
        }
    }
    return true;
}

bool exit_program_mode()
{
    auto result = true;

    // Read (cmd = 0x29) all 1K bytes, starting at address 0, 128 bytes at a time.
    uint8_t cmd[] = {0x01, 0x29, 0xfc, 0x03, 0x00, 0x00, 0x80};

    constexpr const uint16_t BLOCK_SIZE = 128;

    for (uint16_t addr = 0; addr != 0x500; addr += BLOCK_SIZE)
    {
        cmd[5] = addr & 0xFF;
        cmd[4] = (addr >> 8) & 0xFF;
        if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
        {
            ERROR("Read EEPROM transmit failed");
            result = false;
            break;
        }

        uint8_t buffer[BLOCK_SIZE + 10];

        if (HAL_UART_Receive(&huart3, buffer, BLOCK_SIZE + 10, 1000) != HAL_OK)
        {
            ERROR("Read EEPROM receive failed");
            result = false;
            break;
        }

        for (size_t i = 0; i != BLOCK_SIZE; i += 16) {
            printf("%04X: ", addr + i);
            for (size_t j = 0; j != 16; j++) {
                int c = buffer[i + j + 10];
                printf(" %c ", isprint(c) ? (char) c : '.');
            }
            printf("\r\n");
            printf("%04X: ", addr + i);
            for (size_t j = 0; j != 16; j++) {
                printf("%02X ", buffer[i + j + 10]);
            }
            printf("\r\n");
            HAL_Delay(10);
        }
    }


    // Ensure we start out disconnected.
    gpio::BT_CMD::on();
    HAL_Delay(10);
    gpio::BT_RESET::off();
    HAL_Delay(1);       // Spec says minimum 63ns.
    gpio::BT_RESET::on();
    HAL_Delay(400);     // Spec says 354ms.

    huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    huart3.Init.BaudRate = 115200;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        CxxErrorHandler();
    }

    return result;
}

bool set_reset()
{
    return true;
}

bool parse_write_result(const char* function)
{
    uint8_t result[7];

    if (HAL_UART_Receive(&huart3, result, sizeof(result), 1000) != HAL_OK)
    {
        ERROR("%s receive failed", function);
        return false;
    }

    if (result[6] != 0)
    {
        ERROR("%s operation failed", function);
    } else {
        INFO("%s succeeded", function);
    }

    return result[6] == 0;

}

bool set_name()
{
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x13, 0x00, 0x0b, 0x10
        , 'T', 'N', 'C', '3', ' ', 'M', 'o', 'b', 'i', 'l', 'i', 'n', 'k', 'd'
        , 0x00, 0x00};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool set_pin()
{
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x13, 0x00, 0x5b, 0x10
        , '1', '2', '3', '4', '5', '6', 0x00, 0x00
        , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool set_misc()
{
    // System options. Security parameters.
    // 0x01AD: 01 04 00 19 41 00
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x09, 0x01, 0xad, 0x06
        , 0x02, 0x04, 0x00, 0x59, 0x09, 0x00};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool set_le_service_name()
{
    // LE Service Name.
    // 0x0217: 08 KISS TNC
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x0c, 0x02, 0x17, 0x09
        , 0x08, 'K', 'I', 'S', 'S', ' ', 'T', 'N', 'C'};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool set_le_service_uuid()
{
    // LE Service UUID.
    // 0x0354: 6F E6 FD 82 C1 36 65 A4 16 4E 88 55  5E 6A EB 27
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x13, 0x03, 0x54, 0x10
        , 0x6F, 0xE6, 0xFD, 0x82, 0xC1, 0x36, 0x65, 0xA4
        , 0x16, 0x4E, 0x88, 0x55, 0x5E, 0x6A, 0xEB, 0x27};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool set_le_tx_attribute_uuid()
{
    // TX attribute UUID.
    // 0x0364: BB 68 1F 96 B0 01 49 AE C9 46 2A BA 02 00 00 00
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x13, 0x03, 0x64, 0x10
        , 0xBB, 0x68, 0x1F, 0x96, 0xB0, 0x01, 0x49, 0xAE
        , 0xC9, 0x46, 0x2A, 0xBA, 0x02, 0x00, 0x00, 0x00};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool set_le_rx_attribute_uuid()
{
    // RX attribute UUID.
    // 0x0374: BB 68 1F 96 B0 01 49 AE C9 46 2A BA 01 00 00 00
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x13, 0x03, 0x74, 0x10
        , 0xBB, 0x68, 0x1F, 0x96, 0xB0, 0x01, 0x49, 0xAE
        , 0xC9, 0x46, 0x2A, 0xBA, 0x01, 0x00, 0x00, 0x00};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool set_le_attribute_properties()
{
    // RX notify; TX write/write without response.
    // 0x0384: 10 0C
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x05, 0x03, 0x84, 0x02, 0x10, 0x0c};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("set_le_attribute_properties transmit failed");
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool set_le_manufacturer()
{
    // LE manufacturer string.
    // 0x0300: Mobilinkd LLC
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x13, 0x03, 0x00, 0x10
        , 'M', 'o', 'b', 'i', 'l', 'i', 'n', 'k', 'd', ' ', 'L', 'L', 'C'
        , 0x00, 0x00, 0x00};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool set_le_advertising()
{
    // undocumented but required for proper advertising.
    // 0x03A0: 31 bytes
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x22, 0x03, 0xa0, 0x1f
        , 0x1B, 0x05, 0x09, 0x54, 0x4E, 0x43, 0x33, 0x11
        , 0x06, 0x6F, 0xE6, 0xFD, 0x82, 0xC1, 0x36, 0x65
        , 0xA4, 0x16, 0x4E, 0x88, 0x55, 0x5E, 0x6A, 0xEB
        , 0x27, 0x02, 0x01, 0x02, 0x00, 0x00, 0x00};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 10) != HAL_OK)
    {
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
}

bool configure_le_service()
{
    bool result = true;

    result &= set_le_service_name();
    result &= set_le_service_uuid();
    result &= set_le_tx_attribute_uuid();
    result &= set_le_rx_attribute_uuid();
    result &= set_le_attribute_properties();
    result &= set_le_manufacturer();
    result &= set_le_advertising();

    return result;
}

/**
 * Set the baud rate to 115200, turn on RTS/CTS flow control and then
 * reset the BLE module.
 *
 * @return true on success, otherwise false.
 */
bool set_uart()
{
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.BaudRate = 115200;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
      CxxErrorHandler();
  }

  return true;
}

bool set_work()
{
    return true;
}

bool set_power()
{
  return true;
}

bool set_secure()
{
    return true;
}

bool set_reliable()
{
    return true;
}

}}} // mobilinkd::tnc::bm78

int bm78_initialized()
{
    using namespace mobilinkd::tnc;
    using namespace mobilinkd::tnc::bm78;

    return HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == BT_INIT_MAGIC;
}

int bm78_initialize()
{
    using namespace mobilinkd::tnc;
    using namespace mobilinkd::tnc::bm78;

    int result = 0;

    if (!enter_program_mode()) result = 1;
    else if (!set_name()) result = 2;
    else if (!set_pin()) result = 3;
    else if (!set_misc()) result = 4;
    else if (!configure_le_service()) result = 5;
    exit_program_mode();

#if 0
    bt_status_init();

    if (not enter_command_mode()) result = 1;
    else if (not set_uart()) result = 4;
    else if (not set_work()) result = 7;
    else if (not set_power()) result = 8;
    else if (not set_secure()) result = 9;
    else if (not set_gpio()) result = 3;
    else if (not set_name()) result = 2;
    else if (not set_reset()) result = 10;
    exit_command_mode();
#if 1
    if (result == 0) {
        /* Write BT_INIT_MAGIC to RTC back-up register RTC_BKP_DR1 to indicate
           that the HC-05 module has been initialized.  */
        HAL_PWR_EnableBkUpAccess();
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, BT_INIT_MAGIC);
        HAL_PWR_DisableBkUpAccess();
    }
#endif
#endif
    return result;
}

void bm78_state_change(void) {}
int bm78_disable(void) {return 0;}
int bm78_enable(void) {return 0;}

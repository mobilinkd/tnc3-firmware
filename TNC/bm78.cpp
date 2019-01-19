// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "bm78.h"
#include "bm78_eeprom.hpp"
#include "GPIO.hpp"
#include "Log.h"
#include "main.h"
#include "HdlcFrame.hpp"

#include "stm32l4xx_hal.h"

#include <cstdint>
#include <cstring>
#include <algorithm>
#include <array>

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart3;
extern CRC_HandleTypeDef hcrc;

namespace mobilinkd { namespace tnc { namespace bm78 {

/**
 * The BM78 module is a dual-mode BT3.0 & BLE5.0 UART modules.  It supports
 * transparent data transfer in one or both modes.  The module documentation
 * only clearly describes how to configure the modules using a (horrible)
 * Windows application.  Microchip publishes a library for use with their
 * PIC chips that we initially used as a guide for implementing our own
 * configuration code.  We now use a hybrid mechanism, writing out the
 * entire configuration and tweaking a few settings.
 *
 * The module must be booted into EEPROM programming mode in order to make
 * changes.  This mode uses 115200 baud with no hardware flow control.
 *
 * We program the module for the following features:
 *
 *  - The module is set for 115200 baud with hardware flow control.
 *  - The name is changed to TNC3.
 *  - The BT3.0 pairing PIN is set to 1234
 *  - The BLE5.0 pairing PIN is set to "123456".
 *  - The module power setting is set as low as possible for BLE.
 */

void bm78_reset()
{
  // Must use HAL_Delay() here as osDelay() may not be available.
  mobilinkd::tnc::gpio::BT_RESET::off();
  HAL_Delay(1);
  mobilinkd::tnc::gpio::BT_RESET::on();
}

/**
 * Enter BM78 EEPROM programming mode.
 *
 * This pulls EAN low on the BM78 and resets the module to enter EEPROM
 * programming mode and disable HW flow control on the UART.
 *
 * @note The timing of this process is not well documented.
 */
void enter_program_mode()
{
    // Ensure we start out disconnected.
    gpio::BT_CMD::off();
    HAL_Delay(10);
    gpio::BT_RESET::off();
    HAL_Delay(1);       // Spec says minimum 63ns.
    gpio::BT_RESET::on();
    HAL_Delay(200);     // Timing for EEPROM programming is not specified.

    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.BaudRate = 115200;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        CxxErrorHandler();
    }
}

bool read_mac_address()
{
    // Read (cmd = 0x29) the first 6 bytes at address 0.
    uint8_t cmd[] = { 0x01, 0x29, 0xfc, 0x03, 0x00, 0x00, 0x06 };
    constexpr const uint16_t BLOCK_SIZE = 6;
    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 100) != HAL_OK)
    {
        ERROR("read_mac_address transmit failed");
        return false;
    }

    uint8_t buffer[BLOCK_SIZE + 10];
    if (HAL_UART_Receive(&huart3, buffer, sizeof(buffer), 1000) != HAL_OK)
    {
        ERROR("read_mac_address receive failed");
        return false;
    }

    for (size_t i = 0; i != BLOCK_SIZE; ++i)
    {
        mac_address[5 - i] = buffer[i + 10]; // Reverse the bytes
    }

    return true;
}

/**
 * Exit BM78 EEPROM programming mode and return to pass-through mode.
 *
 * This asserts EAN on the BM78 and resets the module to exit EEPROM
 * programming mode, reconfigures the UART for HW flow control, then
 * waits for the module to become ready.
 */
void exit_program_mode()
{
    gpio::BT_CMD::on();
    HAL_Delay(10);
    gpio::BT_RESET::off();
    HAL_Delay(1);       // Spec says minimum 63ns.
    gpio::BT_RESET::on();

    huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    huart3.Init.BaudRate = 115200;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        CxxErrorHandler();
    }

    bm78_wait_until_ready();
}

/**
 * Parse the result of the previous write.  Return true if the write was
 * successful and false if either the response could not be read or the
 * write failed.
 *
 * @pre A block of EEPROM data was written to the BM78.
 * @post The write response has been written.
 *
 * The result of the write is 7 bytes in length.  The write was successful
 * if the last byte returned is a 0.
 *
 * @param function is the name of the calling function and is used when
 *  logging to indicate which write operation failed.
 * @return true when the write was successful, otherwise false.
 */
bool parse_write_result(const char* function)
{
    uint8_t result[7];

    HAL_StatusTypeDef status;
    if ((status = HAL_UART_Receive(&huart3, result, sizeof(result), 1000)) != HAL_OK)
    {
        ERROR("%s receive failed (%d)", function, status);
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

/**
 * Parse the EEPROM data and write the segments to the BM78 EEPROM.
 *
 * @pre The BM78 is in programming mode.
 * @post The BM78 is programmed.
 *
 * The data from the EEPROM programming UI is a sparsely populated memory
 * map.  That data has been converted into <address> <length> <data> format
 * in the eeprom_data block.  The last block has a zero length, indicating
 * EOF.  The address is two bytes, big endian, followed by a one byte
 * length and *length* bytes of data.
 *
 * It is important that memory areas that are not populated by the UI are
 * not overwritten as they may contain device-specific data elements.  For
 * example, the first 6 bytes of data are the Bluetooth device MAC address.
 *
 * The write command is 7 bytes long.  The first three bytes are static:
 * {0x01, 0x27, 0xfc}.  The 4th byte is *length* + 3.  The 5th and 6th
 * are the address, and the 7th is *length*.  The data to be written
 * follows.
 *
 * Once the data has been written, the BM78 module sends a response.  This
 * must be read and parse before the next block is written.
 *
 * @return true if the BM78 is programmed, otherwise false.
 */
bool write_eeprom()
{
    const uint8_t* data = eeprom_data;

    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x00, 0x00, 0x00, 0x00};

    bool result = true;
    while (result)
    {
        auto len = data[2];

        cmd[0] = 0x01;
        cmd[1] = 0x27;
        cmd[2] = 0xfc;
        cmd[3] = len + 3;
        cmd[4] = data[0];
        cmd[5] = data[1];
        cmd[6] = len;

        if (len == 0)
            break;

        data += 3;

        if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 1000) != HAL_OK)
        {
            ERROR("%s transmit header failed", __PRETTY_FUNCTION__);
            return false;
        }

        if (HAL_UART_Transmit(&huart3, const_cast<uint8_t*>(data), len, 1000) != HAL_OK)
        {
            ERROR("%s transmit data failed", __PRETTY_FUNCTION__);
            return false;
        } else {
            DEBUG("%s transmit succeeded %d bytes at 0x%02X%02X",
                __FUNCTION__, cmd[6], cmd[4], cmd[5]);
        }

        result = parse_write_result(__PRETTY_FUNCTION__);

        data += len;
    }

    return result;
}

bool write_serial()
{
    uint8_t cmd[] = {0x01, 0x27, 0xfc, 0x13, 02, 0x42, 0x10};

    if (HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 1000) != HAL_OK)
    {
        ERROR("%s transmit header failed", __PRETTY_FUNCTION__);
        return false;
    }

    if (HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(serial_number_64),
        16, 1000) != HAL_OK)
    {
        ERROR("%s transmit data failed", __PRETTY_FUNCTION__);
        return false;
    }

    return parse_write_result(__PRETTY_FUNCTION__);
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
        ERROR("%s transmit failed", __PRETTY_FUNCTION__);
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

void bm78_wait_until_ready()
{
    auto start = HAL_GetTick();
    // Must wait until P1_5 (BT_STATE2) is high and P0_4 (BT_STATE1) is low.
    GPIO_PinState bt_state1, bt_state2;
    do {
        if (HAL_GetTick() > start + 2000) CxxErrorHandler(); // Timed out.

        HAL_Delay(100);
        bt_state2 = HAL_GPIO_ReadPin(BT_STATE2_GPIO_Port, BT_STATE2_Pin);
        bt_state1 = HAL_GPIO_ReadPin(BT_STATE1_GPIO_Port, BT_STATE1_Pin);
        DEBUG("bt_state2=%d, bt_state1=%d", bt_state2, bt_state1);
    } while (!((bt_state2 == GPIO_PIN_SET) and (bt_state1 == GPIO_PIN_RESET)));
}

uint32_t eeprom_crc()
{
    const uint8_t* data = eeprom_data;

    while (true)
    {
        data++;
        data++;
        auto len = *data++;
        if (!len) break;
        data += len;
    }

    uint32_t size = data - eeprom_data;

    return HAL_CRC_Calculate(&hcrc, (uint32_t*)eeprom_data, size);
}

int bm78_initialized()
{
    using namespace mobilinkd::tnc;
    using namespace mobilinkd::tnc::bm78;

    auto crc = eeprom_crc();

    return HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == crc;
}

void bm78_initialize_mac_address()
{
    using namespace mobilinkd::tnc::bm78;

    enter_program_mode();
    read_mac_address();
    exit_program_mode();
}

int bm78_initialize()
{
    using namespace mobilinkd::tnc;
    using namespace mobilinkd::tnc::bm78;

    int result = 0;

    enter_program_mode();
    if (!write_eeprom()) result = 1;
    else if (!write_serial()) result = 2;
    else if (!read_mac_address()) result = 3;
    exit_program_mode();

#if 1
    if (result == 0) {
        /* Write CRC to RTC back-up register RTC_BKP_DR1 to indicate
           that the BM78 module has been initialized.  */
        HAL_PWR_EnableBkUpAccess();
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, eeprom_crc());
        HAL_PWR_DisableBkUpAccess();
    }
#endif
    return result;
}

void bm78_state_change(void) {}
int bm78_disable(void) {return 0;}
int bm78_enable(void) {return 0;}

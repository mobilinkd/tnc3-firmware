// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Log.h"
#include "SerialPort.hpp"
#include "PortInterface.h"
#include "HdlcFrame.hpp"
#include "bm78.h"
#include "Kiss.hpp"
#include "main.h"

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#include <cstdlib>
#include <cstring>
#include <atomic>

extern UART_HandleTypeDef huart3;
extern osMessageQId ioEventQueueHandle;

uint8_t rxBuffer;
std::atomic<int> uart_error{HAL_UART_ERROR_NONE};

std::atomic<bool> txDoneFlag{true};

uint8_t tmpBuffer[mobilinkd::tnc::TX_BUFFER_SIZE];
uint8_t tmpBuffer2[mobilinkd::tnc::TX_BUFFER_SIZE];

void log_frame(mobilinkd::tnc::hdlc::IoFrame* frame)
{
    int pos = 0;
    for (auto c: *frame) {
        if (isprint(int(c))) pos += sprintf((char*)tmpBuffer2 + pos, " %c ", c);
        else pos += sprintf((char*)tmpBuffer2 + pos, "/%02x", c);
        if (pos > 80) {
          DEBUG((char*)tmpBuffer2);
          pos = 0;
        }
    }
    DEBUG((char*)tmpBuffer2);
}

extern "C" void startSerialTask(void const* arg)
{
    using namespace mobilinkd::tnc;

    auto serialPort = static_cast<const SerialPort*>(arg);

    const uint8_t FEND = 0xC0;
    const uint8_t FESC = 0xDB;
    const uint8_t TFEND = 0xDC;
    const uint8_t TFESC = 0xDD;

    enum State {WAIT_FBEGIN, WAIT_FRAME_TYPE, WAIT_FEND, WAIT_ESCAPED};

    State state = WAIT_FBEGIN;

    hdlc::IoFrame* frame = hdlc::acquire_wait();

    HAL_UART_Receive_IT(&huart3, &rxBuffer, 1);

    uint32_t last_sent_time = osKernelSysTick();
    uint32_t current_sent_time = 0;

    while (true) {
        osEvent evt = osMessageGet(serialPort->queue(), osWaitForever);

        if (evt.status != osEventMessage) {
            HAL_UART_Receive_IT(&huart3, &rxBuffer, 1);
            continue;
        }

        if (evt.value.v & 0x100) {
            hdlc::release(frame);
            ERROR("UART Error: %08lx", evt.value.v);
            uart_error.store(HAL_UART_ERROR_NONE);
            frame = hdlc::acquire_wait();
            HAL_UART_Receive_IT(&huart3, &rxBuffer, 1);
            continue;
        }

        uint8_t c = evt.value.v;
        INFO("%02x - %c", c, c);
        switch (state) {
        case WAIT_FBEGIN:
            if (c == FEND) state = WAIT_FRAME_TYPE;
            break;
        case WAIT_FRAME_TYPE:
            if (c == FEND) break;   // Still waiting for FRAME_TYPE.
            frame->type(c);
            state = WAIT_FEND;
            break;
        case WAIT_FEND:
            switch (c) {
            case FESC:
                state = WAIT_ESCAPED;
                break;
            case FEND:
                frame->source(hdlc::IoFrame::SERIAL_DATA);
                if (osMessagePut(ioEventQueueHandle, reinterpret_cast<uint32_t>(frame), osWaitForever) != osOK)
                {
                    hdlc::release(frame);
                }
                current_sent_time = osKernelSysTick();
                if (last_sent_time + 50 > current_sent_time) {
                    uint32_t delay = (last_sent_time + 50) - current_sent_time;
                    osDelay(delay);
                }
                frame = hdlc::acquire_wait();
                state = WAIT_FBEGIN;
                break;
            default:
                if (not frame->push_back(c)) {
                    hdlc::release(frame);
                    state = WAIT_FBEGIN;  // Drop frame;
                    frame = hdlc::acquire_wait();
                }
            }
            break;
        case WAIT_ESCAPED:
            state = WAIT_FEND;
            switch (c) {
            case TFESC:
                if (not frame->push_back(FESC)) {
                    hdlc::release(frame);
                    state = WAIT_FBEGIN;  // Drop frame;
                    frame = hdlc::acquire_wait();
                }
                break;
            case TFEND:
                if (not frame->push_back(FEND)) {
                    hdlc::release(frame);
                    state = WAIT_FBEGIN;  // Drop frame;
                    frame = hdlc::acquire();
                }
                break;
            default:
                hdlc::release(frame);
                state = WAIT_FBEGIN;  // Drop frame;
                frame = hdlc::acquire_wait();
            }
            break;
        }
    }
}


extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef*)
{
    txDoneFlag = true;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *)
{
    osMessagePut(mobilinkd::tnc::getSerialPort()->queue(), (uint32_t) rxBuffer, 0);

    HAL_UART_Receive_IT(&huart3, &rxBuffer, 1);
}


extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    osMessagePut(mobilinkd::tnc::getSerialPort()->queue(),
        (uint32_t) (huart->gState<<16) | huart->ErrorCode | 0x100, 0);
    uart_error.store((huart->gState<<16) | huart->ErrorCode);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_READY;
}

namespace mobilinkd { namespace tnc {

void SerialPort::init()
{
    if (serialTaskHandle_) return;

    osMessageQDef(uartQueue, 32, void*);
    queue_ = osMessageCreate(osMessageQ(uartQueue), 0);

    osMutexDef(uartMutex);
    mutex_ = osMutexCreate(osMutex(uartMutex));

    osThreadDef(serialTask, startSerialTask, osPriorityAboveNormal, 0, 128);
    serialTaskHandle_ = osThreadCreate(osThread(serialTask), this);
    DEBUG("serialTaskHandle_ = %p", serialTaskHandle_);
}

bool SerialPort::open()
{
    if (open_ or !serialTaskHandle_) return open_;

    open_ = true;
    return open_;
}

void SerialPort::close()
{
    open_ = false;
}

bool SerialPort::write(const uint8_t* data, uint32_t size, uint8_t type, uint32_t timeout)
{
    if (!open_) return false;

    uint32_t start = osKernelSysTick();

    if (osMutexWait(mutex_, timeout) != osOK)
        return false;

    using ::mobilinkd::tnc::kiss::slip_encoder;

    auto slip_iter = slip_encoder((const char*)data, size);
    auto slip_end = slip_encoder();

    size_t pos = 0;
    memset(tmpBuffer, 0, TX_BUFFER_SIZE);

    tmpBuffer[pos++] = 0xC0;   // FEND
    tmpBuffer[pos++] = type;   // KISS Data Frame

    while (slip_iter != slip_end) {
        tmpBuffer[pos++] = *slip_iter++;
        if (pos == TX_BUFFER_SIZE) {

            while (!txDoneFlag) osThreadYield();
            memcpy(TxBuffer, tmpBuffer, TX_BUFFER_SIZE);
            txDoneFlag = false;

            while (open_ and HAL_UART_Transmit_DMA(&huart3, TxBuffer, TX_BUFFER_SIZE) == HAL_BUSY)
            {
                if (osKernelSysTick() > start + timeout) {
                    osMutexRelease(mutex_);
                    txDoneFlag = true;
                    return false;
                }
                osThreadYield();
            }
            pos = 0;
            memset(TxBuffer, 0, TX_BUFFER_SIZE);
        }
    }

    // Buffer has room for at least one more byte.
    tmpBuffer[pos++] = 0xC0;
    while (!txDoneFlag) osThreadYield();
    memcpy(TxBuffer, tmpBuffer, pos);
    txDoneFlag = false;
    while (open_ and HAL_UART_Transmit_DMA(&huart3, TxBuffer, pos) == HAL_BUSY)
    {
        if (osKernelSysTick() > start + timeout) {
            osMutexRelease(mutex_);
            txDoneFlag = true;
            return false;
        }
        osThreadYield();
    }

    osMutexRelease(mutex_);

    return true;
}

bool SerialPort::write(const uint8_t* data, uint32_t size, uint32_t timeout)
{
    if (!open_) return false;

    uint32_t start = osKernelSysTick();

    if (osMutexWait(mutex_, timeout) != osOK)
        return false;

    size_t pos = 0;
    memset(TxBuffer, 0, TX_BUFFER_SIZE);

    auto first = data;
    auto last = data + size;

    while (first != last) {
        TxBuffer[pos++] = *first++;
        if (pos == TX_BUFFER_SIZE) {
            while (open_ and HAL_UART_Transmit(&huart3, TxBuffer, TX_BUFFER_SIZE, timeout) == HAL_BUSY)
            {
                if (osKernelSysTick() > start + timeout) {
                    osMutexRelease(mutex_);
                    return false;
                }
                osThreadYield();
            }
            pos = 0;
            memset(TxBuffer, 0, TX_BUFFER_SIZE);
        }
    }

    while (open_ and HAL_UART_Transmit(&huart3, TxBuffer, TX_BUFFER_SIZE, timeout) == HAL_BUSY)
    {
        if (osKernelSysTick() > start + timeout) {
            osMutexRelease(mutex_);
            return false;
        }
        osThreadYield();
    }

    while (open_ and HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, timeout) == HAL_BUSY)
    {
        if (osKernelSysTick() > start + timeout) {
            osMutexRelease(mutex_);
            return false;
        }
        osThreadYield();
    }

    osMutexRelease(mutex_);

    return true;
}

/*
 * Abort the DMA transmission. Release the mutex and the frame.  Set the
 * txDoneFlag so other writes may be attempted.
 *
 * This really sucks. The BM78 seems to just give up the ghost in BLE mode
 * when connected for long periods of time (and long is relative, but
 * typically more than an hour).  To deal with this, we reset the device
 * and the client needs to attempt to reconnect when disconnection is
 * detected.
 */
bool SerialPort::abort_tx(hdlc::IoFrame* frame)
{
    HAL_UART_AbortTransmit(&huart3);
    hdlc::release(frame);
    WARN("SerialPort::write timed out -- DMA aborted.");
    HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin, GPIO_PIN_SET);
    bm78_wait_until_ready();
    txDoneFlag = true;
    osMutexRelease(mutex_);
    return false;
}

bool SerialPort::write(hdlc::IoFrame* frame, uint32_t timeout)
{
    DEBUG("SerialPort::write sending frame");

    if (!open_) {
        WARN("SerialPort::write not open");
        hdlc::release(frame);
        return false;
    }

    uint32_t start = osKernelSysTick();

    if (osMutexWait(mutex_, timeout) != osOK) {
        WARN("SerialPort::write timed out");
        hdlc::release(frame);
        return false;
    }

    using ::mobilinkd::tnc::kiss::slip_encoder2;

    hdlc::IoFrame::iterator begin = frame->begin();
    hdlc::IoFrame::iterator end = frame->begin();
    std::advance(end, frame->size() - 2);           // Drop FCS

    auto slip_iter = slip_encoder2(begin);
    auto slip_end = slip_encoder2(end);

    size_t pos = 0;

    tmpBuffer[pos++] = 0xC0;   // FEND
    tmpBuffer[pos++] = static_cast<int>(frame->type());   // KISS Data Frame

    while (slip_iter != slip_end)
    {
        tmpBuffer[pos++] = *slip_iter++;
        if (pos == TX_BUFFER_SIZE) {
            while (!txDoneFlag) {
                // txDoneFlag set in HAL_UART_TxCpltCallback() above when DMA completes.
                if (osKernelSysTick() > (start + timeout)) {
                    return abort_tx(frame); // Abort DMA xfer on timeout.
                } else {
                    osThreadYield();
                }
            }
            memcpy(TxBuffer, tmpBuffer, TX_BUFFER_SIZE);
            txDoneFlag = false;
            while (open_ and HAL_UART_Transmit_DMA(&huart3, TxBuffer, TX_BUFFER_SIZE) == HAL_BUSY)
            {
                // This should not happen.  HAL_BUSY should not occur when txDoneFlag set.
                if (osKernelSysTick() > (start + timeout)) {
                    return abort_tx(frame); // Abort DMA xfer on timeout.
                } else {
                    osThreadYield();
                }
            }
            pos = 0;
        }
    }

    // Buffer has room for at least one more byte.
    tmpBuffer[pos++] = 0xC0;

    while (!txDoneFlag) {
        // txDoneFlag set in HAL_UART_TxCpltCallback() above when DMA completes.
        if (osKernelSysTick() > (start + timeout)) {
            return abort_tx(frame); // Abort DMA xfer on timeout.
        } else {
            osThreadYield();
        }
    }

    memcpy(TxBuffer, tmpBuffer, TX_BUFFER_SIZE);
    txDoneFlag = false;
    while (open_ and HAL_UART_Transmit_DMA(&huart3, TxBuffer, pos) == HAL_BUSY) {
        // This should not happen.  HAL_BUSY should not occur when txDoneFlag set.
        if (osKernelSysTick() > (start + timeout)) {
            return abort_tx(frame); // Abort DMA xfer on timeout.
        } else {
            osThreadYield();
        }
    }

    osMutexRelease(mutex_);
    hdlc::release(frame);

    DEBUG("SerialPort::write COMPLETE");

    return true;
}


SerialPort* getSerialPort()
{
    static SerialPort instance;
    return &instance;
}

}} // mobilinkd::tnc

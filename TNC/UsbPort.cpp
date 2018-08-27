// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include <Log.h>
#include "UsbPort.hpp"
#include "HdlcFrame.hpp"
#include "Kiss.hpp"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "cmsis_os.h"

extern "C" void TNC_Error_Handler(int dev, int err);

extern osMessageQId ioEventQueueHandle;

extern "C" void cdc_receive(const uint8_t* buf, uint32_t len) {
  using namespace mobilinkd::tnc::hdlc;
  if (mobilinkd::tnc::getUsbPort()->queue() != 0) {
    auto frame = mobilinkd::tnc::hdlc::ioFramePool().acquire();
    if (frame) {
      for (uint32_t i = 0; i != len; ++i) {
        frame->push_back(*buf++);
      }
      frame->source(IoFrame::SERIAL_DATA);
      if (osMessagePut(mobilinkd::tnc::getUsbPort()->queue(), (uint32_t) frame,
        osWaitForever) != osOK) {
        mobilinkd::tnc::hdlc::release(frame);
      }
    }
  }
}


extern "C" void startCDCTask(void const* arg)
{
    using namespace mobilinkd::tnc;

    auto usbPort = static_cast<const UsbPort*>(arg);

    const uint8_t FEND = 0xC0;
    const uint8_t FESC = 0xDB;
    const uint8_t TFEND = 0xDC;
    const uint8_t TFESC = 0xDD;

    uint8_t frame_type = kiss::FRAME_DATA;

    enum State {WAIT_FBEGIN, WAIT_FRAME_TYPE, WAIT_FEND, WAIT_ESCAPED};

    State state = WAIT_FBEGIN;

    hdlc::IoFrame* frame = hdlc::ioFramePool().acquire();

    while (true) {
        osEvent evt = osMessageGet(usbPort->queue(), osWaitForever);
        if (evt.status != osEventMessage) {
            continue;
        }

        auto input = static_cast<hdlc::IoFrame*>(evt.value.p);

        if (!usbPort->isOpen()) {
            hdlc::release(input);
            continue;
        }

        for (auto& c : *input) {
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
                    osMessagePut(ioEventQueueHandle, reinterpret_cast<uint32_t>(frame), osWaitForever);
                    frame = hdlc::ioFramePool().acquire();
                    state = WAIT_FBEGIN;
                    break;
                default:
                    if (not frame->push_back(c)) {
                        hdlc::release(frame);
                        state = WAIT_FBEGIN;  // Drop frame;
                        frame = hdlc::ioFramePool().acquire();
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
                        frame = hdlc::ioFramePool().acquire();
                    }
                    break;
                case TFEND:
                    if (not frame->push_back(FEND)) {
                        hdlc::release(frame);
                        state = WAIT_FBEGIN;  // Drop frame;
                        frame = hdlc::ioFramePool().acquire();
                    }
                    break;
                default:
                    hdlc::release(frame);
                    state = WAIT_FBEGIN;  // Drop frame;
                    frame = hdlc::ioFramePool().acquire();
                }
                break;
            }
        }
        hdlc::release(input);
    }
}

namespace mobilinkd { namespace tnc {

void UsbPort::init()
{
    if (cdcTaskHandle_) return;

    osMessageQDef(cdcQueue, 4, void*);
    queue_ = osMessageCreate(osMessageQ(cdcQueue), 0);

    osMutexDef(usbMutex);
    mutex_ = osMutexCreate(osMutex(usbMutex));

    osThreadDef(cdcTask, startCDCTask, osPriorityNormal, 0, 128);
    cdcTaskHandle_ = osThreadCreate(osThread(cdcTask), this);
}

bool UsbPort::open()
{
    if (open_ or !cdcTaskHandle_) return open_;

    open_ = true;
    return open_;
}

void UsbPort::close()
{
    open_ = false;
}

bool UsbPort::write(const uint8_t* data, uint32_t size, uint8_t type, uint32_t timeout)
{
    if (!open_) return false;

    uint32_t start = osKernelSysTick();

    if (osMutexWait(mutex_, timeout) != osOK)
        return false;

    using ::mobilinkd::tnc::kiss::slip_encoder;

    auto slip_iter = slip_encoder((const char*)data, size);
    auto slip_end = slip_encoder();

    size_t pos = 0;

    TxBuffer[pos++] = 0xC0;   // FEND
    TxBuffer[pos++] = type;   // KISS Data Frame

    while (slip_iter != slip_end) {
        TxBuffer[pos++] = *slip_iter++;
        if (pos == TX_BUFFER_SIZE) {
            while (open_ and CDC_Transmit_FS(TxBuffer, pos) == USBD_BUSY) {
                if (osKernelSysTick() > start + timeout) {
                    osMutexRelease(mutex_);
                    return false;
                }
                osThreadYield();
            }
            pos = 0;
        }
    }

    // Buffer has room for at least one more byte.
    TxBuffer[pos++] = 0xC0;
    while (open_ and CDC_Transmit_FS(TxBuffer, pos) == USBD_BUSY)
        osThreadYield();

    osMutexRelease(mutex_);

    return true;
}

bool UsbPort::write(const uint8_t* data, uint32_t size, uint32_t timeout)
{
    if (!open_) return false;

    uint32_t start = osKernelSysTick();

    if (osMutexWait(mutex_, timeout) != osOK)
        return false;

    size_t pos = 0;

    auto first = data;
    auto last = data + size;

    while (first != last) {
        TxBuffer[pos++] = *first++;
        if (pos == TX_BUFFER_SIZE) {
            while (open_ and CDC_Transmit_FS(TxBuffer, pos) == USBD_BUSY) {
                if (osKernelSysTick() > start + timeout) {
                    osMutexRelease(mutex_);
                    return false;
                }
                osThreadYield();
            }
            pos = 0;
        }
    }

    while (open_ and CDC_Transmit_FS(TxBuffer, pos) == USBD_BUSY) {
        if (osKernelSysTick() > start + timeout) {
            osMutexRelease(mutex_);
            return false;
        }
        osThreadYield();
    }
    while (open_ and CDC_Transmit_FS((uint8_t*)"\r\n", 2) == USBD_BUSY) {
        if (osKernelSysTick() > start + timeout) {
            osMutexRelease(mutex_);
            return false;
        }
        osThreadYield();
    }

    osMutexRelease(mutex_);

    return true;
}

bool UsbPort::transmit_buffer(size_t pos, uint32_t start, uint32_t timeout)
{
  while (open_ and CDC_Transmit_FS(TxBuffer, pos) == USBD_BUSY) {
    if ((timeout != osWaitForever) and
      (osKernelSysTick() > start + timeout))
    {
      if (USBD_LL_IsStallEP(
        static_cast<USBD_HandleTypeDef*>(hUsbDeviceFS.pClassData),
        CDC_IN_EP))
      {
        USBD_LL_FlushEP(
          static_cast<USBD_HandleTypeDef*>(hUsbDeviceFS.pClassData),
          CDC_IN_EP);
        USBD_LL_ClearStallEP(
          static_cast<USBD_HandleTypeDef*>(hUsbDeviceFS.pClassData),
          CDC_IN_EP);
      }
      return false;
    }
    osThreadYield();
  }
  return true;
}

bool UsbPort::write(hdlc::IoFrame* frame, uint32_t timeout)
{
    if (!open_) {
        hdlc::release(frame);
        return false;
    }

    uint32_t start = osKernelSysTick();

    if (osMutexWait(mutex_, timeout) != osOK) {
      hdlc::release(frame);
      TNC_Error_Handler(6,0);
      return false;
    }

    using ::mobilinkd::tnc::kiss::slip_encoder2;

    hdlc::IoFrame::iterator begin = frame->begin();
    hdlc::IoFrame::iterator end = frame->begin();
    std::advance(end, frame->size() - 2);           // Drop FCS

    auto slip_iter = slip_encoder2(begin);
    auto slip_end = slip_encoder2(end);

    size_t pos = 0;

    TxBuffer[pos++] = 0xC0;   // FEND
    TxBuffer[pos++] = static_cast<int>(frame->type());   // KISS Data Frame

    auto result = true;

    while (result and (slip_iter != slip_end)) {
      TxBuffer[pos++] = *slip_iter++;
      if (pos == TX_BUFFER_SIZE) {
        result = transmit_buffer(pos, start, timeout);
        pos = 0;
      }
    }

    if (result) {
      // Buffer has room for at least one more byte.  It cannot be full here.
      TxBuffer[pos++] = 0xC0;
      result = transmit_buffer(pos, start, timeout);
    }

    hdlc::release(frame);
    osMutexRelease(mutex_);
    return result;
}


UsbPort* getUsbPort()
{
    static UsbPort instance;
    return &instance;
}

}} // mobilinkd::tnc

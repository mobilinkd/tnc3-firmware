// Copyright 2016 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC__USB_PORT_HPP_
#define MOBILINKD__TNC__USB_PORT_HPP_

#include "PortInterface.hpp"
namespace mobilinkd { namespace tnc {

/**
 * This interface defines the semi-asynchronous interface used for reading
 * and writing via the USB CDC interface.
 *
 * init() must be called once and only once.  It must be called before the
 * USB device is started.
 */
struct UsbPort : PortInterface
{
    virtual ~UsbPort() {}
    virtual bool open();
    virtual bool isOpen() const { return open_; }

    virtual void close();
    virtual osMessageQId queue() const { return queue_; }
    virtual bool write(const uint8_t* data, uint32_t size, uint8_t type,
        uint32_t timeout);
    virtual bool write(const uint8_t* data, uint32_t size, uint32_t timeout);
    virtual bool write(hdlc::IoFrame* frame, uint32_t timeout = osWaitForever);

    void init();

private:
    bool transmit_buffer(size_t pos, uint32_t start, uint32_t timeout);

    bool open_{false};                  // opened/closed
    osMutexId mutex_{0};                // TX Mutex
    osMessageQId queue_{0};             // ISR read queue
    osThreadId cdcTaskHandle_{0};       // CDC read handler
};

UsbPort* getUsbPort();

}} // mobilinkd::tnc

#endif // MOBILINKD__TNC__USB_PORT_HPP_

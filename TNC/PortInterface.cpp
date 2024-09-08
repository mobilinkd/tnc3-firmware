// Copyright 2016-2024 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "PortInterface.hpp"
#include "SerialPort.hpp"
#ifdef TNC_HAS_USB
#include "UsbPort.hpp"
#endif
#include "NullPort.hpp"

#include <algorithm>
#include <cstring>

namespace mobilinkd { namespace tnc {

uint8_t TxBuffer[TX_BUFFER_SIZE];
PortInterface* ioport = nullptr;


int write(hdlc::IoFrame* frame, uint32_t timeout)
{
    if (mobilinkd::tnc::ioport == nullptr) return -1;

    return mobilinkd::tnc::ioport->write(frame, timeout);
}

}} // mobilinkd::tnc

int writeLog(const uint8_t* data, uint32_t size, uint32_t timeout)
{
    if (mobilinkd::tnc::ioport == nullptr) return -1;
    return mobilinkd::tnc::ioport->write(data, size, 7, timeout);
}

int writeTNC(const uint8_t* data, uint32_t size, uint32_t timeout)
{
    if (mobilinkd::tnc::ioport == nullptr) return -1;
    return mobilinkd::tnc::ioport->write(data, size, timeout);
}

int printTNC(const char* zstring, uint32_t timeout)
{
    if (mobilinkd::tnc::ioport == nullptr) return -1;
    return mobilinkd::tnc::ioport->write((uint8_t*) zstring, strlen(zstring), timeout);
}

void init_ioport()
{
    mobilinkd::tnc::ioport = mobilinkd::tnc::getNullPort();
    initNull();
}

#ifdef TNC_HAS_USB
void initCDC()
{
    mobilinkd::tnc::getUsbPort()->init();
}

int openCDC()
{
    mobilinkd::tnc::getSerialPort()->close();
    mobilinkd::tnc::PortInterface* tmp = mobilinkd::tnc::getUsbPort();
    tmp->open();
    if (mobilinkd::tnc::ioport != tmp and tmp->isOpen())
    {
        std::swap(tmp, mobilinkd::tnc::ioport);
        if (tmp) tmp->close();
        return true;
    }
    return mobilinkd::tnc::ioport == tmp;
}

void closeCDC()
{
    openNull();
}

int writeCDC(const uint8_t* data, uint32_t size, uint32_t timeout)
{
    return mobilinkd::tnc::getUsbPort()->write(data, size, timeout);
}
#endif

void initSerial()
{
    mobilinkd::tnc::getSerialPort()->init();
}

int openSerial()
{
#ifndef NUCLEOTNC
    mobilinkd::tnc::getUsbPort()->close();
#endif
    mobilinkd::tnc::PortInterface* tmp = mobilinkd::tnc::getSerialPort();
    tmp->open();
    if (mobilinkd::tnc::ioport != tmp and tmp->isOpen())
    {
        std::swap(tmp, mobilinkd::tnc::ioport);
        if (tmp) tmp->close();
        return true;
    }
    return mobilinkd::tnc::ioport == tmp;
}

void closeSerial()
{
    openNull();
}

int writeSerial(const uint8_t* data, uint32_t size, uint32_t timeout)
{
    return mobilinkd::tnc::getSerialPort()->write(data, size, timeout);
}

void initNull()
{
    mobilinkd::tnc::getNullPort()->init();
}

int openNull()
{
    auto tmp = mobilinkd::tnc::getNullPort();
    if (mobilinkd::tnc::ioport != tmp and tmp->isOpen())
    {
        if (mobilinkd::tnc::ioport) mobilinkd::tnc::ioport->close();
        mobilinkd::tnc::ioport = tmp;
        return true;
    }
    return mobilinkd::tnc::ioport == tmp;
}

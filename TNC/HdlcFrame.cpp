// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#include "HdlcFrame.hpp"
#include "Log.h"
#include "cmsis_os.h"

namespace mobilinkd { namespace tnc { namespace hdlc {

FrameSegmentPool frameSegmentPool __attribute__((section(".bss2")));

IoFramePool& ioFramePool() {
    static IoFramePool pool;
    return pool;
}

void release(IoFrame* frame)
{
    ioFramePool().release(frame);
//    printf("< %d\r\n", ioFramePool().size());
}

IoFrame* acquire()
{
    auto result = ioFramePool().acquire();
    if (result == nullptr) CxxErrorHandler();
//    printf("> %d\r\n", ioFramePool().size());
    return result;
}


IoFrame* acquire_wait()
{
    IoFrame* result = nullptr;
    while ((result = ioFramePool().acquire()) == nullptr) {
        osThreadYield();
    }
//    printf("> %d\r\n", ioFramePool().size());
    return result;
}

}}} // mobilinkd::tnc::hdlc

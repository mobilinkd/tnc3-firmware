// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Digipeater.hpp"
#include "Digipeater.h"
#include "IOEventTask.h"
#include "HdlcFrame.hpp"

#include <algorithm>
#include <iterator>

extern osMessageQId hdlcOutputQueueHandle;

/*
 * APRS Digipeater implementation.
 *
 * Note that these rules are for digipeating only.  We have other rules for
 * packets sent to this station directly.
 *
 * 0. Ignore all packets that are not UI packets.
 * 1. Ignore all packets with destination address == MYCALL or any ALIASES.
 * 2. Compute CRC32 on source and destination address, and information fields.
 * 3. Ignore all packets with same CRC32 heard in past 30 seconds.
 * 4. Ignore all digipeated VIA entries.
 * 5. Find VIA entries matching preempted aliases.
 * 5.1. http://www.aprs.org/aprs12/preemptive-digipeating.txt
 * 6. Find first non-digipeated VIA entry.
 * 6.1. If it matches one of our set and used aliases, relay.
 */

void startDigipeaterTask(void* arg)
{
  using mobilinkd::tnc::Digipeater;
  using mobilinkd::tnc::hdlc::IoFrame;

  auto digi = static_cast<Digipeater*>(arg);
  for(;;)
  {
    osEvent evt = osMessageGet(digipeaterQueueHandle, osWaitForever);
    if (evt.status != osEventMessage) continue;

    uint32_t cmd = evt.value.v;
    if (cmd < FLASH_BASE) // Assumes FLASH_BASE < SRAM_BASE.
    {
      // this is a command, not a packet.
      return;
    }

    digi->clean_history();

    auto frame = static_cast<IoFrame*>(evt.value.p);

    if (!digi->can_repeat(frame)) continue;

    auto digi_frame = digi->rewrite_frame(frame);

  }
}

const uint8_t test_beacon[] = {
    0x82, 0xa0, 0xb0, 0x64, 0x60, 0x70, 0x60, 0xae,
    0xb0, 0x72, 0x9e, 0x40, 0x40, 0x66, 0xae, 0x92,
    0x88, 0x8a, 0x64, 0x40, 0x63, 0x03, 0xf0, 0x3d,
    0x34, 0x31, 0x35, 0x34, 0x2e, 0x37, 0x20, 0x4e,
    0x2f, 0x30, 0x38, 0x37, 0x34, 0x31, 0x2e, 0x32,
    0x20, 0x57, 0x33, 0x50, 0x48, 0x47, 0x33, 0x32,
    0x36, 0x30, 0x2d, 0x4d, 0x6f, 0x62, 0x69, 0x6c,
    0x69, 0x6e, 0x6b, 0x64, 0x20, 0x50, 0x72, 0x6f,
    0x74, 0x6f, 0x74, 0x79, 0x70, 0x65, 0x20, 0x42,
    0x4c, 0x45, 0x20, 0x54, 0x4e, 0x43, 0x0d
};

void beacon(void* arg)
{
    using namespace mobilinkd::tnc;

    auto frame = hdlc::acquire_wait();

    std::copy(test_beacon, test_beacon + sizeof(test_beacon),
        std::back_inserter(*frame));

    if (osMessagePut(hdlcOutputQueueHandle,
        reinterpret_cast<uint32_t>(frame),
        osWaitForever) != osOK)
    {
        ERROR("Failed to write frame to TX queue");
        hdlc::release(frame);
    }
}

namespace mobilinkd { namespace tnc {

}}  // mobilinkd::tnc

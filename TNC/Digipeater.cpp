// Copyright 2017 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "Digipeater.hpp"
#include "Digipeater.h"
#include "IOEventTask.h"

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

void beacon(void* arg)
{

}

namespace mobilinkd { namespace tnc {

}}  // mobilinkd::tnc

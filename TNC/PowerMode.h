// Copyright 2018 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#ifndef TNC_POWERMODE_H_
#define TNC_POWERMODE_H_

enum class PowerMode {
    RUN,        // Power on, connected.
    LPRUN,      // Power on, not connected.
    SLEEP,      // Power off, USB connected.
    STOP        // Power off, no USB connection.
};

extern volatile PowerMode power_mode;

#endif // TNC_POWERMODE_H_

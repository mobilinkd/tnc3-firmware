This is the firmware for the TNC3 version 2.1.1 hardware.

# Building

Use Eclipse with CDT and the GNU MCU Eclipse plugins.



# Debugging

Logging is enabled in debug builds and is output via ITM (SWO).  The
firmware is distributed with an openocd stlink config file that enables
ITM output to a named pipe -- `swv`.

To read from this pipe, open a terminal and run:

`while true; do tr -d '\01' < swv; done`

If you change the MCU's core clock, you need to adjust the timing in the
`stlink-tnc3.cfg` config file.


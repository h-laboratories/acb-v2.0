# ACB v2.0 Firmware
This repo contains the firmware for the ACB v2.0.

*The latest .bin for production firmware is in the release section on the right.*

**When compiling in the Arduino IDE make sure to use the settings outlined in [#arduino-ide-settings](Arduino IDE Settings)**

## Example firmware
There are also several firmware examples in the `/examples` dir. These examples are created using the Arduino IDE and can be programmed to the ACB v2.0 via one of two methods:
1) By using the ACB debugger mount + STLink.
2) By flashing via USB as you would regular ACB firmware.

The debugger mount allows rapid debugging directly in the Arduino IDE instead of needing to boot the ACB into DFU mode.

## Arduino IDE Settings

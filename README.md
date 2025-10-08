# ACB v2.0 Firmware
This repo contains the firmware for the ACB v2.0. The latest .bin for production firmware is in the release section on the right.

**When compiling in the Arduino IDE make sure to use the settings outlined in [Arduino IDE Settings](#arduino-ide-settings)**

## Example firmware
There are also several firmware examples in the `/examples` dir. These examples are created using the Arduino IDE and can be programmed to the ACB v2.0 via one of two methods:
1) By using the ACB debugger mount + STLink.
2) By flashing via USB as you would regular ACB firmware.

The debugger mount allows rapid debugging directly in the Arduino IDE instead of needing to boot the ACB into DFU mode.

## Arduino IDE Settings
Make sure to install the STM32 & Simple FOC Libraries in the Arduino IDE. In the Tools menu select:
1. Board > STM32 based boards > Generic STM32G4 series
2. Board part number > Generic "G474RETX"
3. USB Support > CDC (generic 'Serial' supersede U(S)ART)
4. U(S)ART support > Disabled 

## TODO
- [ ] Fix exception handling for higher voltages
- [ ] Current filtering
- [ ] Auto PID tuning
- [ ] CAN port from STM IDE implementation
- [ ] Allow PWM frequency updates
- [ ] Warnings for high/low encoder sensing
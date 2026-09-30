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

## CAN bootloader (reprogram over CAN)
The first 32 KB of flash hold a small CAN bootloader (`src/acb_can_bootloader`) so any ACB on the bus can be reflashed without touching USB or BOOT0. The main firmware includes `can_boot.cpp`, which answers `PING` and reboots into the bootloader on `ENTER`. Protocol and IDs are in `src/acb_can_bootloader/acb_can_protocol.h`; the PC side is `tools/acb_can_flash.py` (needs a slcan adapter such as a CANable and `pip install python-can pyserial`).

Flash layout:

| Range | Contents |
|---|---|
| `0x08000000 - 0x08006FFF` | bootloader (build with `upload.maximum_size=28672`) |
| `0x08007000 - 0x08007FFF` | boot config page (CAN node id, default 1) |
| `0x08008000 - 0x0807CFFF` | application (build with `build.flash_offset=0x8000`) |
| `0x0807D000 - 0x0807EFFF` | anti-cogging map (`src/acb_v2.0_firmware/cogging.cpp`) |
| `0x0807F000 - 0x0807FFFF` | reserved for the application's EEPROM emulation |

### One-time install (DFU)
```bash
python tools/build.py bootloader app --dfu --run
```
This builds both images with arduino-cli and writes them at their addresses with STM32CubeProgrammer. Alternatively flash `build/bootloader/acb_can_bootloader.ino.bin` at `0x08000000` and the app at `0x08008000` with any DFU/ST-Link tool.

### Everyday use
```bash
python tools/acb_can_flash.py scan                                    # list nodes on the bus
python tools/acb_can_flash.py flash build/app/acb_v2.0_firmware.ino.bin --node 1
python tools/build.py app --can --node 1                              # build + flash in one go
python tools/acb_can_flash.py set-node-id 3 --node 1                  # give a board a different id
```
`flash` asks the running application to reboot into the bootloader, erases, writes (about 15 KB/s at 500 kbit/s), verifies a CRC-32 and restarts the application. Without the `--stay` flag the board always returns to the application.

### Recovery
After every reset the bootloader listens for 300 ms before starting the application, and it stays resident if the application vector table is invalid. If a board does not answer, run `flash` or `enter` and power-cycle the board while the tool is waiting.

### Arduino IDE
To build the application from the IDE with the correct offset, copy `tools/boards.local.txt` next to the STM32 core's `boards.txt` (see the comment in that file) and pick the board part number **ACB v2.0 (G474RETx, CAN bootloader app)**. The IDE's DFU upload then writes at `0x08008000` and leaves the bootloader intact.

## Anti-cogging
`src/acb_v2.0_firmware/cogging.cpp` implements ODrive-style anti-cogging: a calibration steps the rotor through 2048 positions per revolution in angle mode, waits for position and velocity to settle at each, and records the current needed to hold it. Once enabled, the interpolated map value is added to the current setpoint as feed-forward torque in every closed-loop mode with a current torque controller. The map is keyed to the absolute rotor angle (incremental encoder plus the MA730 offset read at boot) and stored in flash at `0x0807D000` together with the pole-pair count and sensor direction it was measured with.

```bash
python tools/acb_can_flash.py cog calib --watch --node 1   # motor must be aligned; takes a few minutes
python tools/acb_can_flash.py cog status --node 1
python tools/acb_can_flash.py disable --node 1 && python tools/acb_can_flash.py cog save --node 1
python tools/acb_can_flash.py cog dump --out cogging.csv --node 1
python tools/acb_can_flash.py cog disable --node 1         # A/B compare against the raw motor
```
The equivalent USB serial commands are `cog_calib`, `cog_abort`, `cog_status`, `cog_enable`, `cog_disable` and `cog_save`.

## TODO
- [ ] Fix exception handling for higher voltages
- [ ] Current filtering
- [ ] Auto PID tuning
- [x] CAN bootloader (`src/acb_can_bootloader`, `tools/acb_can_flash.py`)
- [ ] CAN control protocol for the application (only PING / ENTER / reset so far)
- [ ] Allow PWM frequency updates
- [ ] Warnings for high/low encoder sensing
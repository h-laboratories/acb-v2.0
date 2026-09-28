# ACB v2.0 — PCB Overview

A summary of the components on the ACB v2.0 board and how they are wired into the STM32G474RET6, derived from [`src/acb_v2.0_firmware/config.h`](src/acb_v2.0_firmware/config.h), the driver modules, and the example sketches.

## Microcontroller

**STM32G474RET6** — Cortex-M4F @ 170 MHz, 512 KB flash, 128 KB SRAM, LQFP-64.

- Compiled against the Arduino STM32 core as a "Generic G474RETX".
- Serial console runs over USB CDC at **2,000,000 baud** (`SERIAL_BAUD_RATE`).
- DFU bootloader is reachable over USB; the dedicated `BUTTON` (PB8) triggers a `NVIC_SystemReset()` in firmware.
- Independent Watchdog (IWDG) configured for a 2 s timeout (`IWDG_TIMEOUT_MS`).
- Internal temperature + VREFINT used via ADC channels `ATEMP` (16) and `AVREF` (0) — factory calibration constants read from `TS_CAL1_ADDR` / `TS_CAL2_ADDR` / `VREFINT_CAL_ADDR`.

## Power stage

### DRV8323RSRGZR — 3-phase gate driver (SPI variant, integrated CSAs)

TI smart gate driver with built-in current-sense amplifiers, VDS/VGS protection, and SPI configuration.

| Signal      | MCU pin | Direction | Notes |
|-------------|---------|-----------|-------|
| `DRV_EN`    | PB9     | MCU → DRV | Enable line, brought HIGH at the end of `setup()`. |
| `DRV_CAL`   | PB7     | MCU → DRV | CSA offset calibration pulse (HIGH ≥100 µs during setup). |
| `DRV_FAULT` | PB10    | DRV → MCU | Open-drain fault, configured `INPUT_PULLUP`. |
| `SPI_CS_DRV`| PB5     | MCU → DRV | SPI chip select (`DRV8323_CS_PIN`). |

SPI registers (`0x00`–`0x06`) and bit positions are mapped in [`DRV8323RSRGZR.h`](src/acb_v2.0_firmware/DRV8323RSRGZR.h); fault status is decoded by `DRV8323RSRGZR::checkFaults()`.

### 6× MOSFET half-bridges driven by 6 PWM channels

| Phase | High-side | Low-side |
|-------|-----------|----------|
| A     | PC0 (`PWM_H_A`)  | PC13 (`PWM_L_A`) |
| B     | PC1 (`PWM_H_B`)  | PB0 (`PWM_L_B`)  |
| C     | PC2 (`PWM_H_C`)  | PB1 (`PWM_L_B`)  |

Driven by SimpleFOC's `BLDCDriver6PWM` at **20 kHz** (`DEFAULT_PWM_FREQUENCY`), Space Vector PWM modulation. Power-supply voltage is sampled live and the driver's `voltage_power_supply` is updated whenever it drifts by more than 0.1 V.

### Phase-current sensing

Three low-side shunts feed the DRV8323's internal CSAs, whose outputs come out to the MCU ADC pins:

| Channel | MCU pin |
|---------|---------|
| Iₐ      | PA0 (`CURR_A`) |
| I_b     | PA1 (`CURR_B`) |
| I_c     | PA2 (`CURR_C`) |

- Shunt: **8 mΩ** (`SHUNT_RESISTANCE`).
- CSA gain: **20 V/V** (`CURRENT_GAIN`).
- Configured as a SimpleFOC `LowsideCurrentSense` with `skip_align = true` to suppress startup motion.

## Position sensing

### MA730GQ — MPS magnetic absolute encoder (SPI)

| Signal     | MCU pin | Notes |
|------------|---------|-------|
| `SPI_CS_POS` | PB6   | Chip select (`MA730GQ_CS_PIN`). |
| `SPI_MOSI` | PA7     | Shared SPI bus. |
| `SPI_MISO` | PA6     | |
| `SPI_SCK`  | PA5     | |

- 16-bit absolute angle. The firmware uses the SPI absolute reading **at startup only**, to bias the FOC zero-electric-angle relative to a stored `absolute_angle_zero_calibration`.
- Driver: [`MA730GQ.cpp/.h`](src/acb_v2.0_firmware/MA730GQ.cpp) — implements raw angle, register R/W, and unit conversions.

### Quadrature encoder interface (ABZ from the MA730GQ)

The same MA730GQ also outputs ABZ pulses, wired into the STM32 as a regular quadrature input. At runtime FOC uses **this** path (not SPI) because it is interrupt-driven and zero-latency.

| Signal      | MCU pin | Notes |
|-------------|---------|-------|
| `ENCODER_A` | PA15    | Quadrature A, hardware interrupt. |
| `ENCODER_B` | PB3     | Quadrature B, hardware interrupt. |
| `ENCODER_Z` | PC10    | Index pulse (defined but no ISR currently attached in `acb_v2.0_firmware.ino`). |

`ENCODER_PPR = 1024`, quadrature ON (×4 → 4096 counts/rev).

## Shared SPI bus

The MA730GQ and DRV8323 share one SPI peripheral (`SPI1` on the G474). The bus is bit-banged at `SPI_CLOCK_DIV128`, MSB-first, with `SPI_MODE0` at the global `SPI.begin()`. The MA730GQ in particular wants Mode 3 — its driver toggles modes around its own transactions (see `MA730GQ.cpp` / `MA730GQ_README.md`).

## CAN bus

The G474 has **no classic bxCAN** — only **FDCAN**. On the G474RET6 the transceiver pins map to **FDCAN2** (not FDCAN1/"CAN1"):

| Signal       | MCU pin | STM32 alt. function |
|--------------|---------|---------------------|
| `FDCAN2_RX`  | PB12    | AF9 (`GPIO_AF9_FDCAN2`) |
| `FDCAN2_TX`  | PB13    | AF9 (`GPIO_AF9_FDCAN2`) |

The MCU drives an external CAN transceiver IC (transceiver part isn't named in the firmware — likely an SN65HVD230 / TJA1051-class 3.3 V transceiver based on the rest of the BOM). See [`examples/can_test/can_test.ino`](examples/can_test/can_test.ino) for the HAL-level setup: FDCAN2 in classic-CAN mode at 500 kbit/s, kernel clock taken from PCLK1 (there is no HSE), accept-all global filter into RX FIFO0. CAN integration into the main firmware is still an open TODO in the README.

## Board monitoring

### Bus voltage

`BUS_V` = **PB15**, fed by a `1 kΩ / 33 kΩ` divider (`BUS_VOLTAGE_DIVIDER = 1e3 / 34e3`).

Firmware clamps boot if outside **11.5 V – 30 V**.

### NTC thermistor (board temperature)

`TEMP` = **PA8**, 10 kΩ NTC + 10 kΩ fixed resistor in a divider. Steinhart-Hart with `B = 4200` (`TEMP_NTC_B_CONSTANT`), 25 °C nominal (`TEMP_NTC_NOMINAL = 10k`).

### Internal MCU temperature

ADC channel 16 + VREFINT on channel 0, scaled using STM32 factory calibration at `0x1FFF75A8` / `0x1FFF75CA` / `0x1FFF75AA`. `calculateInternalTemperature()` currently returns `-1` (TODO in source).

## User I/O

| Component   | MCU pin | Notes |
|-------------|---------|-------|
| Status LED  | PC6 (`STATUS_LED`) | Solid HIGH after boot. |
| Comms LED   | PC7 (`COM_LED`)    | Reserved for serial-activity indication. |
| Push button | PB8 (`BUTTON`)     | `INPUT_PULLUP`, falling-edge ISR with 50 ms debounce; triggers MCU reset. |

## Programming / debug

- **USB CDC** for normal use (serial + DFU flashing).
- **SWD via STLink** on the dedicated debugger mount (see README §"Example firmware").
- No external HSE crystal is configured — the example CAN sketch shows the PLL fed from **HSI** (16 MHz / 4 × 85 = 170 MHz `SYSCLK`).

## Configuration storage

`ACBConfig` (in [`config_manager.h`](src/acb_v2.0_firmware/config_manager.h)) is persisted to **emulated EEPROM** in flash. The block is validated by magic number `0xACB4` (`EEPROM_CONFIG_MAGIC_NUMBER`) at `EEPROM_CONFIG_START_ADDR = 0`. Stored fields include the three PID sets (velocity / angle / current), `pole_pairs`, min/max angle limits, FOC `zero_electric_angle` + `sensor_direction`, and the absolute-angle calibration offset.

## Pin map at a glance

```
PA0  ── CURR_A  (ADC, phase A current)
PA1  ── CURR_B  (ADC, phase B current)
PA2  ── CURR_C  (ADC, phase C current)
PA5  ── SPI_SCK
PA6  ── SPI_MISO
PA7  ── SPI_MOSI
PA8  ── TEMP    (ADC, NTC)
PA15 ── ENCODER_A
PB0  ── PWM_L_B
PB1  ── PWM_L_C
PB3  ── ENCODER_B
PB5  ── SPI_CS_DRV   (DRV8323)
PB6  ── SPI_CS_POS   (MA730GQ)
PB7  ── DRV_CAL
PB8  ── BUTTON
PB9  ── DRV_EN
PB10 ── DRV_FAULT
PB12 ── CAN1_RX
PB13 ── CAN1_TX
PB15 ── BUS_V   (ADC, bus voltage divider)
PC0  ── PWM_H_A
PC1  ── PWM_H_B
PC2  ── PWM_H_C
PC6  ── STATUS_LED
PC7  ── COM_LED
PC10 ── ENCODER_Z
PC13 ── PWM_L_A
```

## Default tuning constants

From [`config.h`](src/acb_v2.0_firmware/config.h):

- Motor: **19 pole-pairs** default, **4 V** driver voltage limit, motor voltage limit = ½ driver limit.
- PID defaults: velocity (0.25 / 1.0 / 0.001), angle (20 / 1 / 0), current (1.0 / 0.1 / 0.001).
- Angle limits: ±180°.
- Low-pass filter constants: 0.05 across velocity / angle / current_d / current_q.

## Notes worth remembering

- The DRV8323 fault line and the MA730GQ angle output are the two most useful diagnostic signals; both have firmware helpers (`drv8323_fault_check` command and the `getEncoderMagStatus` command — see `CommandManager`).
- Booting the board outside the **12–30 V** envelope calls `exit(1)` after a warning — there is no soft recovery, you must power-cycle.
- The on-startup absolute-angle correction is non-trivial: it reads the MA730GQ once, reads the quadrature `Encoder` once, and adjusts `motor.zero_electric_angle` modulo 2π. If absolute-angle calibration drifts, this is where to look (`acb_v2.0_firmware.ino` lines ~307–324).
- The DRV8323 and MA730GQ live on the **same SPI bus** with different preferred modes — be careful changing one driver's mode without restoring it.

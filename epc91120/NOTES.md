# EPC91120 bring-up notes

Board: EPC91120 3-phase GaN inverter for humanoid joints (3x EPC23102, STM32G431CBU6).
Quick start guide: https://epc-co.com/epc/portals/0/epc/documents/guides/EPC91120_qsg.pdf (v1.3, Aug 2025).
Debug probe: STLINK-V3MINIE (SN 003100323235510F37333439), VCP enumerates as COM9.

## From the quick start guide
- Input 15-55 V spec (UVLO 6 V); bench runs it at 12 V and the 3.3 V rail reads 3.28 V over SWD.
- Phase current sense: MCS1823-350BRN inline Hall sensors on phases U and V, 26.4 mV/A, 1.65 V offset, +-62.5 A range, over-current outputs wired-OR to the MCU.
- DC bus and phase voltage sense: resistor dividers, 44.89 mV/V.
- Encoder: magnetic, 1024 PPR ABZ with Z index plus SPI absolute (chip not named in the guide).
- RS485 port; 14-pin JTAG/STDC14 connector; Start/Stop and Reset buttons; green LED = 5 V, yellow LED = 3.3 V.
- Factory firmware: ST motor-control stack, 100 kHz PWM, 50 ns dead time, spins the (Unitree A1) motor at 50 rpm; controlled by ST's real-time GUI over the debug connector or via RS485.
- `factory_firmware_backup.bin` in this folder is the full 128 KB flash image read on 2026-09-30 (RDP level 0).

## Pin map decoded from live peripheral registers (factory firmware idle, SWD hot-plug)
| Function | Pin | Evidence |
|---|---|---|
| TIM1_CH1 / CH2 / CH3 (high-side PWM U/V/W) | PA8 / PA9 / PA10 | AF6, pull-down |
| TIM1_CH1N / CH2N / CH3N (low-side PWM) | PA7 / PB0 / PB1 | AF6 |
| TIM1 config | centre-aligned (CMS=3), ARR 1700, PSC 0, RCR 3, CKD=1, DTG=4 (~47 ns), BKP active-high, OSSR/OSSI set, CH4 as ADC trigger | TIM1 regs |
| VBUS sense | PA1 = ADC1_IN2 | ADC1 SQR1 = ch2, left-aligned DR 0x2A10 -> 0.54 V = 12.0 V at 44.89 mV/V |
| Start/Stop button | PC13 input | EXTI13 rising edge, SYSCFG EXTICR4 = port C |
| USART2 TX/RX (GUI link, likely RS485 and/or STDC14 VCP) | PA2 / PA3 | AF7, BRR 92 -> 1.8432 Mbaud, DMA TX/RX |
| SWD | PA13 / PA14 | default |
| Unused/analog | PA0, PA4, PA5, PA6, PA11, PA12, PB2, PB5-PB15, PC14, PC15 | MODER = analog |
| Left at JTAG defaults | PA15, PB3, PB4 | AF0 |

Not determinable from the idle dump (ST stack programs them only when running): the two current-sense ADC
channels (candidates PA0, PA4, PA5, PA6, PB2, PB11, PB12, PB14, PB15), TIM1 break/fault input, EPC23102 STB
(enable) pins, encoder ABZ/SPI pins (no encoder peripheral is enabled: the factory firmware is sensorless),
RS485 DE pin, LED pins. Next step: dump the same registers while the factory firmware is running, or get the
schematic from EPC.

// ACB v2.0 CAN bootloader protocol -- shared by the bootloader, the application
// (src/acb_v2.0_firmware/acb_can_protocol.h is an identical copy) and
// tools/acb_can_flash.py. Keep all three in sync.
//
// Classic CAN, 11-bit IDs, 500 kbit/s. Every node has a 7-bit node id (1..127,
// default 1, stored in the boot-config flash page). Node id 0 is broadcast.
//
//   host -> node   0x600 | node   command frame, byte0 = command
//   host -> node   0x580 | node   raw data frame (8 payload bytes) for WRITE
//   node -> host   0x680 | node   response frame, byte0 = command it answers
//
// Flash layout (STM32G474RET6, 512 KB):
//   0x08000000..0x08006FFF  bootloader          (28 KB max)
//   0x08007000..0x08007FFF  boot config page    (AcbBootCfg: node id)
//   0x08008000..0x0807EFFF  application         (link with build.flash_offset=0x8000)
//   0x0807F000..0x0807FFFF  reserved            (EEPROM emulation used by the app)
#pragma once
#include <stdint.h>

#define ACB_CAN_PROTO_VERSION   1
#define ACB_CAN_BITRATE         500000UL

#define ACB_CAN_ID_CMD_BASE     0x600u
#define ACB_CAN_ID_DATA_BASE    0x580u
#define ACB_CAN_ID_RESP_BASE    0x680u
#define ACB_CAN_ID_BASE_MASK    0x780u   // (id & mask) == one of the bases
#define ACB_CAN_NODE_MASK       0x07Fu
#define ACB_CAN_NODE_BROADCAST  0x00u
#define ACB_CAN_NODE_MAX        0x7Fu
#define ACB_CAN_DEFAULT_NODE_ID 1u

#define ACB_FLASH_BASE          0x08000000u
#define ACB_BOOT_CFG_ADDR       0x08007000u
#define ACB_APP_ADDR            0x08008000u
#define ACB_APP_END             0x0807F000u   // exclusive
#define ACB_MAX_CHUNK           2048u         // max bytes per WRITE_BEGIN

#define ACB_BOOTCFG_MAGIC       0xACB0CF61u
#define ACB_BOOT_MAGIC_STAY     0xB007CA11u   // in TAMP->BKP0R: stay in bootloader after reset

typedef struct {
  uint32_t magic;      // ACB_BOOTCFG_MAGIC
  uint8_t  node_id;    // 1..127
  uint8_t  reserved[3];
} AcbBootCfg;

// Commands (byte0 of a command frame; responses echo it in byte0)
#define ACB_CMD_HELLO        0x00  // node -> host, unsolicited when the bootloader becomes resident
#define ACB_CMD_PING         0x01  // -> [0x01, proto, version, flags, uid32 LE]
#define ACB_CMD_ERASE        0x02  // [addr LE32, len LE24]  -> [0x02, status]
#define ACB_CMD_WRITE_BEGIN  0x03  // [addr LE32, len LE16]  -> [0x03, status] only on error
#define ACB_CMD_WRITE_DONE   0x04  // -> [0x04, status, crc32 LE] after the chunk is programmed
#define ACB_CMD_CRC          0x05  // [addr LE32, len LE24]  -> [0x05, status, crc32 LE]
#define ACB_CMD_GO           0x06  // [mode] 0 = reset into app, 1 = reset and stay in bootloader -> [0x06, status]
#define ACB_CMD_SET_NODE_ID  0x07  // [id] -> [0x07, status, id]; node then resets into the bootloader
#define ACB_CMD_INFO         0x08  // -> [0x08, flash_kb LE16, page_bytes LE16, app_start_kb LE16, dbank]
#define ACB_CMD_ENTER        0x7F  // app: reset into bootloader (-> [0x7F, 0] first); bootloader: re-send HELLO

// Application control commands (handled by the application, can_control.cpp).
// Floats are IEEE-754 little-endian. Mode numbering follows SimpleFOC MotionControlType.
#define ACB_CMD_SET_MODE        0x10  // [mode] 0 torque, 1 velocity, 2 angle, 3 velocity_openloop, 4 angle_openloop -> [0x10, status, mode]
#define ACB_CMD_SET_TARGET      0x11  // [float32] rad/s, rad, or torque units depending on mode -> [0x11, status]
#define ACB_CMD_ENABLE          0x12  // [0|1] -> [0x12, status]
#define ACB_CMD_SET_POLE_PAIRS  0x13  // [pp] -> [0x13, status, pp]   (persist with SAVE_CONFIG)
#define ACB_CMD_SET_VLIMIT      0x14  // [float32 volts] motor phase voltage limit (<= bus/2) -> [0x14, status]
#define ACB_CMD_SAVE_CONFIG     0x15  // -> [0x15, status]
#define ACB_CMD_GET_STATE       0x20  // -> [0x20, mode | enabled<<4, velocity i16 (0.01 rad/s), angle i32 (1e-4 rad)]
#define ACB_CMD_GET_TELEMETRY   0x21  // -> [0x21, bus u16 (0.01 V), board temp i16 (0.1 C), mcu temp i16 (0.1 C), drv_fault]
#define ACB_CMD_DRV_CLEAR_FAULTS 0x16 // -> [0x16, status]   clear latched DRV8323 faults
#define ACB_CMD_DRV_WRITE_REG    0x17 // [addr, val LE16] -> [0x17, status, addr, readback LE16]
#define ACB_CMD_DRV_FAULTS       0x22 // -> [0x22, nFAULT pin asserted (1), FaultStatus1 LE16, VGSStatus2 LE16]
#define ACB_CMD_DRV_READ_REG     0x23 // [addr] -> [0x23, addr, val LE16]
#define ACB_CMD_SERIAL_STREAM    0x18 // [hz] print "pos_deg <deg> vel_rpm <rpm>" on USB serial at hz (0 = off) -> [0x18, status, hz]
#define ACB_CMD_RECALIBRATE      0x19 // [align_volts f32, optional] run sensor alignment (initFOC) -> [0x19, status, sensor_dir i8, zero_elec_angle f32]
#define ACB_CMD_SET_ILIMIT       0x1A // [float32 amps] motor current limit (velocity PID output in foc_current mode) -> [0x1A, status]
#define ACB_CMD_SET_TORQUE_MODE  0x1B // [0 voltage, 1 dc_current, 2 foc_current] -> [0x1B, status, mode]
#define ACB_CMD_GET_CURRENTS     0x24 // -> [0x24, ia mA i16, ib mA i16, ic mA i16, 0]   raw phase currents from the sense amps
#define ACB_CMD_GET_DQ           0x25 // -> [0x25, iq mA i16, id mA i16, uq mV i16, 0]  FOC currents / q voltage
#define ACB_CMD_DRV_SPI_CFG      0x1C // [mode 0-3, clk_khz LE16] SPI settings used for DRV8323 transactions -> [0x1C, status]
#define ACB_CMD_COG_CALIB        0x1D // [1 start | 0 abort, pos_thr counts u8, vel_thr (0.01 rad/s) u8, dwell ms u8, timeout (10 ms) u8] -> [0x1D, status]
#define ACB_CMD_COG_ENABLE       0x1E // [0|1] apply the anti-cogging map as current feed-forward -> [0x1E, status]
#define ACB_CMD_COG_SAVE         0x1F // -> [0x1F, status]  write the map to flash (motor must be disabled)
#define ACB_CMD_COG_STATUS       0x26 // -> [0x26, state (0 idle,1 calibrating,2 done,3 aborted), flags (valid | enabled<<1 | saved<<2), index LE16, n LE16, timeouts]
#define ACB_CMD_COG_GET          0x27 // [idx LE16] -> [0x27, idx LE16, map[idx] mA i16, map[idx+1] mA i16]

// PING / HELLO flags
#define ACB_FLAG_APP_VALID      0x01
#define ACB_FLAG_IN_BOOTLOADER  0x02

// Status codes
#define ACB_ST_OK            0
#define ACB_ST_ARG           1   // bad argument / address range
#define ACB_ST_FLASH         2   // erase/program failed
#define ACB_ST_STATE         3   // command not valid in this state
#define ACB_ST_VERIFY        4   // readback / CRC mismatch
#define ACB_ST_UNSUPPORTED   5
#define ACB_ST_NOT_IN_BL     6   // sent by the application for bootloader-only commands

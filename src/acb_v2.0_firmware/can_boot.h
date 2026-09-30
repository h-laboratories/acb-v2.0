// Application-side hook for the ACB CAN bootloader (src/acb_can_bootloader).
//
// Brings up FDCAN2 (PB12/PB13, 500 kbit/s) and answers the small subset of the
// bootloader protocol an application needs:
//   ACB_CMD_PING   -> identify (flags say "in application")
//   ACB_CMD_ENTER  -> ack, then reset into the bootloader
//   ACB_CMD_GO     -> ack, then plain reset
// Call canBootInit() once in setup() and canBootPoll() from loop().
#pragma once
#include <stdint.h>

void    canBootInit();
void    canBootPoll();
uint8_t canBootNodeId();

// Implemented in can_control.cpp: handle an application-level command.
// On entry resp[0] = cmd, resp[1] = ACB_ST_OK, *respLen = 2. Return false if
// the command is unknown (the caller then answers UNSUPPORTED / NOT_IN_BL).
void canControlLoop();   // can_control.cpp: periodic work (serial position stream); call from loop()
bool canAppCommand(const uint8_t* data, uint8_t len, uint8_t* resp, uint8_t* respLen);

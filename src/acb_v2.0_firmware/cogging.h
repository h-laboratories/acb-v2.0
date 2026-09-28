// Anti-cogging map: ODrive-style calibration and feed-forward for SimpleFOC.
//
// Calibration steps the rotor through COG_MAP_N evenly spaced positions in angle
// mode, waits at each until position error and velocity settle, and records the
// current the velocity loop needed to hold that position. In normal operation the
// interpolated map value is added to the current setpoint as feed-forward torque.
//
// The map is keyed to the absolute rotor angle: incremental encoder mechanical
// angle + the MA730 offset captured at boot (g_encoderAbsOffset), so it survives
// reboots. It is persisted to its own flash region below the EEPROM page.
#pragma once
#include <Arduino.h>

#define COG_MAP_N        2048u          // bins per mechanical revolution (2 encoder counts each at 1024 PPR)
#define COG_FLASH_ADDR   0x0807D000u    // 8 KB region, see README flash layout
#define COG_FLASH_LEN    0x2000u

enum CogState : uint8_t { COG_IDLE = 0, COG_CALIBRATING = 1, COG_DONE = 2, COG_ABORTED = 3 };

struct CogCalParams {
  uint8_t  pos_thr_counts;   // settle window in encoder counts (default 1)
  float    vel_thr;          // settle velocity threshold, rad/s (default 0.1)
  uint16_t dwell_ms;         // must stay settled this long before recording (default 30)
  uint16_t timeout_ms;       // give up waiting and record anyway (default 400)
};

void     coggingLoad();                       // read map from flash at boot
bool     coggingStart(const CogCalParams* p); // p may be NULL for defaults; false if motor not ready
void     coggingAbort();
void     coggingUpdate();                     // call every loop before motor.move()
void     coggingApply();                      // call every loop after motor.move()
uint8_t  coggingSave();                       // ACB_ST_*; motor must be disabled
bool     coggingSetEnabled(bool en);          // false if there is no valid map

CogState coggingState();
bool     coggingValid();
bool     coggingEnabled();
bool     coggingSaved();
uint16_t coggingIndex();
uint8_t  coggingTimeouts();
int16_t  coggingEntry(uint16_t idx);          // mA
float    coggingMapAngle();                   // absolute rotor angle used for indexing, [0, 2pi)

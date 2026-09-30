// Application-level CAN control commands (ACB_CMD_SET_MODE .. ACB_CMD_GET_TELEMETRY).
// Dispatched from canBootPoll() in can_boot.cpp via canAppCommand().
#include <Arduino.h>
#include <string.h>
#include "config.h"
#include "config_manager.h"
#include "DRV8323RSRGZR.h"
#include "CommandManager.h"
#include "acb_can_protocol.h"
#include "can_boot.h"

extern BLDCMotor motor;
extern BLDCDriver6PWM driver;
#include "windowed_encoder.h"
extern WindowedEncoder encoder;
extern LowsideCurrentSense current_sense;
extern DRV8323RSRGZR drv8323;
extern CommandManager command_manager;
#include "cogging.h"
extern uint8_t g_drvSpiMode;
extern uint32_t g_drvSpiHz;
extern float bus_voltage, board_temperature, internal_temperature;

static float rdf32(const uint8_t* p) { float f; memcpy(&f, p, 4); return f; }
static void wru16(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; }
static void wri16(uint8_t* p, long v) {
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static void wrf32(uint8_t* p, float f) { memcpy(p, &f, 4); }
static void wri32(uint8_t* p, long v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

static uint8_t g_streamHz = 0;

void canControlLoop() {
  if (!g_streamHz) return;
  static uint32_t last = 0;
  const uint32_t period = 1000u / g_streamHz;
  if (millis() - last < period) return;
  last = millis();
  Serial.print("pos_deg ");
  Serial.print(motor.shaft_angle * 180.0f / PI, 2);
  Serial.print(" vel_rpm ");
  Serial.println(motor.shaft_velocity * 60.0f / (2.0f * PI), 1);
}

static bool isOpenLoop() {
  return motor.controller == MotionControlType::velocity_openloop || motor.controller == MotionControlType::angle_openloop;
}

bool canAppCommand(const uint8_t* d, uint8_t n, uint8_t* r, uint8_t* rlen) {
  // On entry r[0] = d[0], r[1] = ACB_ST_OK, *rlen = 2.
  switch (d[0]) {
    case ACB_CMD_SET_MODE: {
      if (n < 2 || d[1] > 4) { r[1] = ACB_ST_ARG; return true; }
      motor.controller = (MotionControlType)d[1];
      r[2] = d[1]; *rlen = 3;
      return true;
    }
    case ACB_CMD_SET_TARGET: {
      if (n < 5) { r[1] = ACB_ST_ARG; return true; }
      motor.target = rdf32(&d[1]);
      return true;
    }
    case ACB_CMD_ENABLE: {
      if (n < 2) { r[1] = ACB_ST_ARG; return true; }
      if (d[1]) motor.enable(); else motor.disable();
      return true;
    }
    case ACB_CMD_SET_POLE_PAIRS: {
      if (n < 2 || d[1] < 1) { r[1] = ACB_ST_ARG; return true; }
      motor.pole_pairs = d[1];
      acb_config.pole_pairs = d[1];
      r[2] = d[1]; *rlen = 3;
      return true;
    }
    case ACB_CMD_SET_VLIMIT: {
      if (n < 5) { r[1] = ACB_ST_ARG; return true; }
      const float v = rdf32(&d[1]);
      const float vmax = (bus_voltage > 1.0f) ? bus_voltage * 0.5f : driver.voltage_limit;
      if (!(v >= 0.0f && v <= vmax)) { r[1] = ACB_ST_ARG; return true; }
      motor.voltage_limit = v;
      if (driver.voltage_limit < v) driver.voltage_limit = v;
      // BLDCMotor::init() copies voltage_limit into these once; keep them in step.
      motor.PID_current_q.limit = v;
      motor.PID_current_d.limit = v;
      if (motor.torque_controller == TorqueControlType::voltage) motor.PID_velocity.limit = v;
      return true;
    }
    case ACB_CMD_SAVE_CONFIG:
      saveConfig();
      return true;

    case ACB_CMD_GET_STATE: {
      float vel, ang;
      if (isOpenLoop()) {          // loopFOC() skips the sensor in open loop: read the encoder directly
        encoder.update();
        vel = encoder.getVelocity();
        ang = encoder.getAngle();
      } else {
        vel = motor.shaft_velocity;
        ang = motor.shaft_angle;
      }
      r[1] = ((uint8_t)motor.controller & 0x0F) | (motor.enabled ? 0x10 : 0);
      wri16(&r[2], lroundf(vel * 100.0f));
      wri32(&r[4], lroundf(ang * 10000.0f));
      *rlen = 8;
      return true;
    }
    case ACB_CMD_GET_TELEMETRY: {
      wru16(&r[1], (uint32_t)lroundf(bus_voltage * 100.0f));
      wri16(&r[3], lroundf(board_temperature * 10.0f));
      wri16(&r[5], lroundf(internal_temperature * 10.0f));
      r[7] = (digitalRead(DRV_FAULT) == LOW) ? 1 : 0;
      *rlen = 8;
      return true;
    }
    case ACB_CMD_SET_ILIMIT: {
      if (n < 5) { r[1] = ACB_ST_ARG; return true; }
      const float a = rdf32(&d[1]);
      if (!(a > 0.0f && a <= 40.0f)) { r[1] = ACB_ST_ARG; return true; }
      motor.current_limit = a;
      if (motor.torque_controller != TorqueControlType::voltage) motor.PID_velocity.limit = a;
      return true;
    }
    case ACB_CMD_RECALIBRATE: {
      if (n >= 5) {
        const float va = rdf32(&d[1]);
        if (va > 0.0f && va <= 6.0f) motor.voltage_sensor_align = va;
      }
      command_manager.handle_recalibrate_sensors();   // blocks for a few seconds; leaves motor disabled
      r[2] = (uint8_t)(int8_t)acb_config.sensor_direction;
      wrf32(&r[3], acb_config.zero_electric_angle);
      *rlen = 7;
      return true;
    }
    case ACB_CMD_SET_TORQUE_MODE: {
      if (n < 2 || d[1] > 2) { r[1] = ACB_ST_ARG; return true; }
      motor.torque_controller = (TorqueControlType)d[1];
      motor.PID_velocity.limit = (d[1] == 0) ? motor.voltage_limit : motor.current_limit;
      r[2] = d[1]; *rlen = 3;
      return true;
    }
    case ACB_CMD_GET_CURRENTS: {
      PhaseCurrent_s c = current_sense.getPhaseCurrents();
      wri16(&r[1], lroundf(c.a * 1000.0f));
      wri16(&r[3], lroundf(c.b * 1000.0f));
      wri16(&r[5], lroundf(c.c * 1000.0f));
      r[7] = 0; *rlen = 8;
      return true;
    }
    case ACB_CMD_GET_DQ: {
      wri16(&r[1], lroundf(motor.current.q * 1000.0f));
      wri16(&r[3], lroundf(motor.current.d * 1000.0f));
      wri16(&r[5], lroundf(motor.voltage.q * 1000.0f));
      r[7] = 0; *rlen = 8;
      return true;
    }
    case ACB_CMD_SERIAL_STREAM: {
      if (n < 2 || d[1] > 100) { r[1] = ACB_ST_ARG; return true; }
      g_streamHz = d[1];
      r[2] = d[1]; *rlen = 3;
      return true;
    }
    case ACB_CMD_DRV_SPI_CFG: {
      if (n < 4 || d[1] > 3) { r[1] = ACB_ST_ARG; return true; }
      g_drvSpiMode = d[1];
      const uint32_t khz = d[2] | (d[3] << 8);
      if (khz >= 50 && khz <= 10000) g_drvSpiHz = khz * 1000u;
      return true;
    }
    case ACB_CMD_DRV_CLEAR_FAULTS:
      drv8323.resetFaults();
      return true;
    case ACB_CMD_DRV_FAULTS: {
      r[1] = (digitalRead(DRV_FAULT) == LOW) ? 1 : 0;
      wru16(&r[2], drv8323.readRegister(0x00));
      wru16(&r[4], drv8323.readRegister(0x01));
      *rlen = 6;
      return true;
    }
    case ACB_CMD_DRV_READ_REG: {
      if (n < 2 || d[1] > 0x06) { r[1] = ACB_ST_ARG; return true; }
      r[1] = d[1];
      wru16(&r[2], drv8323.readRegister(d[1]));
      *rlen = 4;
      return true;
    }
    case ACB_CMD_DRV_WRITE_REG: {
      if (n < 4 || d[1] < 0x02 || d[1] > 0x06) { r[1] = ACB_ST_ARG; return true; }
      drv8323.writeRegister(d[1], (uint16_t)(d[2] | (d[3] << 8)));
      r[2] = d[1];
      wru16(&r[3], drv8323.readRegister(d[1]));
      *rlen = 5;
      return true;
    }
    case ACB_CMD_COG_CALIB: {
      if (n < 2) { r[1] = ACB_ST_ARG; return true; }
      if (!d[1]) { coggingAbort(); return true; }
      CogCalParams p = {1, 0.1f, 30, 400};
      if (n >= 6) { p.pos_thr_counts = d[2]; p.vel_thr = d[3] * 0.01f; p.dwell_ms = d[4]; p.timeout_ms = (uint16_t)d[5] * 10u; }
      if (isOpenLoop()) motor.controller = MotionControlType::velocity;   // calibration returns to a closed-loop mode
      if (!coggingStart(&p)) r[1] = ACB_ST_STATE;
      return true;
    }
    case ACB_CMD_COG_ENABLE: {
      if (n < 2) { r[1] = ACB_ST_ARG; return true; }
      if (!coggingSetEnabled(d[1] != 0)) r[1] = ACB_ST_STATE;
      return true;
    }
    case ACB_CMD_COG_SAVE:
      r[1] = coggingSave();
      return true;
    case ACB_CMD_COG_STATUS: {
      r[1] = (uint8_t)coggingState();
      r[2] = (coggingValid() ? 1 : 0) | (coggingEnabled() ? 2 : 0) | (coggingSaved() ? 4 : 0);
      wru16(&r[3], coggingIndex());
      wru16(&r[5], COG_MAP_N);
      r[7] = coggingTimeouts();
      *rlen = 8;
      return true;
    }
    case ACB_CMD_COG_GET: {
      if (n < 3) { r[1] = ACB_ST_ARG; return true; }
      const uint16_t i = d[1] | (d[2] << 8);
      if (i >= COG_MAP_N) { r[1] = ACB_ST_ARG; return true; }
      wru16(&r[1], i);
      wri16(&r[3], coggingEntry(i));
      wri16(&r[5], coggingEntry((i + 1) % COG_MAP_N));
      *rlen = 7;
      return true;
    }
    default:
      return false;
  }
}

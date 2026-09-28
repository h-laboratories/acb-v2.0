#include "cogging.h"
#include <SimpleFOC.h>
#include <string.h>
#include "config.h"
#include "acb_can_protocol.h"
#include "config_manager.h"

extern BLDCMotor motor;
extern float g_encoderAbsOffset;   // MA730 absolute angle - incremental mechanical angle, captured at boot

// ------------------------------------------------------------------ state ----
static int16_t   g_map[COG_MAP_N];
static bool      g_valid = false, g_enabled = false, g_saved = false;
static CogState  g_state = COG_IDLE;
static uint16_t  g_idx = 0;
static uint8_t   g_timeouts = 0;
static CogCalParams g_p;
static MotionControlType g_prevCtl;
static uint32_t  g_binStart = 0, g_settleSince = 0;
static bool      g_settling = false;
static float     g_acc = 0.0f;
static uint32_t  g_accN = 0;

struct CogFlashHdr {
  uint32_t magic;        // "COG1"
  uint16_t n;
  uint8_t  pole_pairs;
  int8_t   sensor_dir;
  uint32_t crc;          // over the int16 entries
  uint32_t reserved;
};
#define COG_MAGIC 0x31474F43u   // "COG1" little-endian
static_assert(sizeof(CogFlashHdr) == 16, "header must stay 8-byte aligned");
static_assert(sizeof(CogFlashHdr) + sizeof(g_map) <= COG_FLASH_LEN, "map does not fit its flash region");

// ---------------------------------------------------------------- helpers ----
static float normAngle(float a) {          // [0, 2pi)
  a = fmodf(a, _2PI);
  return a < 0 ? a + _2PI : a;
}
static float normSigned(float a) {         // (-pi, pi]
  a = normAngle(a);
  return a > _PI ? a - _2PI : a;
}
static float dirSign() { return motor.sensor_direction == Direction::CCW ? -1.0f : 1.0f; }

static uint32_t crc32(const uint8_t* p, uint32_t len) {
  uint32_t c = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < len; i++) {
    c ^= p[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

float coggingMapAngle() {
  if (!motor.sensor) return 0.0f;
  return normAngle(motor.sensor->getMechanicalAngle() + g_encoderAbsOffset);
}

// ------------------------------------------------------------------ flash ----
// Same helpers as the CAN bootloader. The map lives in flash bank 2 while the
// application runs from bank 1, so in dual-bank mode erase/program does not
// stall code fetch (the encoder interrupts keep running).
static void flashFlushCaches() {
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
}
static uint32_t flashPageSize()  { return (FLASH->OPTR & FLASH_OPTR_DBANK) ? 0x800u : 0x1000u; }
static uint32_t flashSizeBytes() { return (uint32_t)(*(volatile uint16_t*)FLASHSIZE_BASE) * 1024u; }

static uint8_t flashErase(uint32_t addr, uint32_t len) {
  const uint32_t ps = flashPageSize();
  const uint32_t start = addr & ~(ps - 1), end = (addr + len + ps - 1) & ~(ps - 1);
  uint8_t st = ACB_ST_OK;
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  for (uint32_t a = start; a < end; a += ps) {
    FLASH_EraseInitTypeDef e = {0};
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.NbPages = 1;
    const uint32_t off = a - FLASH_BASE;
    if (FLASH->OPTR & FLASH_OPTR_DBANK) {
      const uint32_t bankSize = flashSizeBytes() / 2;
      e.Banks = (off < bankSize) ? FLASH_BANK_1 : FLASH_BANK_2;
      e.Page = (off % bankSize) / ps;
    } else {
      e.Banks = FLASH_BANK_1;
      e.Page = off / ps;
    }
    uint32_t pageErr = 0;
    if (HAL_FLASHEx_Erase(&e, &pageErr) != HAL_OK) { st = ACB_ST_FLASH; break; }
  }
  HAL_FLASH_Lock();
  flashFlushCaches();
  return st;
}

static uint8_t flashProgram(uint32_t addr, const uint8_t* data, uint32_t len) {
  if ((addr & 7u) || (len & 7u)) return ACB_ST_ARG;
  uint8_t st = ACB_ST_OK;
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  for (uint32_t i = 0; i < len; i += 8) {
    uint64_t dw;
    memcpy(&dw, data + i, 8);
    if (dw == 0xFFFFFFFFFFFFFFFFull) continue;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dw) != HAL_OK) { st = ACB_ST_FLASH; break; }
  }
  HAL_FLASH_Lock();
  flashFlushCaches();
  if (st == ACB_ST_OK && memcmp((const void*)addr, data, len) != 0) st = ACB_ST_VERIFY;
  return st;
}

void coggingLoad() {
  const CogFlashHdr* h = (const CogFlashHdr*)COG_FLASH_ADDR;
  const int16_t* m = (const int16_t*)(COG_FLASH_ADDR + sizeof(CogFlashHdr));
  g_valid = false; g_enabled = false; g_saved = false;
  if (h->magic != COG_MAGIC || h->n != COG_MAP_N) return;
  if (h->pole_pairs != (uint8_t)acb_config.pole_pairs || h->sensor_dir != (int8_t)acb_config.sensor_direction) return;
  if (crc32((const uint8_t*)m, sizeof(g_map)) != h->crc) return;
  memcpy(g_map, m, sizeof(g_map));
  g_valid = true; g_saved = true; g_enabled = true;
}

uint8_t coggingSave() {
  if (!g_valid) return ACB_ST_STATE;
  if (motor.enabled || g_state == COG_CALIBRATING) return ACB_ST_STATE;   // erase stalls the control loop
  CogFlashHdr h = {COG_MAGIC, (uint16_t)COG_MAP_N, (uint8_t)acb_config.pole_pairs, (int8_t)acb_config.sensor_direction,
                   crc32((const uint8_t*)g_map, sizeof(g_map)), 0};
  uint8_t st = flashErase(COG_FLASH_ADDR, sizeof(h) + sizeof(g_map));
  if (st == ACB_ST_OK) st = flashProgram(COG_FLASH_ADDR, (const uint8_t*)&h, sizeof(h));
  if (st == ACB_ST_OK) st = flashProgram(COG_FLASH_ADDR + sizeof(h), (const uint8_t*)g_map, sizeof(g_map));
  g_saved = (st == ACB_ST_OK);
  return st;
}

// ------------------------------------------------------------ calibration ----
static const CogCalParams kDefaults = {1, 0.1f, 30, 400};

bool coggingStart(const CogCalParams* p) {
  if (!motor.sensor || motor.sensor_direction == Direction::UNKNOWN) return false;
  if (motor.torque_controller == TorqueControlType::voltage) return false;    // map is in amps
  g_p = p ? *p : kDefaults;
  if (g_p.pos_thr_counts == 0) g_p.pos_thr_counts = 1;
  if (g_p.vel_thr <= 0.0f) g_p.vel_thr = kDefaults.vel_thr;
  if (g_p.dwell_ms == 0) g_p.dwell_ms = kDefaults.dwell_ms;
  if (g_p.timeout_ms < g_p.dwell_ms) g_p.timeout_ms = g_p.dwell_ms * 4;
  memset(g_map, 0, sizeof(g_map));
  g_valid = false; g_saved = false; g_enabled = false;      // feed-forward off while measuring
  g_prevCtl = motor.controller;
  motor.controller = MotionControlType::angle;
  motor.target = motor.shaft_angle;
  motor.enable();
  g_idx = 0; g_timeouts = 0;
  g_binStart = millis(); g_settling = false; g_acc = 0.0f; g_accN = 0;
  g_state = COG_CALIBRATING;
  return true;
}

static void finish(CogState s) {
  g_state = s;
  motor.controller = g_prevCtl;
  motor.target = (g_prevCtl == MotionControlType::angle) ? motor.shaft_angle : 0.0f;
  if (s == COG_DONE) { g_valid = true; g_enabled = true; }
}

void coggingAbort() {
  if (g_state == COG_CALIBRATING) finish(COG_ABORTED);
}

static void recordAndAdvance(float amps) {
  long mA = lroundf(amps * 1000.0f);
  if (mA > 32767) mA = 32767; else if (mA < -32767) mA = -32767;
  g_map[g_idx] = (int16_t)mA;
  g_idx++;
  g_binStart = millis(); g_settling = false; g_acc = 0.0f; g_accN = 0;
  if (g_idx >= COG_MAP_N) finish(COG_DONE);
}

void coggingUpdate() {
  if (g_state != COG_CALIBRATING) return;
  if (!motor.enabled) { finish(COG_ABORTED); return; }   // someone disabled the motor mid-run

  const float binTarget = (float)g_idx * (_2PI / (float)COG_MAP_N);
  const float delta = normSigned(binTarget - coggingMapAngle());       // in the encoder's native sense
  motor.target = motor.shaft_angle + dirSign() * delta;                // re-derived each loop, frame-safe

  const float posThr = (float)g_p.pos_thr_counts * (_2PI / (4.0f * ENCODER_PPR));
  const uint32_t now = millis();
  const bool settled = fabsf(delta) <= posThr && fabsf(motor.shaft_velocity) <= g_p.vel_thr;

  if (settled) {
    if (!g_settling) { g_settling = true; g_settleSince = now; g_acc = 0.0f; g_accN = 0; }
    g_acc += motor.current_sp; g_accN++;
    if (now - g_settleSince >= g_p.dwell_ms) { recordAndAdvance(g_acc / (float)g_accN); return; }
  } else {
    g_settling = false;
  }
  if (now - g_binStart >= g_p.timeout_ms) {
    if (g_timeouts < 255) g_timeouts++;
    recordAndAdvance(g_accN ? g_acc / (float)g_accN : motor.current_sp);
  }
}

// ------------------------------------------------------------ feed-forward ----
void coggingApply() {
  if (!g_enabled || !g_valid || g_state == COG_CALIBRATING || !motor.enabled) return;
  if (motor.controller == MotionControlType::velocity_openloop || motor.controller == MotionControlType::angle_openloop) return;
  if (motor.torque_controller == TorqueControlType::voltage) return;
  const float pos = coggingMapAngle() * ((float)COG_MAP_N / _2PI);
  uint32_t i = (uint32_t)pos;
  const float f = pos - (float)i;
  i %= COG_MAP_N;
  const uint32_t j = (i + 1u) % COG_MAP_N;
  const float ff = ((float)g_map[i] * (1.0f - f) + (float)g_map[j] * f) * 1e-3f;
  motor.current_sp = _constrain(motor.current_sp + ff, -motor.current_limit, motor.current_limit);
}

bool coggingSetEnabled(bool en) {
  if (en && !g_valid) return false;
  g_enabled = en;
  return true;
}

CogState coggingState()    { return g_state; }
bool     coggingValid()    { return g_valid; }
bool     coggingEnabled()  { return g_enabled; }
bool     coggingSaved()    { return g_saved; }
uint16_t coggingIndex()    { return g_idx; }
uint8_t  coggingTimeouts() { return g_timeouts; }
int16_t  coggingEntry(uint16_t idx) { return idx < COG_MAP_N ? g_map[idx] : 0; }

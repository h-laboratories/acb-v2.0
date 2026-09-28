// ACB v2.0 CAN bootloader ---- STM32G474RET6, FDCAN2 on PB12 (RX) / PB13 (TX)
//
// Lets an ACB be reprogrammed over the CAN bus with tools/acb_can_flash.py.
// Protocol, IDs and flash layout: acb_can_protocol.h.
//
// Boot decision (every reset goes through here first):
//   1. TAMP backup register 0 == ACB_BOOT_MAGIC_STAY -> stay resident
//      (the application writes this when it receives ACB_CMD_ENTER, then resets).
//   2. No valid application vector table at ACB_APP_ADDR -> stay resident.
//   3. Otherwise listen BOOT_WINDOW_MS for ACB_CMD_ENTER (recovery path: the PC
//      tool spams ENTER while you power-cycle the board), then jump to the app.
//
// While resident: STATUS_LED blinks fast, COM_LED toggles on every CAN frame.
//
// Build (tools/build.py bootloader):
//   Generic STM32G4 / G474RETx, USB support = None, U(S)ART = Disabled,
//   upload.maximum_size = 28672 so the image is guaranteed to end before the
//   boot-config page at 0x08007000. Flash once via DFU/ST-Link at 0x08000000.

#include <string.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "acb_can_protocol.h"

#define BL_VERSION        1
#define BOOT_WINDOW_MS    300
#define STATUS_LED_PIN    PC6
#define COM_LED_PIN       PC7

static FDCAN_HandleTypeDef hfdcan2;
static uint8_t  g_nodeId = ACB_CAN_DEFAULT_NODE_ID;

// WRITE state: data frames are buffered here, programmed when the chunk is complete
static uint8_t  g_buf[ACB_MAX_CHUNK];
static uint32_t g_wrAddr = 0, g_wrLen = 0, g_wrGot = 0;
static bool     g_wrActive = false;

// ---------------------------------------------------------------- helpers ----
static inline uint32_t rd32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline uint32_t rd24(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16); }
static inline uint32_t rd16(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8); }
static inline void wr32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static inline void wr16(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; }

// zlib-compatible CRC-32 (matches Python's zlib.crc32)
static uint32_t crc32_update(uint32_t crc, const uint8_t* p, uint32_t n) {
  crc = ~crc;
  while (n--) {
    crc ^= *p++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

static uint32_t uid32() {
  const volatile uint32_t* uid = (const volatile uint32_t*)UID_BASE;
  return uid[0] ^ uid[1] ^ uid[2];
}

// -------------------------------------------------------- backup register ----
static void bkpInit() {
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
}
static uint32_t bkpRead()            { return TAMP->BKP0R; }
static void     bkpWrite(uint32_t v) { TAMP->BKP0R = v; }

// ------------------------------------------------------------------ flash ----
static uint32_t flashPageSize()  { return (FLASH->OPTR & FLASH_OPTR_DBANK) ? 0x800u : 0x1000u; }
static uint32_t flashSizeBytes() { return (uint32_t)(*(volatile uint16_t*)FLASHSIZE_BASE) * 1024u; }

static void flashFlushCaches() {
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
}

static bool inAppRegion(uint32_t addr, uint32_t len) {
  return len > 0 && addr >= ACB_APP_ADDR && addr + len > addr && addr + len <= ACB_APP_END;
}

// Erase every page overlapping [addr, addr+len). Caller checks the region.
static uint8_t flashErase(uint32_t addr, uint32_t len) {
  const uint32_t ps = flashPageSize();
  const uint32_t start = addr & ~(ps - 1);
  const uint32_t end = (addr + len + ps - 1) & ~(ps - 1);
  uint8_t st = ACB_ST_OK;

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  for (uint32_t a = start; a < end; a += ps) {
    FLASH_EraseInitTypeDef e = {0};
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.NbPages = 1;
    const uint32_t off = a - ACB_FLASH_BASE;
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

// Program len bytes (multiple of 8, addr 8-aligned) and verify by readback.
static uint8_t flashProgram(uint32_t addr, const uint8_t* data, uint32_t len) {
  if ((addr & 7u) || (len & 7u)) return ACB_ST_ARG;
  uint8_t st = ACB_ST_OK;

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  for (uint32_t i = 0; i < len; i += 8) {
    uint64_t dw;
    memcpy(&dw, data + i, 8);
    if (dw == 0xFFFFFFFFFFFFFFFFull) continue;   // already erased
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dw) != HAL_OK) { st = ACB_ST_FLASH; break; }
  }
  HAL_FLASH_Lock();
  flashFlushCaches();
  if (st == ACB_ST_OK && memcmp((const void*)addr, data, len) != 0) st = ACB_ST_VERIFY;
  return st;
}

static uint8_t readNodeId() {
  const AcbBootCfg* c = (const AcbBootCfg*)ACB_BOOT_CFG_ADDR;
  if (c->magic == ACB_BOOTCFG_MAGIC && c->node_id >= 1 && c->node_id <= ACB_CAN_NODE_MAX) return c->node_id;
  return ACB_CAN_DEFAULT_NODE_ID;
}

static uint8_t writeNodeId(uint8_t id) {
  uint8_t st = flashErase(ACB_BOOT_CFG_ADDR, flashPageSize());
  if (st != ACB_ST_OK) return st;
  uint8_t rec[8];
  wr32(rec, ACB_BOOTCFG_MAGIC);
  rec[4] = id; rec[5] = rec[6] = rec[7] = 0xFF;
  return flashProgram(ACB_BOOT_CFG_ADDR, rec, 8);
}

// ------------------------------------------------------------ application ----
static bool appValid() {
  const uint32_t sp = *(volatile uint32_t*)ACB_APP_ADDR;
  const uint32_t pc = *(volatile uint32_t*)(ACB_APP_ADDR + 4);
  const bool spOk = sp > 0x20000000u && sp <= 0x20020000u;          // 128 KB SRAM
  const bool pcOk = (pc & 1u) && pc >= ACB_APP_ADDR && pc < ACB_APP_END;
  return spOk && pcOk;
}

// Put the clock tree back to its reset state (HSI 16 MHz, PLL off) so the
// application's own SystemClock_Config starts from known conditions.
static void rccReset() {
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY)) {}
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}
  RCC->CR &= ~RCC_CR_PLLON;
  while (RCC->CR & RCC_CR_PLLRDY) {}
  RCC->CFGR = 0x00000001u;      // HSI selected, no prescalers
  RCC->PLLCFGR = 0x00001000u;   // reset value
  RCC->CIER = 0;
}

static void jumpToApp() {
  const uint32_t sp = *(volatile uint32_t*)ACB_APP_ADDR;
  const uint32_t pc = *(volatile uint32_t*)(ACB_APP_ADDR + 4);

  HAL_FDCAN_Stop(&hfdcan2);
  HAL_FDCAN_DeInit(&hfdcan2);
  __HAL_RCC_FDCAN_CLK_DISABLE();
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(COM_LED_PIN, LOW);

  __disable_irq();
  SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;
  for (uint32_t i = 0; i < 8; i++) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
  rccReset();
  flashFlushCaches();
  SCB->VTOR = ACB_APP_ADDR;
  __set_CONTROL(0);
  __set_MSP(sp);
  __DSB(); __ISB();
  __enable_irq();
  ((void (*)(void))pc)();
  while (1) {}
}

// -------------------------------------------------------------------- CAN ----
static void canInit(uint8_t node) {
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &g);

  // FDCAN kernel clock from PCLK1 (no HSE on this board)
  RCC_PeriphCLKInitTypeDef pc = {0};
  pc.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  pc.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&pc);
  __HAL_RCC_FDCAN_CLK_ENABLE();

  // 20 tq per bit: 1 + 15 + 4, 80 % sample point
  const uint32_t tqPerBit = 20;
  uint32_t prescaler = HAL_RCC_GetPCLK1Freq() / (ACB_CAN_BITRATE * tqPerBit);
  if (prescaler < 1) prescaler = 1;

  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = prescaler;
  hfdcan2.Init.NominalSyncJumpWidth = 4;
  hfdcan2.Init.NominalTimeSeg1 = 15;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = prescaler;
  hfdcan2.Init.DataSyncJumpWidth = 4;
  hfdcan2.Init.DataTimeSeg1 = 15;
  hfdcan2.Init.DataTimeSeg2 = 4;
  hfdcan2.Init.StdFiltersNbr = 2;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  HAL_FDCAN_Init(&hfdcan2);

  // Only our command IDs (own node + broadcast) and our data ID reach FIFO0.
  FDCAN_FilterTypeDef f = {0};
  f.IdType = FDCAN_STANDARD_ID;
  f.FilterType = FDCAN_FILTER_DUAL;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterIndex = 0;
  f.FilterID1 = ACB_CAN_ID_CMD_BASE | node;
  f.FilterID2 = ACB_CAN_ID_CMD_BASE | ACB_CAN_NODE_BROADCAST;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &f);
  f.FilterIndex = 1;
  f.FilterID1 = ACB_CAN_ID_DATA_BASE | node;
  f.FilterID2 = ACB_CAN_ID_DATA_BASE | node;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &f);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  HAL_FDCAN_Start(&hfdcan2);
}

static bool canSend(uint32_t id, const uint8_t* d, uint8_t len) {
  const uint32_t t0 = millis();
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0) {
    if (millis() - t0 > 20) return false;   // nobody ACKing us
  }
  FDCAN_TxHeaderTypeDef h;
  h.Identifier = id;
  h.IdType = FDCAN_STANDARD_ID;
  h.TxFrameType = FDCAN_DATA_FRAME;
  h.DataLength = len;                        // raw byte count (0..8) in this HAL
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch = FDCAN_BRS_OFF;
  h.FDFormat = FDCAN_CLASSIC_CAN;
  h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  h.MessageMarker = 0;
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &h, (uint8_t*)d) == HAL_OK;
}

static bool canRecv(uint32_t* id, uint8_t* d, uint8_t* len) {
  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) == 0) return false;
  FDCAN_RxHeaderTypeDef h;
  if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &h, d) != HAL_OK) return false;
  *id = h.Identifier;
  *len = (h.DataLength > 8) ? 8 : (uint8_t)h.DataLength;
  return true;
}

static void respond(const uint8_t* d, uint8_t len) {
  canSend(ACB_CAN_ID_RESP_BASE | g_nodeId, d, len);
}

static void sendHello(uint8_t cmd) {
  uint8_t r[8];
  r[0] = cmd;
  r[1] = ACB_CAN_PROTO_VERSION;
  r[2] = BL_VERSION;
  r[3] = ACB_FLAG_IN_BOOTLOADER | (appValid() ? ACB_FLAG_APP_VALID : 0);
  wr32(&r[4], uid32());
  respond(r, 8);
}

// --------------------------------------------------------------- commands ----
static void handleData(const uint8_t* d, uint8_t n) {
  if (!g_wrActive) return;
  uint32_t room = g_wrLen - g_wrGot;
  if (n > room) n = room;
  memcpy(&g_buf[g_wrGot], d, n);
  g_wrGot += n;
  if (g_wrGot < g_wrLen) return;

  g_wrActive = false;
  uint8_t r[8] = {ACB_CMD_WRITE_DONE, 0, 0, 0, 0, 0, 0, 0};
  r[1] = flashProgram(g_wrAddr, g_buf, g_wrLen);
  wr32(&r[2], crc32_update(0, g_buf, g_wrLen));
  respond(r, 6);
}

static void resetAfterResponse(bool stay) {
  delay(20);                 // let the response frame leave the Tx FIFO
  if (stay) bkpWrite(ACB_BOOT_MAGIC_STAY);
  NVIC_SystemReset();
}

static void handleCommand(const uint8_t* d, uint8_t n) {
  uint8_t r[8] = {d[0], ACB_ST_OK, 0, 0, 0, 0, 0, 0};
  switch (d[0]) {
    case ACB_CMD_PING:
    case ACB_CMD_ENTER:
      sendHello(d[0] == ACB_CMD_ENTER ? ACB_CMD_HELLO : ACB_CMD_PING);
      return;

    case ACB_CMD_INFO:
      wr16(&r[1], flashSizeBytes() / 1024);
      wr16(&r[3], flashPageSize());
      wr16(&r[5], (ACB_APP_ADDR - ACB_FLASH_BASE) / 1024);
      r[7] = (FLASH->OPTR & FLASH_OPTR_DBANK) ? 1 : 0;
      respond(r, 8);
      return;

    case ACB_CMD_ERASE: {
      if (n < 8) { r[1] = ACB_ST_ARG; respond(r, 2); return; }
      const uint32_t addr = rd32(&d[1]), len = rd24(&d[5]);
      r[1] = inAppRegion(addr, len) ? flashErase(addr, len) : ACB_ST_ARG;
      g_wrActive = false;
      respond(r, 2);
      return;
    }

    case ACB_CMD_WRITE_BEGIN: {
      if (n < 7) { r[1] = ACB_ST_ARG; respond(r, 2); return; }
      const uint32_t addr = rd32(&d[1]), len = rd16(&d[5]);
      if (!inAppRegion(addr, len) || len > ACB_MAX_CHUNK || (len & 7u) || (addr & 7u)) {
        g_wrActive = false;
        r[1] = ACB_ST_ARG; respond(r, 2); return;
      }
      g_wrAddr = addr; g_wrLen = len; g_wrGot = 0; g_wrActive = true;   // quiet on success
      return;
    }

    case ACB_CMD_CRC: {
      if (n < 8) { r[1] = ACB_ST_ARG; respond(r, 2); return; }
      const uint32_t addr = rd32(&d[1]), len = rd24(&d[5]);
      const uint32_t fend = ACB_FLASH_BASE + flashSizeBytes();
      if (len == 0 || addr < ACB_FLASH_BASE || addr + len > fend || addr + len < addr) { r[1] = ACB_ST_ARG; respond(r, 2); return; }
      wr32(&r[2], crc32_update(0, (const uint8_t*)addr, len));
      respond(r, 6);
      return;
    }

    case ACB_CMD_GO: {
      const uint8_t mode = (n >= 2) ? d[1] : 0;
      respond(r, 2);
      resetAfterResponse(mode == 1);
      return;   // not reached
    }

    case ACB_CMD_SET_NODE_ID: {
      const uint8_t id = (n >= 2) ? d[1] : 0;
      if (id < 1 || id > ACB_CAN_NODE_MAX) { r[1] = ACB_ST_ARG; r[2] = id; respond(r, 3); return; }
      r[1] = writeNodeId(id);
      r[2] = id;
      respond(r, 3);
      if (r[1] == ACB_ST_OK) resetAfterResponse(true);   // come back up with the new id
      return;
    }

    default:
      r[1] = ACB_ST_UNSUPPORTED;
      respond(r, 2);
      return;
  }
}

static void service() {
  uint32_t id; uint8_t d[8]; uint8_t n;
  while (canRecv(&id, d, &n)) {
    digitalToggle(COM_LED_PIN);
    if ((id & ACB_CAN_ID_BASE_MASK) == ACB_CAN_ID_DATA_BASE) { handleData(d, n); continue; }
    if (n >= 1) handleCommand(d, n);
  }
}

// ------------------------------------------------------------------- main ----
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(COM_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(COM_LED_PIN, LOW);

  bkpInit();
  bool stay = (bkpRead() == ACB_BOOT_MAGIC_STAY);
  bkpWrite(0);                      // one-shot: the next reset runs the app unless told otherwise
  if (!appValid()) stay = true;

  g_nodeId = readNodeId();
  canInit(g_nodeId);

  if (!stay) {
    const uint32_t t0 = millis();
    while (millis() - t0 < BOOT_WINDOW_MS) {
      uint32_t id; uint8_t d[8]; uint8_t n;
      if (canRecv(&id, d, &n) && (id & ACB_CAN_ID_BASE_MASK) == ACB_CAN_ID_CMD_BASE && n >= 1 && d[0] == ACB_CMD_ENTER) {
        stay = true;
        break;
      }
    }
    if (!stay) jumpToApp();         // does not return
  }

  sendHello(ACB_CMD_HELLO);
}

void loop() {
  service();
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink >= 100) { lastBlink = millis(); digitalToggle(STATUS_LED_PIN); }
}

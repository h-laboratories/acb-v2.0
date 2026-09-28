#include <Arduino.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "acb_can_protocol.h"
#include "can_boot.h"

#define CAN_APP_VERSION 1   // reported in PING byte 2 while the application is running

static FDCAN_HandleTypeDef hfdcan2;
static uint8_t g_nodeId = ACB_CAN_DEFAULT_NODE_ID;
static bool    g_ready = false;

static inline void wr32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

static uint8_t readNodeId() {
  const AcbBootCfg* c = (const AcbBootCfg*)ACB_BOOT_CFG_ADDR;
  if (c->magic == ACB_BOOTCFG_MAGIC && c->node_id >= 1 && c->node_id <= ACB_CAN_NODE_MAX) return c->node_id;
  return ACB_CAN_DEFAULT_NODE_ID;
}

static void requestBootloader() {
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
  TAMP->BKP0R = ACB_BOOT_MAGIC_STAY;
}

static bool canSend(uint32_t id, const uint8_t* d, uint8_t len) {
  const uint32_t t0 = millis();
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0) {
    if (millis() - t0 > 5) return false;
  }
  FDCAN_TxHeaderTypeDef h;
  h.Identifier = id;
  h.IdType = FDCAN_STANDARD_ID;
  h.TxFrameType = FDCAN_DATA_FRAME;
  h.DataLength = len;
  h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  h.BitRateSwitch = FDCAN_BRS_OFF;
  h.FDFormat = FDCAN_CLASSIC_CAN;
  h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  h.MessageMarker = 0;
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &h, (uint8_t*)d) == HAL_OK;
}

uint8_t canBootNodeId() { return g_nodeId; }

void canBootInit() {
  g_nodeId = readNodeId();

  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &g);

  RCC_PeriphCLKInitTypeDef pc = {0};
  pc.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  pc.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&pc);
  __HAL_RCC_FDCAN_CLK_ENABLE();

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
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) return;

  // Only command frames for this node or broadcast reach us.
  FDCAN_FilterTypeDef f = {0};
  f.IdType = FDCAN_STANDARD_ID;
  f.FilterIndex = 0;
  f.FilterType = FDCAN_FILTER_DUAL;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1 = ACB_CAN_ID_CMD_BASE | g_nodeId;
  f.FilterID2 = ACB_CAN_ID_CMD_BASE | ACB_CAN_NODE_BROADCAST;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &f);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  g_ready = (HAL_FDCAN_Start(&hfdcan2) == HAL_OK);
}

void canBootPoll() {
  if (!g_ready) return;
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0) {
    FDCAN_RxHeaderTypeDef h;
    uint8_t d[8];
    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &h, d) != HAL_OK) return;
    const uint8_t n = (h.DataLength > 8) ? 8 : (uint8_t)h.DataLength;
    if (n < 1) continue;

    const uint32_t respId = ACB_CAN_ID_RESP_BASE | g_nodeId;
    uint8_t r[8] = {d[0], ACB_ST_OK, 0, 0, 0, 0, 0, 0};

    switch (d[0]) {
      case ACB_CMD_PING: {
        const volatile uint32_t* uid = (const volatile uint32_t*)UID_BASE;
        r[1] = ACB_CAN_PROTO_VERSION;
        r[2] = CAN_APP_VERSION;
        r[3] = ACB_FLAG_APP_VALID;            // running the app, not the bootloader
        wr32(&r[4], uid[0] ^ uid[1] ^ uid[2]);
        canSend(respId, r, 8);
        break;
      }
      case ACB_CMD_ENTER:
        canSend(respId, r, 2);
        delay(20);                           // let the ack leave the Tx FIFO
        requestBootloader();
        NVIC_SystemReset();
        break;
      case ACB_CMD_GO:
        canSend(respId, r, 2);
        delay(20);
        NVIC_SystemReset();
        break;
      default: {
        uint8_t rl = 2;
        if (canAppCommand(d, n, r, &rl)) {
          canSend(respId, r, rl);
        } else {
          r[1] = (d[0] >= ACB_CMD_ERASE && d[0] <= ACB_CMD_INFO) ? ACB_ST_NOT_IN_BL : ACB_ST_UNSUPPORTED;
          canSend(respId, r, 2);
        }
        break;
      }
    }
  }
}

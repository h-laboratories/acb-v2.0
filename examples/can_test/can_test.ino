// CAN bring-up test for the ACB v2.0  ----  CHIP: STM32G474RET6
//
// IMPORTANT NOTES (why the previous version of this sketch did not work):
//
//  1. The STM32G4 family has NO classic bxCAN peripheral. It only has FDCAN.
//     The old sketch used CAN_HandleTypeDef / HAL_CAN_* / stm32g4xx_hal_can.h,
//     which is the F-series API and does not exist in the G4 HAL. This rewrite
//     uses the FDCAN HAL (stm32g4xx_hal_fdcan.h) in *classic CAN* mode.
//
//  2. On the G474RET6 the transceiver pins PB12 / PB13 map to FDCAN2, NOT
//     "CAN1" / FDCAN1:
//         PB12 -> FDCAN2_RX  (AF9)
//         PB13 -> FDCAN2_TX  (AF9)
//
//  3. The FDCAN kernel-clock source resets to HSE. This board has no HSE
//     crystal, so we explicitly select PCLK1 as the FDCAN clock below, or the
//     peripheral would have no clock at all.
//
//  Bit rate: 500 kbit/s, classic CAN, 8-byte frames. Sends ID 0x123 every 2 s
//  and prints any frame it receives. Any frame received on PING_ID (0x7E0) is
//  echoed back on PONG_ID (0x7E1) with the same payload, so a PC-side tool
//  (tools/can_wiring_test.py) can verify a full round trip.
//  Leave the Arduino core's clock setup
//  (170 MHz SYSCLK) alone -- do NOT re-run SystemClock_Config() here.

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

FDCAN_HandleTypeDef hfdcan2;

// Frame we transmit periodically
const uint32_t CAN_MESSAGE_ID = 0x123;
const uint8_t  CAN_DATA[8]     = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

// Ping/pong IDs used by tools/can_wiring_test.py
const uint32_t PING_ID = 0x7E0;   // PC -> ACB
const uint32_t PONG_ID = 0x7E1;   // ACB -> PC (echo of PING payload)

// Target classic-CAN bit rate
const uint32_t CAN_BITRATE = 500000UL;

unsigned long previousMillis = 0;
const unsigned long interval = 2000;  // 2 s

static void Error_Handler_CAN(const char* msg) {
  while (1) {
    Serial.print("CAN ERROR: ");
    Serial.println(msg);
    delay(1000);
  }
}

// Queue one classic 8-byte data frame. Returns true if it was accepted into
// the Tx FIFO (not whether it was ACKed on the bus).
static bool canSend(uint32_t id, const uint8_t* data, uint8_t len) {
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier          = id;
  TxHeader.IdType              = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
  TxHeader.DataLength          = (uint32_t)len;   // 0..8 = raw byte count in this HAL
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker       = 0;
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, (uint8_t*)data) == HAL_OK;
}

// Configure PB12 (FDCAN2_RX) and PB13 (FDCAN2_TX), AF9.
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin       = GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Point the FDCAN kernel clock at PCLK1 (APB1). This is common to all FDCAN
// instances and only needs to be done once. Without it the peripheral would
// run from the (absent) HSE.
static void MX_FDCAN_ClockConfig(void) {
  RCC_PeriphCLKInitTypeDef periphClk = {0};
  periphClk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  periphClk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&periphClk) != HAL_OK) {
    Error_Handler_CAN("FDCAN clock select failed");
  }
}

static void MX_FDCAN2_Init(void) {
  __HAL_RCC_FDCAN_CLK_ENABLE();

  // Compute the nominal-bit-time prescaler from the live PCLK1 so this stays
  // correct even if the core's clock tree changes. We use 20 time quanta per
  // bit: 1 (sync) + 15 (seg1) + 4 (seg2) -> 80% sample point.
  const uint32_t tqPerBit = 20;
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t prescaler = pclk1 / (CAN_BITRATE * tqPerBit);
  if (prescaler < 1) prescaler = 1;

  hfdcan2.Instance                  = FDCAN2;
  hfdcan2.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;   // classic CAN, not FD
  hfdcan2.Init.Mode                 = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission   = ENABLE;
  hfdcan2.Init.TransmitPause        = DISABLE;
  hfdcan2.Init.ProtocolException    = DISABLE;
  hfdcan2.Init.NominalPrescaler     = prescaler;             // ~17 @ 170 MHz PCLK1
  hfdcan2.Init.NominalSyncJumpWidth = 4;
  hfdcan2.Init.NominalTimeSeg1      = 15;                    // prop + phase1
  hfdcan2.Init.NominalTimeSeg2      = 4;                     // phase2
  // Data-phase fields are unused in classic mode but must be valid.
  hfdcan2.Init.DataPrescaler        = prescaler;
  hfdcan2.Init.DataSyncJumpWidth    = 4;
  hfdcan2.Init.DataTimeSeg1         = 15;
  hfdcan2.Init.DataTimeSeg2         = 4;
  hfdcan2.Init.StdFiltersNbr        = 0;                     // accept-all via global filter
  hfdcan2.Init.ExtFiltersNbr        = 0;
  hfdcan2.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
    Error_Handler_CAN("HAL_FDCAN_Init failed");
  }

  // Accept every standard/extended frame that doesn't match a filter into
  // RX FIFO0, and reject remote frames.
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    Error_Handler_CAN("global filter config failed");
  }

  Serial.print("FDCAN2 PCLK1=");
  Serial.print(pclk1);
  Serial.print(" Hz, prescaler=");
  Serial.print(prescaler);
  Serial.print(" -> bitrate=");
  Serial.print(pclk1 / (prescaler * tqPerBit));
  Serial.println(" bit/s");
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("ACB v2.0 - FDCAN2 CAN test (500 kbit/s, classic)");

  MX_GPIO_Init();
  MX_FDCAN_ClockConfig();
  MX_FDCAN2_Init();

  if (HAL_FDCAN_Start(&hfdcan2) == HAL_OK) {
    Serial.println("FDCAN2 started - sending ID 0x123 every 2 s, printing RX");
  } else {
    Error_Handler_CAN("HAL_FDCAN_Start failed");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // ---- Transmit every 2 s ----
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (canSend(CAN_MESSAGE_ID, CAN_DATA, 8)) {
      Serial.print("TX ok @ ");
      Serial.print(currentMillis);
      Serial.print(" ms - ID: 0x");
      Serial.print(CAN_MESSAGE_ID, HEX);
      Serial.print(", Data:");
      for (int i = 0; i < 8; i++) {
        Serial.print(CAN_DATA[i] < 16 ? " 0" : " ");
        Serial.print(CAN_DATA[i], HEX);
      }
      Serial.println();
    } else {
      // Most common cause here is no other node + no ACK with the Tx FIFO full.
      Serial.println("TX failed (FIFO full / bus error - is a second CAN node present?)");
    }
  }

  // ---- Receive: drain RX FIFO0 ----
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
      break;
    }

    uint8_t len = RxHeader.DataLength;  // raw byte count in this HAL (0..8)
    Serial.print("RX - ID: 0x");
    Serial.print(RxHeader.Identifier, HEX);
    Serial.print(" | Len: ");
    Serial.print(len);
    Serial.print(" | Data:");
    for (uint8_t i = 0; i < len && i < 8; i++) {
      Serial.print(RxData[i] < 16 ? " 0" : " ");
      Serial.print(RxData[i], HEX);
    }
    Serial.println();

    // Ping/pong: echo the payload back so the PC can confirm a round trip.
    if (RxHeader.IdType == FDCAN_STANDARD_ID && RxHeader.Identifier == PING_ID) {
      if (canSend(PONG_ID, RxData, len > 8 ? 8 : len)) {
        Serial.println("   -> echoed on 0x7E1");
      } else {
        Serial.println("   -> echo TX failed (Tx FIFO full)");
      }
    }
  }

  delay(1);
}

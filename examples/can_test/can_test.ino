// CHIP STM32G474RET6
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_can.h"

// CAN handle
CAN_HandleTypeDef hcan1;

// CAN message ID and data
const uint32_t CAN_MESSAGE_ID = 0x123;
const uint8_t CAN_DATA[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 2000; // 2 seconds in milliseconds

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);

void setup() {
  // Initialize HAL
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CAN1_Init();
  
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("CAN Test - Sending message every 2 seconds");
  
  // Start CAN
  if (HAL_CAN_Start(&hcan1) == HAL_OK) {
    Serial.println("CAN bus initialized successfully");
  } else {
    Serial.println("CAN bus initialization failed");
  }
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check if 2 seconds have passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Create CAN message
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    
    // Configure message header
    TxHeader.StdId = CAN_MESSAGE_ID;
    TxHeader.ExtId = 0x01;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
    
    // Copy data
    memcpy(TxData, CAN_DATA, sizeof(CAN_DATA));
    
    // Send the message
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    
    if (status == HAL_OK) {
      Serial.print("Message sent successfully at ");
      Serial.print(currentMillis);
      Serial.print(" ms - ID: 0x");
      Serial.print(CAN_MESSAGE_ID, HEX);
      Serial.print(", Data: ");
      for (int i = 0; i < 8; i++) {
        if (TxData[i] < 16) Serial.print("0");
        Serial.print(TxData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("Failed to send CAN message");
    }
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure the main internal regulator output voltage
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!HAL_PWREx_GetVoltageRange() == PWR_REGULATOR_VOLTAGE_SCALE1) {}

  // Initializes the RCC Oscillators according to the specified parameters
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    // Clock configuration failed
    while(1) {}
  }

  // Initializes the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    // Clock configuration failed
    while(1) {}
  }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  // Enable GPIO clocks
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  // Configure CAN pins
  // PB12 - CAN1_RX
  // PB13 - CAN1_TX
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_CAN1_Init(void) {
  CAN_FilterTypeDef sFilterConfig;
  
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    // CAN initialization failed
    while(1) {}
  }
  
  // Configure CAN filter
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
    // CAN filter configuration failed
    while(1) {}
  }
}

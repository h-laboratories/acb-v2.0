#include "hal_conf_extra.h"

#include "config.h"
#include "config_manager.h"
#include "MA730GQ.h"
#include "DRV8323RSRGZR.h"
#include "CommandManager.h"
#include <SimpleFOC.h>
#include <SPI.h>
#include "stm32g4xx_hal_fdcan.h"

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);  // Will be updated from config after loadConfig()

BLDCDriver6PWM driver = BLDCDriver6PWM(PWM_H_A, PWM_L_A, PWM_H_B, PWM_L_B, PWM_H_C, PWM_L_C);

LowsideCurrentSense current_sense = LowsideCurrentSense(SHUNT_RESISTANCE, CURRENT_GAIN, CURR_A, CURR_B, CURR_C);

/* ENCODER SETUP */
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, ENCODER_PPR);

/* MA730GQ SPI ENCODER SETUP */
MA730GQ spi_encoder = MA730GQ(MA730GQ_CS_PIN);

/* DRV8323RSRGZR SPI DRIVER SETUP */
DRV8323RSRGZR drv8323 = DRV8323RSRGZR(DRV8323_CS_PIN);

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doI(){encoder.handleIndex();}
/* ENCODER END */

// CAN message data
const uint32_t CAN_MESSAGE_ID = 0x123;
const uint8_t CAN_DATA[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

// CAN reception variables
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint32_t last_can_receive_time = 0;
uint32_t can_receive_count = 0;
/* USER CODE END PV */

// CAN
FDCAN_HandleTypeDef hfdcan2;


/* BUTTON INTERRUPT */
void buttonISR() {
  // Small delay to debounce the button
  delay(50);
  
  // Check if button is still pressed after debounce
  if (digitalRead(BUTTON) == LOW) {
    // Reset the STM32 when button is pressed
    NVIC_SystemReset();
  }
}
/* BUTTON END */

/* COMMAND MANAGER */
CommandManager command_manager(&motor, &spi_encoder);

/* BOARD MONITORING VARIABLES */
float board_temperature = 0.0f;
float bus_voltage = 0.0f;
float internal_temperature = 0.0f;

/* CURRENT MONITORING VARIABLES */
float current_a = 0.0f;
float current_b = 0.0f;
float current_c = 0.0f;

/* CAN MESSAGE TIMING VARIABLES */
unsigned long last_can_message_time = 0;
const unsigned long CAN_MESSAGE_INTERVAL = 1000; // 1 second in milliseconds


void IOSetup(){
  pinMode(DRV_EN, OUTPUT);
  pinMode(DRV_CAL, OUTPUT);
  pinMode(DRV_FAULT, INPUT_PULLUP);
  
  // Configure SPI CS pins
  pinMode(SPI_CS_POS, OUTPUT);
  pinMode(SPI_CS_DRV, OUTPUT);
  
  // Set CS pins high initially
  digitalWrite(SPI_CS_POS, HIGH);
  digitalWrite(SPI_CS_DRV, HIGH);

  // LED pins 
  pinMode(COM_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);

  // Board monitoring
  pinMode(BUS_V, INPUT);
  pinMode(TEMP, INPUT);

  // pinMode(CAN_RX, INPUT);
  // pinMode(CAN_TX, OUTPUT);
  
  // Button setup with interrupt
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, FALLING);
  
  delay(10);  // Allow sensor to stabilize
}

/**
 * Initialize SPI communication for MA730GQ encoder and DRV8323 driver
 */
void initSPI() {
  SPI.setMISO(SPI_MISO);
  SPI.setMOSI(SPI_MOSI);
  SPI.setSCLK(SPI_SCK);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV128);  
}

/**
 * Calculate board temperature from NTC thermistor reading
 * @return Temperature in Celsius
 */
float calculateBoardTemperature() {
  int adc_value = analogRead(TEMP);
  float voltage = (adc_value / 1023.0f) * 3.3f;  // Convert ADC to voltage (3.3V reference)
  
  // Calculate NTC resistance using voltage divider formula
  float ntc_resistance = (TEMP_R_FIXED * voltage) / (3.3f - voltage);
  
  // Calculate temperature using Steinhart-Hart equation
  float temp_kelvin = 1.0f / (1.0f/TEMP_NTC_B_CONSTANT * log(ntc_resistance/TEMP_NTC_NOMINAL) + 1.0f/298.15f);
  float temp_celsius = temp_kelvin - 273.15f;
  
  return temp_celsius;
}

/**
 * Calculate bus voltage from voltage divider reading
 * @return Bus voltage in Volts
 */
float calculateBusVoltage() {
  int adc_value = analogRead(BUS_V);
  float voltage = (adc_value / 1023.0f) * 3.3f;  // Convert ADC to voltage (3.3V reference)
  
  // Calculate actual bus voltage using voltage divider formula
  float actual_voltage = voltage / BUS_VOLTAGE_DIVIDER;
  
  return actual_voltage;
}

/**
 * TODO: Fix. 
 * Calculate internal STM32 temperature from internal sensor.
 * @return Internal temperature in Celsius
 */
float calculateInternalTemperature() {
  uint16_t raw_temp = analogRead(ATEMP);  // Internal temperature sensor
  uint16_t raw_vref = analogRead(AVREF);  // Internal VREFINT (1.212V typ)
  
  // Check if we're getting valid readings
  // if (raw_temp == 0 || raw_vref == 0) {
  //   Serial.println("Warning: Internal temperature sensor returning zero values");
  //   return 0.0f;
  // }
  
  // Load calibration values from system memory
  uint16_t TS_CAL1 = *(uint16_t*)TS_CAL1_ADDR;
  uint16_t TS_CAL2 = *(uint16_t*)TS_CAL2_ADDR;
  uint16_t VREFINT_CAL = *(uint16_t*)VREFINT_CAL_ADDR;

  // Calculate actual VDDA (in V)
  float vdda = 3.0f * (float)VREFINT_CAL / (float)raw_vref;

  // Scale the raw temperature reading to equivalent at VDDA=3V
  float raw_temp_scaled = (float)raw_temp * (vdda / 3.0f);

  // Calculate temperature using two-point calibration formula
  float temperature = ((TS_CAL2_TEMP - TS_CAL1_TEMP) / ((float)TS_CAL2 - (float)TS_CAL1)) *
                      (raw_temp_scaled - (float)TS_CAL1) + TS_CAL1_TEMP;

  
  // return temperature;
  return -1;
}

/**
 * Check angle limits and switch to position mode if exceeded
 * This function monitors the motor position when in velocity or torque mode
 * and automatically switches to position mode at the nearest limit if exceeded
 */
void check_angle_limits() {
  // Only check limits if motor is in velocity or torque mode
  if (motor.controller != MotionControlType::velocity && 
      motor.controller != MotionControlType::torque) {
    return;
  }
  
  // Get current position in degrees
  float current_position = motor.shaft_angle * 180.0f / PI;
  
  // Check if position is outside limits
  if (current_position < acb_config.min_angle) {
    // Position is below minimum, switch to position mode at minimum
    motor.target = acb_config.min_angle * PI / 180.0f;
    motor.controller = MotionControlType::angle;
    
    Serial.print("set_position ");
    Serial.println(acb_config.min_angle);
  } else if (current_position > acb_config.max_angle) {
    // Position is above maximum, switch to position mode at maximum
    motor.target = acb_config.max_angle * PI / 180.0f;
    motor.controller = MotionControlType::angle;
    
    Serial.print("set_position ");
    Serial.println(acb_config.max_angle);
  }
}


// // CAN
void initCAN(){

  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 160;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Serial.println("Failed to init CAN");
    Error_Handler();
  }

  GPIO_InitTypeDef GPIO_InitStruct = {};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
  HAL_RCCEx_GetPeriphCLKConfig(&PeriphClkInit);

  PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Serial.println("Failed to init CAN clock");
    Error_Handler();
    
  }
  __HAL_RCC_FDCAN_CLK_ENABLE();

  // __HAL_RCC_GPIOB_CLK_ENABLE();
  /**FDCAN2 GPIO Configuration
  PB12     ------> FDCAN2_RX
  PB13     ------> FDCAN2_TX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
    Serial.println("Failed to start CAN");
    Error_Handler();
  }
}
/**
 * @brief Send CAN message over FDCAN2
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise error code
 */
HAL_StatusTypeDef SendCANMessage(void)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  
  // Configure message header
  TxHeader.Identifier = CAN_MESSAGE_ID;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  
  // Copy data
  for (int i = 0; i < 8; i++) {
    TxData[i] = CAN_DATA[i];
  }
  // Serial.println("Sending CAN message");
  // Send the message
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}

/**
 * @brief Receive CAN message from FDCAN2
 * @retval HAL_StatusTypeDef HAL_OK if message received, HAL_ERROR if no message, otherwise error code
 */
HAL_StatusTypeDef ReceiveCANMessage(void)
{
  HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData);
  
  if (status == HAL_OK) {
    // Message received successfully
    can_receive_count++;
    last_can_receive_time = HAL_GetTick();
    
    // Optional: Process received message here
    // You can add your message processing logic here
    // For example: check message ID, process data, etc.
  }
  
  return status;
}
/* USER CODE END 0 */

void setup() {
  _delay(1000);
  
  Serial.begin(SERIAL_BAUD_RATE);
  _delay(10);
  Serial.println("---------- ACB BOOTING ----------");
  
  // Setups + loads
  initSPI();
  IOSetup();
  loadConfig();

  // initCAN();
  // Setup CAN
  // can_bus.begin(true);
  // can_bus.setBaudRate(CAN_RATE);


  spi_encoder.init();
  drv8323.init();

  board_temperature = calculateBoardTemperature();
  bus_voltage = calculateBusVoltage();
  internal_temperature = calculateInternalTemperature();

  if (bus_voltage > 30 || bus_voltage < 11.5){
    Serial.print("ACBv2.0 limited to 30V. (Detected ");
    Serial.print(bus_voltage,2);
    Serial.println("v)");
    Serial.println("Please use an input voltage to 12v-30v");
    exit(1);
  }
    
  
  // Initialize encoder
  encoder.quadrature = Quadrature::ON;
  encoder.init();
  
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);
  
  // Configure driver
  driver.voltage_power_supply = bus_voltage;
  driver.voltage_limit = 4;
  driver.pwm_frequency = DEFAULT_PWM_FREQUENCY;

  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  
  // Link current sense
  current_sense.linkDriver(&driver);
  motor.linkDriver(&driver);
  
  // Configure motor
  motor.voltage_limit = driver.voltage_limit/2;
  motor.voltage_sensor_align = 1;
  motor.velocity_index_search = 5;
  motor.pole_pairs = acb_config.pole_pairs;
  
  // Set up motion control
  motor.controller = MotionControlType::velocity;
  // motor.torque_controller = TorqueControlType::foc_current;
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
  motor.LPF_velocity = 0.05;
  motor.LPF_angle = 0.05;
  motor.LPF_current_d = 0.05;
  motor.LPF_current_q = 0.05;

  motor.target = 0;  

  // Apply PID settings from configuration
  motor.PID_velocity.P = acb_config.velocity_p;
  motor.PID_velocity.I = acb_config.velocity_i;
  motor.PID_velocity.D = acb_config.velocity_d;
  
  motor.P_angle.P = acb_config.angle_p;
  motor.P_angle.I = acb_config.angle_i;
  motor.P_angle.D = acb_config.angle_d;
  
  motor.PID_current_q.P = acb_config.current_p;
  motor.PID_current_q.I = acb_config.current_i;
  motor.PID_current_q.D = acb_config.current_d;
    
  motor.PID_current_d.P = acb_config.current_p;
  motor.PID_current_d.I = acb_config.current_i;
  motor.PID_current_d.D = acb_config.current_d;

  // Apply sensor calibration values from configuration
  motor.sensor_direction = (acb_config.sensor_direction == 1) ? Direction::CW : Direction::CCW;

  // Calibrate driver
  digitalWrite(DRV_EN, HIGH);
  delay(10);
  digitalWrite(DRV_CAL, HIGH);
  delayMicroseconds(100);
  digitalWrite(DRV_CAL, LOW);
  
  // Initialize motor
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
  delay(10);
  current_sense.skip_align = true; // This stops sstartup movement, but the encoder.update() in theory should be fine if align enabled.
  current_sense.init();
  
  motor.linkCurrentSense(&current_sense);

  
  // Apply absolute angle correction
  // Electrical angle = (pole_pairs * mechanical angle) + offset  
  float current_absolute_angle = spi_encoder.getAngleRadians();
  encoder.update(); // This is to handle any movement during startup
  float current_relative_angle = encoder.getMechanicalAngle();
  
  motor.controller = MotionControlType::velocity;
  motor.target = 0;
  motor.LPF_velocity = 0.05;
  motor.LPF_angle = 0.05;
  
  float zero_electric_calibrated = acb_config.zero_electric_angle-((current_absolute_angle-current_relative_angle) * acb_config.pole_pairs);
  zero_electric_calibrated = fmod(zero_electric_calibrated, 2 * PI);
  
  if(zero_electric_calibrated < 0) {
    zero_electric_calibrated += 2*PI;
  }
  motor.zero_electric_angle = zero_electric_calibrated;
  
  motor.initFOC();

  
  // Disable and re-enable driver
  drv8323.resetFaults();
  
  motor.disable();

  Serial.println("ACB v2.0 Firmware Ready");
  Serial.println("Open-Actuator Protocol Active, enter help for commands.");
 
  _delay(1000);
}

void loop() {  
  // Process serial commands
  // float loop_start_time = micros();
  command_manager.process_serial_commands();

  // Check angle limits and switch to position mode if exceeded
  check_angle_limits();

  // Update board monitoring values
  board_temperature = calculateBoardTemperature();
  bus_voltage = calculateBusVoltage();
  internal_temperature = calculateInternalTemperature();
  
  // Update driver power supply if bus voltage changes significantly
  static float last_bus_voltage = bus_voltage;
  if (abs(bus_voltage - last_bus_voltage) > 0.1f) {
    driver.voltage_power_supply = bus_voltage;
    last_bus_voltage = bus_voltage;
  }
  
  /* TODO: Handle driver faults */
  
  // Send CAN message every second
  // unsigned long current_time = millis();
  // if (current_time - last_can_message_time >= CAN_MESSAGE_INTERVAL) {
  //   if(SendCANMessage() == HAL_OK) {
      
  //   }
  //     last_can_message_time = current_time;
  // }
  //   CAN_message_t can_msg;
  //   can_msg.id = 0x00;
  //   can_msg.len = 8;
  //   can_msg.buf[0] = 0x00;
  //   can_msg.buf[1] = 0x11;
  //   can_msg.buf[2] = 0x22;
  //   can_msg.buf[3] = 0x33;
  //   can_msg.buf[4] = 0x44;
  //   can_msg.buf[5] = 0x55;
  //   can_msg.buf[6] = 0x66;
  //   can_msg.buf[7] = 0x77;
  //   can_bus.write(can_msg);
  
  // }
  
  motor.loopFOC();
  motor.move();
}

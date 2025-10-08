#include "config.h"
#include "config_manager.h"
#include "MA730GQ.h"
#include "DRV8323RSRGZR.h"
#include "CommandManager.h"
#include <SimpleFOC.h>
#include <SPI.h>

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


/* IWDG (Independent Watchdog) Functions */

/**
 * Initialize the Independent Watchdog Timer (IWDG)
 * This watchdog will reset the STM32 if not refreshed within the timeout period
 */
void initIWDG() {
  // Enable IWDG clock (LSI oscillator should be enabled by default)
  // LSI is typically 32kHz on STM32G474
  
  // Unlock IWDG registers
  IWDG->KR = 0x5555;
  
  // Set prescaler (64 prescaler gives ~32kHz/64 = 500Hz)
  IWDG->PR = 64;
  
  // Set reload value for 2 second timeout
  IWDG->RLR = IWDG_RELOAD_VALUE;
  
  // Lock IWDG registers
  IWDG->KR = 0x0000;
  
  // Start the watchdog
  IWDG->KR = 0xCCCC;
  
  Serial.println("IWDG initialized - 2 second timeout");
}

/**
 * Refresh the Independent Watchdog Timer
 * This must be called regularly to prevent system reset
 * If this function is not called within the timeout period (2 seconds),
 * the STM32 will automatically reset, helping recover from software issues
 */
void refreshIWDG() {
  IWDG->KR = 0xAAAA;  // Refresh watchdog counter
}

/**
 * Test function to demonstrate IWDG functionality
 * WARNING: This will cause a system reset after 2 seconds
 * Only use for testing purposes
 */
void testIWDG() {
  Serial.println("IWDG Test: System will reset in 2 seconds...");
  Serial.println("Stopping watchdog refresh to trigger reset");
  // Do not call refreshIWDG() in the main loop to test watchdog reset
}

/* FAULTS*/


// HardFault handler (catches unhandled faults)
void HardFault_Handler(void) {
    // Read fault status registers for diagnosis
    volatile uint32_t cfsr = SCB->CFSR;  // Configurable Fault Status Register
    volatile uint32_t hfsr = SCB->HFSR;  // HardFault Status Register
    volatile uint32_t mmfar = SCB->MMFAR;  // MemManage Fault Address (if applicable)
    volatile uint32_t bfar = SCB->BFAR;    // BusFault Address (if applicable)
    Serial.println("HARD FAULT");
        NVIC_SystemReset();


    // Optional: Toggle an LED or send via UART for indication
    // e.g., HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Assuming LED on PA5

    NVIC_SystemReset();

    // Infinite loop to halt (use debugger to inspect variables here)
    while (1) {}
}

// Memory Management Fault (e.g., invalid memory access)
void MemManage_Handler(void) {
  Serial.println("MEM FAULT");
      NVIC_SystemReset();

    // Similar to above: Read SCB->CFSR for details (MMFSR bits)
    while (1) {}
}

// Bus Fault (e.g., invalid address during load/store)
void BusFault_Handler(void) {
  Serial.println("BUS FAULT");
      NVIC_SystemReset();

    // Read SCB->CFSR (BFSR bits) and SCB->BFAR
    while (1) {}
}

// Usage Fault (e.g., undefined instruction, divide by zero)
void UsageFault_Handler(void) {
    // Read SCB->CFSR (UFSR bits)
        NVIC_SystemReset();

    Serial.println("USAGE FAULT");
    while (1) {}
}

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
CommandManager command_manager(&motor);

/* BOARD MONITORING VARIABLES */
float board_temperature = 0.0f;
float bus_voltage = 0.0f;
float internal_temperature = 0.0f;

/* CURRENT MONITORING VARIABLES */
float current_a = 0.0f;
float current_b = 0.0f;
float current_c = 0.0f;


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
  
  // Button setup with interrupt
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, FALLING);
  
  // Enable internal temperature sensor for STM32G474
  // ADC12_COMMON->CCR |= ADC_CCR_VSENSESEL;  // Enable temperature sensor
  delay(10);  // Allow sensor to stabilize
  
  // Note: STM32G474 has 12-bit ADC, but analogRead() may be configured for 10-bit
  // If readings are still zero, check ADC resolution configuration
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

void setup() {
  IOSetup();
  
  // Initialize Independent Watchdog Timer (IWDG)
  initIWDG();
  
  // Enable UsageFault, BusFault, and MemManage fault handlers
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;
  
  int c1 = 0;
  int c2 = 1 / c1;


  Serial.begin(SERIAL_BAUD_RATE);
  // Initialize SPI communication
  // initSPI();
  SimpleFOCDebug::enable(&Serial);

  
  // Initialize MA730GQ encoder
  spi_encoder.init();
  
  // Initialize DRV8323 driver
  drv8323.init();

  // Calculate initial board temperature, bus voltage, and internal temperature
  board_temperature = calculateBoardTemperature();
  bus_voltage = calculateBusVoltage();
  internal_temperature = calculateInternalTemperature();
  // Load configuration from EEPROM
  loadConfig();
  
  // Update motor pole pairs from configuration
  motor.pole_pairs = acb_config.pole_pairs;
  // driver.dead_zone = 0.03;
  
  // Initialize encoder
  encoder.quadrature = Quadrature::ON;
  encoder.init();
  
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  // current_sense.init();
  
  // Configure driver
  driver.voltage_power_supply = bus_voltage;
  // driver.voltage_power_supply = 12;
  driver.voltage_limit = 4;
  driver.pwm_frequency = DEFAULT_PWM_FREQUENCY;

  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  
  // Link current sense
  // current_sense.linkDriver(&driver);
  motor.linkDriver(&driver);
  
  // Configure motor limits
  motor.voltage_limit = driver.voltage_limit/2;
  motor.voltage_sensor_align = 2;
  motor.velocity_index_search = 5;
  
  // Set up motion control
  motor.controller = MotionControlType::velocity;
  
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
  
  // Apply sensor calibration values from configuration
  motor.zero_electric_angle = acb_config.zero_electric_angle;
  motor.sensor_direction = (acb_config.sensor_direction == 1) ? Direction::CW : Direction::CCW;

  
  // Initialize current sense
  // current_sense.init();
  // motor.linkCurrentSense(&current_sense);
  delay(10);
  // Calibrate driver
  digitalWrite(DRV_CAL, HIGH);
  delay(1);
  digitalWrite(DRV_CAL, LOW);
  delay(10);
  digitalWrite(DRV_EN, HIGH);
  delay(100);
  
  // Initialize motor
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
  _delay(1000);
  
  motor.initFOC();
  
  // Disable and re-enable driver
  drv8323.resetFaults();
  
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.target = 0;
  motor.LPF_velocity = 0.05;
  motor.LPF_angle = 0.05;
  motor.disable();

  motor.LPF_current_d = 0.05;
  motor.LPF_current_q = 0.05;

  Serial.println("ACB v2.0 Firmware Ready");
  Serial.println("Open-Actuator Protocol Active");
  Serial.println("Commands: set_position, set_velocity, set_torque, get_position, get_velocity, get_torque, enable, disable, home, stop, reset_position, cmd_mode, get_full_state");
  Serial.println("PID Commands: get_velocity_pid, set_velocity_pid, get_angle_pid, set_angle_pid, get_current_pid, set_current_pid, save_config");
  Serial.println("Motion Commands: get_downsample, set_downsample");
  Serial.println("Motor Commands: get_pole_pairs, set_pole_pairs");
  Serial.println("Monitoring Commands: get_temperature, get_bus_voltage, get_internal_temperature, get_current_a, get_current_b, get_current_c, drv8323_fault_check");
  Serial.println("Calibration Commands: recalibrate_sensors");
  _delay(1000);
}

void loop() {
  // Refresh Independent Watchdog Timer to prevent reset
  refreshIWDG();
  
  // Process serial commands
  // float loop_start_time = micros();
  command_manager.process_serial_commands();

  // Update board monitoring values
  // board_temperature = calculateBoardTemperature();
  // bus_voltage = calculateBusVoltage();
  // internal_temperature = calculateInternalTemperature();

  // PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  // current_sense.getABCurrents(PhaseCurrent_s current);
  // current_a = currents.a;
  // current_b = currents.b;
  // current_c = currents.c;
  // current_a = current_sense.getDCCurrent();
  
  // Update driver power supply if bus voltage changes significantly
  // static float last_bus_voltage = bus_voltage;
  // if (abs(bus_voltage - last_bus_voltage) > 0.1f) {
  //   driver.voltage_power_supply = bus_voltage;
  //   last_bus_voltage = bus_voltage;
  // }
  
  /* TODO: Handle driver faults */
  
  motor.loopFOC();
  motor.move();
}

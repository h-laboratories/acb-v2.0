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

void setup() {
  _delay(1000);
  
  Serial.begin(SERIAL_BAUD_RATE);
  
  // Setups + loads
  initSPI();
  IOSetup();
  loadConfig();

  spi_encoder.init();
  drv8323.init();

  board_temperature = calculateBoardTemperature();
  bus_voltage = calculateBusVoltage();
  internal_temperature = calculateInternalTemperature();

  if (bus_voltage > 30 || bus_voltage < 12){
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
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
  motor.LPF_velocity = 0.05;
  motor.LPF_angle = 0.05;
  motor.LPF_current_d = 0.005;
  motor.LPF_current_q = 0.005;

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
  // Initialize current sense
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
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
  
  current_sense.init();
  current_sense.skip_align = true; // This stops sstartup movement, but the encoder.update() in theory should be fine if align enabled.
  motor.linkCurrentSense(&current_sense);

  motor.initFOC();
  
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

  Serial.print("Applied absolute angle calibration correction: ");
  Serial.print(motor.zero_electric_angle);
  Serial.println(" rad");

  
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
  
  motor.loopFOC();
  motor.move();
}

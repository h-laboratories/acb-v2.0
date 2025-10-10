#include "config_manager.h"

// Global configuration instance
ACBConfig acb_config;

/**
 * Initialize configuration with default values
 */
void initConfig() {
  acb_config.magic_number = EEPROM_CONFIG_MAGIC_NUMBER;
  
  // PID Configuration defaults
  acb_config.velocity_p = DEFAULT_VELOCITY_P;
  acb_config.velocity_i = DEFAULT_VELOCITY_I;
  acb_config.velocity_d = DEFAULT_VELOCITY_D;
  acb_config.angle_p = DEFAULT_ANGLE_P;
  acb_config.angle_i = DEFAULT_ANGLE_I;
  acb_config.angle_d = DEFAULT_ANGLE_D;
  acb_config.current_p = DEFAULT_CURRENT_P;
  acb_config.current_i = DEFAULT_CURRENT_I;
  acb_config.current_d = DEFAULT_CURRENT_D;
  
  // Sensor Calibration defaults
  acb_config.zero_electric_angle = 0.0f;
  acb_config.sensor_direction = 1;  // Default to CW direction
  
  // Motor Configuration defaults
  acb_config.pole_pairs = DEFAULT_POLE_PAIRS;
  
  // Angle Limits Configuration defaults
  acb_config.min_angle = DEFAULT_MIN_ANGLE;
  acb_config.max_angle = DEFAULT_MAX_ANGLE;
  
  // Add more default values here as needed
  // Example:
  // acb_config.motor_voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  // acb_config.encoder_ppr = ENCODER_PPR;
  // acb_config.debug_mode = false;
}

/**
 * Load configuration from EEPROM
 * @return true if valid configuration was loaded, false if defaults were used
 */
bool loadConfig() {

  EEPROM.get(EEPROM_CONFIG_START_ADDR, acb_config);
  
  // Check if magic number matches (valid data)
  if (acb_config.magic_number != EEPROM_CONFIG_MAGIC_NUMBER) {
    Serial.println("No valid config found, using defaults");
    initConfig();
    saveConfig();
    return false;
  }
  
  Serial.println("Configuration loaded from EEPROM");
  printConfig();
  return true;
}

/**
 * Save configuration to EEPROM
 */
void saveConfig() {
  EEPROM.put(EEPROM_CONFIG_START_ADDR, acb_config);
  // Serial.println("Configuration saved to EEPROM");
}

/**
 * Reset configuration to default values
 */
void resetConfig() {
  initConfig();
  saveConfig();
  Serial.println("Configuration reset to defaults");
}

/**
 * Print current configuration
 */
void printConfig() {
  Serial.println("Current Configuration:");
  Serial.print("Velocity PID: P=");
  Serial.print(acb_config.velocity_p);
  Serial.print(", I=");
  Serial.print(acb_config.velocity_i);
  Serial.print(", D=");
  Serial.println(acb_config.velocity_d);
  
  Serial.print("Angle PID: P=");
  Serial.print(acb_config.angle_p);
  Serial.print(", I=");
  Serial.print(acb_config.angle_i);
  Serial.print(", D=");
  Serial.println(acb_config.angle_d);
  
  Serial.print("Current PID: P=");
  Serial.print(acb_config.current_p);
  Serial.print(", I=");
  Serial.print(acb_config.current_i);
  Serial.print(", D=");
  Serial.println(acb_config.current_d);
  
  Serial.print("Sensor Calibration: Zero Electric Angle=");
  Serial.print(acb_config.zero_electric_angle);
  Serial.print(", Sensor Direction=");
  Serial.println(acb_config.sensor_direction);
  
  Serial.print("Motor Configuration: Pole Pairs=");
  Serial.println(acb_config.pole_pairs);
  
  Serial.print("Angle Limits: Min=");
  Serial.print(acb_config.min_angle);
  Serial.print(", Max=");
  Serial.println(acb_config.max_angle);
  
  // Add more configuration printing here as needed
}

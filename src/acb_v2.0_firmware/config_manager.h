#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <EEPROM.h>
#include "config.h"

/* GENERIC CONFIGURATION STRUCTURE */
struct ACBConfig {
  uint16_t magic_number;  // Magic number to verify valid data
  
  // PID Configuration
  float velocity_p;
  float velocity_i;
  float velocity_d;
  float angle_p;
  float angle_i;
  float angle_d;
  float current_p;
  float current_i;
  float current_d;
  
  // Sensor Calibration Values
  float zero_electric_angle;
  int sensor_direction;
  
  // Motor Configuration
  int pole_pairs;
  
  // Angle Limits Configuration
  float min_angle;  // Minimum allowed angle in degrees
  float max_angle;  // Maximum allowed angle in degrees
  
  // Add more configuration variables here as needed
  // Example:
  // float motor_voltage_limit;
  // uint16_t encoder_ppr;
  // bool debug_mode;
};

extern ACBConfig acb_config;

/* CONFIGURATION MANAGEMENT FUNCTIONS */

/**
 * Initialize configuration with default values
 */
void initConfig();

/**
 * Load configuration from EEPROM
 * @return true if valid configuration was loaded, false if defaults were used
 */
bool loadConfig();

/**
 * Save configuration to EEPROM
 */
void saveConfig();

/**
 * Reset configuration to default values
 */
void resetConfig();

/**
 * Print current configuration
 */
void printConfig();

#endif // CONFIG_MANAGER_H

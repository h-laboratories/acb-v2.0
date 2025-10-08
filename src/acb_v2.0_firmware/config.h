#include <SimpleFOC.h>
#ifndef ACB_DEFINITIONS_H
#define ACB_DEFINITIONS_H

// ACB v2.0 Pin Definitions
// This file contains all pin definitions and constants for the ACB v2.0 motor controller

// PWM Pin Definitions
#define PWM_H_A PC0 
#define PWM_L_A PC13
#define PWM_H_B PC1
#define PWM_L_B PB0
#define PWM_H_C PC2
#define PWM_L_C PB1

// Encoder Pin Definitions
#define ENCODER_A PA15
#define ENCODER_B PB3
#define ENCODER_Z PC10
#define ENCODER_PPR 1024

// Current Sense Pin Definitions
#define CURR_A  PA0
#define CURR_B  PA1
#define CURR_C  PA2

// Board monitoring
#define BUS_V PB15
#define TEMP  PA8
#define BUS_VOLTAGE_DIVIDER (1e3 / (1e3 + 33e3))
#define TEMP_NTC_NOMINAL 10e3
#define TEMP_NTC_B_CONSTANT 4200
#define TEMP_R_FIXED 10e3
#define BUTTON PB8

// Internal STM32 ADC channels
#define ATEMP 16  // Internal temperature sensor channel
#define AVREF 0   // Internal voltage reference channel

// Calibration memory addresses (from STM32G474 reference manual/datasheet)
#define TS_CAL1_ADDR 0x1FFF75A8  // TS_CAL1 at 30°C, VDDA=3V
#define TS_CAL2_ADDR 0x1FFF75CA  // TS_CAL2 at 130°C, VDDA=3V
#define VREFINT_CAL_ADDR 0x1FFF75AA  // VREFINT_CAL at VDDA=3V

// Calibration temperatures
#define TS_CAL1_TEMP 30.0f
#define TS_CAL2_TEMP 130.0f

// LEDs
#define COM_LED     PC7
#define STATUS_LED  PC6

// Driver Control Pin Definitions
#define DRV_FAULT PB10
#define DRV_EN  PB9
#define DRV_CAL PB7

// SPI Pin Definitions
#define SPI_MOSI PA7
#define SPI_MISO PA6
#define SPI_SCK PA5
#define SPI_CS_DRV PB5
#define SPI_CS_POS PB6

// MA730GQ Encoder Configuration
#define MA730GQ_CS_PIN SPI_CS_POS  // Use the same CS pin as defined for SPI_CS_POS

// DRV8323RSRGZR Driver Configuration
#define DRV8323_CS_PIN SPI_CS_DRV  // Use the same CS pin as defined for SPI_CS_DRV


// Motor Configuration Constants
#define MOTOR_POLE_PAIRS 19
#define DEFAULT_POLE_PAIRS 19

// Current Sense Configuration
#define SHUNT_RESISTANCE 0.003f  // Ohms
#define CURRENT_GAIN 20.0f       // Current sense amplifier gain

// Driver Configuration
#define DEFAULT_PWM_FREQUENCY 20000
#define DEFAULT_VOLTAGE_LIMIT 4

// Serial Communication
#define SERIAL_BAUD_RATE 115200
#define DEBUG_SERIAL_BAUD_RATE 2000000

// Timing Constants
#define DRV_CAL_DELAY_MS 10
#define MOTOR_READY_DELAY_MS 1000
#define PRINT_INTERVAL_MS 5

// Motion Control Types
#define DEFAULT_CONTROLLER MotionControlType::velocity

// Default Target Values
#define DEFAULT_TARGET_VELOCITY 40.0f
#define DEFAULT_VOLTAGE_LIMIT_FACTOR 0.5f

// Default PID Values
#define DEFAULT_VELOCITY_P 0.25f
#define DEFAULT_VELOCITY_I 1.0f
#define DEFAULT_VELOCITY_D 0.0f
#define DEFAULT_ANGLE_P 1.0f
#define DEFAULT_ANGLE_I 1.0f
#define DEFAULT_ANGLE_D 0.0f
#define DEFAULT_CURRENT_P 0.5f
#define DEFAULT_CURRENT_I 0.1f
#define DEFAULT_CURRENT_D 0.0f

// EEPROM Storage Addresses
#define EEPROM_CONFIG_START_ADDR 0
#define EEPROM_CONFIG_MAGIC_NUMBER 0xACB2

// IWDG (Independent Watchdog) Configuration
#define IWDG_TIMEOUT_MS 2000  // 2 second timeout
#define IWDG_PRESCALER IWDG_PRESCALER_64  // 64 prescaler for ~32kHz LSI
#define IWDG_RELOAD_VALUE ((IWDG_TIMEOUT_MS * 32000) / (64 * 1000))  // Calculate reload value

#endif // ACB_DEFINITIONS_H

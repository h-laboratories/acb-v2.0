// MA730GQ SPI Encoder Test Example
// This example demonstrates how to use the MA730GQ sensor class
// to read angle data and configure the encoder registers

#include <SimpleFOC.h>
#include <SPI.h>

// Include the MA730GQ sensor class
#include "../../src/acb_v2.0_firmware/MA730GQ.h"
#include "../../src/acb_v2.0_firmware/config.h"

// Create MA730GQ encoder instance
MA730GQ spi_encoder = MA730GQ(MA730GQ_CS_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("MA730GQ SPI Encoder Test");
  
  // Initialize SPI communication
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  
  // Initialize MA730GQ encoder
  spi_encoder.init();
  
  // Read and display register h5 (0x5) from MA730GQ encoder
  uint16_t h5_value = spi_encoder.readRegister(0x5);
  Serial.print("MA730GQ Register h5 value: 0x");
  if (h5_value < 0x1000) Serial.print("0");
  if (h5_value < 0x100) Serial.print("0");
  if (h5_value < 0x10) Serial.print("0");
  Serial.println(h5_value, HEX);
  Serial.print("MA730GQ Register h5 value (decimal): ");
  Serial.println(h5_value);
  
  // Example: Configure encoder settings
  Serial.println("Configuring MA730GQ encoder...");
  
  // Set ppt to 128 (pulses per turn)
  uint16_t ppt_val = 128-1;
  spi_encoder.writeRegister(0x04, ppt_val << 6);
  spi_encoder.writeRegister(0x05, 0xFF & (ppt_val >> 2));
  spi_encoder.writeRegister(0x10, 200);
  
  // Write register 0x6 with configuration value
  spi_encoder.writeRegister(0x6, 0b10010000);
  
  // Verify the write by reading back
  uint8_t reg6_value = spi_encoder.readRegister(0x6);
  Serial.print("Register 0x6 value after write: 0x");
  Serial.println(reg6_value, HEX);
  
  Serial.println("MA730GQ encoder ready!");
  Serial.println("Reading angle data...");
  Serial.println("Angle (radians)");
}

void loop() {
  static unsigned long last_print = 0;
  
  unsigned long now = millis();
  if (now - last_print >= 10) {  // Print every 10ms
    // Read raw angle from MA730GQ via SPI
    uint16_t raw_angle = spi_encoder.readAngle();
    
    // Convert angle to radians
    float raw_radians = spi_encoder.angleToRadians(raw_angle);
    
    // Print angle in radians
    Serial.print(raw_radians, 4);
    Serial.println();
    
    last_print = now;
  }
}


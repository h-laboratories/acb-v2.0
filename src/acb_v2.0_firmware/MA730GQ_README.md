# MA730GQ SPI Encoder Driver

This module provides a C++ class interface for the MA730GQ magnetic encoder over SPI communication.

## Features

- **SPI Communication**: Full SPI protocol implementation for MA730GQ encoder
- **Register Access**: Read and write encoder configuration registers
- **Angle Reading**: High-resolution 16-bit angle reading
- **Filtering**: Built-in velocity-based and low-pass filtering for noisy readings
- **Angle Conversion**: Convert between raw values, radians, and degrees

## Hardware Connections

The MA730GQ encoder should be connected as follows:

| MA730GQ Pin | Arduino Pin | Description |
|-------------|-------------|-------------|
| VCC         | 3.3V/5V     | Power supply |
| GND         | GND         | Ground |
| CS          | PB6         | Chip select (SPI_CS_POS) |
| SCK         | PA5         | SPI clock (SPI_SCK) |
| MISO        | PA6         | SPI data out (SPI_MISO) |
| MOSI        | PA7         | SPI data in (SPI_MOSI) |

## Usage

### Basic Setup

```cpp
#include "MA730GQ.h"

// Create encoder instance
MA730GQ spi_encoder = MA730GQ(MA730GQ_CS_PIN);

void setup() {
  // Initialize SPI communication
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  
  // Initialize MA730GQ encoder
  spi_encoder.init();
}

void loop() {
  // Read angle in radians
  float angle_rad = spi_encoder.getAngleRadians();
  
  // Read angle in degrees
  float angle_deg = spi_encoder.getAngleDegrees();
}
```

### Register Operations

```cpp
// Read a register
uint16_t reg_value = spi_encoder.readRegister(0x05);

// Write to a register
spi_encoder.writeRegister(0x06, 0b10010000);
```


## Configuration

The encoder can be configured by writing to its registers:

```cpp
// Set pulses per turn to 128
uint16_t ppt_val = 128-1;
spi_encoder.writeRegister(0x04, ppt_val << 6);
spi_encoder.writeRegister(0x05, 0xFF & (ppt_val >> 2));

// Configure other settings
spi_encoder.writeRegister(0x10, 200);
spi_encoder.writeRegister(0x06, 0b10010000);
```

## API Reference

### Constructor
- `MA730GQ(uint8_t cs_pin)` - Create encoder instance with chip select pin

### Initialization
- `void init()` - Initialize MA730GQ encoder (CS pin setup only)
- Note: SPI must be initialized separately in main code

### Angle Reading
- `uint16_t readAngle()` - Read raw 16-bit angle value
- `float getAngleRadians()` - Get filtered angle in radians
- `float getAngleDegrees()` - Get filtered angle in degrees
- `float angleToRadians(uint16_t angle_16bit)` - Convert 16-bit value to radians

### Register Operations
- `uint16_t readRegister(uint8_t reg_address)` - Read encoder register
- `void writeRegister(uint8_t reg_address, uint8_t data)` - Write encoder register

### Filtering

## Example

See `examples/ma730gq_test/ma730gq_test.ino` for a complete working example.

## Notes

- The encoder uses SPI Mode 3 (CPOL=1, CPHA=1)
- Clock divider is set to SPI_CLOCK_DIV128 for reliable communication
- Filtering parameters can be adjusted in the header file if needed
- The encoder provides 16-bit resolution (0-65535) for full 360° rotation


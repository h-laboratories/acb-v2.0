#include "MA730GQ.h"

MA730GQ::MA730GQ(uint8_t cs_pin) : _cs_pin(cs_pin) {
    // Initialize encoder
}

void MA730GQ::init() {
    // Set CS pin high initially (SPI must be initialized separately)
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    // Sets the Low/High sensitivity for the magnet
    writeRegister(
        0x06,
        (0b00000000 << 5) | (0b00000111 << 2)
    );
    // Sets the filter window (factory default = 119)
    writeRegister(
        0x0E,
        85
    );
    // Sets hysteresis (factory default = 156)
    writeRegister(
        0x10,
        156
    );

}

uint16_t MA730GQ::readAngle() {
    uint16_t angle = 0;
       
    // Pull CS low to start communication
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1); // Small delay for setup time
    
    // Send all zeros and receive angle
    angle = SPI.transfer16(0x0000);
    
    // Pull CS high to end communication
    digitalWrite(_cs_pin, HIGH);
    delayMicroseconds(1); // Small delay for hold time
    
    return angle;
}

uint16_t MA730GQ::readRegister(uint8_t reg_address) {
    uint8_t result = 0;
    uint16_t buff = 0;
    
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1);

    uint16_t read_request = 0x4000 | ((reg_address & 0x1F) << 8);
    SPI.transfer16(read_request);
    digitalWrite(_cs_pin, HIGH);

    delayMicroseconds(1);
    digitalWrite(_cs_pin, LOW);
    
    delayMicroseconds(1);
    buff = SPI.transfer16(0x0000);
    buff = (buff >> 8) & 0xFF;
    result = buff;
    
    // Pull CS high to end communication
    digitalWrite(_cs_pin, HIGH);
    delayMicroseconds(1); // Small delay for hold time
    
    return result;
}

void MA730GQ::writeRegister(uint8_t reg_address, uint8_t data) {
    // Pull CS low to start communication
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1); // Small delay for setup time
    
    // Frame 1: Send write request
    // Bits 15-13: Write command (100)
    // Bits 12-8: 5-bit register address
    // Bits 7-0: 8-bit data
    uint16_t write_request = 0x8000 | ((reg_address & 0x1F) << 8) | (data & 0xFF);
    SPI.transfer16(write_request);
    digitalWrite(_cs_pin, HIGH);
    delay(20);
    
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1); // Small delay for setup time
    
    // Frame 2: Dummy transaction (some devices require this)
    uint16_t res = SPI.transfer16(0x0000);
    if(res >> 8 != data){
        Serial.printf("Failed to write data! Got: %x, expected: %x\n", res, data);
    }
    
    // Pull CS high to end communication
    digitalWrite(_cs_pin, HIGH);
}



float MA730GQ::angleToRadians(uint16_t angle_16bit) {
    return ((float)angle_16bit) * 2.0 * PI / 65536.0;
}

float MA730GQ::getAngleRadians() {
    uint16_t raw_angle = readAngle();
    return angleToRadians(raw_angle);
}

float MA730GQ::getAngleDegrees() {
    return getAngleRadians() * 180.0 / PI;
}



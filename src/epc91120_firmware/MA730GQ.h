#ifndef MA730GQ_H
#define MA730GQ_H

#include <Arduino.h>
#include <SPI.h>

/**
 * MA730GQ SPI Encoder Driver
 * 
 * This class provides an interface to the MA730GQ magnetic encoder
 * over SPI communication. It includes functions for reading angle data,
 * reading/writing registers, and filtering noisy readings.
 */

class MA730GQ {
public:
    /**
     * Constructor for MA730GQ encoder
     * @param cs_pin: Chip select pin for the encoder
     */
    MA730GQ(uint8_t cs_pin);
    
    /**
     * Initialize MA730GQ encoder (CS pin setup only)
     * Note: SPI must be initialized separately in main code
     */
    void init();
    
    /**
     * Read angle from MA730GQ encoder over SPI
     * Single transaction: send all zeros, receive angle value
     * @return: 16-bit angle value
     */
    uint16_t readAngle();
    
    /**
     * Read a register from MA730GQ encoder over SPI
     * Uses two-frame protocol as per datasheet:
     * Frame 1: Read request (3-bit command 010 + 5-bit address + 8 zeros)
     * Frame 2: Receive 16-bit register value
     * @param reg_address: Register address to read (0x00-0x1F)
     * @return: 16-bit register value
     */
    uint16_t readRegister(uint8_t reg_address);
    
    /**
     * Write a register to MA730GQ encoder over SPI
     * Uses two-frame protocol as per datasheet:
     * Frame 1: Write request (3-bit command 100 + 5-bit address + 8-bit data)
     * Frame 2: Dummy transaction (send zeros, receive response)
     * @param reg_address: Register address to write (0x00-0x1F)
     * @param data: 8-bit data value to write
     */
    void writeRegister(uint8_t reg_address, uint8_t data);
    
    
    /**
     * Convert 16-bit angle value to radians
     * @param angle_16bit: 16-bit angle value (0-65535)
     * @return: Angle in radians (0 to 2π)
     */
    float angleToRadians(uint16_t angle_16bit);
    
    /**
     * Get the current angle in radians
     * @return: Current angle in radians
     */
    float getAngleRadians();
    
    /**
     * Get the current angle in degrees
     * @return: Current angle in degrees
     */
    float getAngleDegrees();
    

private:
    uint8_t _cs_pin;
    
};

#endif // MA730GQ_H


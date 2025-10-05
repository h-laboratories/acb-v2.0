#ifndef DRV8323RSRGZR_H
#define DRV8323RSRGZR_H

#include <Arduino.h>
#include <SPI.h>

/**
 * DRV8323RSRGZR SPI MOSFET Driver Manager
 * 
 * This class provides basic SPI communication interface to the DRV8323RSRGZR 
 * 3-phase gate driver. It implements the SPI protocol for reading and writing registers.
 * 
 * SPI Protocol:
 * - 16-bit transactions
 * - Bit 15: R/W (1=Read, 0=Write)
 * - Bits 14-11: 4-bit register address
 * - Bits 10-0: 11-bit data
 * - Response: First 5 bits are don't care, last 11 bits are data
 */

class DRV8323RSRGZR {
public:
    /**
     * Constructor for DRV8323RSRGZR driver
     * @param cs_pin: Chip select pin for the driver
     */
    DRV8323RSRGZR(uint8_t cs_pin);
    
    /**
     * Initialize DRV8323RSRGZR driver (CS pin setup only)
     * Note: SPI must be initialized separately in main code
     */
    void init();
    
    /**
     * Read a register from DRV8323RSRGZR over SPI
     * @param reg_address: Register address to read (0x00-0x0F)
     * @return: 11-bit register value
     */
    uint16_t readRegister(uint8_t reg_address);
    
    /**
     * Write a register to DRV8323RSRGZR over SPI
     * @param reg_address: Register address to write (0x00-0x0F)
     * @param data: 11-bit data value to write
     */
    void writeRegister(uint8_t reg_address, uint16_t data);
    
    /**
     * Check for faults and print human-readable fault information
     * @return: true if any faults detected, false otherwise
     */
    bool checkFaults();
    static const uint8_t FAULT_STATUS_1 = 0x00;
    static const uint8_t VGS_STATUS_2 = 0x01;
    static const uint8_t DRIVER_CONTROL = 0x02;
    static const uint8_t GATE_DRIVE_HS = 0x03;
    static const uint8_t GATE_DRIVE_LS = 0x04;
    static const uint8_t OCP_CONTROL = 0x05;
    static const uint8_t CSA_CONTROL = 0x06;

    // Bit positions
    static const uint8_t FAULT = 10;
    static const uint8_t VDS_OCP = 9;
    static const uint8_t GDF = 8;
    static const uint8_t UVLO = 7;
    static const uint8_t OTSD = 6;
    static const uint8_t VDS_HA = 5;
    static const uint8_t VDS_LA = 4;
    static const uint8_t VDS_HB = 3;
    static const uint8_t VDS_LB = 2;
    static const uint8_t VDS_HC = 1;
    static const uint8_t VDS_LC = 0;
    static const uint8_t SA_OC = 10;
    static const uint8_t SB_OC = 9;
    static const uint8_t SC_OC = 8;
    static const uint8_t OTW = 7;
    static const uint8_t CPUV = 6;
    static const uint8_t VGS_HA = 5;
    static const uint8_t VGS_LA = 4;
    static const uint8_t VGS_HB = 3;
    static const uint8_t VGS_LB = 2;
    static const uint8_t VGS_HC = 1;
    static const uint8_t VGS_LC = 0;
    static const uint8_t DIS_CPUV = 9;
    static const uint8_t DIS_GDF = 8;
    static const uint8_t OTW_REP = 7;
    static const uint8_t PWM_MODE = 5;
    static const uint8_t PWM_COM = 4;
    static const uint8_t PWM_DIR = 3;
    static const uint8_t COAST = 2;
    static const uint8_t BRAKE = 1;
    static const uint8_t CLR_FLT = 0;
    static const uint8_t LOCK = 8;
    static const uint8_t IDRIVEP_HS = 4;
    static const uint8_t IDRIVEN_HS = 0;
    static const uint8_t CBC = 10;
    static const uint8_t TDRIVE = 8;
    static const uint8_t IDRIVEP_LS = 4;
    static const uint8_t IDRIVEN_LS = 0;
    static const uint8_t TRETRY = 10;
    static const uint8_t DEAD_TIME = 8;
    static const uint8_t OCP_MODE = 6;
    static const uint8_t OCP_DEG = 4;
    static const uint8_t VDS_LVL = 0;
    static const uint8_t CSA_FET = 10;
    static const uint8_t VREF_DIV = 9;
    static const uint8_t LS_REF = 8;
    static const uint8_t CSA_GAIN = 6;
    static const uint8_t DIS_SEN = 5;
    static const uint8_t CSA_CAL_A = 4;
    static const uint8_t CSA_CAL_B = 3;
    static const uint8_t CSA_CAL_C = 2;
    static const uint8_t SEN_LVL = 0;

private:
    uint8_t _cs_pin;
};

#endif // DRV8323RSRGZR_H


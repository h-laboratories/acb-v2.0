#include "DRV8323RSRGZR.h"

DRV8323RSRGZR::DRV8323RSRGZR(uint8_t cs_pin) : _cs_pin(cs_pin) {
    // Initialize driver
}

void DRV8323RSRGZR::init() {
    // Set CS pin high initially (SPI must be initialized separately)
    // pinMode(_cs_pin, OUTPUT);
    // digitalWrite(_cs_pin, HIGH);
}

uint16_t DRV8323RSRGZR::readRegister(uint8_t reg_address) {    
    // digitalWrite(PB9, HIGH);
    // delay(1);
    // Pull CS low to start communication
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(10); // Small delay for setup time
    
    // Construct read command: R/W=1, 4-bit address, 11-bit data=0
    uint16_t read_cmd = 0x8000 | ((reg_address & 0x0F) << 11);
    read_cmd = 0x0000 | ((reg_address & 0x0F) << 11);
    uint16_t response = SPI.transfer16(read_cmd);
    // uint8_t res_1 = SPI.transfer(read_cmd >> 8);
    // uint8_t res_2 = SPI.transfer(read_cmd & 0xFF);

    // uint16_t response = (uint16_t)(res_1 << 8) | res_2;
    // Pull CS high to end communication
    Serial.print("RAW: ");
    Serial.print(response, HEX);
    Serial.println("");
    delayMicroseconds(10); // Small delay for hold time
    digitalWrite(_cs_pin, HIGH);
    // delay(1);
    // digitalWrite(PB9, LOW);

    // Extract 11-bit data from response (bits 10-0)
    return response & 0x07FF;
}

void DRV8323RSRGZR::writeRegister(uint8_t reg_address, uint16_t data) { 
    // Pull CS low to start communication
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1); // Small delay for setup time
    
    // Construct write command: R/W=0, 4-bit address, 11-bit data
    uint16_t write_cmd = ((reg_address & 0x0F) << 11) | (data & 0x07FF);
    SPI.transfer16(write_cmd);
    
    // Pull CS high to end communication
    digitalWrite(_cs_pin, HIGH);
    delayMicroseconds(1); // Small delay for hold time
}

bool DRV8323RSRGZR::checkFaults() {
    // Read fault status registers
    uint16_t fault_status_1 = readRegister(FAULT_STATUS_1);
    uint16_t vgs_status_2 = readRegister(VGS_STATUS_2);
    
    bool has_faults = false;
    
    // Check Fault Status 1 register
    if (fault_status_1 != 0) {
        Serial.println("DRV8323 Fault Status 1:");
        
        if (fault_status_1 & (1 << FAULT)) {
            Serial.println("  - General Fault");
            has_faults = true;
        }
        if (fault_status_1 & (1 << VDS_OCP)) {
            Serial.println("  - VDS Overcurrent Protection");
            has_faults = true;
        }
        if (fault_status_1 & (1 << GDF)) {
            Serial.println("  - Gate Driver Fault");
            has_faults = true;
        }
        if (fault_status_1 & (1 << UVLO)) {
            Serial.println("  - Undervoltage Lockout");
            has_faults = true;
        }
        if (fault_status_1 & (1 << OTSD)) {
            Serial.println("  - Overtemperature Shutdown");
            has_faults = true;
        }
        if (fault_status_1 & (1 << VDS_HA)) {
            Serial.println("  - VDS Fault High Side A");
            has_faults = true;
        }
        if (fault_status_1 & (1 << VDS_LA)) {
            Serial.println("  - VDS Fault Low Side A");
            has_faults = true;
        }
        if (fault_status_1 & (1 << VDS_HB)) {
            Serial.println("  - VDS Fault High Side B");
            has_faults = true;
        }
        if (fault_status_1 & (1 << VDS_LB)) {
            Serial.println("  - VDS Fault Low Side B");
            has_faults = true;
        }
        if (fault_status_1 & (1 << VDS_HC)) {
            Serial.println("  - VDS Fault High Side C");
            has_faults = true;
        }
        if (fault_status_1 & (1 << VDS_LC)) {
            Serial.println("  - VDS Fault Low Side C");
            has_faults = true;
        }
    }
    
    // Check VGS Status 2 register
    if (vgs_status_2 != 0) {
        Serial.println("DRV8323 VGS Status 2:");
        
        if (vgs_status_2 & (1 << SA_OC)) {
            Serial.println("  - Shunt A Overcurrent");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << SB_OC)) {
            Serial.println("  - Shunt B Overcurrent");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << SC_OC)) {
            Serial.println("  - Shunt C Overcurrent");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << OTW)) {
            Serial.println("  - Overtemperature Warning");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << CPUV)) {
            Serial.println("  - Charge Pump Undervoltage");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << VGS_HA)) {
            Serial.println("  - VGS Fault High Side A");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << VGS_LA)) {
            Serial.println("  - VGS Fault Low Side A");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << VGS_HB)) {
            Serial.println("  - VGS Fault High Side B");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << VGS_LB)) {
            Serial.println("  - VGS Fault Low Side B");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << VGS_HC)) {
            Serial.println("  - VGS Fault High Side C");
            has_faults = true;
        }
        if (vgs_status_2 & (1 << VGS_LC)) {
            Serial.println("  - VGS Fault Low Side C");
            has_faults = true;
        }
    }
    
    if (!has_faults) {
        Serial.println("DRV8323: No faults detected");
    }
    
    return has_faults;
}
#include "wiring_time.h"
#include "DRV8323RSRGZR.h"
#include "config.h"

// The DRV8323 samples SDI on the falling SCLK edge (SPI mode 1); the encoder
// and the rest of the firmware use mode 0, so switch per transaction.
uint8_t  g_drvSpiMode = 1;          // runtime-adjustable for bring-up (ACB_CMD_DRV_SPI_CFG)
uint32_t g_drvSpiHz   = 1000000;
static void drvSpiBegin() {
    static const uint8_t modes[4] = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};
    SPI.beginTransaction(SPISettings(g_drvSpiHz, MSBFIRST, modes[g_drvSpiMode & 3]));
}
static void drvSpiEnd() {
    SPI.endTransaction();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
}

DRV8323RSRGZR::DRV8323RSRGZR(uint8_t cs_pin) : _cs_pin(cs_pin) {}

void DRV8323RSRGZR::init() {}

void DRV8323RSRGZR::resetFaults(){
    digitalWrite(DRV_EN, LOW);
    delayMicroseconds(100);
    digitalWrite(DRV_EN, HIGH);
}

uint16_t DRV8323RSRGZR::readRegister(uint8_t reg_address) {    
    // digitalWrite(PB9, HIGH);
    // delay(1);
    // Pull CS low to start communication
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(10); // Small delay for setup time
    
    // Construct read command: R/W=1 (bit 15), 4-bit address, 11-bit data=0.
    // NOTE: previously bit 15 was cleared, which turned every read into a
    // write of zeros to the addressed register.
    uint16_t read_cmd = 0x8000 | ((reg_address & 0x0F) << 11);
    drvSpiBegin();
    uint16_t response = SPI.transfer16(read_cmd);
    drvSpiEnd();

    // Pull CS high to end communication
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
    drvSpiBegin();
    SPI.transfer16(write_cmd);
    drvSpiEnd();
    
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
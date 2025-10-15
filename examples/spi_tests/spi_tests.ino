#include <SPI.h>

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


void IOSetup(){
    pinMode(DRV_EN, OUTPUT);
    pinMode(DRV_FAULT, INPUT_PULLUP);
    
    pinMode(SPI_CS_POS, OUTPUT);
    pinMode(SPI_CS_DRV, OUTPUT);

    pinMode(SPI_MISO, INPUT_PULLDOWN);

    digitalWrite(SPI_CS_POS, HIGH);
    digitalWrite(SPI_CS_DRV, HIGH);
    
    digitalWrite(DRV_EN, HIGH);
    delayMicroseconds(100);

    digitalWrite(DRV_CAL, HIGH);
    delayMicroseconds(500);
    digitalWrite(DRV_CAL, LOW);
}

void initSPI() {
    SPI.setMISO(SPI_MISO);
    SPI.setMOSI(SPI_MOSI);
    SPI.setSCLK(SPI_SCK);
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV128);  
}



void writeData(uint16_t data, uint8_t addr){
    Serial.println("here");
    digitalWrite(SPI_CS_DRV, LOW);
    delayMicroseconds(2);
    Serial.println("here");
    uint16_t cmd = 0x0000 | (addr << 11) | data;
    // SPI.transfer16(cmd, false);
    delayMicroseconds(2);
    Serial.println("here");
    digitalWrite(SPI_CS_DRV, HIGH);
    delayMicroseconds(1);
}

uint16_t readData(uint8_t addr){
    digitalWrite(SPI_CS_DRV, LOW);
    delayMicroseconds(2);
    uint16_t cmd = 0x8000 | (addr << 11);
    // uint16_t data = SPI.transfer16(cmd);
    uint16_t data =0;
    delayMicroseconds(2);
    digitalWrite(SPI_CS_DRV, HIGH);
    delayMicroseconds(1);
    return data & 0x01FF;
}

void setup() {
    IOSetup();
    // initSPI();
    pinMode(SPI_MISO, INPUT_PULLUP);
    digitalWrite(SPI_MISO, HIGH);
    Serial.begin(115200);
    delay(1000);
    Serial.println("starting2");
    writeData(
        (0 << 10) | 
        (0b01 << 8) |
        (0b01 << 6) |
        (0b01 << 4) | 
        0b1001 
        , 0x05);
        Serial.println("Main loop");
}

void loop() {
    uint16_t data = readData(0x05);
    Serial.printf("Address 0x%u data: 0x", 0x05);
    Serial.println(data, HEX);
    delay(100);
}
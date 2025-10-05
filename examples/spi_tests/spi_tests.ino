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

    pinMode(SPI_MISO, INPUT_PULLUP);
    digitalWrite(SPI_CS_POS, HIGH);
    digitalWrite(SPI_CS_DRV, HIGH);
    
    digitalWrite(DRV_EN, HIGH);
    delayMicroseconds(100);
    digitalWrite(DRV_EN, LOW);
    delay(10);
    digitalWrite(DRV_EN, HIGH);

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

void setup() {
    IOSetup();
    initSPI();
    Serial.begin(115200);
    
}

void loop() {
    for (int i = 0; i < 7; i++) {

        digitalWrite(SPI_CS_DRV, LOW);
        delayMicroseconds(1);
        uint16_t cmd = 0x8000 | (i << 11);
        uint16_t data = SPI.transfer16(i);
        digitalWrite(SPI_CS_DRV, HIGH);
        delayMicroseconds(1);
        
        Serial.printf("Address 0x%u data: 0x", i);
        Serial.println(data, HEX);
        delay(1);
    }
    delay(1000);
}
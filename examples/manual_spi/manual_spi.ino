// This file manually implements SPI for slow speed devices

#define SPI_MOSI PA7
#define SPI_MISO PA6
#define SPI_SCK PA5
#define SPI_CS_DRV PB5
#define DRV_EN  PB9


#define clock_delay 1 // in microseconds
#define cs_wait_time 100 // CS wait time in microseconds
void setup(){
  Serial.begin(115200);
  
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(SPI_SCK, OUTPUT);
  pinMode(SPI_CS_DRV, OUTPUT);
  pinMode(DRV_EN, OUTPUT);


  digitalWrite(SPI_CS_DRV, HIGH);
  digitalWrite(DRV_EN, HIGH);
}

// Manual SPI read function for address 0x06
uint16_t spi_read_register(uint8_t address) {
  // Construct 16-bit frame: R/W=1, 4-bit address, 11-bit data
  // For read: R/W=1, address=0x06, data=0 (11 bits)
  uint16_t frame = 0x8000 | (address << 11); // R/W=1, address in bits 14-11
  
  uint16_t received_data = 0;
  
  // Pull CS low to start transaction
  digitalWrite(SPI_CS_DRV, LOW);
  delayMicroseconds(cs_wait_time);
  
  // SPI Mode 0: CPOL=0, CPHA=0
  // Clock starts low, data sampled on rising edge, setup on falling edge
  digitalWrite(SPI_SCK, LOW);
  delayMicroseconds(clock_delay);
  
  // Send 16 bits MSB first
  for (int i = 15; i >= 0; i--) {
    // Setup data on falling edge (SPI Mode 0)
    digitalWrite(SPI_MOSI, (frame >> i) & 0x01);
    delayMicroseconds(clock_delay);
    
    // Clock rising edge - sample data
    digitalWrite(SPI_SCK, HIGH);
    delayMicroseconds(clock_delay);
    
    // Read MISO on rising edge
    received_data |= (digitalRead(SPI_MISO) << i);
    
    // Clock falling edge - setup next bit
    digitalWrite(SPI_SCK, LOW);
    delayMicroseconds(clock_delay);
  }
  delayMicroseconds(cs_wait_time);
  // End transaction
  digitalWrite(SPI_CS_DRV, HIGH);
  delayMicroseconds(cs_wait_time);
  
  return received_data;
}

void loop(){
  // Read from address 0x06
  for (int i = 0; i < 7; i++)
  {  uint16_t data = spi_read_register(i);
  
  // Print the result
  Serial.printf("Address 0x%u data: 0x", i);
  Serial.println(data, HEX);
  delay(1);
  }
  delay(1000); // Wait 1 second between reads
}
// SPI encoder calibration example for MA730GQ
#include <SimpleFOC.h>
#include <SPI.h>

// ACB Pin definitions

#define PWM_H_A PC0 
#define PWM_L_A PC13
#define PWM_H_B PC1
#define PWM_L_B PB0
#define PWM_H_C PC2
#define PWM_L_C PB1


#define ENCODER_A PA15
#define ENCODER_B PB3
#define ENCODER_Z PC10

#define CURR_A  PA0
#define CURR_B  PA1
#define CURR_C  PA2

#define DRV_FAULT PB10
#define DRV_EN  PB9
#define DRV_CAL PB7

#define SPI_MOSI PA7
#define SPI_MISO PA6
#define SPI_SCK PA5
#define SPI_CS_DRV PB5
#define SPI_CS_POS PB6

#define BUS_V PB15

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(19);
BLDCDriver6PWM driver = BLDCDriver6PWM(PWM_H_A, PWM_L_A, PWM_H_B, PWM_L_B, PWM_H_C, PWM_L_C);
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 1024, ENCODER_Z); 

// Interrupt routine intialisation

void doA(){
  encoder.handleA();
}
void doB(){
  encoder.handleB();}
void doI(){
  encoder.handleIndex();
}


//target variable
float target_velocity = 10;


// MA730GQ SPI communication functions
/**
 * Initialize SPI communication for MA730GQ encoder
 */
void initSPI() {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV128); // Adjust clock speed as needed
  
  // Set CS pins high initially
  digitalWrite(SPI_CS_POS, HIGH);
  digitalWrite(SPI_CS_DRV, HIGH);
}

/**
 * Read a register from MA730GQ encoder over SPI
 * Uses two-frame protocol as per datasheet:
 * Frame 1: Read request (3-bit command 010 + 5-bit address + 8 zeros)
 * Frame 2: Receive 16-bit register value
 * @param reg_address: Register address to read (0x00-0x1F)
 * @return: 16-bit register value
 */
uint8_t readMA730GQRegister(uint8_t reg_address) {
  uint8_t result = 0;
  uint16_t buff = 0;
  
  digitalWrite(SPI_CS_POS, LOW);
  delayMicroseconds(1);

  uint16_t read_request = 0x4000 | ((reg_address & 0x1F) << 8);
  SPI.transfer16(read_request);
  digitalWrite(SPI_CS_POS, HIGH);

  delayMicroseconds(1);
  digitalWrite(SPI_CS_POS, LOW);
  
  delayMicroseconds(1);
  buff = SPI.transfer16(0x0000);
  Serial.println(buff, HEX);
  buff = (buff >> 8) & 0xFF;
  result = buff;
  
  // Pull CS high to end communication
  digitalWrite(SPI_CS_POS, HIGH);
  delayMicroseconds(1); // Small delay for hold time
  
  return result;
}

/**
 * Read angle from MA730GQ encoder over SPI
 * Single transaction: send all zeros, receive angle value
 * @return: 16-bit angle value
 */
uint16_t readMA730GQAngle() {
  uint16_t angle = 0;
  
  // Pull CS low to start communication
  digitalWrite(SPI_CS_POS, LOW);
  delayMicroseconds(1); // Small delay for setup time
  
  // Send all zeros and receive angle
  angle = SPI.transfer16(0x0000);
  
  // Pull CS high to end communication
  digitalWrite(SPI_CS_POS, HIGH);
  delayMicroseconds(1); // Small delay for hold time
  
  return angle;
}



/**
 * Convert 16-bit angle value to radians
 * @param angle_16bit: 16-bit angle value (0-65535)
 * @return: Angle in radians (0 to 2π)
 */
float angleToRadians(uint16_t angle_16bit) {
  return ((float)angle_16bit) * 2.0 * PI / 65536.0;
}

/**
 * Write a register to MA730GQ encoder over SPI
 * Uses two-frame protocol as per datasheet:
 * Frame 1: Write request (3-bit command 100 + 5-bit address + 8-bit data)
 * Frame 2: Dummy transaction (send zeros, receive response)
 * @param reg_address: Register address to write (0x00-0x1F)
 * @param data: 8-bit data value to write
 */
void writeMA730GQRegister(uint8_t reg_address, uint8_t data) {
  // SPI.beginTransaction(SPISettings());
  // Pull CS low to start communication
  digitalWrite(SPI_CS_POS, LOW);
  // _delay(2);

  delayMicroseconds(1); // Small delay for setup time
  // delay(1);
  
  
  // Frame 1: Send write request
  // Bits 15-13: Write command (100)
  // Bits 12-8: 5-bit register address
  // Bits 7-0: 8-bit data
  uint16_t write_request = 0x8000 | ((reg_address & 0x1F) << 8) | (data & 0xFF);
  // Serial.println(write_request, HEX);
  SPI.transfer16(write_request, true);
  digitalWrite(SPI_CS_POS, HIGH);
  _delay(20);
  digitalWrite(SPI_CS_POS, LOW);
    delayMicroseconds(1); // Small delay for setup time
  // delayMicroseconds(2);
  
  // Frame 2: Dummy transaction (some devices require this)
  uint16_t res = SPI.transfer16(0x0000);
  if(res >> 8 != data){
    Serial.printf("Failed to write data! Got: %x, expected: %x\n", res, data);
    // Serial.printf("Sent request 0x%x\n", write_request);
  }
  
  // Pull CS high to end communication
  digitalWrite(SPI_CS_POS, HIGH);
  // delayMicroseconds(1); // Small delay for hold time
  // delay(1);
  // SPI.endTransaction();
}

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }


void IOSetup(){
  // Configure IO not handled by SimpleFOC
  pinMode(DRV_EN, OUTPUT);
  pinMode(DRV_CAL, OUTPUT);
  
  // Configure SPI CS pins
  pinMode(SPI_CS_POS, OUTPUT);
  pinMode(SPI_CS_DRV, OUTPUT);
  // pinMode(SPI_MOSI, OUTPUT);

  pinMode(BUS_V, INPUT);
}

void setup() {
  IOSetup();
  // monitoring port
  Serial.begin(115200);
  Serial.println("Encoder SPI Test");


  // Initialize SPI communication


  driver.voltage_power_supply = 12;
  driver.voltage_limit = 1;
  driver.pwm_frequency = 50000;
  driver.init();

  motor.linkDriver(&driver);
  motor.voltage_limit = driver.voltage_limit / 2;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();

  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::ON;

  // check if you need internal pullups
  encoder.pullup = Pullup::USE_INTERN;
  digitalWrite(DRV_EN, HIGH);
  // initialize encoder hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB);

  initSPI();
  
  // Read register h5 (0x5) from MA730GQ encoder
  uint16_t h5_value = readMA730GQRegister(0x5);
  Serial.print("MA730GQ Register h5 value: 0x");
  if (h5_value < 0x1000) Serial.print("0");
  if (h5_value < 0x100) Serial.print("0");
  if (h5_value < 0x10) Serial.print("0");
  Serial.println(h5_value, HEX);
  Serial.print("MA730GQ Register h5 value (decimal): ");
  Serial.println(h5_value);
  
  // Set ppt to 128
  uint16_t ppt_val = 128-1;
  // writeMA730GQRegister(0x04, ppt_val  << 6);
  // writeMA730GQRegister(0x05, 0xFF & (ppt_val  >> 2));
  writeMA730GQRegister(0x10, 84);


  // Write register 0x6 with value 0b01010100 (0x54)
  // writeMA730GQRegister(0x6, 0b00011100);
  // Serial.println("Written 0b01010100 to register 0x6");
  // uint8_t out = readMA730GQRegister(0x6);
  // Serial.println(out, HEX);
  motor.move(10);
  Serial.println("Encoder readyy");

  // while(1){
    // writeMA730GQRegister(0x04, ppt_val  << 6);
  // writeMA730GQRegister(0x05, 0xFF & (ppt_val  >> 2));

  // writeMA730GQRegister(0x10, 200);
  // _delay(500);
  // }
    float bus_v = analogRead(BUS_V);

  bus_v = (bus_v / 1024.0) * 3.3;
  bus_v = bus_v / (1e3 / (1e3 + 33e3));

    Serial.print("Bus voltage: ");
  Serial.println(bus_v);
  while(1);
  _delay(3000);
}

void loop() {
  // IMPORTANT - call as frequently as possible
  // update the sensor values 
  // motor.move(10);
  encoder.update();
  // display the angle and the angular velocity to the terminal
  static unsigned long last_print = 0;
  
  unsigned long now = millis();
  if (now - last_print >= 10) {
    // Read angle from MA730GQ via SPI (filtering removed)
    // Serial.print("\t");
    // Serial.print("Encoder Angle: ");
    Serial.print(encoder.getMechanicalAngle());
    // Serial.print("\t");
    // Serial.print("Encoder Velocity: ");
    // Serial.println(encoder.getVelocity());



    Serial.print("\n");
    last_print = now;
  }
}
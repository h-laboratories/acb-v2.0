// Open loop motor control example
#define HAL_IWDG_MODULE_ENABLED 1

#include <IWatchdog.h>
#include <SimpleFOC.h>
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
// #include "stm32g4xx_hal_iwdg.h"
// #include "stm32g4xx_hal_def.h"
#include <IWatchdog.h>

#include <stm32g4xx_hal_flash.h>
#include <stm32g4xx_hal_flash_ex.h>

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

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(19);
BLDCDriver6PWM driver = BLDCDriver6PWM(PWM_H_A, PWM_L_A, PWM_H_B, PWM_L_B, PWM_H_C, PWM_L_C);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, 20.0*10, CURR_A, CURR_B, CURR_C);
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 1024); 

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
// void doI(){encoder.handleIndex();}

float target = 0;

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void doTarget(char* cmd) { command.scalar(&target, cmd); }

// IWDG_HandleTypeDef hiwdg;

void initSPI() {
  SPI.setMISO(SPI_MISO);
  SPI.setMOSI(SPI_MOSI);
  SPI.setSCLK(SPI_SCK);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);  
}


void IOSetup(){
  // Configure IO not handled by SimpleFOC
  pinMode(DRV_EN, OUTPUT);
  pinMode(DRV_CAL, OUTPUT);
  pinMode(DRV_FAULT, INPUT);
  // Configure SPI CS pins
  pinMode(SPI_CS_POS, OUTPUT);
  // Set CS pins high initially
  digitalWrite(SPI_CS_POS, HIGH);

  // Configure PG10 as GPIO input with pullup
  // Note: This pin is normally the NRST (reset) pin, but after calling configureNRSTAsGPIO()
  // in setup(), it becomes available as a regular GPIO pin
  pinMode(7, OUTPUT);
  pinMode(PG10, OUTPUT);
  
}

void initIWDG(){

   if (IWatchdog.isReset(true)) {
    
    Serial.println("Reset IWDG");
    IWatchdog.clearReset();
  }
  IWatchdog.begin(10000000);

  // hiwdg.Instance = IWDG;
  // hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  // hiwdg.Init.Window = 100;
  // hiwdg.Init.Reload = 100;

  // // IWDG_KR = 0x0000CCCC;
  


  // if(HAL_IWDG_Init(&hiwdg)!= HAL_OK){
  //   Serial.println("IWDG Not started");
  // }
}
void refreshIWDG(){
  IWatchdog.reload();
// if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK){
//     Serial.println("Couldn't refresh HAL");
//   }
}
uint8_t readRegister(uint8_t addr){
  uint8_t result = 0;
  uint16_t buff = 0;
  
  digitalWrite(SPI_CS_POS, LOW);
  delayMicroseconds(1);
  delay(1);

  uint16_t read_request = 0x4000 | ((addr & 0x1F) << 8);
  SPI.transfer16(read_request);
  digitalWrite(SPI_CS_POS, HIGH);

  delayMicroseconds(1);
  digitalWrite(SPI_CS_POS, LOW);
  
  delayMicroseconds(1);
  buff = SPI.transfer16(0x0000);
  buff = (buff >> 8) & 0xFF;
  result = buff;
  
  // Pull CS high to end communication
  digitalWrite(SPI_CS_POS, HIGH);
  delayMicroseconds(1); // Small delay for hold time
  
  return result;
}

uint8_t writeRegister(uint8_t addr, uint8_t data){
  // Pull CS low to start communication
    digitalWrite(SPI_CS_POS, LOW);
    delayMicroseconds(1); // Small delay for setup time
    
    // Frame 1: Send write request
    // Bits 15-13: Write command (100)
    // Bits 12-8: 5-bit register address
    // Bits 7-0: 8-bit data
    uint16_t write_request = 0x8000 | ((addr & 0x1F) << 8) | (data & 0xFF);
    SPI.transfer16(write_request);
    digitalWrite(SPI_CS_POS, HIGH);
    delay(20);
    
    digitalWrite(SPI_CS_POS, LOW);
    delayMicroseconds(1); // Small delay for setup time
    
    // Frame 2: Dummy transaction (some devices require this)
    uint16_t res = SPI.transfer16(0x0000);
    if(res >> 8 != data){
        Serial.printf("Failed to write data! Got: %x, expected: %x\n", res, data);
    }
    
    // Pull CS high to end communication
    digitalWrite(SPI_CS_POS, HIGH);
}

float getEncoderAngle(){
  uint16_t result = 0;
  
  
  digitalWrite(SPI_CS_POS, LOW);
  delayMicroseconds(1);

  uint16_t encoder_value = SPI.transfer16(0x0000);

  delayMicroseconds(1);
  digitalWrite(SPI_CS_POS, HIGH);

  // encoder_value = encoder_value >> 6;
  float max_value_10bit = 1023;
  float max_value_12bit = 4095;
  float max_value_14bit = 16383;
  float max_value_16bit = 65535;
  float angle = ((float)encoder_value) / max_value_16bit;
  angle = angle * 2 * PI;
  // angle += (random(-1000, 1000) / 1e6);

  
  return angle;
  
}

void readyEncoder(){

}

// GenericSensor encoder = GenericSensor(getEncoderAngle, readyEncoder); 

void setup() {
  
  Serial.begin(2000000);
  delay(1000);

  Serial.println("--------------------");
  Serial.println("ACBv2.0 Test script (basic_foc_spi_encoder_current_sense)");

  IOSetup();
  initSPI();
  initIWDG();
  
  SimpleFOCDebug::enable(&Serial);
  encoder.quadrature = Quadrature::ON;

  encoder.init();
    encoder.enableInterrupts(doA, doB);

  motor.linkSensor(&encoder);
  
  driver.voltage_power_supply = 20;
  driver.pwm_frequency = 20000;
  
  driver.voltage_limit = 4;
  if(!driver.init()){
    // Serial.println("Driver init failed!");
    return;
  }
  current_sense.linkDriver(&driver);
  motor.linkDriver(&driver);

  motor.voltage_limit = driver.voltage_limit/2;   // [V]
  motor.voltage_sensor_align = 2;
  motor.velocity_index_search = 1;

  motor.PID_velocity.P = .2;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0.01;
  
  motor.target = 20;

  motor.LPF_velocity.Tf = 0.05f;

  // open loop control config
  // motor.torque_controller = TorqueControlType;
  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  

  digitalWrite(DRV_CAL, HIGH);
  _delay(1);
  digitalWrite(DRV_CAL, LOW);
  _delay(1);
  digitalWrite(DRV_EN, HIGH);
  _delay(10);
  refreshIWDG();

  // if(digitalRead(DRV_FAULT) == LOW){
  //   Serial.println("Driver fault");
  // }

  command.add('M',doMotor,"motor");
  motor.useMonitoring(Serial);
  // motor.monitor_downsample=0;
  // init motor hardware
  if(!motor.init()){
    // Serial.println("Motor init failed!");
    return;
  }
  // current_sense.
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  // current_sense.skip_align = true;
  // current_sense.skip_align=true;
  // _delay(500);
  // // add target command T
  //   digitalWrite(DRV_EN, LOW);
  //   _delay(100);
  //     digitalWrite(DRV_EN, HIGH);
  


  _delay(100);
  refreshIWDG();
  current_sense.skip_align = true;
  motor.initFOC();

  // while(1){

  //   encoder.update();

  //   static unsigned long lastPrintTime = 0;
  //   unsigned long now = millis();
  //   if (now - lastPrintTime >= 250) {
  //     Serial.println(encoder.getAngle());
  //     lastPrintTime = now;
  //   }
  //   // _delay(250);
  // }

  // motor.LPF_velocity = 0.01;
  // motor.motion_downsample = 10;
  // motor.disable();
  // motor.move(100);
  
  digitalWrite(DRV_EN, LOW);
  _delay(10);
  digitalWrite(DRV_EN, HIGH);
  Serial.println("Motor ready!");
  _delay(1000);

  // Serial.println("Motor current offset: ");
  // Serial.println(current_sense.offset_ia);
  // Serial.println(current_sense.offset_ib);
  // Serial.println(current_sense.offset_ic);

  motor.target=2*PI;
  motor.enable();

  // while(1);
  // motor.move(10);
}


void loop() {

  refreshIWDG();

  motor.loopFOC();
  // motor.move(target);
  // encoder.update();
  motor.move();
  // motor.move(10);
  motor.monitor();
  command.run();
  
}

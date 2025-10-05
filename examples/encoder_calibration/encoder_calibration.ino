// Open loop motor control example
#include <SimpleFOC.h>

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
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 1024, ENCODER_Z); 

// Interrupt routine intialisation
// channel A and B callbacks
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

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }


void IOSetup(){
  // Configure IO not handled by SimpleFOC
  pinMode(DRV_EN, OUTPUT);
  pinMode(DRV_CAL, OUTPUT);
}

void setup() {
  IOSetup();
  // monitoring port
  Serial.begin(115200);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 4;
  driver.pwm_frequency = 2000;
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
  // hardware interrupt enable
  // encoder.enableInterrupts(doA, doB);

  Serial.println("Encoder ready");
  _delay(1000);
}

void loop() {
  // IMPORTANT - call as frequently as possible
  // update the sensor values 
  motor.move(10);
  // encoder.update();
  // display the angle and the angular velocity to the terminal
  static unsigned long last_print = 0;
  unsigned long now = millis();
  if (now - last_print >= 10) {
    Serial.print(encoder.getAngle());
    Serial.print("\t");
    Serial.println(encoder.getVelocity());
    last_print = now;
  }
}
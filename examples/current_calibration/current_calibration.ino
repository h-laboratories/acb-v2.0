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
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, 20.0 * 16, CURR_A, CURR_B, CURR_C);

//target variable
float target_velocity = 0;

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
  // use monitoring with serial 
  Serial.begin(2000000);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);
  driver.pwm_frequency = 50000;
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 4;
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  current_sense.linkDriver(&driver);
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 2;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity;

  // Enable the MOSFET driver
  digitalWrite(DRV_EN, HIGH);

  digitalWrite(DRV_CAL, HIGH);
  _delay(10);
  digitalWrite(DRV_CAL, LOW);

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  // add target command T
  motor.initFOC();
  

  Serial.println("Motor ready!");
  _delay(1000);
}


void loop() {
  motor.loopFOC();
  motor.move(10);
  
    // static unsigned long last_print = 0;
    // unsigned long now = millis();
    // if (now - last_print >= 5) {
    //   PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    //     Serial.print(currents.a*1000); // milli Amps
    //     Serial.print("\t");
    //     Serial.print(currents.b*1000); // milli Amps
    //     Serial.print("\t");
    //     Serial.println(currents.c*1000); // milli Amps
    //     last_print = now;
    // }
}

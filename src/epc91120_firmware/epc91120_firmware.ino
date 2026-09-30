// EPC91120 (3x EPC23102 GaN half-bridge, STM32G431CBU6) SimpleFOC firmware.
//
// Pin map from the EPC91120 schematic (Rev 1.0) and the factory firmware's
// peripheral registers, see epc91120/NOTES.md:
//   TIM1 CH1/2/3  PA8 / PA9 / PA10   high-side U/V/W (EPC23102 HSIN)
//   TIM1 CH1N/2N/3N PA7 / PB0 / PB1  low-side U/V/W (EPC23102 LSIN)
//   Isns U / V    PA0 / PB11         MCS1823 inline Hall sensors, 26.4 mV/A, 1.65 V at 0 A
//   Vdc           PA1                44.89 mV/V divider
//   OCDn          PA11               over-current, wired-OR, active low
//   MA732 A/B/Z   PA15 / PB3 / PB10  1024 PPR quadrature (+ index)
//   MA732 SPI2    MISO PB14, MOSI PB15, SCK PF1, CS PF0
//   USART2        TX PA2, RX PA3     ST-LINK virtual COM port (STDC14 pins 13/14)
//   Start/Stop    PC13               button, active high
//   RS485         D PB4, DE PB5, nRE PB6, R PB7 (no hardware UART TX on PB4; unused here)
//
// Control: SimpleFOC Commander on the serial port, 115200 baud. Examples:
//   M?           list motor commands        MC1     velocity mode (0 torque, 2 angle, 3/4 open loop)
//   ME1 / ME0    enable / disable           M20     target 20 rad/s
//   MLC2         current limit 2 A          MVP0.8  velocity P     MVI20  velocity I
//   E            encoder status             B       bus voltage    S      status line
//   A            re-run sensor alignment    O0/O1   over-current trip disable/enable
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include "MA730GQ.h"
#include "windowed_encoder.h"
#include "spi_angle_sensor.h"

// ------------------------------------------------------------------ pins ----
#define PIN_PWM_UH  PA8
#define PIN_PWM_UL  PA7
#define PIN_PWM_VH  PA9
#define PIN_PWM_VL  PB0
#define PIN_PWM_WH  PA10
#define PIN_PWM_WL  PB1
#define PIN_ISNS_U  PA0
#define PIN_ISNS_V  PB11
#define PIN_VDC     PA1
#define PIN_OCDN    PA11
#define PIN_ENC_A   PA15
#define PIN_ENC_B   PB3
#define PIN_ENC_Z   PB10
#define PIN_SPI_MISO PB14
#define PIN_SPI_MOSI PB15
#define PIN_SPI_SCK  PF1
#define PIN_SPI_CS   PF0
#define PIN_BUTTON  PC13

#define MOTOR_POLE_PAIRS   11        // 24 slot / 22 pole outrunner on the bench
#define ENCODER_PPR        1024
#define ISNS_MV_PER_A      26.4f
#define VDC_GAIN           0.04489f  // V per V
#define PWM_HZ             40000
#define DEAD_ZONE          0.004f    // fraction of the PWM period: 100 ns at 40 kHz (EPC23102 needs ~50 ns)
#define DEFAULT_CURRENT_LIMIT 2.0f   // A; bench PSU budget
#define ALIGN_VOLTAGE      0.5f      // ~2 A into a 0.26 R winding during initFOC alignment

HardwareSerial SerialVcp(PA3, PA2);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_PWM_UH, PIN_PWM_UL, PIN_PWM_VH, PIN_PWM_VL, PIN_PWM_WH, PIN_PWM_WL);
// The MCS1823 sensors are inline (continuous), so any PWM sampling instant is valid. SimpleFOC's
// LowsideCurrentSense is used because on STM32 it samples with timer-triggered injected ADC conversions
// (no analogRead(): ~25 kHz loop instead of ~2.5 kHz). shunt*gain = 26.4 mV/A.
LowsideCurrentSense current_sense = LowsideCurrentSense(ISNS_MV_PER_A * 1e-3f, 1.0f, PIN_ISNS_U, PIN_ISNS_V);
WindowedEncoder encoder = WindowedEncoder(PIN_ENC_A, PIN_ENC_B, ENCODER_PPR, PIN_ENC_Z);
MA730GQ spi_encoder = MA730GQ(PIN_SPI_CS);   // MA732 is register-compatible for angle + magnet status
SpiAngleSensor spi_sensor = SpiAngleSensor(&spi_encoder);   // FOC position source (A/B pulses are noisy on this board)
Commander command = Commander(SerialVcp);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doZ() { encoder.handleIndex(); }

static float bus_voltage = 0.0f;
static bool  oc_trip_enabled = true;
static bool  oc_tripped = false;
static uint32_t loop_count = 0, loop_stamp = 0; static float loop_hz = 0.0f;

static float readBusVoltage() {           // only safe before current_sense.init() claims the ADC
  analogReadResolution(12);
  const float v = analogRead(PIN_VDC) * 3.3f / 4095.0f / VDC_GAIN;
  analogReadResolution(10);               // SimpleFOC's STM32 inline current sense assumes 10-bit analogRead()
  return v;
}

// ---------------------------------------------------------- commander ----
void onMotor(char* cmd) { command.motor(&motor, cmd); }
void onEncoder(char* cmd) {
  (void)cmd;
  uint8_t st = spi_encoder.readRegister(0x1B);
  SerialVcp.print(F("abs ")); SerialVcp.print(spi_encoder.getAngleRadians(), 4);
  SerialVcp.print(F(" abz ")); SerialVcp.print(encoder.getMechanicalAngle(), 4);
  SerialVcp.print(F(" shaft ")); SerialVcp.print(motor.shaft_angle, 4);
  SerialVcp.print(F(" vel ")); SerialVcp.print(motor.shaft_velocity, 3);
  SerialVcp.print(F(" MGH ")); SerialVcp.print((st >> 7) & 1);
  SerialVcp.print(F(" MGL ")); SerialVcp.print((st >> 6) & 1);
  SerialVcp.print(F(" dir ")); SerialVcp.print((int)motor.sensor_direction);
  SerialVcp.print(F(" zero_el ")); SerialVcp.println(motor.zero_electric_angle, 4);
}
void onBus(char* cmd) { (void)cmd; SerialVcp.print(F("bus ")); SerialVcp.print(bus_voltage, 2); SerialVcp.println(F(" V (read at boot)")); }
void onStatus(char* cmd) {
  (void)cmd;
  SerialVcp.print(F("mode ")); SerialVcp.print((int)motor.controller);
  SerialVcp.print(F(" enabled ")); SerialVcp.print((int)motor.enabled);
  SerialVcp.print(F(" target ")); SerialVcp.print(motor.target, 3);
  SerialVcp.print(F(" vel ")); SerialVcp.print(motor.shaft_velocity, 3);
  SerialVcp.print(F(" iq ")); SerialVcp.print(motor.current.q, 3);
  SerialVcp.print(F(" id ")); SerialVcp.print(motor.current.d, 3);
  SerialVcp.print(F(" uq ")); SerialVcp.print(motor.voltage.q, 3);
  PhaseCurrent_s c = current_sense.getPhaseCurrents();
  SerialVcp.print(F(" ia ")); SerialVcp.print(c.a, 2); SerialVcp.print(F(" ib ")); SerialVcp.print(c.b, 2);
  SerialVcp.print(F(" ilimit ")); SerialVcp.print(motor.current_limit, 2);
  SerialVcp.print(F(" oc_pin ")); SerialVcp.print(digitalRead(PIN_OCDN));
  SerialVcp.print(F(" oc_tripped ")); SerialVcp.print(oc_tripped);
  SerialVcp.print(F(" loop_hz ")); SerialVcp.println(loop_hz, 0);
}
void onAlign(char* cmd) {
  (void)cmd;
  motor.disable();
  motor.sensor_direction = Direction::UNKNOWN;
  motor.zero_electric_angle = NOT_SET;
  motor.enable();
  motor.initFOC();
  motor.disable();
  SerialVcp.print(F("align: dir ")); SerialVcp.print((int)motor.sensor_direction);
  SerialVcp.print(F(" zero_el ")); SerialVcp.println(motor.zero_electric_angle, 4);
}
void onOcTrip(char* cmd) { oc_trip_enabled = (cmd[0] != '0'); oc_tripped = false; SerialVcp.print(F("oc trip ")); SerialVcp.println(oc_trip_enabled); }
void onWindow(char* cmd) {
  float ms = atof(cmd); if (ms >= 0) spi_sensor.window_s = ms * 1e-3f;
  SerialVcp.print(F("velocity window ms ")); SerialVcp.println(spi_sensor.window_s * 1e3f, 1);
}

void setup() {
  // Power stage idle before anything else.
  const uint8_t gates[] = {PIN_PWM_UH, PIN_PWM_UL, PIN_PWM_VH, PIN_PWM_VL, PIN_PWM_WH, PIN_PWM_WL};
  for (uint8_t p : gates) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pinMode(PIN_OCDN, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT);

  SerialVcp.begin(115200);
  delay(200);
  SerialVcp.println(F("\nEPC91120 SimpleFOC firmware"));

  bus_voltage = readBusVoltage();
  SerialVcp.print(F("bus ")); SerialVcp.print(bus_voltage, 2); SerialVcp.println(F(" V"));

  SPI.setMISO(PIN_SPI_MISO); SPI.setMOSI(PIN_SPI_MOSI); SPI.setSCLK(PIN_SPI_SCK);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST); SPI.setDataMode(SPI_MODE0); SPI.setClockDivider(SPI_CLOCK_DIV32);
  pinMode(PIN_SPI_CS, OUTPUT); digitalWrite(PIN_SPI_CS, HIGH);   // no register writes: keep the MA732 factory config
  uint8_t st = spi_encoder.readRegister(0x1B);
  SerialVcp.print(F("MA732 abs ")); SerialVcp.print(spi_encoder.getAngleRadians(), 4);
  SerialVcp.print(F(" rad  MGH ")); SerialVcp.print((st >> 7) & 1); SerialVcp.print(F(" MGL ")); SerialVcp.println((st >> 6) & 1);

  encoder.quadrature = Quadrature::ON;          // A/B kept only as a reference reading (E command)
  encoder.init();
  encoder.enableInterrupts(doA, doB, doZ);
  spi_sensor.init();
  motor.linkSensor(&spi_sensor);

  driver.voltage_power_supply = bus_voltage > 5.0f ? bus_voltage : 15.0f;
  driver.voltage_limit = driver.voltage_power_supply * 0.5f;
  driver.pwm_frequency = PWM_HZ;
  driver.dead_zone = DEAD_ZONE;
  if (!driver.init()) { SerialVcp.println(F("driver init failed")); while (true) delay(100); }
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);

  motor.voltage_limit = driver.voltage_limit;
  motor.current_limit = DEFAULT_CURRENT_LIMIT;
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.PID_current_q.P = 0.2f; motor.PID_current_q.I = 5.0f; motor.PID_current_q.D = 0;
  motor.PID_current_d.P = 0.2f; motor.PID_current_d.I = 5.0f; motor.PID_current_d.D = 0;
  motor.LPF_current_q.Tf = 0.005f; motor.LPF_current_d.Tf = 0.005f;
  motor.PID_velocity.P = 0.8f; motor.PID_velocity.I = 20.0f; motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = 20.0f;
  motor.velocity_limit = 400.0f;
  motor.init();

  if (!current_sense.init()) { SerialVcp.println(F("current sense init failed")); }
  current_sense.skip_align = false;      // let SimpleFOC verify sensor/phase pairing on this new board
  encoder.update();
  motor.linkCurrentSense(&current_sense);

  motor.disable();                               // no boot-time alignment: send A when the motor is free to move

  command.add('M', onMotor, "motor");
  command.add('E', onEncoder, "encoder status");
  command.add('B', onBus, "bus voltage");
  command.add('S', onStatus, "status");
  command.add('A', onAlign, "re-align sensor");
  command.add('O', onOcTrip, "over-current trip 0/1");
  command.add('W', onWindow, "velocity window ms");
  command.verbose = VerboseMode::user_friendly;
  SerialVcp.println(F("ready: A align (moves), ME1 enable, M<rad/s> target, S status, E encoder, M? help"));
  loop_stamp = millis();
}

void loop() {
  loop_count++;
  if (millis() - loop_stamp >= 500) { loop_hz = loop_count * 1000.0f / (millis() - loop_stamp); loop_count = 0; loop_stamp = millis(); }

  if (oc_trip_enabled && motor.enabled && digitalRead(PIN_OCDN) == LOW) {
    motor.disable(); oc_tripped = true;
    SerialVcp.println(F("!! over-current (OCDn low): motor disabled"));
  }
  motor.loopFOC();
  motor.move();
  command.run();
}

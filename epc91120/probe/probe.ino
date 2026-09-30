// EPC91120 pin probe. Keeps the EPC23102 gate inputs low (power stage off),
// then reports every spare GPIO and ADC-capable pin so the encoder ABZ pins
// and the second current-sense channel can be found by turning the rotor by
// hand and watching which inputs toggle / which analog channel sits at the
// MCS1823 1.65 V zero-current level. Output on USART2 (PA2/PA3), 115200 baud,
// which is the GUI/RS485 link and reaches the ST-LINK VCP.
#include <Arduino.h>

HardwareSerial Ser(PA3, PA2);

// TIM1 CH1-3 / CH1N-3N on the EPC91120 (from the factory firmware's GPIO AF registers).
static const uint8_t kGatePins[] = {PA8, PA9, PA10, PA7, PB0, PB1};

struct Pin { const char* name; uint8_t pin; };
static const Pin kDigital[] = {
  {"PA0", PA0}, {"PA4", PA4}, {"PA5", PA5}, {"PA6", PA6}, {"PA11", PA11}, {"PA12", PA12}, {"PA15", PA15},
  {"PB2", PB2}, {"PB3", PB3}, {"PB4", PB4}, {"PB5", PB5}, {"PB6", PB6}, {"PB7", PB7}, {"PB9", PB9},
  {"PB10", PB10}, {"PB11", PB11}, {"PB12", PB12}, {"PB13", PB13}, {"PB14", PB14}, {"PB15", PB15},
  {"PC13", PC13}, {"PC14", PC14}, {"PC15", PC15}, {"PF0", PF0}, {"PF1", PF1},
};
static const Pin kAnalog[] = {
  {"PA0", PA0}, {"PA1", PA1}, {"PA4", PA4}, {"PA5", PA5}, {"PA6", PA6},
  {"PB2", PB2}, {"PB11", PB11}, {"PB12", PB12}, {"PB14", PB14}, {"PB15", PB15},
};
static const int ND = sizeof(kDigital) / sizeof(kDigital[0]);
static const int NA = sizeof(kAnalog) / sizeof(kAnalog[0]);
static uint8_t lastState[ND];
static uint32_t toggles[ND];

void setup() {
  for (uint8_t p : kGatePins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }   // power stage idle first
  Ser.begin(115200);
  delay(300);
  Ser.println();
  Ser.println("EPC91120 probe: gate pins PA7 PA8 PA9 PA10 PB0 PB1 held LOW");
  for (int i = 0; i < ND; i++) { pinMode(kDigital[i].pin, INPUT_PULLUP); lastState[i] = digitalRead(kDigital[i].pin); toggles[i] = 0; }
  analogReadResolution(12);
  Ser.println("commands: d = digital snapshot, a = analog snapshot, t = toggle counts (reset), r = reset counts");
}

static void printDigital() {
  Ser.print("D:");
  for (int i = 0; i < ND; i++) { Ser.print(' '); Ser.print(kDigital[i].name); Ser.print('='); Ser.print(digitalRead(kDigital[i].pin)); }
  Ser.println();
}
static void printAnalog() {
  Ser.print("A:");
  for (int i = 0; i < NA; i++) {
    pinMode(kAnalog[i].pin, INPUT_ANALOG);
    int v = analogRead(kAnalog[i].pin);
    Ser.print(' '); Ser.print(kAnalog[i].name); Ser.print('='); Ser.print(v * 3.3f / 4095.0f, 3);
  }
  Ser.println();
  for (int i = 0; i < ND; i++) pinMode(kDigital[i].pin, INPUT_PULLUP);   // restore digital probing
}
static void printToggles(bool reset) {
  Ser.print("T:");
  for (int i = 0; i < ND; i++) if (toggles[i]) { Ser.print(' '); Ser.print(kDigital[i].name); Ser.print('='); Ser.print(toggles[i]); }
  Ser.println();
  if (reset) for (int i = 0; i < ND; i++) toggles[i] = 0;
}

void loop() {
  // count edges on every digital candidate (fast poll)
  for (int i = 0; i < ND; i++) {
    uint8_t s = digitalRead(kDigital[i].pin);
    if (s != lastState[i]) { toggles[i]++; lastState[i] = s; }
  }
  static uint32_t lastReport = 0;
  if (millis() - lastReport >= 1000) { lastReport = millis(); printToggles(false); }
  while (Ser.available()) {
    char c = Ser.read();
    if (c == 'd') printDigital();
    else if (c == 'a') printAnalog();
    else if (c == 't') printToggles(true);
    else if (c == 'r') { for (int i = 0; i < ND; i++) toggles[i] = 0; Ser.println("counts reset"); }
  }
}

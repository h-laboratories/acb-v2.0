/*
 * MCP2515 CAN Controller for Arduino Uno
 * 
 * This example demonstrates how to use the MCP2515 CAN controller
 * with SPI communication on Arduino Uno.
 * 
 * Hardware connections:
 * - MCP2515 CS pin to Arduino pin 10
 * - MCP2515 SCK pin to Arduino pin 13 (SCK)
 * - MCP2515 SI pin to Arduino pin 11 (MOSI)
 * - MCP2515 SO pin to Arduino pin 12 (MISO)
 * - MCP2515 INT pin to Arduino pin 2 (interrupt)
 * - MCP2515 VCC to 5V
 * - MCP2515 GND to GND
 * 
 * CAN bus connections:
 * - CAN_H to CAN transceiver CAN_H
 * - CAN_L to CAN transceiver CAN_L
 * - 120Ω termination resistor between CAN_H and CAN_L
 */

#include <SPI.h>

// MCP2515 Register definitions
#define MCP2515_RESET           0xC0
#define MCP2515_READ            0x03
#define MCP2515_WRITE           0x02
#define MCP2515_READ_STATUS     0xA0
#define MCP2515_RTS             0x80
#define MCP2515_BIT_MODIFY      0x05

// MCP2515 Control registers
#define MCP2515_CANCTRL         0x0F
#define MCP2515_CANSTAT         0x0E
#define MCP2515_CNF1            0x2A
#define MCP2515_CNF2            0x29
#define MCP2515_CNF3            0x28
#define MCP2515_CANINTE         0x2B
#define MCP2515_CANINTF         0x2C
#define MCP2515_EFLG            0x2D

// MCP2515 Buffer registers
#define MCP2515_TXB0CTRL         0x30
#define MCP2515_TXB1CTRL         0x40
#define MCP2515_TXB2CTRL         0x50
#define MCP2515_RXB0CTRL         0x60
#define MCP2515_RXB1CTRL         0x70

// MCP2515 Filter and mask registers
#define MCP2515_RXF0SIDH        0x00
#define MCP2515_RXF0SIDL        0x01
#define MCP2515_RXF0EID8        0x02
#define MCP2515_RXF0EID0        0x03
#define MCP2515_RXM0SIDH        0x20
#define MCP2515_RXM0SIDL        0x21
#define MCP2515_RXM0EID8        0x22
#define MCP2515_RXM0EID0        0x23

// MCP2515 Control bits
#define MCP2515_MODE_NORMAL     0x00
#define MCP2515_MODE_SLEEP      0x20
#define MCP2515_MODE_LOOPBACK   0x40
#define MCP2515_MODE_LISTENONLY 0x60
#define MCP2515_MODE_CONFIG     0x80
#define MCP2515_MODE_MASK       0xE0

// MCP2515 Interrupt flags
#define MCP2515_INT_RX0IF       0x01
#define MCP2515_INT_RX1IF       0x02
#define MCP2515_INT_TX0IF       0x04
#define MCP2515_INT_TX1IF       0x08
#define MCP2515_INT_TX2IF       0x10
#define MCP2515_INT_ERRIF       0x20
#define MCP2515_INT_WAKIF       0x40
#define MCP2515_INT_MERRF       0x80

// Pin definitions
const int CS_PIN = 10;           // Chip select pin
const int INT_PIN = 2;           // Interrupt pin

// CAN message structure
struct CANMessage {
  uint32_t id;
  uint8_t data[8];
  uint8_t length;
  bool extended;
};

// Global variables
volatile bool canMessageReceived = false;
CANMessage receivedMessage;

/**
 * @brief Initialize SPI communication with MCP2515
 */
void initSPI() {
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);  // Set SPI clock to 4MHz
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
}

/**
 * @brief Write a single byte to MCP2515 register
 * @param address Register address to write to
 * @param data Data byte to write
 */
void mcp2515_write_register(uint8_t address, uint8_t data) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(MCP2515_WRITE);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
}

/**
 * @brief Read a single byte from MCP2515 register
 * @param address Register address to read from
 * @return Data byte read from register
 */
uint8_t mcp2515_read_register(uint8_t address) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(MCP2515_READ);
  SPI.transfer(address);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return data;
}

/**
 * @brief Reset MCP2515 controller
 */
void mcp2515_reset() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(MCP2515_RESET);
  digitalWrite(CS_PIN, HIGH);
  delay(10);
}

/**
 * @brief Set MCP2515 operation mode
 * @param mode Operation mode to set
 * @return true if mode was set successfully, false otherwise
 */
bool mcp2515_set_mode(uint8_t mode) {
  mcp2515_write_register(MCP2515_CANCTRL, mode);
  
  // Wait for mode change
  uint8_t timeout = 0;
  while ((mcp2515_read_register(MCP2515_CANSTAT) & MCP2515_MODE_MASK) != mode) {
    if (timeout++ > 10) {
      return false;
    }
    delay(1);
  }
  return true;
}

/**
 * @brief Configure CAN bit timing for 4Mbps
 */
void mcp2515_configure_bitrate() {
  // Configuration for 4Mbps at 16MHz crystal
  // CNF1: BRP = 0, SJW = 0 (1 TQ)
  mcp2515_write_register(MCP2515_CNF1, 0x00);
  // CNF2: BTLMODE = 1, SAM = 0, PS1 = 1, PS2 = 0
  mcp2515_write_register(MCP2515_CNF2, 0x90);
  // CNF3: PS2 = 0
  mcp2515_write_register(MCP2515_CNF3, 0x82);
}

/**
 * @brief Configure CAN filters to accept all messages
 */
void mcp2515_configure_filters() {
  // Configure RX filter 0 to accept all messages
  mcp2515_write_register(MCP2515_RXF0SIDH, 0x00);
  mcp2515_write_register(MCP2515_RXF0SIDL, 0x00);
  mcp2515_write_register(MCP2515_RXF0EID8, 0x00);
  mcp2515_write_register(MCP2515_RXF0EID0, 0x00);
  
  // Configure RX mask 0 to accept all messages
  mcp2515_write_register(MCP2515_RXM0SIDH, 0x00);
  mcp2515_write_register(MCP2515_RXM0SIDL, 0x00);
  mcp2515_write_register(MCP2515_RXM0EID8, 0x00);
  mcp2515_write_register(MCP2515_RXM0EID0, 0x00);
  
  // Enable RX buffer 0 and 1
  mcp2515_write_register(MCP2515_RXB0CTRL, 0x60);
  mcp2515_write_register(MCP2515_RXB1CTRL, 0x60);
}

/**
 * @brief Initialize MCP2515 CAN controller
 * @return true if initialization successful, false otherwise
 */
bool mcp2515_init() {
  // Set pin modes
  pinMode(CS_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT_PULLUP);
  
  // Initialize SPI
  initSPI();
  
  // Reset MCP2515
  mcp2515_reset();
  
  // Configure bit timing
  mcp2515_configure_bitrate();
  
  // Configure filters
  mcp2515_configure_filters();
  
  // Set to normal mode
  if (!mcp2515_set_mode(MCP2515_MODE_NORMAL)) {
    return false;
  }
  
  // Enable interrupts
  mcp2515_write_register(MCP2515_CANINTE, MCP2515_INT_RX0IF | MCP2515_INT_RX1IF);
  
  return true;
}

/**
 * @brief Interrupt service routine for CAN message reception
 */
void canInterrupt() {
  canMessageReceived = true;
}

/**
 * @brief Read CAN message from RX buffer
 * @param buffer Buffer number (0 or 1)
 * @param message Pointer to CAN message structure
 * @return true if message was read successfully, false otherwise
 */
bool mcp2515_read_message(uint8_t buffer, CANMessage* message) {
  uint8_t base_addr = (buffer == 0) ? 0x60 : 0x70;
  uint8_t ctrl_reg = (buffer == 0) ? MCP2515_RXB0CTRL : MCP2515_RXB1CTRL;
  
  // Check if message is available
  uint8_t status = mcp2515_read_register(ctrl_reg);
  if (!(status & 0x03)) {
    return false;
  }
  
  // Read message data
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(MCP2515_READ);
  SPI.transfer(base_addr);
  
  // Read ID (4 bytes)
  uint8_t id_high = SPI.transfer(0x00);
  uint8_t id_low = SPI.transfer(0x00);
  uint8_t eid8 = SPI.transfer(0x00);
  uint8_t eid0 = SPI.transfer(0x00);
  
  // Read DLC
  uint8_t dlc = SPI.transfer(0x00) & 0x0F;
  
  // Read data bytes
  for (int i = 0; i < 8; i++) {
    message->data[i] = SPI.transfer(0x00);
  }
  
  digitalWrite(CS_PIN, HIGH);
  
  // Extract ID
  message->id = ((uint32_t)id_high << 3) | ((id_low & 0xE0) >> 5);
  message->length = dlc;
  message->extended = false; // Standard ID for now
  
  // Clear interrupt flag
  mcp2515_write_register(MCP2515_CANINTF, (buffer == 0) ? MCP2515_INT_RX0IF : MCP2515_INT_RX1IF);
  
  return true;
}

/**
 * @brief Print CAN message to serial output
 * @param message Pointer to CAN message structure
 */
void printCANMessage(const CANMessage* message) {
  Serial.print("ID: 0x");
  Serial.print(message->id, HEX);
  Serial.print(" | Length: ");
  Serial.print(message->length);
  Serial.print(" | Data: ");
  
  for (int i = 0; i < message->length; i++) {
    if (message->data[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(message->data[i], HEX);
    if (i < message->length - 1) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

/**
 * @brief Setup function - runs once at startup
 */
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MCP2515 CAN Controller Example");
  Serial.println("==============================");
  
  // Initialize MCP2515
  if (mcp2515_init()) {
    Serial.println("MCP2515 initialized successfully");
  } else {
    Serial.println("MCP2515 initialization failed!");
    while (1) {
      delay(1000);
    }
  }
  
  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(INT_PIN), canInterrupt, FALLING);
  
  Serial.println("Waiting for CAN messages...");
  Serial.println("Format: ID: 0xXXX | Length: X | Data: XX XX XX...");
  Serial.println();
}

/**
 * @brief Main loop function - runs continuously
 */
void loop() {
  // Check for received messages
  if (canMessageReceived) {
    canMessageReceived = false;
    
    // Try to read from both RX buffers
    if (mcp2515_read_message(0, &receivedMessage)) {
      printCANMessage(&receivedMessage);
    } else if (mcp2515_read_message(1, &receivedMessage)) {
      printCANMessage(&receivedMessage);
    }
  }
  
  // Small delay to prevent excessive CPU usage
  delay(1);
}

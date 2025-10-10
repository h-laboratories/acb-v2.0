//STM32G474RET6

// STM32G474 Independent Watchdog (IWDG) implementation
// Hardware watchdog timer that will reset the system if not fed within 2 seconds

#include "stm32g4xx_hal.h"

IWDG_HandleTypeDef hiwdg;

void setup() {
  // put your setup code here, to run once:
  
  // Initialize serial communication for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("Initializing STM32G474 IWDG for 2 seconds");
  
  // Configure the Independent Watchdog Timer (IWDG)
  // For 2 seconds timeout with LSI = 32kHz:
  // Prescaler = 32, Reload = 2000
  // Timeout = (Prescaler * Reload) / LSI_freq = (32 * 2000) / 32000 = 2 seconds
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;  // 32 prescaler
  hiwdg.Init.Reload = 2000;                  // Reload value for 2 seconds
  
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
    Serial.println("Error: IWDG initialization failed!");
    while(1); // Halt if initialization fails
  }
  
  Serial.println("IWDG initialized successfully - 2 second timeout");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Feed the watchdog to prevent system reset
  // This must be called at least once every 2 seconds
  HAL_IWDG_Refresh(&hiwdg);
  
  // Add a small delay to prevent the loop from running too fast
  delay(100);
  
  // Your main application code should go here
  // Make sure to call HAL_IWDG_Refresh() at least once every 2 seconds
}

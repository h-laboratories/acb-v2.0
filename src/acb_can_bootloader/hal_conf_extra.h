// Enable the HAL modules the CAN bootloader needs (the STM32duino core only
// compiles a HAL module when its macro is defined).
#pragma once
#ifndef HAL_FDCAN_MODULE_ENABLED
#define HAL_FDCAN_MODULE_ENABLED
#endif
#ifndef HAL_FLASH_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#endif
#ifndef HAL_PWR_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#endif

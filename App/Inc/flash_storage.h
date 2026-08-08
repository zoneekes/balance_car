#ifndef __FLASH_STORAGE_H__
#define __FLASH_STORAGE_H__

#include "stm32f1xx_hal.h"

// User flash storage address (adjust to your MCU flash size / layout)
// Default: last 1KB page of STM32F103C8T6 (64KB flash). If you have a different device, update accordingly.
#ifndef FLASH_SAVE_ADDR
#define FLASH_SAVE_ADDR  ((uint32_t)0x0800FC00)
#endif

// Save current PID params to flash. Returns 0 on success, non-zero on error.
int flash_save_pid_params(void);
// Load PID params from flash. Returns 0 on success (data valid), non-zero on error.
int flash_load_pid_params(void);

#endif

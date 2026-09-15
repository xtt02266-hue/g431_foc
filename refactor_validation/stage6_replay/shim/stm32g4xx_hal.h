#ifndef VERIF6_STM32G4XX_HAL_H
#define VERIF6_STM32G4XX_HAL_H
#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } ADC_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;

uint32_t HAL_GetTick(void);
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) { }
static inline void __enable_irq(void) { }
#endif

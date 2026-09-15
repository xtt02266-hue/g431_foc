#ifndef VERIF_STM32G4XX_HAL_H
#define VERIF_STM32G4XX_HAL_H

/* 主机验证用最小 HAL 头替身。只声明 motor_system.c 编译所需的最少类型/函数，
 * 不改变被测源文件本身。所有 HAL 行为由 test_support.c 的桩实现。 */
#include <stdint.h>
#include <stddef.h>

typedef enum
{
    HAL_OK      = 0x00,
    HAL_ERROR   = 0x01,
    HAL_BUSY    = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } ADC_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } DMA_HandleTypeDef;

uint32_t HAL_GetTick(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* CMSIS 临界区内在函数的主机替身：单线程验证无需真正屏蔽中断。 */
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) { }
static inline void __enable_irq(void) { }

#endif

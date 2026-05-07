#ifndef USER_IO_H
#define USER_IO_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

HAL_StatusTypeDef UserIO_StartDma(void);

uint16_t Pot_ReadRaw(void);

uint8_t Button1_ReadLevel(void);
uint8_t Button2_ReadLevel(void);

void Led_Set(uint8_t on);
void Led_Toggle(void);

#endif

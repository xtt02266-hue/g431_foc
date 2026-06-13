#ifndef MT6826S_H
#define MT6826S_H

#include <stdint.h>

#define MT6826S_ANGLE_MAX_15BIT  32767U

void MT6826S_Init(void);
uint16_t MT6826S_ReadRawAngle15(void);
uint8_t MT6826S_IsOk(void);

#endif

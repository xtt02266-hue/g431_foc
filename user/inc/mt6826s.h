#ifndef MT6826S_H
#define MT6826S_H

#include <stdint.h>

#define MT6826S_ANGLE_MAX_15BIT  32767U

#define MT6826S_STATUS_OVERSPEED     (1U << 0)
#define MT6826S_STATUS_STRONG_FIELD  (1U << 1)
#define MT6826S_STATUS_UNDERVOLTAGE  (1U << 2)

void MT6826S_Init(void);
uint16_t MT6826S_ReadRawAngle15(void);
uint8_t MT6826S_IsOk(void);
uint8_t MT6826S_GetStatus(void);
uint32_t MT6826S_GetCrcErrorCount(void);
uint32_t MT6826S_GetTransferErrorCount(void);

#endif

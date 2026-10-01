#ifndef MT6826S_H
#define MT6826S_H

#include <stdint.h>

#define MT6826S_ANGLE_MAX_15BIT  32767U

#define MT6826S_STATUS_OVERSPEED     (1U << 0)
#define MT6826S_STATUS_STRONG_FIELD  (1U << 1)
#define MT6826S_STATUS_UNDERVOLTAGE  (1U << 2)

void MT6826S_Init(void);
/* Start one non-blocking SPI1 burst. A request is ignored while DMA is busy. */
void MT6826S_RequestReadDMA(void);
uint8_t MT6826S_IsOk(void);
uint8_t MT6826S_GetStatus(void);
uint32_t MT6826S_GetCrcErrorCount(void);
uint32_t MT6826S_GetTransferErrorCount(void);
/* Main-loop only. 0=busy, 1=done, 2=SPI error. Never overlap angle DMA. */
uint8_t MT6826S_ReadRegister(uint16_t address, uint8_t *value);
uint8_t MT6826S_WriteRegister(uint16_t address, uint8_t value);
/* After successful/uncertain self-calibration only a power cycle is valid. */
void MT6826S_SuspendAngleReads(void);

#endif

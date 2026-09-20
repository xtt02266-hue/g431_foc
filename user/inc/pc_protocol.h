#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H

#include <stdint.h>

#define PC_PROTOCOL_VERSION 2U
#define PC_PROTOCOL_MAX_PAYLOAD 256U

void PC_Protocol_Init(void);
void PC_Protocol_FeedFromISR(const uint8_t *data, uint16_t length);
void PC_Protocol_NotifyTxCompleteFromISR(void);
void PC_Protocol_Task(void);
void PC_Protocol_SendTelemetry(void);
uint32_t PC_Protocol_GetCrcErrorCount(void);
uint32_t PC_Protocol_GetRxOverflowCount(void);
uint32_t PC_Protocol_GetTelemetryDropCount(void);

#endif

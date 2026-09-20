#ifndef PC_TRANSPORT_H
#define PC_TRANSPORT_H

#include <stdint.h>

void PC_Transport_Init(void);
uint8_t PC_Transport_TrySend(uint8_t *data, uint16_t length);

#endif

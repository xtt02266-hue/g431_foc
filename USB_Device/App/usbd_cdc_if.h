#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

#define APP_RX_DATA_SIZE 512U
#define APP_TX_DATA_SIZE 64U

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);

#ifdef __cplusplus
}
#endif
#endif

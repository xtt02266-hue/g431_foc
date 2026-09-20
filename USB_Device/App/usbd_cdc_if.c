#include "usbd_cdc_if.h"
#include "pc_protocol.h"
#include <stddef.h>

static uint8_t s_rx_buffer[APP_RX_DATA_SIZE];
static uint8_t s_tx_placeholder[APP_TX_DATA_SIZE];
static USBD_CDC_LineCodingTypeDef s_line_coding = { 921600U, 0U, 0U, 8U };

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, s_tx_placeholder, 0U);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, s_rx_buffer);
    return (int8_t)USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    return (int8_t)USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t command, uint8_t *buffer,
                             uint16_t length)
{
    (void)length;
    switch (command) {
    case CDC_SET_LINE_CODING:
        s_line_coding.bitrate = (uint32_t)buffer[0] |
                                ((uint32_t)buffer[1] << 8U) |
                                ((uint32_t)buffer[2] << 16U) |
                                ((uint32_t)buffer[3] << 24U);
        s_line_coding.format = buffer[4];
        s_line_coding.paritytype = buffer[5];
        s_line_coding.datatype = buffer[6];
        break;
    case CDC_GET_LINE_CODING:
        buffer[0] = (uint8_t)s_line_coding.bitrate;
        buffer[1] = (uint8_t)(s_line_coding.bitrate >> 8U);
        buffer[2] = (uint8_t)(s_line_coding.bitrate >> 16U);
        buffer[3] = (uint8_t)(s_line_coding.bitrate >> 24U);
        buffer[4] = s_line_coding.format;
        buffer[5] = s_line_coding.paritytype;
        buffer[6] = s_line_coding.datatype;
        break;
    default:
        break;
    }
    return (int8_t)USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *buffer, uint32_t *length)
{
    if ((buffer != NULL) && (length != NULL) && (*length != 0U)) {
        PC_Protocol_FeedFromISR(buffer, (uint16_t)*length);
    }
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, s_rx_buffer);
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (int8_t)USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *buffer, uint32_t *length,
                                  uint8_t endpoint)
{
    (void)buffer;
    (void)length;
    (void)endpoint;
    PC_Protocol_NotifyTxCompleteFromISR();
    return (int8_t)USBD_OK;
}

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS
};

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
    USBD_CDC_HandleTypeDef *cdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if ((cdc == NULL) || (cdc->TxState != 0U)) {
        return USBD_BUSY;
    }
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buffer, length);
    return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

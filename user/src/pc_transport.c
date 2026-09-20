#include "pc_transport.h"
#include "pc_protocol.h"
#include "board_profile.h"

#if BOARD_COMM_USB_CDC
#include "usb_device.h"
#include "usbd_cdc_if.h"

void PC_Transport_Init(void)
{
    MX_USB_Device_Init();
}

uint8_t PC_Transport_TrySend(uint8_t *data, uint16_t length)
{
    return (CDC_Transmit_FS(data, length) == USBD_OK) ? 1U : 0U;
}

#else
#include "usart.h"

#define PC_UART_RX_BUFFER_SIZE 512U
static uint8_t s_uart_rx[PC_UART_RX_BUFFER_SIZE];

static void PC_Transport_StartUartReceive(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_uart_rx,
                                     sizeof(s_uart_rx)) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}

void PC_Transport_Init(void)
{
    PC_Transport_StartUartReceive();
}

uint8_t PC_Transport_TrySend(uint8_t *data, uint16_t length)
{
    if (huart2.gState != HAL_UART_STATE_READY) {
        return 0U;
    }
    return (HAL_UART_Transmit_DMA(&huart2, data, length) == HAL_OK) ? 1U : 0U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance != USART2) {
        return;
    }
    PC_Protocol_FeedFromISR(s_uart_rx, size);
    PC_Transport_StartUartReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) {
        return;
    }
    (void)HAL_UART_AbortReceive(huart);
    PC_Transport_StartUartReceive();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        PC_Protocol_NotifyTxCompleteFromISR();
    }
}
#endif

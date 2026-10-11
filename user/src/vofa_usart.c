#include "vofa_usart.h"
#include "usart.h"
#include "pc_protocol.h"
#include <string.h>

// VOFA+ JustFloat 协议的固定帧尾：0x00 0x00 0x80 0x7F
const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

// 定义全局的接收和发送缓冲区
uint8_t vofa_rx_buffer[VOFA_RX_BUFFER_SIZE];
uint8_t vofa_tx_buffer[VOFA_TX_MAX_FLOAT_COUNT * sizeof(float) + sizeof(vofa_tail)];

// 前台按原缓冲区和事件模式启动 UART DMA 接收。
static void vofa_start_receive(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, vofa_rx_buffer, VOFA_RX_BUFFER_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}

// 初始化串口接收与协议模块，保留 DMA 空闲事件配置。
void vofa_init(void)
{
    pc_protocol_init();
    // 开启串口 DMA + 空闲中断接收，适用于上位机发来的不定长指令
    vofa_start_receive();
}

// HAL 库不定长接收完成回调函数（当串口总线空闲或接收满时触发）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        pc_protocol_feed_from_isr(vofa_rx_buffer, Size);

        // 处理完毕后，重新打开 DMA 接收，准备接收下一帧
        vofa_start_receive();
    }
}

// Overrun/noise/frame errors can stop an RX DMA session.  Recover the receiver
// here so one transient UART error cannot leave the lower computer TX-only.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        (void)HAL_UART_AbortReceive(huart);
        vofa_start_receive();
    }
}

/*
 * 底层 发送 Float 数组函数
 * 升级为：DMA 无阻塞发送方式，彻底释放 FOC 算力
 */
void vofa_just_float_send(float *data, uint8_t count)
{
    if (count == 0 || (count * sizeof(float) + 4) > sizeof(vofa_tx_buffer)) {
        return;
    }

    // 检查 DMA 是否正在发送中？
    // 如果上一帧还没发完，我们选择【直接丢弃当前帧】，坚决不能在这里死等（死等就变成阻塞了，会卡死 FOC 中断！）
    if (huart2.gState != HAL_UART_STATE_READY) {
        return;
    }

    // 1. 将数据本体和帧尾拼接成一整段放入连续缓存区
    uint16_t data_len = count * sizeof(float);
    memcpy(vofa_tx_buffer, data, data_len);          // 放入有效数据
    memcpy(&vofa_tx_buffer[data_len], vofa_tail, 4); // 放入帧尾

    // 2. 发起 DMA 请求，后台硬件会自动把整个包搬运到串口，CPU就跑去继续干别的事了
    HAL_UART_Transmit_DMA(&huart2, vofa_tx_buffer, data_len + 4);
}

// 1. 发送电流采样值 (A相、B相、C相)
void vofa_send_currents(float iu_a, float iv_a, float iw_a)
{
    float send_buf[3];
    send_buf[0] = iu_a;
    send_buf[1] = iv_a;
    send_buf[2] = iw_a;
    vofa_just_float_send(send_buf, 3);
}

// 2. 发送机械角度和电角度 (便于验证极对数和零位)
void vofa_send_angles(float mech_angle, float elec_angle)
{
    float send_buf[2];
    send_buf[0] = mech_angle;
    send_buf[1] = elec_angle;
    vofa_just_float_send(send_buf, 2);
}

// 3. 发送 FOC 变换后的 dq 轴电流
void vofa_send_dq_currents(float id, float iq)
{
    float send_buf[2];
    send_buf[0] = id;
    send_buf[1] = iq;
    vofa_just_float_send(send_buf, 2);
}

// 4. 发送综合调试数据
void vofa_send_all(float elec_angle, float id, float iq, float target_iq)
{
    float send_buf[4];
    send_buf[0] = elec_angle;
    send_buf[1] = id;
    send_buf[2] = iq;
    send_buf[3] = target_iq;
    vofa_just_float_send(send_buf, 4);
}

#include "vofa_usart.h"
#include "usart.h"
#include <string.h>

// VOFA+ JustFloat 协议的固定帧尾：0x00 0x00 0x80 0x7F
const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

// 定义全局的接收和发送缓冲区
uint8_t vofa_rx_buffer[VOFA_RX_BUFFER_SIZE];
uint8_t vofa_tx_buffer[84]; // 20 floats + 4-byte JustFloat frame tail

void VOFA_Init(void)
{
    // 开启串口 DMA + 空闲中断接收，适用于上位机发来的不定长指令
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, vofa_rx_buffer, VOFA_RX_BUFFER_SIZE);
    
    // 关闭 DMA 的半传输完成中断（HT），通常上位机通信不需要处理一半的数据，关掉能省点 CPU
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

// HAL 库不定长接收完成回调函数（当串口总线空闲或接收满时触发）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        // 在这里处理 VOFA+ 或者其他上位机发下来的指令帧
        // 收到的有效数据长度为 Size，数据在 vofa_rx_buffer[0] 到 vofa_rx_buffer[Size-1] 中
        
        // TODO: 添加指令解析代码，比如设定期望电流、PID参数等
        
        // 处理完毕后，重新打开 DMA 接收，准备接收下一帧
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, vofa_rx_buffer, VOFA_RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}

/*
 * 底层 发送 Float 数组函数
 * 升级为：DMA 无阻塞发送方式，彻底释放 FOC 算力
 */
void VOFA_JustFloat_Send(float *data, uint8_t count)
{
    if (count == 0 || (count * sizeof(float) + 4) > sizeof(vofa_tx_buffer)) return;
    
    // 检查 DMA 是否正在发送中？
    // 如果上一帧还没发完，我们选择【直接丢弃当前帧】，坚决不能在这里死等（死等就变成阻塞了，会卡死 FOC 中断！）
    if (huart2.gState != HAL_UART_STATE_READY)
    {
        return; 
    }
    
    // 1. 将数据本体和帧尾拼接成一整段放入连续缓存区
    uint16_t data_len = count * sizeof(float);
    memcpy(vofa_tx_buffer, data, data_len);                     // 放入有效数据
    memcpy(&vofa_tx_buffer[data_len], vofa_tail, 4);            // 放入帧尾
    
    // 2. 发起 DMA 请求，后台硬件会自动把整个包搬运到串口，CPU就跑去继续干别的事了
    HAL_UART_Transmit_DMA(&huart2, vofa_tx_buffer, data_len + 4);
}

// 1. 发送电流采样值 (A相、B相、C相)
void VOFA_Send_Currents(float iu_a, float iv_a, float iw_a)
{
    float send_buf[3];
    send_buf[0] = iu_a;
    send_buf[1] = iv_a;
    send_buf[2] = iw_a;
    VOFA_JustFloat_Send(send_buf, 3);
}

// 2. 发送机械角度和电角度 (便于验证极对数和零位)
void VOFA_Send_Angles(float mech_angle, float elec_angle)
{
    float send_buf[2];
    send_buf[0] = mech_angle;
    send_buf[1] = elec_angle;
    VOFA_JustFloat_Send(send_buf, 2);
}

// 3. 发送 FOC 变换后的 dq 轴电流
void VOFA_Send_DQ_Currents(float id, float iq)
{
    float send_buf[2];
    send_buf[0] = id;
    send_buf[1] = iq;
    VOFA_JustFloat_Send(send_buf, 2);
}

// 4. 发送综合调试数据
void VOFA_Send_All(float elec_angle, float id, float iq, float target_iq)
{
    float send_buf[4];
    send_buf[0] = elec_angle;
    send_buf[1] = id;
    send_buf[2] = iq;
    send_buf[3] = target_iq;
    VOFA_JustFloat_Send(send_buf, 4);
}

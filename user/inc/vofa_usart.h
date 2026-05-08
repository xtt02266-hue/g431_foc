#ifndef VOFA_USART_H
#define VOFA_USART_H

#include "stm32g4xx_hal.h"

/* 根据实际使用的串口进行修改，当前默认使用 huart2 */
extern UART_HandleTypeDef huart2;

// 定义接收缓存区大小
#define VOFA_RX_BUFFER_SIZE 128
extern uint8_t vofa_rx_buffer[VOFA_RX_BUFFER_SIZE];

/* 
 * 初始化/启动接收等（如有需要）
 */
void VOFA_Init(void);

/*
 * 底层发送函数: 发送不定长 Float 数组 (符合 VOFA+ JustFloat 协议)
 * 警告：为了防止阻塞 FOC 中断，默认使用轮询但在 FOC 调试中建议配合 DMA 
 */
void VOFA_JustFloat_Send(float *data, uint8_t count);

/*
 * 面向应用层的具体发送函数
 */
// 1. 发送电流采样值 (A相、B相、C相)
void VOFA_Send_Currents(float iu_a, float iv_a, float iw_a);

// 2. 发送机械角度和电角度 (便于验证极对数和零位)
void VOFA_Send_Angles(float mech_angle, float elec_angle);

// 3. 发送 FOC 变换后的 dq 轴电流
void VOFA_Send_DQ_Currents(float id, float iq);

// 4. 发送综合调试数据 (按需添加变量)
void VOFA_Send_All(float elec_angle, float id, float iq, float target_iq);

#endif // VOFA_USART_H
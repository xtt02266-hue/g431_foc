#include "as5600.h"
#include "i2c.h"

uint8_t as5600_rx_buffer[2] = {0};
volatile uint16_t as5600_current_angle = 0;
volatile uint8_t as5600_i2c_error = 0;  // 0=正常, 非0=I2C通信故障

// 请求 DMA 进行无阻塞读取
void AS5600_RequestRead_DMA(void)
{
    // 如果 I2C 外设处于空闲状态，则发起 DMA 获取请求
    if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
    {
        // 推荐使用 Mem_Read_DMA，一步搞定发送寄存器地址和接收数据
        HAL_I2C_Mem_Read_DMA(&hi2c1, AS5600_Address, RAW_ANGLE_HI, I2C_MEMADD_SIZE_8BIT, as5600_rx_buffer, 2);
    }
}

// 获取最新缓存的角度（原函数的无阻塞形式）
uint16_t AS5600_ReadRawAngle(void)
{
    return as5600_current_angle;
}

// 当 DMA 接收完成时，由 HAL 库自动调用此回调
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        // 解析 DMA 搬运过来的 2 字节数据
        as5600_current_angle = (uint16_t)((as5600_rx_buffer[0] & 0x0F) << 8) | as5600_rx_buffer[1];
        as5600_i2c_error = 0;  // 通信成功，清除错误标志
        
        // 读完一次立刻发起下一次请求，实现"后台永动机"式的持续角度刷新
        AS5600_RequestRead_DMA();
    }
}

// I2C 通信错误回调（超时、仲裁丢失、NACK 等）
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        as5600_i2c_error = 1;  // 标记 I2C 故障
        
        // 尝试恢复：重新初始化 I2C 外设
        HAL_I2C_DeInit(hi2c);
        MX_I2C1_Init();
        
        AS5600_RequestRead_DMA();
    }
}

// 获取 I2C 通信状态（供外部诊断）
uint8_t AS5600_IsI2cOk(void)
{
    return (as5600_i2c_error == 0) ? 1 : 0;
}

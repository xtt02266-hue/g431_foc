#include "as5600.h"
#include "i2c.h"

// 读取 AS5600 原始角度寄存器（12 位）。
uint16_t AS5600_ReadRawAngle(void)
{
    uint8_t buf[2] = {0};
    // 统一使用 I2C1 句柄。
    I2C_HandleTypeDef *hi2c = &hi2c1;

    // 发送空写以确保总线处于已知状态。
    HAL_I2C_Master_Transmit(hi2c, AS5600_Address, 0, 0, I2C_TIMEOUT);

    // 设置待读取寄存器地址。
    uint8_t regAddr = RAW_ANGLE_HI;
    HAL_I2C_Master_Transmit(hi2c, AS5600_Address, &regAddr, 1, I2C_TIMEOUT);

    // 再次空写，用于分隔事务。
    HAL_I2C_Master_Transmit(hi2c, AS5600_Address, 0, 0, I2C_TIMEOUT);

    // 读取高低字节。
    HAL_I2C_Master_Receive(hi2c, AS5600_Address | 0x01, buf, 2, I2C_TIMEOUT);

    // 高字节低 4 位与低字节拼接得到 12 位角度。
    return (uint16_t)((buf[0] & 0x0F) << 8) | buf[1];
}

#include "as5600.h"
#include "i2c.h"

uint16_t AS5600_ReadRawAngle(void)
{
    uint8_t buf[2] = {0};
    I2C_HandleTypeDef *hi2c = &hi2c1;

    HAL_I2C_Master_Transmit(hi2c, AS5600_Address, 0, 0, I2C_TIMEOUT);

    uint8_t regAddr = RAW_ANGLE_HI;
    HAL_I2C_Master_Transmit(hi2c, AS5600_Address, &regAddr, 1, I2C_TIMEOUT);

    HAL_I2C_Master_Transmit(hi2c, AS5600_Address, 0, 0, I2C_TIMEOUT);

    HAL_I2C_Master_Receive(hi2c, AS5600_Address | 0x01, buf, 2, I2C_TIMEOUT);

    return (uint16_t)((buf[0] & 0x0F) << 8) | buf[1];
}

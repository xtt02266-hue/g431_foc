#include "oled.h"

#include "gpio.h"
#include "OLED_Font.h"

// 以 SystemCoreClock 估算软件 I2C 忙等待次数；us 为请求微秒数，实际时长受编译和指令开销影响。
static void oled_delay_us(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 10;
    while (ticks--)
        ;
}

// I2C SCL 引脚写入电平。
static void oled_w_scl(uint8_t bit_value)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (GPIO_PinState)(bit_value));
    oled_delay_us(6);
}

// I2C SDA 引脚写入电平。
static void oled_w_sda(uint8_t bit_value)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (GPIO_PinState)(bit_value));
    oled_delay_us(6);
}

//#define oled_w_scl(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (GPIO_PinState)(x))
//#define oled_w_sda(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (GPIO_PinState)(x))
// I2C 引脚初始状态设置。
static void oled_i2c_init(void)
{
    HAL_Delay(10);

    oled_w_scl(1);
    oled_w_sda(1);
}

/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
static void oled_i2c_start(void)
{
    oled_w_sda(1);
    oled_w_scl(1);
    oled_w_sda(0);
    oled_w_scl(0);
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
static void oled_i2c_stop(void)
{
    oled_w_sda(0);
    oled_w_scl(1);
    oled_w_sda(1);
}

/**
  * @brief  I2C发送一个字节
  * @param  byte 要发送的一个字节
  * @retval 无
  */
static void oled_i2c_send_byte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        oled_w_sda(!!(byte & (0x80 >> i)));
        oled_w_scl(1);
        oled_w_scl(0);
    }
    oled_w_scl(1); //额外的一个时钟，不处理应答信号
    oled_w_scl(0);
}

/**
  * @brief  OLED写命令
  * @param  command 要写入的命令
  * @retval 无
  */
static void oled_write_command(uint8_t command)
{
    oled_i2c_start();
    oled_i2c_send_byte(0x78); //从机地址
    oled_i2c_send_byte(0x00); //写命令
    oled_i2c_send_byte(command);
    oled_i2c_stop();
}

/**
  * @brief  OLED写数据
  * @param  data 要写入的数据
  * @retval 无
  */
static void oled_write_data(uint8_t data)
{
    oled_i2c_start();
    oled_i2c_send_byte(0x78); //从机地址
    oled_i2c_send_byte(0x40); //写数据
    oled_i2c_send_byte(data);
    oled_i2c_stop();
}

/**
  * @brief  OLED设置光标位置
  * @param  y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  x 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
static void oled_set_cursor(uint8_t y, uint8_t x)
{
    oled_write_command(0xB0 | y);                 //设置Y位置
    oled_write_command(0x10 | ((x & 0xF0) >> 4)); //设置X位置高4位
    oled_write_command(0x00 | (x & 0x0F));        //设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void oled_clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) {
        oled_set_cursor(j, 0);
        for (i = 0; i < 128; i++) {
            oled_write_data(0x00);
        }
    }
}

/**
  * @brief  OLED显示一个字符
  * @param  line 行位置，范围：1~4
  * @param  column 列位置，范围：1~16
  * @param  character 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void oled_show_char(uint8_t line, uint8_t column, char character)
{
    uint8_t i;
    oled_set_cursor((line - 1) * 2, (column - 1) * 8); //设置光标位置在上半部分
    for (i = 0; i < 8; i++) {
        oled_write_data(OLED_F8x16[((uint16_t)(character - ' ') * 16U) + i]); //显示上半部分内容
    }
    oled_set_cursor((line - 1) * 2 + 1, (column - 1) * 8); //设置光标位置在下半部分
    for (i = 0; i < 8; i++) {
        oled_write_data(OLED_F8x16[((uint16_t)(character - ' ') * 16U) + i + 8U]); //显示下半部分内容
    }
}

/**
  * @brief  OLED显示字符串
  * @param  line 起始行位置，范围：1~4
  * @param  column 起始列位置，范围：1~16
  * @param  text 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void oled_show_string(uint8_t line, uint8_t column, char *text)
{
    uint8_t i;
    for (i = 0; text[i] != '\0'; i++) {
        oled_show_char(line, column + i, text[i]);
    }
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
static uint32_t oled_pow(uint32_t x, uint32_t y)
{
    uint32_t result = 1;
    while (y--) {
        result *= x;
    }
    return result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  line 起始行位置，范围：1~4
  * @param  column 起始列位置，范围：1~16
  * @param  number 要显示的数字，范围：0~4294967295
  * @param  length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void oled_show_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t i;
    for (i = 0; i < length; i++) {
        oled_show_char(line, column + i, number / oled_pow(10, length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  line 起始行位置，范围：1~4
  * @param  column 起始列位置，范围：1~16
  * @param  number 要显示的数字，范围：-2147483648~2147483647
  * @param  length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void oled_show_signed_num(uint8_t line, uint8_t column, int32_t number, uint8_t length)
{
    uint8_t i;
    uint32_t magnitude;
    if (number >= 0) {
        oled_show_char(line, column, '+');
        magnitude = number;
    } else {
        oled_show_char(line, column, '-');
        magnitude = -number;
    }
    for (i = 0; i < length; i++) {
        oled_show_char(line, column + i + 1, magnitude / oled_pow(10, length - i - 1) % 10 + '0');
    }
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  line 起始行位置，范围：1~4
  * @param  column 起始列位置，范围：1~16
  * @param  number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void oled_show_hex_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t i, digit;
    for (i = 0; i < length; i++) {
        digit = number / oled_pow(16, length - i - 1) % 16;
        if (digit < 10) {
            oled_show_char(line, column + i, digit + '0');
        } else {
            oled_show_char(line, column + i, digit - 10 + 'A');
        }
    }
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  line 起始行位置，范围：1~4
  * @param  column 起始列位置，范围：1~16
  * @param  number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void oled_show_bin_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
    uint8_t i;
    for (i = 0; i < length; i++) {
        oled_show_char(line, column + i, number / oled_pow(2, length - i - 1) % 2 + '0');
    }
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void oled_init(void)
{

    HAL_Delay(100);

    oled_i2c_init(); //端口初始化

    oled_write_command(0xAE); //关闭显示

    oled_write_command(0xD5); //设置显示时钟分频比/振荡器频率
    oled_write_command(0x80);

    oled_write_command(0xA8); //设置多路复用率
    oled_write_command(0x3F);

    oled_write_command(0xD3); //设置显示偏移
    oled_write_command(0x00);

    oled_write_command(0x40); //设置显示开始行

    oled_write_command(0xA1); //设置左右方向，0xA1正常 0xA0左右反置

    oled_write_command(0xC8); //设置上下方向，0xC8正常 0xC0上下反置

    oled_write_command(0xDA); //设置COM引脚硬件配置
    oled_write_command(0x12);

    oled_write_command(0x81); //设置对比度控制
    oled_write_command(0xCF);

    oled_write_command(0xD9); //设置预充电周期
    oled_write_command(0xF1);

    oled_write_command(0xDB); //设置VCOMH取消选择级别
    oled_write_command(0x30);

    oled_write_command(0xA4); //设置整个显示打开/关闭

    oled_write_command(0xA6); //设置正常/倒转显示

    oled_write_command(0x8D); //设置充电泵
    oled_write_command(0x14);

    oled_write_command(0xAF); //开启显示

    oled_clear(); //OLED清屏
}

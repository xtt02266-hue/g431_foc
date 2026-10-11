#ifndef OLED_H
#define OLED_H

#include <stdint.h>

// OLED 驱动接口。
void oled_init(void);
// 清空 OLED 显示缓存对应的全部页面，使用既有软件 I2C 写入时序。
void oled_clear(void);
// 在指定行列显示单个字符；保持现有字模和行列约定。
void oled_show_char(uint8_t line, uint8_t column, char character);
// 从指定行列显示以零结尾的字符串，沿用原有字符宽度和调用方边界约束。
void oled_show_string(uint8_t line, uint8_t column, char *text);
// 按指定宽度显示十进制无符号数，保留补零规则。
void oled_show_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
// 按指定宽度显示有符号十进制数，保留正负号规则。
void oled_show_signed_num(uint8_t line, uint8_t column, int32_t number, uint8_t length);
// 按指定宽度显示十六进制数。
void oled_show_hex_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
// 按指定宽度显示二进制数。
void oled_show_bin_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

#endif

#ifndef MT6826S_H
#define MT6826S_H

#include <stdint.h>

#define MT6826S_ANGLE_MAX_15BIT 32767U

#define MT6826S_STATUS_OVERSPEED    (1U << 0)
#define MT6826S_STATUS_STRONG_FIELD (1U << 1)
#define MT6826S_STATUS_UNDERVOLTAGE (1U << 2)

// 初始化 DMA 缓冲区和通信诊断历史，片选恢复空闲高电平。
void mt6826s_init(void);
/* Start one non-blocking SPI1 burst. A request is ignored while DMA is busy. */
void mt6826s_request_read_dma(void);
// 读取最近编码器通信有效标志；角度新鲜度由上层检查。
uint8_t mt6826s_is_ok(void);
// 读取最近突发帧中的芯片状态，不发起 SPI 查询。
uint8_t mt6826s_get_status(void);
// 读取累计 CRC 错误次数；错误帧不发布新角度。
uint32_t mt6826s_get_crc_error_count(void);
// 读取累计 SPI/DMA 传输错误次数。
uint32_t mt6826s_get_transfer_error_count(void);
/* Main-loop only. 0=busy, 1=done, 2=SPI error. Never overlap angle DMA. */
uint8_t mt6826s_read_register(uint16_t address, uint8_t *value);
// 前台写入寄存器并恢复 DMA 仲裁状态；返回值沿用寄存器事务状态码。
uint8_t mt6826s_write_register(uint16_t address, uint8_t value);
/* After successful/uncertain self-calibration only a power cycle is valid. */
void mt6826s_suspend_angle_reads(void);

#endif

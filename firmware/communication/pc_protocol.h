#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H

#include <stdint.h>

#define PC_PROTOCOL_VERSION     2U
#define PC_PROTOCOL_MAX_PAYLOAD 256U

// 复位解析、发送、令牌及重放缓存，保留协议初始序号和诊断计数。
void pc_protocol_init(void);
// 接收 ISR 将明确长度的字节片段写入固定环形队列；前台负责解析命令。
void pc_protocol_feed_from_isr(const uint8_t *data, uint16_t length);
// 前台推进解析、DMA 重试和周期状态发送；时间参数单位 ms。
void pc_protocol_task(void);
// 按既有遥测字段序列化观测量，保留采集记录配对及发送成功后的消费次序。
void pc_protocol_send_telemetry(void);
// 读取累计接收 CRC 错误次数。
uint32_t pc_protocol_get_crc_error_count(void);
// 读取接收环形队列溢出次数。
uint32_t pc_protocol_get_rx_overflow_count(void);
// 读取遥测发送未被接受的累计次数。
uint32_t pc_protocol_get_telemetry_drop_count(void);

#endif

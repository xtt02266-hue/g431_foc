#include "mt6826s.h"
#include "spi.h"
#include "gpio.h"
#include "motor_encoder.h"

#define MT6826S_CS_GPIO_PORT GPIOA      // 编码器 SPI 软件片选所在端口
#define MT6826S_CS_GPIO_PIN  GPIO_PIN_4 // PA4 软件片选，低电平有效

/* 突发读取命令为 1010b，后接 12 位起始寄存器地址 0x003。 */
#define MT6826S_BURST_COMMAND_HIGH 0xA0U
#define MT6826S_BURST_COMMAND_LOW  0x03U // 突发读取起始寄存器地址低字节
#define MT6826S_BURST_FRAME_BYTES  6U    // DMA 命令与角度/状态/CRC 的完整字节数
#define MT6826S_CRC8_POLYNOMIAL    0x07U // 芯片帧 CRC-8 多项式 x^8+x^2+x+1

/* SPI DMA 占用以下缓冲区直至完成或异常回调；保持缓冲区静态生命周期。 */
/* 角度 DMA 缓冲区在请求后由 SPI/DMA 独占，完成或异常回调释放片选和忙状态。
 * 前台寄存器事务通过原锁与角度 DMA 仲裁；错误计数由完成/异常/超时路径更新。
 */
static const uint8_t g_mt6826s_tx[MT6826S_BURST_FRAME_BYTES] = {
    MT6826S_BURST_COMMAND_HIGH, MT6826S_BURST_COMMAND_LOW, 0x00U, 0x00U, 0x00U, 0x00U};
static uint8_t g_mt6826s_rx[MT6826S_BURST_FRAME_BYTES];
static volatile uint8_t g_mt6826s_busy = 0U;
static volatile uint8_t g_mt6826s_ok = 0U;
static volatile uint16_t g_mt6826s_last_angle = 0U;
static volatile uint8_t g_mt6826s_status = 0U;
static volatile uint32_t g_mt6826s_crc_error_count = 0U;
static volatile uint32_t g_mt6826s_transfer_error_count = 0U;
static uint32_t g_mt6826s_start_ms = 0U;
static uint8_t g_mt6826s_timeout_reported = 0U;
static volatile uint8_t g_register_lock, g_angle_reads_suspended;

// 按 MT6826S 的 CRC-8 多项式校验指定字节范围，保留初值与位处理方向。
static uint8_t mt6826s_calculate_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    for (uint8_t byte_index = 0U; byte_index < length; ++byte_index) {
        crc ^= data[byte_index];
        for (uint8_t bit_index = 0U; bit_index < 8U; ++bit_index) {
            crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1) ^ MT6826S_CRC8_POLYNOMIAL) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// 拉低 PA4 片选，开始原有 SPI 事务。
static void mt6826s_select(void)
{
    HAL_GPIO_WritePin(MT6826S_CS_GPIO_PORT, MT6826S_CS_GPIO_PIN, GPIO_PIN_RESET);
}

// 拉高 PA4 片选，结束 SPI 事务；回调和异常退出共用。
static void mt6826s_unselect(void)
{
    HAL_GPIO_WritePin(MT6826S_CS_GPIO_PORT, MT6826S_CS_GPIO_PIN, GPIO_PIN_SET);
}

// 初始化 DMA 缓冲区和通信诊断历史，片选恢复空闲高电平。
void mt6826s_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    mt6826s_unselect();

    GPIO_InitStruct.Pin = MT6826S_CS_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MT6826S_CS_GPIO_PORT, &GPIO_InitStruct);

    mt6826s_unselect();
    g_mt6826s_ok = 0U;
    g_mt6826s_last_angle = 0U;
    g_mt6826s_status = 0U;
    g_mt6826s_crc_error_count = 0U;
    g_mt6826s_transfer_error_count = 0U;
    g_mt6826s_busy = 0U;
    g_mt6826s_timeout_reported = 0U;
    g_register_lock = g_angle_reads_suspended = 0U;
}

// 请求突发角度 DMA；忙碌、寄存器占用或暂停状态按原规则跳过。
void mt6826s_request_read_dma(void)
{
    if (g_register_lock || g_angle_reads_suspended) {
        return;
    }
    if (g_mt6826s_busy != 0U) {
        /* A lost DMA completion must invalidate the angle instead of holding
         * the last valid reading indefinitely. Do not touch an active DMA. */
        if ((g_mt6826s_timeout_reported == 0U) && ((uint32_t)(HAL_GetTick() - g_mt6826s_start_ms) > 2U)) {
            g_mt6826s_timeout_reported = 1U;
            g_mt6826s_ok = 0U;
            ++g_mt6826s_transfer_error_count;
        }
        return;
    }

    g_mt6826s_busy = 1U;
    g_mt6826s_start_ms = HAL_GetTick();
    g_mt6826s_timeout_reported = 0U;
    mt6826s_select();
    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive_DMA(&hspi1, g_mt6826s_tx, g_mt6826s_rx, MT6826S_BURST_FRAME_BYTES);
    if (status != HAL_OK) {
        mt6826s_unselect();
        g_mt6826s_ok = 0U;
        ++g_mt6826s_transfer_error_count;
        /* HAL may have started RX DMA before TX setup failed. Keep this
         * request latched so the ADC IRQ cannot reuse either DMA buffer. */
        g_mt6826s_timeout_reported = 1U;
    }
}

// SPI DMA 完成 ISR 检查 CRC 和状态后发布角度；坏帧保留最近有效时间戳。
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1) {
        return;
    }

    mt6826s_unselect();

    /* During the two command bytes MISO is high impedance.  The following
     * bytes are registers 0x003 (ANGLE[14:7]), 0x004 (ANGLE[6:0],0),
     * 0x005 (STATUS[2:0]) and 0x006 (CRC8 over the preceding 24 bits). */
    uint8_t calculated_crc = mt6826s_calculate_crc8(&g_mt6826s_rx[2], 3U);
    if (calculated_crc != g_mt6826s_rx[5]) {
        /* Drop this frame without publishing its angle or refreshing the
         * timestamp. The last CRC-valid sample remains usable only within
         * the existing 2 ms freshness deadline; repeated CRC loss goes stale. */
        ++g_mt6826s_crc_error_count;
        g_mt6826s_busy = 0U;
        return;
    }

    /*
     * STATUS[2:0] are warning flags, not a frame-valid indication.  The
     * datasheet still defines ANGLE and CRC for frames with warning bits set.
     * Rejecting every warning frame used to hold the previous angle for whole
     * rotor sectors (most visibly when the motor field asserted STATUS[1]),
     * which then produced a large artificial speed spike when the warning
     * cleared.  Keep publishing CRC-valid angles and report STATUS separately.
     */
    g_mt6826s_status = g_mt6826s_rx[4] & 0x07U;
    g_mt6826s_last_angle =
        ((((uint16_t)g_mt6826s_rx[2]) << 7) | (((uint16_t)g_mt6826s_rx[3]) >> 1)) & MT6826S_ANGLE_MAX_15BIT;
    motor_encoder_on_sample(g_mt6826s_last_angle);
    g_mt6826s_ok = 1U;
    g_mt6826s_busy = 0U;
}

// SPI 异常 ISR 结束片选、释放忙状态并记录通信错误。
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1) {
        return;
    }
    mt6826s_unselect();
    g_mt6826s_ok = 0U;
    ++g_mt6826s_transfer_error_count;
    /* The other DMA channel may still be active after an error. The motor
     * will stop on invalid encoder data; do not reuse these buffers. */
    g_mt6826s_timeout_reported = 1U;
}

// 读取最近编码器通信有效标志；角度新鲜度由上层检查。
uint8_t mt6826s_is_ok(void)
{
    return g_mt6826s_ok;
}

// 读取最近突发帧中的芯片状态，不发起 SPI 查询。
uint8_t mt6826s_get_status(void)
{
    return g_mt6826s_status;
}

// 读取累计 CRC 错误次数；错误帧不发布新角度。
uint32_t mt6826s_get_crc_error_count(void)
{
    return g_mt6826s_crc_error_count;
}

// 读取累计 SPI/DMA 传输错误次数。
uint32_t mt6826s_get_transfer_error_count(void)
{
    return g_mt6826s_transfer_error_count;
}

// 前台在原有互斥条件下执行最长 1 ms 的寄存器事务；0 忙、1 成功、2 错误。
static uint8_t mt6826s_register_transfer(uint16_t address, uint8_t write, uint8_t *value)
{
    if (address > 0xFFFU || value == 0 || g_angle_reads_suspended) {
        return 2U;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (g_mt6826s_busy || g_register_lock) {
        if (!mask) {
            __enable_irq();
        }
        return 0U;
    }
    g_register_lock = 1U;
    if (!mask) {
        __enable_irq();
    }
    uint8_t tx[3] = {(uint8_t)((write ? 0x60U : 0x30U) | (address >> 8U)), (uint8_t)address, write ? *value : 0U};
    uint8_t rx[3] = {0};
    mt6826s_select();
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3U, 1U);
    mt6826s_unselect();
    if (result != HAL_OK) {
        g_mt6826s_ok = 0U;
        ++g_mt6826s_transfer_error_count;
    } else if (!write) {
        *value = rx[2];
    }
    g_register_lock = 0U;
    return result == HAL_OK ? 1U : 2U;
}

// 前台读取芯片寄存器；返回 0 忙、1 成功、2 错误，输出字节保持原规则。
uint8_t mt6826s_read_register(uint16_t address, uint8_t *value)
{
    return mt6826s_register_transfer(address, 0U, value);
}

// 前台写入寄存器并恢复 DMA 仲裁状态；返回值沿用寄存器事务状态码。
uint8_t mt6826s_write_register(uint16_t address, uint8_t value)
{
    return mt6826s_register_transfer(address, 1U, &value);
}

// 暂停后续角度采样请求并失效缓存，用于自校准后的重新上电门控。
void mt6826s_suspend_angle_reads(void)
{
    g_angle_reads_suspended = 1U;
}

#include "motor_cogging.h"
#include "motor_cogging_builtin.h"
#include "motor_config.h"
#include "motor_encoder.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define COGGING_FLASH_MAGIC            0x434F4747UL                    // 齿槽 Flash 记录标识 COGG
#define COGGING_FLASH_VERSION          2U                              // 现有 Flash 记录版本；保持旧存储布局
#define COGGING_RECEIVED_BYTES         (MOTOR_COGGING_TABLE_SIZE / 8U) // 完整表的收齐位图容量，单位字节
#define COGGING_TRANSACTION_TIMEOUT_MS 2000U                           // 表上传事务有效间隔，单位 ms

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    uint8_t enabled;
    uint8_t reserved0[3];
    float gain;
    float max_iq_a;
    float fade_start_rpm;
    float fade_end_rpm;
    uint16_t phase_offset_counts;
    uint16_t table_count;
    uint32_t table_crc;
    uint32_t table_revision;
    int16_t table[MOTOR_COGGING_TABLE_SIZE];
    uint32_t builtin_table_crc; // 内置表来源标签，沿用原保留字位置；旧记录为 0 时不覆盖新默认表
    uint32_t record_crc;
} motor_cogging_flash_record_t;

_Static_assert((sizeof(motor_cogging_flash_record_t) % 8U) == 0U, "Cogging flash record must be double-word aligned");
_Static_assert(sizeof(motor_cogging_flash_record_t) <= FLASH_PAGE_SIZE, "Cogging flash record exceeds one flash page");
_Static_assert(sizeof(s_motor_cogging_builtin_table) == MOTOR_COGGING_TABLE_SIZE * sizeof(int16_t),
               "Built-in cogging table must contain 512 points");

extern uint8_t __cogging_params_flash_start__[];

/* 前台命令写暂存表，控制路径只读活动表并更新渐入增益。
 * 表提交、配置和 Flash 操作的同步由原调用路径负责；双表及交换顺序保持不变。
 * g_received 是收齐位图，事务时间戳单位 ms；表值乘缩放系数后为 Iq（A）。
 */
static motor_cogging_config_t g_config;
static int16_t g_table_a[MOTOR_COGGING_TABLE_SIZE];
static int16_t g_table_b[MOTOR_COGGING_TABLE_SIZE];
static int16_t *g_active_table = g_table_a;
static int16_t *g_staging_table = g_table_b;
static uint8_t g_received[COGGING_RECEIVED_BYTES];
static uint16_t g_transaction_counter;
static uint16_t g_active_transaction;
static uint32_t g_transaction_last_ms;
static uint32_t g_expected_crc;
static uint32_t g_active_crc;
static uint32_t g_table_revision;
static uint16_t g_ramp_elapsed_ms;
static float g_effective_gain;
static uint8_t g_persisted;

// 按反射 CRC-32 多项式计算表或记录校验值；输入是明确长度的字节范围。
uint32_t motor_cogging_crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t index = 0U; index < length; ++index) {
        crc ^= bytes[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

// 检查增益、电流限幅、速度衰减区间和编码器相位偏移；无效配置返回 0。
uint8_t motor_cogging_is_config_valid(const motor_cogging_config_t *config)
{
    if ((config == NULL) || (config->enabled > 1U) || !isfinite(config->gain) || !isfinite(config->max_iq_a) ||
        !isfinite(config->fade_start_rpm) || !isfinite(config->fade_end_rpm)) {
        return 0U;
    }
    if ((config->gain < 0.0f) || (config->gain > 10.0f) || (config->max_iq_a < 0.0f) ||
        (config->max_iq_a > MOTOR_TORQUE_CURRENT_LIMIT_A) || (config->fade_start_rpm < 0.0f) ||
        (config->fade_end_rpm <= config->fade_start_rpm) || (config->fade_end_rpm > MOTOR_HOST_SPEED_MAX_RPM) ||
        (config->phase_offset_counts > MOTOR_ENCODER_COUNT_MASK_U16) ||
        ((config->enabled != 0U) && (config->max_iq_a <= 0.0f))) {
        return 0U;
    }
    return 1U;
}

// 装入固件内置候选表及用户实测配置；CRC 正确才使能补偿。
static void motor_cogging_set_defaults(void)
{
    g_config.enabled = 0U;
    g_config.gain = 0.90f;
    g_config.max_iq_a = 0.080f;
    g_config.fade_start_rpm = 100.0f;
    g_config.fade_end_rpm = 200.0f;
    g_config.phase_offset_counts = 0U;
    memcpy(g_table_a, s_motor_cogging_builtin_table, sizeof(g_table_a));
    memset(g_table_b, 0, sizeof(g_table_b));
    g_active_table = g_table_a;
    g_staging_table = g_table_b;
    g_active_crc = motor_cogging_crc32(g_active_table, sizeof(g_table_a));
    g_table_revision = 0U;
    g_persisted = 0U;
    if (g_active_crc == MOTOR_COGGING_BUILTIN_CRC32) {
        g_config.enabled = 1U;
        g_table_revision = MOTOR_COGGING_BUILTIN_REVISION;
        g_persisted = 1U;
    } else {
        memset(g_table_a, 0, sizeof(g_table_a));
        g_active_crc = motor_cogging_crc32(g_active_table, sizeof(g_table_a));
    }
}

// 只恢复与本次内置表来源匹配的有效记录；旧表不覆盖新默认值，失败不擦写参数页。
static uint8_t motor_cogging_load_from_flash(void)
{
    motor_cogging_flash_record_t record;
    const void *address = (const void *)(uintptr_t)__cogging_params_flash_start__;
    memcpy(&record, address, sizeof(record));
    if ((record.magic != COGGING_FLASH_MAGIC) || (record.version != COGGING_FLASH_VERSION) ||
        (record.record_size != sizeof(record)) || (record.table_count != MOTOR_COGGING_TABLE_SIZE) ||
        (record.builtin_table_crc != MOTOR_COGGING_BUILTIN_CRC32) ||
        (record.record_crc != motor_cogging_crc32(&record, offsetof(motor_cogging_flash_record_t, record_crc)))) {
        return 0U;
    }

    motor_cogging_config_t config = {record.enabled,        record.gain,         record.max_iq_a,
                                     record.fade_start_rpm, record.fade_end_rpm, record.phase_offset_counts};
    if ((motor_cogging_is_config_valid(&config) == 0U) ||
        (record.table_crc != motor_cogging_crc32(record.table, sizeof(record.table)))) {
        return 0U;
    }

    g_config = config;
    memcpy(g_table_a, record.table, sizeof(g_table_a));
    g_active_table = g_table_a;
    g_staging_table = g_table_b;
    g_active_crc = record.table_crc;
    g_table_revision = record.table_revision;
    g_persisted = 1U;
    return 1U;
}

// 初始化内置表及事务状态，再恢复来源匹配的已保存配置；不启动电机或擦写 Flash。
void motor_cogging_init(void)
{
    // 先提供随固件下载的默认补偿表，无需上位机导入。
    motor_cogging_set_defaults();
    g_transaction_counter = 0U;
    g_active_transaction = 0U;
    g_ramp_elapsed_ms = 0U;
    g_effective_gain = 0.0f;
    // 保留本版本主动保存的配置，包括编码器重标定时保存的禁用状态。
    (void)motor_cogging_load_from_flash();
}

// 接受有效配置并更新补偿参数；电流单位 A，速度单位 rpm，相位单位计数。
void motor_cogging_set_config(uint8_t enabled, float gain, float max_iq_a, float fade_start_rpm, float fade_end_rpm,
                              uint16_t phase_offset_counts)
{
    const motor_cogging_config_t config = {enabled, gain, max_iq_a, fade_start_rpm, fade_end_rpm, phase_offset_counts};

    if (motor_cogging_is_config_valid(&config) != 0U) {
        g_config = config;
        g_persisted = 0U;
        motor_cogging_runtime_stop();
    }
}

// 复制当前配置供诊断或协议读取；不改变活动表。
void motor_cogging_get_config(motor_cogging_config_t *config)
{
    if (config != NULL) {
        *config = g_config;
    }
}

// 开始完整表上传事务；点数必须匹配固定表容量，失败返回 0。
uint8_t motor_cogging_table_begin(uint16_t count, uint32_t expected_crc)
{
    if (count != MOTOR_COGGING_TABLE_SIZE) {
        return 0U;
    }
    ++g_transaction_counter;
    if (g_transaction_counter == 0U) {
        ++g_transaction_counter;
    }
    g_active_transaction = g_transaction_counter;
    g_transaction_last_ms = HAL_GetTick();
    g_expected_crc = expected_crc;
    memset(g_received, 0, sizeof(g_received));
    return 1U;
}

// 读取前台分配的事务号；后台清理活动事务不会改变此计数。
uint16_t motor_cogging_get_transaction_id(void)
{
    return g_transaction_counter;
}

// 按事务号、偏移和点数写暂存表；记录收齐位图，活动表仍可读取。
uint8_t motor_cogging_table_write_chunk(uint16_t transaction_id, uint16_t offset, uint8_t count, const int16_t *values)
{
    if ((transaction_id == 0U) || (transaction_id != g_active_transaction) || (values == NULL) ||
        ((uint32_t)(HAL_GetTick() - g_transaction_last_ms) > COGGING_TRANSACTION_TIMEOUT_MS) || (count == 0U) ||
        (count > MOTOR_COGGING_CHUNK_MAX_POINTS) || ((uint32_t)offset + count > MOTOR_COGGING_TABLE_SIZE)) {
        return 0U;
    }
    for (uint16_t i = 0U; i < count; ++i) {
        if ((values[i] > MOTOR_COGGING_TABLE_MAX_COUNTS) || (values[i] < -MOTOR_COGGING_TABLE_MAX_COUNTS)) {
            return 0U;
        }
    }
    g_transaction_last_ms = HAL_GetTick();
    memcpy(&g_staging_table[offset], values, (uint32_t)count * sizeof(int16_t));
    for (uint16_t i = 0U; i < count; ++i) {
        uint16_t index = (uint16_t)(offset + i);
        g_received[index >> 3U] |= (uint8_t)(1U << (index & 7U));
    }
    return 1U;
}

// 检查事务、收齐位图和 CRC 后交换活动/暂存表，保持原有提交边界。
uint8_t motor_cogging_table_commit(uint16_t transaction_id)
{
    if ((transaction_id == 0U) || (transaction_id != g_active_transaction) ||
        ((uint32_t)(HAL_GetTick() - g_transaction_last_ms) > COGGING_TRANSACTION_TIMEOUT_MS)) {
        return 0U;
    }
    for (uint16_t i = 0U; i < sizeof(g_received); ++i) {
        if (g_received[i] != 0xFFU) {
            return 0U;
        }
    }
    uint32_t crc = motor_cogging_crc32(g_staging_table, MOTOR_COGGING_TABLE_SIZE * sizeof(int16_t));
    if (crc != g_expected_crc) {
        return 0U;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int16_t *old_active = g_active_table;
    g_active_table = g_staging_table;
    g_staging_table = old_active;
    g_active_crc = crc;
    ++g_table_revision;
    if (g_table_revision == 0U) {
        ++g_table_revision;
    }
    g_active_transaction = 0U;
    g_persisted = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

// 按偏移复制活动表的固定容量片段；输出计数乘缩放系数后为 A。
uint8_t motor_cogging_table_read_chunk(uint16_t offset, uint8_t count, int16_t *values)
{
    if ((values == NULL) || (count == 0U) || (count > MOTOR_COGGING_CHUNK_MAX_POINTS) ||
        ((uint32_t)offset + count > MOTOR_COGGING_TABLE_SIZE)) {
        return 0U;
    }
    memcpy(values, &g_active_table[offset], (uint32_t)count * sizeof(int16_t));
    return 1U;
}

// 擦写专属 Flash 页并回读校验；调用方负责停机条件与中断保护。
uint8_t motor_cogging_save_to_flash(void)
{
    motor_cogging_flash_record_t record = {0};
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    uint32_t address = (uint32_t)(uintptr_t)__cogging_params_flash_start__;
    HAL_StatusTypeDef result;
    record.magic = COGGING_FLASH_MAGIC;
    record.version = COGGING_FLASH_VERSION;
    record.record_size = sizeof(record);
    record.enabled = g_config.enabled;
    record.gain = g_config.gain;
    record.max_iq_a = g_config.max_iq_a;
    record.fade_start_rpm = g_config.fade_start_rpm;
    record.fade_end_rpm = g_config.fade_end_rpm;
    record.phase_offset_counts = g_config.phase_offset_counts;
    record.table_count = MOTOR_COGGING_TABLE_SIZE;
    record.table_crc = g_active_crc;
    record.table_revision = g_table_revision;
    memcpy(record.table, g_active_table, sizeof(record.table));
    record.builtin_table_crc = MOTOR_COGGING_BUILTIN_CRC32;
    record.record_crc = motor_cogging_crc32(&record, offsetof(motor_cogging_flash_record_t, record_crc));

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0U;
    }
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages = 1U;
    result = HAL_FLASHEx_Erase(&erase, &page_error);
    if (result == HAL_OK) {
        for (uint32_t offset = 0U; offset < sizeof(record); offset += 8U) {
            uint64_t double_word;
            memcpy(&double_word, ((const uint8_t *)&record) + offset, 8U);
            result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + offset, double_word);
            if (result != HAL_OK) {
                break;
            }
        }
    }
    HAL_FLASH_Lock();
    if ((result != HAL_OK) || (memcmp((const void *)(uintptr_t)address, &record, sizeof(record)) != 0)) {
        g_persisted = 0U;
        return 0U;
    }
    g_persisted = 1U;
    return 1U;
}

// 复位运行补偿增益渐入历史，避免沿用上次运行的渐入位置。
void motor_cogging_runtime_start(void)
{
    g_ramp_elapsed_ms = 0U;
    g_effective_gain = 0.0f;
}

// 停机时清零有效补偿增益及渐入计时。
void motor_cogging_runtime_stop(void)
{
    g_ramp_elapsed_ms = 0U;
    g_effective_gain = 0.0f;
}

// 1 ms 更新运行补偿渐入增益；模式不适用或配置关闭时复位渐入历史。
void motor_cogging_task1ms(uint8_t applicable_mode_running)
{
    if ((applicable_mode_running == 0U) || (g_config.enabled == 0U)) {
        motor_cogging_runtime_stop();
        return;
    }
    if (g_ramp_elapsed_ms < MOTOR_COGGING_RAMP_TIME_MS) {
        ++g_ramp_elapsed_ms;
    }
    g_effective_gain = g_config.gain * ((float)g_ramp_elapsed_ms / (float)MOTOR_COGGING_RAMP_TIME_MS);
    if (g_effective_gain > g_config.gain) {
        g_effective_gain = g_config.gain;
    }
}

// 按机械角计数插值表值，叠加速度衰减与限幅后输出电磁方向 Iq（A）。
float motor_cogging_compensation(uint16_t mechanical_angle_counts, float mechanical_speed_rpm)
{
    if ((g_config.enabled == 0U) || (g_effective_gain <= 0.0f) || !isfinite(mechanical_speed_rpm)) {
        return 0.0f;
    }
    float speed = fabsf(mechanical_speed_rpm);
    float fade = 1.0f;
    if (speed >= g_config.fade_end_rpm) {
        return 0.0f;
    }
    if (speed > g_config.fade_start_rpm) {
        fade = (g_config.fade_end_rpm - speed) / (g_config.fade_end_rpm - g_config.fade_start_rpm);
    }
    uint16_t shifted =
        (uint16_t)((mechanical_angle_counts + g_config.phase_offset_counts) & MOTOR_ENCODER_COUNT_MASK_U16);
    uint16_t index = shifted >> 6U;
    uint16_t next = (uint16_t)((index + 1U) & (MOTOR_COGGING_TABLE_SIZE - 1U));
    float fraction = (float)(shifted & 0x003FU) * (1.0f / 64.0f);
    float sample =
        (float)g_active_table[index] + ((float)g_active_table[next] - (float)g_active_table[index]) * fraction;
    float iq = sample * MOTOR_COGGING_TABLE_SCALE_A * g_effective_gain * fade;
    if (iq > g_config.max_iq_a) {
        return g_config.max_iq_a;
    }
    if (iq < -g_config.max_iq_a) {
        return -g_config.max_iq_a;
    }
    return iq;
}

// 读取已提交活动表 CRC；不触发重新计算。
uint32_t motor_cogging_get_active_table_crc(void)
{
    return g_active_crc;
}

// 读取活动表修订号，供状态上报和版本一致性检查。
uint32_t motor_cogging_get_table_revision(void)
{
    return g_table_revision;
}

// 读取已应用渐入后的补偿增益；不等同于配置增益。
float motor_cogging_get_effective_gain(void)
{
    return g_effective_gain;
}

// 查询当前表及配置是否可在重启后恢复，包含固件内置默认值与参数页保存值。
uint8_t motor_cogging_is_persisted(void)
{
    return g_persisted;
}

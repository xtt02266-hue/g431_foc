#include "motor_cogging.h"
#include "motor_config.h"
#include "motor_encoder.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define COGGING_FLASH_MAGIC       0x434F4747UL
#define COGGING_FLASH_VERSION     2U
#define COGGING_RECEIVED_BYTES    (MOTOR_COGGING_TABLE_SIZE / 8U)
#define COGGING_TRANSACTION_TIMEOUT_MS 2000U

typedef struct
{
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
    uint32_t reserved1;
    uint32_t record_crc;
} MotorCoggingFlashRecord;

_Static_assert((sizeof(MotorCoggingFlashRecord) % 8U) == 0U,
               "Cogging flash record must be double-word aligned");
_Static_assert(sizeof(MotorCoggingFlashRecord) <= FLASH_PAGE_SIZE,
               "Cogging flash record exceeds one flash page");

extern uint8_t __cogging_params_flash_start__[];

static MotorCoggingConfig g_config;
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

uint32_t Motor_Cogging_Crc32(const void *data, uint32_t length)
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

uint8_t Motor_Cogging_IsConfigValid(const MotorCoggingConfig *config)
{
    if ((config == NULL) || (config->enabled > 1U) ||
        !isfinite(config->gain) || !isfinite(config->max_iq_a) ||
        !isfinite(config->fade_start_rpm) ||
        !isfinite(config->fade_end_rpm)) return 0U;
    if ((config->gain < 0.0f) || (config->gain > 10.0f) ||
        (config->max_iq_a < 0.0f) ||
        (config->max_iq_a > MOTOR_TORQUE_CURRENT_LIMIT_A) ||
        (config->fade_start_rpm < 0.0f) ||
        (config->fade_end_rpm <= config->fade_start_rpm) ||
        (config->fade_end_rpm > MOTOR_HOST_SPEED_MAX_RPM) ||
        (config->phase_offset_counts > MOTOR_ENCODER_COUNT_MASK_U16) ||
        ((config->enabled != 0U) && (config->max_iq_a <= 0.0f))) return 0U;
    return 1U;
}

static void Motor_Cogging_SetDefaults(void)
{
    g_config.enabled = 0U;
    g_config.gain = 0.3f;
    g_config.max_iq_a = 0.03f;
    g_config.fade_start_rpm = 30.0f;
    g_config.fade_end_rpm = 100.0f;
    g_config.phase_offset_counts = 0U;
    memset(g_table_a, 0, sizeof(g_table_a));
    memset(g_table_b, 0, sizeof(g_table_b));
    g_active_table = g_table_a;
    g_staging_table = g_table_b;
    g_active_crc = Motor_Cogging_Crc32(g_active_table, sizeof(g_table_a));
    g_table_revision = 0U;
    g_persisted = 0U;
}

static uint8_t Motor_Cogging_LoadFromFlash(void)
{
    MotorCoggingFlashRecord record;
    const void *address = (const void *)(uintptr_t)__cogging_params_flash_start__;
    memcpy(&record, address, sizeof(record));
    if ((record.magic != COGGING_FLASH_MAGIC) ||
        (record.version != COGGING_FLASH_VERSION) ||
        (record.record_size != sizeof(record)) ||
        (record.table_count != MOTOR_COGGING_TABLE_SIZE) ||
        (record.record_crc != Motor_Cogging_Crc32(
            &record, offsetof(MotorCoggingFlashRecord, record_crc)))) return 0U;

    MotorCoggingConfig config = {
        record.enabled, record.gain, record.max_iq_a,
        record.fade_start_rpm, record.fade_end_rpm,
        record.phase_offset_counts
    };
    if ((Motor_Cogging_IsConfigValid(&config) == 0U) ||
        (record.table_crc != Motor_Cogging_Crc32(
            record.table, sizeof(record.table)))) return 0U;

    g_config = config;
    memcpy(g_table_a, record.table, sizeof(g_table_a));
    g_active_table = g_table_a;
    g_staging_table = g_table_b;
    g_active_crc = record.table_crc;
    g_table_revision = record.table_revision;
    g_persisted = 1U;
    return 1U;
}

void Motor_Cogging_Init(void)
{
    Motor_Cogging_SetDefaults();
    g_transaction_counter = 0U;
    g_active_transaction = 0U;
    g_ramp_elapsed_ms = 0U;
    g_effective_gain = 0.0f;
    (void)Motor_Cogging_LoadFromFlash();
}

void Motor_Cogging_SetConfig(const MotorCoggingConfig *config)
{
    if (Motor_Cogging_IsConfigValid(config) != 0U) {
        g_config = *config;
        g_persisted = 0U;
        Motor_Cogging_RuntimeStop();
    }
}

void Motor_Cogging_GetConfig(MotorCoggingConfig *config)
{
    if (config != NULL) *config = g_config;
}

uint8_t Motor_Cogging_TableBegin(uint16_t count, uint32_t expected_crc,
                                 uint16_t *transaction_id)
{
    if ((count != MOTOR_COGGING_TABLE_SIZE) || (transaction_id == NULL)) return 0U;
    ++g_transaction_counter;
    if (g_transaction_counter == 0U) ++g_transaction_counter;
    g_active_transaction = g_transaction_counter;
    g_transaction_last_ms = HAL_GetTick();
    g_expected_crc = expected_crc;
    memset(g_received, 0, sizeof(g_received));
    *transaction_id = g_active_transaction;
    return 1U;
}

uint8_t Motor_Cogging_TableWriteChunk(uint16_t transaction_id,
                                      uint16_t offset, uint8_t count,
                                      const int16_t *values)
{
    if ((transaction_id == 0U) ||
        (transaction_id != g_active_transaction) || (values == NULL) ||
        ((uint32_t)(HAL_GetTick() - g_transaction_last_ms) >
         COGGING_TRANSACTION_TIMEOUT_MS) ||
        (count == 0U) || (count > MOTOR_COGGING_CHUNK_MAX_POINTS) ||
        ((uint32_t)offset + count > MOTOR_COGGING_TABLE_SIZE)) return 0U;
    for (uint16_t i = 0U; i < count; ++i) {
        if ((values[i] > MOTOR_COGGING_TABLE_MAX_COUNTS) ||
            (values[i] < -MOTOR_COGGING_TABLE_MAX_COUNTS)) return 0U;
    }
    g_transaction_last_ms = HAL_GetTick();
    memcpy(&g_staging_table[offset], values, (uint32_t)count * sizeof(int16_t));
    for (uint16_t i = 0U; i < count; ++i) {
        uint16_t index = (uint16_t)(offset + i);
        g_received[index >> 3U] |= (uint8_t)(1U << (index & 7U));
    }
    return 1U;
}

uint8_t Motor_Cogging_TableCommit(uint16_t transaction_id)
{
    if ((transaction_id == 0U) ||
        (transaction_id != g_active_transaction) ||
        ((uint32_t)(HAL_GetTick() - g_transaction_last_ms) >
         COGGING_TRANSACTION_TIMEOUT_MS)) return 0U;
    for (uint16_t i = 0U; i < sizeof(g_received); ++i) {
        if (g_received[i] != 0xFFU) return 0U;
    }
    uint32_t crc = Motor_Cogging_Crc32(
        g_staging_table, MOTOR_COGGING_TABLE_SIZE * sizeof(int16_t));
    if (crc != g_expected_crc) return 0U;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int16_t *old_active = g_active_table;
    g_active_table = g_staging_table;
    g_staging_table = old_active;
    g_active_crc = crc;
    ++g_table_revision;
    if (g_table_revision == 0U) ++g_table_revision;
    g_active_transaction = 0U;
    g_persisted = 0U;
    if (primask == 0U) __enable_irq();
    return 1U;
}

uint8_t Motor_Cogging_TableReadChunk(uint16_t offset, uint8_t count,
                                     int16_t *values)
{
    if ((values == NULL) || (count == 0U) ||
        (count > MOTOR_COGGING_CHUNK_MAX_POINTS) ||
        ((uint32_t)offset + count > MOTOR_COGGING_TABLE_SIZE)) return 0U;
    memcpy(values, &g_active_table[offset], (uint32_t)count * sizeof(int16_t));
    return 1U;
}

uint8_t Motor_Cogging_SaveToFlash(void)
{
    MotorCoggingFlashRecord record = {0};
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
    record.record_crc = Motor_Cogging_Crc32(
        &record, offsetof(MotorCoggingFlashRecord, record_crc));

    if (HAL_FLASH_Unlock() != HAL_OK) return 0U;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages = 1U;
    result = HAL_FLASHEx_Erase(&erase, &page_error);
    if (result == HAL_OK) {
        for (uint32_t offset = 0U; offset < sizeof(record); offset += 8U) {
            uint64_t double_word;
            memcpy(&double_word, ((const uint8_t *)&record) + offset, 8U);
            result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                       address + offset, double_word);
            if (result != HAL_OK) break;
        }
    }
    HAL_FLASH_Lock();
    if ((result != HAL_OK) ||
        (memcmp((const void *)(uintptr_t)address, &record, sizeof(record)) != 0)) {
        g_persisted = 0U;
        return 0U;
    }
    g_persisted = 1U;
    return 1U;
}

void Motor_Cogging_RuntimeStart(void)
{
    g_ramp_elapsed_ms = 0U;
    g_effective_gain = 0.0f;
}

void Motor_Cogging_RuntimeStop(void)
{
    g_ramp_elapsed_ms = 0U;
    g_effective_gain = 0.0f;
}

void Motor_Cogging_Task1ms(uint8_t applicable_mode_running)
{
    if ((applicable_mode_running == 0U) || (g_config.enabled == 0U)) {
        Motor_Cogging_RuntimeStop();
        return;
    }
    if (g_ramp_elapsed_ms < MOTOR_COGGING_RAMP_TIME_MS) ++g_ramp_elapsed_ms;
    g_effective_gain = g_config.gain *
        ((float)g_ramp_elapsed_ms / (float)MOTOR_COGGING_RAMP_TIME_MS);
    if (g_effective_gain > g_config.gain) g_effective_gain = g_config.gain;
}

float Motor_Cogging_Compensation(uint16_t mechanical_angle_counts,
                                 float mechanical_speed_rpm)
{
    if ((g_config.enabled == 0U) || (g_effective_gain <= 0.0f) ||
        !isfinite(mechanical_speed_rpm)) return 0.0f;
    float speed = fabsf(mechanical_speed_rpm);
    float fade = 1.0f;
    if (speed >= g_config.fade_end_rpm) return 0.0f;
    if (speed > g_config.fade_start_rpm) {
        fade = (g_config.fade_end_rpm - speed) /
               (g_config.fade_end_rpm - g_config.fade_start_rpm);
    }
    uint16_t shifted = (uint16_t)((mechanical_angle_counts +
                                   g_config.phase_offset_counts) &
                                  MOTOR_ENCODER_COUNT_MASK_U16);
    uint16_t index = shifted >> 6U;
    uint16_t next = (uint16_t)((index + 1U) &
                               (MOTOR_COGGING_TABLE_SIZE - 1U));
    float fraction = (float)(shifted & 0x003FU) * (1.0f / 64.0f);
    float sample = (float)g_active_table[index] +
                   ((float)g_active_table[next] - (float)g_active_table[index]) *
                       fraction;
    float iq = sample * MOTOR_COGGING_TABLE_SCALE_A * g_effective_gain * fade;
    if (iq > g_config.max_iq_a) return g_config.max_iq_a;
    if (iq < -g_config.max_iq_a) return -g_config.max_iq_a;
    return iq;
}

uint32_t Motor_Cogging_GetActiveTableCrc(void) { return g_active_crc; }
uint32_t Motor_Cogging_GetTableRevision(void) { return g_table_revision; }
float Motor_Cogging_GetEffectiveGain(void) { return g_effective_gain; }
uint8_t Motor_Cogging_IsPersisted(void) { return g_persisted; }

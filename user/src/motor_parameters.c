#include "motor_parameters.h"
#include "as5600.h"
#include "motor_current_loop.h"
#include "motor_identify.h"
#include "motor_music.h"
#include "motor_sensorless.h"
#include "svpwm.h"
#include "stm32g4xx_hal.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define MOTOR_PARAMETERS_FLASH_MAGIC       0x4D504152UL
#define MOTOR_PARAMETERS_FLASH_VERSION     1U
#define MOTOR_PARAMETERS_MAX_RESISTANCE    100.0f
#define MOTOR_PARAMETERS_MAX_INDUCTANCE    1.0f
#define MOTOR_PARAMETERS_TWO_PI            6.2831854f
#define MOTOR_PARAMETERS_MAX_POLE_PAIRS    64U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    float resistance;
    float inductance;
    float zero_angle_offset;
    uint32_t pole_pairs;
    int32_t uvw_dir;
    uint32_t crc32;
} MotorParametersFlashRecord;

_Static_assert((sizeof(MotorParametersFlashRecord) % 8U) == 0U,
               "Flash record must be double-word aligned");

extern uint8_t __motor_params_flash_start__;

static volatile MotorParametersStatus g_parameters_status = MOTOR_PARAMETERS_NO_DATA;
static volatile uint8_t g_has_stored_data = 0U;
static uint8_t g_auto_identify_pending = 0U;

static uint32_t Motor_Parameters_Crc32(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;

    for (size_t index = 0U; index < length; index++) {
        crc ^= bytes[index];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }

    return ~crc;
}

static uint8_t Motor_Parameters_AreValuesValid(const MotorIdentifiedParams *params)
{
    if (params == NULL) {
        return 0U;
    }

    if (!((params->resistance > 0.0f) &&
          (params->resistance <= MOTOR_PARAMETERS_MAX_RESISTANCE))) {
        return 0U;
    }

    if (!((params->inductance > 0.0f) &&
          (params->inductance <= MOTOR_PARAMETERS_MAX_INDUCTANCE))) {
        return 0U;
    }

    if ((params->pole_pairs == 0U) ||
        (params->pole_pairs > MOTOR_PARAMETERS_MAX_POLE_PAIRS)) {
        return 0U;
    }

    if (!((params->zero_angle_offset >= 0.0f) &&
          (params->zero_angle_offset < MOTOR_PARAMETERS_TWO_PI))) {
        return 0U;
    }

    return ((params->uvw_dir == 1) || (params->uvw_dir == -1)) ? 1U : 0U;
}

static uint8_t Motor_Parameters_DecodeRecord(
    const MotorParametersFlashRecord *record,
    MotorIdentifiedParams *params)
{
    uint32_t expected_crc;

    if ((record->magic != MOTOR_PARAMETERS_FLASH_MAGIC) ||
        (record->version != MOTOR_PARAMETERS_FLASH_VERSION) ||
        (record->record_size != sizeof(MotorParametersFlashRecord))) {
        return 0U;
    }

    expected_crc = Motor_Parameters_Crc32(
        record, offsetof(MotorParametersFlashRecord, crc32));
    if (record->crc32 != expected_crc) {
        return 0U;
    }

    if ((record->pole_pairs == 0U) ||
        (record->pole_pairs > MOTOR_PARAMETERS_MAX_POLE_PAIRS) ||
        ((record->uvw_dir != 1) && (record->uvw_dir != -1))) {
        return 0U;
    }

    params->resistance = record->resistance;
    params->inductance = record->inductance;
    params->zero_angle_offset = record->zero_angle_offset;
    params->pole_pairs = (uint16_t)record->pole_pairs;
    params->uvw_dir = (int8_t)record->uvw_dir;

    return Motor_Parameters_AreValuesValid(params);
}

static uint8_t Motor_Parameters_Load(MotorIdentifiedParams *params)
{
    MotorParametersFlashRecord record;
    const void *flash_address = (const void *)(uintptr_t)&__motor_params_flash_start__;

    memcpy(&record, flash_address, sizeof(record));
    return Motor_Parameters_DecodeRecord(&record, params);
}

static uint8_t Motor_Parameters_Save(const MotorIdentifiedParams *params)
{
    MotorParametersFlashRecord record = {0};
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    uint32_t flash_address = (uint32_t)(uintptr_t)&__motor_params_flash_start__;
    HAL_StatusTypeDef result;

    if (Motor_Parameters_AreValuesValid(params) == 0U) {
        return 0U;
    }

    record.magic = MOTOR_PARAMETERS_FLASH_MAGIC;
    record.version = MOTOR_PARAMETERS_FLASH_VERSION;
    record.record_size = sizeof(MotorParametersFlashRecord);
    record.resistance = params->resistance;
    record.inductance = params->inductance;
    record.zero_angle_offset = params->zero_angle_offset;
    record.pole_pairs = params->pole_pairs;
    record.uvw_dir = params->uvw_dir;
    record.crc32 = Motor_Parameters_Crc32(
        &record, offsetof(MotorParametersFlashRecord, crc32));

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0U;
    }

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (flash_address - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages = 1U;
    result = HAL_FLASHEx_Erase(&erase, &page_error);

    if (result == HAL_OK) {
        for (uint32_t offset = 0U;
             offset < sizeof(MotorParametersFlashRecord);
             offset += 8U) {
            uint64_t double_word;

            memcpy(&double_word, ((const uint8_t *)&record) + offset, sizeof(double_word));
            result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                       flash_address + offset,
                                       double_word);
            if (result != HAL_OK) {
                break;
            }
        }
    }

    HAL_FLASH_Lock();

    if (result != HAL_OK) {
        return 0U;
    }

    return (memcmp((const void *)(uintptr_t)flash_address,
                   &record,
                   sizeof(record)) == 0) ? 1U : 0U;
}

void Motor_Parameters_Init(void)
{
    MotorIdentifiedParams params;

    if (Motor_Parameters_Load(&params) != 0U) {
        Motor_Identify_UseResult(&params);
        g_has_stored_data = 1U;
        g_auto_identify_pending = 0U;
        g_parameters_status = MOTOR_PARAMETERS_READY;
    } else {
        g_has_stored_data = 0U;
        g_auto_identify_pending = 1U;
        g_parameters_status = MOTOR_PARAMETERS_NO_DATA;
    }
}

uint8_t Motor_Parameters_IdentifyAndSave(void)
{
    uint32_t primask;
    MotorIdentifyState identify_state = Motor_Identify_GetState();
    MotorParametersStatus status = g_parameters_status;

    if ((status == MOTOR_PARAMETERS_IDENTIFYING) ||
        (status == MOTOR_PARAMETERS_SAVE_PENDING) ||
        !((identify_state == IDENTIFY_STATE_IDLE) ||
          (identify_state == IDENTIFY_STATE_DONE) ||
          (identify_state == IDENTIFY_STATE_ERROR))) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    Motor_Music_Stop();
    Motor_Sensorless_Enable(0U);
    g_foc_state.target_d = 0.0f;
    g_foc_state.target_q = 0.0f;
    Motor_CurrentLoop_Enable(0U);
    SVPWM_Disable();
    Motor_OpenLoop_Drive(0.0f, 0.0f);

    g_parameters_status = MOTOR_PARAMETERS_IDENTIFYING;
    Motor_Identify_Start();
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

void Motor_Parameters_ControlTask1ms(void)
{
    MotorIdentifyState identify_state;

    if (g_parameters_status != MOTOR_PARAMETERS_IDENTIFYING) {
        return;
    }

    Motor_Identify_Task();
    identify_state = Motor_Identify_GetState();

    if (identify_state == IDENTIFY_STATE_DONE) {
        g_parameters_status = MOTOR_PARAMETERS_SAVE_PENDING;
    } else if (identify_state == IDENTIFY_STATE_ERROR) {
        SVPWM_Disable();
        g_parameters_status = MOTOR_PARAMETERS_ERROR;
    }
}

void Motor_Parameters_BackgroundTask(void)
{
    MotorIdentifiedParams params;

    /*
     * A blank or invalid parameter page triggers one identification attempt
     * per boot. Wait for a valid encoder sample before energizing the motor.
     */
    if ((g_parameters_status == MOTOR_PARAMETERS_NO_DATA) &&
        (g_auto_identify_pending != 0U)) {
        if (AS5600_IsDataFresh(20U) == 0U) {
            return;
        }

        g_auto_identify_pending = 0U;
        (void)Motor_Parameters_IdentifyAndSave();
        return;
    }

    if (g_parameters_status != MOTOR_PARAMETERS_SAVE_PENDING) {
        return;
    }

    params = Motor_Identify_GetResult();
    if (Motor_Parameters_Save(&params) != 0U) {
        g_has_stored_data = 1U;
        g_parameters_status = MOTOR_PARAMETERS_READY;
    } else {
        g_has_stored_data = 0U;
        g_parameters_status = MOTOR_PARAMETERS_ERROR;
    }
}

uint8_t Motor_Parameters_IsReady(void)
{
    return (g_parameters_status == MOTOR_PARAMETERS_READY) ? 1U : 0U;
}

uint8_t Motor_Parameters_IsBusy(void)
{
    MotorParametersStatus status = g_parameters_status;
    return ((status == MOTOR_PARAMETERS_IDENTIFYING) ||
            (status == MOTOR_PARAMETERS_SAVE_PENDING)) ? 1U : 0U;
}

uint8_t Motor_Parameters_HasStoredData(void)
{
    return g_has_stored_data;
}

MotorParametersStatus Motor_Parameters_GetStatus(void)
{
    return g_parameters_status;
}

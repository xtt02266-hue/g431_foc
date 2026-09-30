#include "motor_friction_storage.h"
#include "stm32g4xx_hal.h"
#include <stddef.h>
#include <string.h>

typedef struct {
    uint32_t magic;
    uint16_t version, size;
    uint8_t enabled, reserved[3];
    float coulomb, viscous, smooth, maximum;
    uint32_t crc;
} FrictionRecord;
_Static_assert(sizeof(FrictionRecord)==32U, "friction flash layout");
extern uint8_t __friction_params_flash_start__[];

static uint32_t crc32(const void *data, size_t size) {
    const uint8_t *p=data; uint32_t crc=0xffffffffU;
    for (size_t i=0;i<size;i++) {
        crc^=p[i];
        for (unsigned j=0;j<8;j++) crc=(crc>>1)^((uint32_t)-(int32_t)(crc&1U)&0xedb88320U);
    }
    return ~crc;
}
uint8_t Motor_FrictionStorage_Load(MotorFrictionConfig *config) {
    FrictionRecord r;
    if (!config) return 0;
    memcpy(&r,__friction_params_flash_start__,sizeof(r));
    if (r.magic!=0x46524943U || r.version!=1 || r.size!=sizeof(r) ||
        r.crc!=crc32(&r,offsetof(FrictionRecord,crc))) return 0;
    MotorFrictionConfig value={r.enabled,r.coulomb,r.viscous,r.smooth,r.maximum};
    if (!Motor_Feedforward_IsFrictionConfigValid(&value)) return 0;
    *config=value;return 1;
}
uint8_t Motor_FrictionStorage_Save(const MotorFrictionConfig *config) {
    if (!Motor_Feedforward_IsFrictionConfigValid(config)) return 0;
    FrictionRecord r={0};
    r.magic=0x46524943U;r.version=1;r.size=sizeof(r);r.enabled=config->enabled;
    r.coulomb=config->coulomb_iq_a;r.viscous=config->viscous_iq_a_per_rpm;
    r.smooth=config->smooth_speed_rpm;r.maximum=config->max_iq_a;
    r.crc=crc32(&r,offsetof(FrictionRecord,crc));
    uintptr_t address=(uintptr_t)__friction_params_flash_start__;
    FLASH_EraseInitTypeDef erase={0};uint32_t error=0;
    erase.TypeErase=FLASH_TYPEERASE_PAGES;erase.Banks=FLASH_BANK_1;
    erase.Page=((uint32_t)address-FLASH_BASE)/FLASH_PAGE_SIZE;erase.NbPages=1;
    if (HAL_FLASH_Unlock()!=HAL_OK) return 0;
    HAL_StatusTypeDef result=HAL_FLASHEx_Erase(&erase,&error);
    if (result==HAL_OK) {
        for (uint32_t offset=0;offset<sizeof(r);offset+=8) {
            uint64_t value;memcpy(&value,(const uint8_t *)&r+offset,8);
            result=HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,(uint32_t)address+offset,value);
            if (result!=HAL_OK) break;
        }
    }
    HAL_FLASH_Lock();
    return result==HAL_OK && !memcmp((const void *)address,&r,sizeof(r));
}

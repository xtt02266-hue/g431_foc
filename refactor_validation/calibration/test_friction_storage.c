#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_friction_storage.h"
#include "stm32g4xx_hal.h"

uint8_t __friction_params_flash_start__[2048];
static unsigned erase_fail,program_fail,unlock_fail,locked;
uint8_t Motor_Feedforward_IsFrictionConfigValid(const MotorFrictionConfig *c) {
    return c && c->enabled<=1 && isfinite(c->coulomb_iq_a) &&
        c->coulomb_iq_a>=0 && c->coulomb_iq_a<=.5f &&
        isfinite(c->viscous_iq_a_per_rpm) && c->viscous_iq_a_per_rpm>=0 && c->viscous_iq_a_per_rpm<=.01f &&
        isfinite(c->smooth_speed_rpm) && c->smooth_speed_rpm>=1 && c->smooth_speed_rpm<=200 &&
        isfinite(c->max_iq_a) && c->max_iq_a>=0 && c->max_iq_a<=1.5f;
}
HAL_StatusTypeDef HAL_FLASH_Unlock(void) { locked=0;return unlock_fail?HAL_ERROR:HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { locked=1;return HAL_OK; }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,uint32_t *error) {
    (void)error;assert(e->NbPages==1);
    if (erase_fail) return HAL_ERROR;
    memset(__friction_params_flash_start__,255,2048);return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type,uint32_t address,uint64_t value) {
    (void)type;if (program_fail) return HAL_ERROR;
    uint32_t offset=address-(uint32_t)(uintptr_t)__friction_params_flash_start__;
    assert(offset+8<=32);memcpy(__friction_params_flash_start__+offset,&value,8);return HAL_OK;
}
int main(void) {
    MotorFrictionConfig expected={1,.04f,.0001f,20,.06f},loaded={0};
    memset(__friction_params_flash_start__,255,2048);
    assert(!Motor_FrictionStorage_Load(&loaded));
    assert(Motor_FrictionStorage_Save(&expected) && locked);
    assert(Motor_FrictionStorage_Load(&loaded));
    assert(loaded.enabled==1 && loaded.coulomb_iq_a==expected.coulomb_iq_a &&
        loaded.viscous_iq_a_per_rpm==expected.viscous_iq_a_per_rpm && loaded.smooth_speed_rpm==20);
    __friction_params_flash_start__[20]^=1;assert(!Motor_FrictionStorage_Load(&loaded));
    assert(Motor_FrictionStorage_Save(&expected));
    MotorFrictionConfig invalid=expected;invalid.coulomb_iq_a=NAN;
    assert(!Motor_FrictionStorage_Save(&invalid));assert(Motor_FrictionStorage_Load(&loaded));
    unlock_fail=1;assert(!Motor_FrictionStorage_Save(&expected));unlock_fail=0;
    erase_fail=1;assert(!Motor_FrictionStorage_Save(&expected) && locked);erase_fail=0;
    program_fail=1;assert(!Motor_FrictionStorage_Save(&expected) && locked);
    assert(!Motor_FrictionStorage_Load(&loaded));
    puts("PASS: friction flash roundtrip, CRC rejection, invalid config, unlock/erase/program failure");
}

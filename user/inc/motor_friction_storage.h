#ifndef MOTOR_FRICTION_STORAGE_H
#define MOTOR_FRICTION_STORAGE_H
#include "motor_feedforward.h"
// 读取摩擦专属 Flash 页，校验固定记录布局、CRC 和配置范围；成功返回 1。
uint8_t motor_friction_storage_load(motor_friction_config_t *config);
// 保存并回读校验摩擦记录；会擦写专属页，调用方负责停机和临界区。
uint8_t motor_friction_storage_save(const motor_friction_config_t *config);
#endif

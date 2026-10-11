#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor_types.h"

/* 系统级功能入口：负责检查参数、传感器和当前运行状态。 */
uint8_t motor_system_start_control(void);

// 在恢复原中断状态的临界区内撤销运行、归零输出并恢复标定前配置。
void motor_system_stop_control(void);

/* 选择既有本地控制模式；仅在模式改变时清零 Iq 和外环历史，不自动启动。
 * 返回 1 表示模式有效并已接受，0 表示数值或当前状态不允许；重复设置相同模式不清零。
 */
uint8_t motor_system_set_control_mode(uint8_t mode);

/* 查询所选模式，供业务逻辑/调试使用；返回值不表示电机已使能或正在输出。 */
uint8_t motor_system_get_control_mode(void);

// 通过统一停机路径取消标定，保持输出撤销与配置恢复次序。
void motor_system_abort_calibration(void);

/* 手动给定仅在 HOST 来源且 TORQUE 正常运行时接受；POT 来源返回 0。
 * 直接给定电磁方向 Iq (A)，Id=0。
 * 有感运行且已选择 TORQUE 时可设置，限幅至 +/- MOTOR_TORQUE_CURRENT_LIMIT_A。
 * 非有限值被拒绝并清零目标。手动模式停机/故障/切换后需重新给定。
 * 电位器模式在恢复运行后自动采用当前旋钮位置。
 * 返回 1 表示接受（超限值会被限幅），0 表示拒绝并清零存储的手动目标。
 */
uint8_t motor_system_set_torque_current(float iq_a);

/* 手动力矩入口，参数单位 N·m：用 Iq=T/Kt 转换后交给电流给定接口检查。
 * Kt 未配置或输入非有限时清零并返回 0；正负沿用 Iq 电磁方向。
 * 返回值及运行条件与 SetTorqueCurrent 相同，POT 来源不接受手动给定。
 */
uint8_t motor_system_set_torque_nm(float torque_nm);

/* 传感器恢复后需主动清故障，防止电机突然自动恢复出力。 */
uint8_t motor_system_clear_fault(void);

/* 主动辨识并保存；必须确认电机无负载且可以自由转动。 */
uint8_t motor_system_identify_and_save(void);

/* 按 motor_songs.h 的曲目 ID 播放有限次数；外部曲目先登记，音乐期间暂停正常控制。 */
uint8_t motor_system_play_song(uint8_t song_id, uint16_t play_count);

// 请求循环播放曲目，沿用音乐期间暂停正常闭环控制的规则。
uint8_t motor_system_play_song_loop(uint8_t song_id);

// 撤销音乐播放请求；系统状态仍由原定时更新路径处理。
void motor_system_stop_music(void);
#endif

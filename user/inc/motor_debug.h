#ifndef MOTOR_DEBUG_H
#define MOTOR_DEBUG_H

/* 返回反向映射并滤波后的电位器计数，仅用于前台调试观测。
 * 读取不关中断、不做 I/O、不调用算法。
 */
float motor_system_get_debug_pot_target(void);

// 将既有电机调试量输出到 OLED；仅在启用该调试模块时调用。
void motor_debug_show_info_oled(void);

#endif

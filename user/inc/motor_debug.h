#ifndef MOTOR_DEBUG_H
#define MOTOR_DEBUG_H

// OLED上显示临时调试信息。
void Motor_ShowDebugInfo_OLED(void);

/* 返回反向映射并滤波后的电位器计数，仅用于前台调试观测。
 * 读取不关中断、不做 I/O、不调用算法。
 */
float Motor_System_GetDebugPotTarget(void);

#endif

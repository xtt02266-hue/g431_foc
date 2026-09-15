#ifndef STAGE6_SUPPORT_H
#define STAGE6_SUPPORT_H

#include <stdint.h>

/* AS5600 读取桩记录：用于证明“仅在到期周期读取”。 */
extern unsigned g_as5600_read_count;
extern unsigned g_as5600_last_read_tick;
extern int      g_as5600_read_this_cycle;
extern uint16_t g_as5600_angle;
extern unsigned g_tick;

void Support_BeginCycle(void);   /* 清本周期读取标记 */

#endif

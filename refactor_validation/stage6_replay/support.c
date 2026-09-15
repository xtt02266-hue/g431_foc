/* Stage 6 回放桩：仅提供 AS5600 角度读取记录与 PID 依赖，不含被测测速算法。 */
#include "support.h"
#include "pid.h"

unsigned g_as5600_read_count = 0;
unsigned g_as5600_last_read_tick = 0;
int      g_as5600_read_this_cycle = 0;
uint16_t g_as5600_angle = 0;
unsigned g_tick = 0;

void Support_BeginCycle(void)
{
    g_as5600_read_this_cycle = 0;
}

uint16_t AS5600_ReadRawAngle(void)
{
    g_as5600_read_count++;
    g_as5600_last_read_tick = g_tick;
    g_as5600_read_this_cycle = 1;
    return g_as5600_angle;
}

uint32_t HAL_GetTick(void) { return 0u; }

/* motor_speed_loop.c 的 PID 依赖（本回放不触发速度环）。 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float out_max, float out_min, float dt)
{
    (void)pid; (void)kp; (void)ki; (void)kd; (void)out_max; (void)out_min; (void)dt;
}
void PID_Calculate(PID_Controller *pid) { (void)pid; }

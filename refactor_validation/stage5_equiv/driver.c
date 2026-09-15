/* Stage 5 行为等价验证驱动：按固定时间线调用真实被测代码
 * HAL_TIM_PeriodElapsedCallback()（内部依次执行参数任务、状态更新、Motor_System_Task、LED），
 * 逐周期打印可观察量与桩调用顺序，供 baseline 与当前实现 A/B 对比。 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "test_support.h"
#include "motor_system.h"
#include "motor_current_loop.h"
#include "motor_speed_loop.h"
#include "motor_position_loop.h"
#include "tim.h"

static int g_cycle = 0;

static unsigned Fbits(float f)
{
    unsigned u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

static void RunCycles(const char *phase, int n)
{
    printf("PHASE %s\n", phase);
    for (int i = 0; i < n; i++) {
        g_test.evlen = 0;
        g_test.event[0] = '\0';
        g_cycle++;
        HAL_TIM_PeriodElapsedCallback(&htim2);
        printf("%03d %d %08X %08X %08X %08X %u %u %u %u %u %u %d %d %s\n",
               g_cycle,
               (int)g_motor_system.state,
               Fbits(g_foc_state.target_q),
               Fbits(g_foc_state.target_d),
               Fbits(g_motor_system.run_data.target_torque_nm),
               Fbits(speed_pid.target),
               g_test.pid_reset_count,
               g_test.pos_reset_count,
               g_test.traj_clear_count,
               g_test.ff_calc_count,
               g_test.pos_run_count,
               g_test.music_task_count,
               g_test.current_loop_enabled,
               g_test.svpwm_enabled,
               (g_test.event[0] != '\0') ? g_test.event : "-");
    }
}

static void BaseReset(void)
{
    Test_ResetAll();
    Motor_System_Init();
}

int main(void)
{
    /* A: 参数未就绪 */
    BaseReset();
    g_test.param_status = MOTOR_PARAMETERS_NO_DATA;
    g_test.as5600_fresh = 1;
    RunCycles("A_param_not_ready", 3);

    /* B: 故障（编码器数据过期） */
    BaseReset();
    g_test.param_status = MOTOR_PARAMETERS_READY;
    g_test.as5600_fresh = 0;
    RunCycles("B_fault", 3);

    /* C: 停止 */
    BaseReset();
    g_test.param_status = MOTOR_PARAMETERS_READY;
    g_test.as5600_fresh = 1;
    Motor_System_StopControl();
    RunCycles("C_stopped", 3);

    /* D: 正常力矩模式 */
    BaseReset();
    g_test.param_status = MOTOR_PARAMETERS_READY;
    g_test.as5600_fresh = 1;
    g_test.music_playing = 0;
    (void)Motor_System_SetControlMode(MOTOR_CONTROL_TORQUE);
    (void)Motor_System_StartControl();
    RunCycles("D_torque", 5);

    /* E: 正常位置模式 */
    (void)Motor_System_SetControlMode(MOTOR_CONTROL_POSITION);
    RunCycles("E_position", 5);

    /* F: 音乐首次进入周期 */
    g_test.music_playing = 1;
    RunCycles("F_music_first", 1);

    /* G: 音乐持续周期 */
    RunCycles("G_music_sustain", 3);

    /* H: 音乐退出后的下一周期 */
    g_test.music_playing = 0;
    RunCycles("H_music_exit_next", 3);

    return 0;
}

/* Stage 6 A/B 回放驱动：对参考实现与迁移后实现喂完全相同的时间线
 * （伪随机但固定的角度、发布速度、fresh/stale 序列），逐周期打印可观察量。
 * 角度读取只由被测实现内部决定，驱动不预读。 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "support.h"
#include "motor_speed_loop.h"
#include "motor_config.h"
#include "as5600.h"

extern float Adaptive_Call(float published_speed_rpm);

static uint16_t recon_ticks = 0;
static float recon_dt_acc = 0.0f;

static unsigned FB(float f)
{
    unsigned u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

static void Fresh(float p, uint16_t angle)
{
    int was_init;
    float ret;
    float dt_recon = 0.0f;
    uint16_t ticks_used = 0;
    float dtacc_used = 0.0f;

    g_tick++;
    Support_BeginCycle();
    g_as5600_angle = angle;

    was_init = speed_est.initialized;
    ret = Adaptive_Call(p);

    if (!was_init) {
        recon_ticks = 0;
        recon_dt_acc = 0.0f;
        dt_recon = MOTOR_SYSTEM_TASK_DT_SEC;
    } else {
        recon_ticks++;
        recon_dt_acc += MOTOR_SYSTEM_TASK_DT_SEC;
        ticks_used = recon_ticks;
        dtacc_used = recon_dt_acc;
        if (g_as5600_read_this_cycle) {
            dt_recon = recon_dt_acc;
            recon_ticks = 0;
            recon_dt_acc = 0.0f;
        }
    }

    printf("%03u FRESH pub=%08X initbefore=%d read=%d rtick=%u angle=%u ret=%08X est=%08X due=%d ticks=%u dtacc=%08X dt=%08X rds=%u\n",
           g_tick, FB(p), was_init, g_as5600_read_this_cycle, g_as5600_last_read_tick,
           angle, FB(ret), FB(speed_est.speed_rpm), g_as5600_read_this_cycle,
           ticks_used, FB(dtacc_used), FB(dt_recon), g_as5600_read_count);
}

static void Stale(void)
{
    g_tick++;
    Support_BeginCycle();
    printf("%03u STALE pub=%08X initbefore=%d read=%d rtick=%u angle=%u ret=%08X est=%08X due=%d ticks=%u dtacc=%08X dt=%08X rds=%u\n",
           g_tick, FB(0.0f), (int)speed_est.initialized, 0, g_as5600_last_read_tick,
           g_as5600_angle, FB(0.0f), FB(speed_est.speed_rpm), 0,
           recon_ticks, FB(recon_dt_acc), FB(0.0f), g_as5600_read_count);
}

static void Reinit(void)
{
    Motor_SpeedEstimator_Init(1.0f);
    printf("---- REINIT alpha=1.0 ----\n");
}

static void ThresholdRun(float p, uint16_t base)
{
    int i;
    for (i = 0; i < 6; i++) {
        Fresh(p, (uint16_t)(base + (uint16_t)(i * 3)));
    }
}

int main(void)
{
    Motor_SpeedEstimator_Init(1.0f);
    printf("== alpha=1.0 ==\n");

    /* 1. initialized=0 的第一次调用 */
    Fresh(0.0f, 4094);

    /* 2. 正向跨零 4094,4095,0,1（高速周期 1，逐周期读取） */
    Fresh(500.0f, 4095);
    Fresh(500.0f, 0);
    Fresh(500.0f, 1);

    /* 3. 反向跨零 */
    Fresh(500.0f, 0);
    Fresh(500.0f, 4095);
    Fresh(500.0f, 4094);

    /* 4. 静止量化抖动 */
    Fresh(500.0f, 100);
    Fresh(500.0f, 100);
    Fresh(500.0f, 101);
    Fresh(500.0f, 100);

    /* 5~7. 阈值 50/200/500 的略低、等于、略高 */
    ThresholdRun(49.9f, 200);
    ThresholdRun(50.0f, 200);
    ThresholdRun(50.1f, 200);
    ThresholdRun(199.9f, 300);
    ThresholdRun(200.0f, 300);
    ThresholdRun(200.1f, 300);
    ThresholdRun(499.9f, 400);
    ThresholdRun(500.0f, 400);
    ThresholdRun(500.1f, 400);

    /* 8. 正速度与负速度 */
    ThresholdRun(300.0f, 1000);
    {
        int i;
        for (i = 0; i < 6; i++) {
            Fresh(-300.0f, (uint16_t)(1000 - i * 7));
        }
    }

    /* 9. 连续多个 stale 周期后恢复 fresh */
    Stale();
    Stale();
    Stale();
    Fresh(100.0f, 500);
    Fresh(100.0f, 505);

    /* 10. 再次 Init 后的行为 */
    Reinit();
    Fresh(0.0f, 200);
    Fresh(0.0f, 205);

    /* 11. 未到采样周期：published=0 但 estimator 内部 speed_rpm != 0 */
    Fresh(500.0f, 300);   /* 到期更新，est 变为非零 */
    Fresh(0.0f, 301);     /* 周期 20，未到期，返回发布的 0，est 保持非零 */
    Fresh(0.0f, 302);
    Fresh(0.0f, 303);

    return 0;
}

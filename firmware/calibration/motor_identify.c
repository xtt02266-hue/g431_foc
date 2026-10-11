#include "motor_identify.h"
#include "motor_encoder.h"
#include "motor_config.h"
#include "motor_current_loop.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>
// #include "motor_current_loop.h" // 后续可能需要调用开环输出电压/电流的接口

static motor_identify_state_t g_identify_state = IDENTIFY_STATE_IDLE;
motor_identified_params_t g_identified_params = {0};

// 状态机等待计时器
static uint32_t g_identify_timer = 0;
static float g_uvw_elec_angle = 0.0f;
static float g_align_sin_sum = 0.0f;
static float g_align_cos_sum = 0.0f;
static uint32_t g_align_sample_count = 0U;

#define MOTOR_TWO_PI                6.2832f // 辨识用一圈电角度 rad，保留原有近似精度
#define MOTOR_HALF_PI               1.5708f // 辨识用四分之一圈 rad，保留原有近似精度
#define MOTOR_OPEN_LOOP_ALIGN_ANGLE 0.0f    // 开环对齐电角度，单位 rad
/*
 * motor_open_loop_drive(theta) 用 sin(theta) 生成相电压。
 * α/β 坐标系下该电压矢量角为 theta-π/2；闭环 Park/逆 Park 则约定 +α 轴为零角。
 */
#define MOTOR_OPEN_LOOP_VECTOR_SHIFT (-MOTOR_HALF_PI)

#define MOTOR_IDENTIFY_L_COOLDOWN_MS   100U   // 电感测量后冷却等待时长，单位 ms
#define MOTOR_IDENTIFY_UVW_LOCK_MS     1000U  // 相序辨识固定矢量锁定时长，单位 ms
#define MOTOR_IDENTIFY_UVW_CYCLES      3.0f   // 相序/极对数辨识的旋转电周期数
#define MOTOR_IDENTIFY_UVW_STEP_RAD    0.025f // 辨识每次更新的电角度增量，单位 rad
#define MOTOR_IDENTIFY_ALIGN_SETTLE_MS 2000U  // 零角对齐稳定等待时长，单位 ms
#define MOTOR_IDENTIFY_ALIGN_SAMPLE_MS 500U   // 零角对齐采样窗口，单位 ms

// 将电角度 rad 环绕至一圈，保留辨识算法原有近似常量。
static float motor_identify_normalize_angle(float angle)
{
    while (angle >= MOTOR_TWO_PI) {
        angle -= MOTOR_TWO_PI;
    }

    while (angle < 0.0f) {
        angle += MOTOR_TWO_PI;
    }

    return angle;
}

// 锁存辨识失败状态并撤销当前开环电压，保留既有失败退出次序。
static void motor_identify_fail(void)
{
    motor_open_loop_drive(0.0f, 0.0f);
    g_identify_timer = 0U;
    g_identify_state = IDENTIFY_STATE_ERROR;
}

// 按电角度 rad 和原始幅值生成开环电压矢量；保留 SVPWM 调用与相位约定。
void motor_open_loop_drive(float elec_angle, float amplitude)
{
    // 幅值限幅 (中心点为500)
    if (amplitude > 500.0f) {
        amplitude = 500.0f;
    }
    if (amplitude < 0.0f) {
        amplitude = 0.0f;
    }

    // 计算三相占空比 (相差120度 => 2.0944弧度)
    // 占空比计算：中心值(500) + 正弦波分量
    float valA = 500.0f + amplitude * sinf(elec_angle);
    float valB = 500.0f + amplitude * sinf(elec_angle - 2.094395f);
    float valC = 500.0f + amplitude * sinf(elec_angle + 2.094395f);

    // 将 0~1000 的设定分辨率映射到实际的定时器 ARR (4250)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(valA * 4.25f));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(valB * 4.25f));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(valC * 4.25f));
}

// ==================== 对外接口函数 ====================

// 启动电机参数辨识流程
// 用途：从主程序或用户按钮调用，开始整个参数校准的全过程
// 特点：只有在状态机处于空闲(IDLE)、已完成(DONE)或出错(ERROR)时才会真正启动，
//       其他状态下调用此函数会被忽略（防止中途被打断）。
void motor_identify_start(void)
{
    if (g_identify_state == IDENTIFY_STATE_IDLE || g_identify_state == IDENTIFY_STATE_DONE ||
        g_identify_state == IDENTIFY_STATE_ERROR) {
        g_identified_params = (motor_identified_params_t){0};
        g_identify_timer = 0U;
        g_uvw_elec_angle = 0.0f;
        g_align_sin_sum = 0.0f;
        g_align_cos_sum = 0.0f;
        g_align_sample_count = 0U;

        // 优先测量电阻，为后续提供安全的对齐电压估算
        g_identify_state = IDENTIFY_STATE_MEASURE_R;
    }
}

// 查询电机参数辨识的当前状态
// 返回值：状态枚举值，可能为：
//   - IDENTIFY_STATE_IDLE        : 空闲（没有辨识或已停止）
//   - IDENTIFY_STATE_MEASURE_R   : 正在测量电阻
//   - IDENTIFY_STATE_MEASURE_L   : 正在测量电感
//   - IDENTIFY_STATE_UVW_AND_POLES : 正在测极对数和相序方向
//   - IDENTIFY_STATE_ALIGN       : 正在对齐零点
//   - IDENTIFY_STATE_DONE        : 辨识完成（此时可以调用 motor_identify_get_result() 取结果）
//   - IDENTIFY_STATE_ERROR       : 辨识失败或异常
// 用途：主程序可以轮询此函数来了解校准进度，在OLED上显示"正在测电阻..."等提示。
uint8_t motor_identify_get_state(void)
{
    return g_identify_state;
}

// 获取辨识出来的电机参数结果
// 返回值：motor_identified_params_t 结构体，包含：
//   - resistance       : 相电阻 (单位：欧姆Ω)，用于计算电机的热损耗和驱动电压限制
//   - inductance       : 相电感 (单位：亨利H)，用于设计电流环的PI控制器参数
//   - pole_pairs       : 极对数(整数，如7、11、14)，用于电角度和机械角度的相互转换
//   - uvw_dir          : UVW相序方向(1=正向接线，-1=反向接线)，影响电机的旋转方向
//   - zero_angle_offset: 磁编码器零点偏置(单位：弧度)，使FOC算法知道磁场零点在哪里
// 注意：只有在 motor_identify_get_state() 返回 IDENTIFY_STATE_DONE 时，
//      这里返回的数据才是有效的、经过完整测试的。
// 用途：辨识完成后调用此函数，把参数保存到Flash或直接应用到FOC闭环控制中。
motor_identified_params_t motor_identify_get_result(void)
{
    return g_identified_params;
}

// 采用已校验的辨识参数，供上电加载 Flash 后恢复电流环参数。
void motor_identify_use_result(float resistance, float inductance, uint16_t pole_pairs, float zero_angle_offset,
                               int8_t uvw_dir)
{
    const motor_identified_params_t params = {resistance, inductance, pole_pairs, zero_angle_offset, uvw_dir};

    g_identified_params = params;
    /* R/L/极对数来自已确认的 GB4310 规格；Flash 只复用安装相关的
     * 相序和零位。参数记录版本会在更换电机时升级，防止读入旧电机标定。
     */
    g_identified_params.resistance = MOTOR_NOMINAL_PHASE_RESISTANCE_OHM;
    g_identified_params.inductance = MOTOR_NOMINAL_PHASE_INDUCTANCE_H;
    g_identified_params.pole_pairs = MOTOR_POLE_PAIRS;
    motor_current_loop_set_motor_identity_params(g_identified_params.pole_pairs, params.zero_angle_offset,
                                                 params.uvw_dir);
    g_identify_timer = 0U;
    g_identify_state = IDENTIFY_STATE_DONE;
}

// 状态机具体实现：云台电机参数辨识
// 由于云台电机内阻大、极对数多，且容易发热，
// 流程先装载规格 R/L，再低速确认相序并对齐零点。
// 1. 装载 GB4310 标称单相电阻。
static void identify_measure_r(void)
{
    motor_open_loop_drive(0.0f, 0.0f);
    g_identified_params.resistance = MOTOR_NOMINAL_PHASE_RESISTANCE_OHM;
    g_identify_timer = 0U;
    g_identify_state = IDENTIFY_STATE_MEASURE_L;
}

// 2. 装载标称相电感（保留原状态名，并非实际测量电感）。
// 目的：给电流环 PI 整定提供已确认的相电感，避免使用旧的 0.6mH 占位值。
static void identify_measure_l(void)
{
    // GB4310 规格为 4.76mH 相间电感，星形绕组换算为 2.38mH 单相电感。

    g_identify_timer++;

    // 先等待 100ms，让电阻测试电流衰减，再进入后续相序/极对数辨识。
    if (g_identify_timer <= MOTOR_IDENTIFY_L_COOLDOWN_MS) {
        motor_open_loop_drive(0.0f, 0.0f);
        return;
    }

    g_identified_params.inductance = MOTOR_NOMINAL_PHASE_INDUCTANCE_H;

    g_identify_timer = 0;
    g_identify_state = IDENTIFY_STATE_UVW_AND_POLES; // 进入测相序极对数
}

// 3. 对齐转子到电角度零点 (获取电角度偏差 Offset)
// 目的：FOC算法需要知道磁场（电角度）的零点对应编码器的哪个物理位置。
static void identify_align(void)
{
    // 调用开环驱动函数，电气角度固定在 0 (0.0f)，幅值使用 200.0f
    // 这相当于用磁力把转子强行“吸附”在位置0，不让它动。
    motor_open_loop_drive(MOTOR_OPEN_LOOP_ALIGN_ANGLE, 200.0f);

    g_identify_timer++;
    if ((g_identify_timer > MOTOR_IDENTIFY_ALIGN_SETTLE_MS) &&
        (g_identify_timer <= (MOTOR_IDENTIFY_ALIGN_SETTLE_MS + MOTOR_IDENTIFY_ALIGN_SAMPLE_MS))) {
        float sample_angle = (float)motor_encoder_get_raw_angle() * MOTOR_ENCODER_RAD_PER_COUNT;
        g_align_sin_sum += sinf(sample_angle);
        g_align_cos_sum += cosf(sample_angle);
        g_align_sample_count++;
    }

    // 先稳定 2 s，再对 500 ms 内的编码器角度做圆周平均。
    if (g_identify_timer > (MOTOR_IDENTIFY_ALIGN_SETTLE_MS + MOTOR_IDENTIFY_ALIGN_SAMPLE_MS)) {
        // 极点吸固后，读取此刻的磁编码器角度作为机械零点
        // 将MT6826S的0~32767原始值换算为弧度(0~2π)。
        float align_mech_angle;

        if (g_align_sample_count == 0U) {
            motor_identify_fail();
            return;
        }

        align_mech_angle = atan2f(g_align_sin_sum, g_align_cos_sum);
        align_mech_angle = motor_identify_normalize_angle(align_mech_angle);

        // 记录完零点后，关闭电机输出电压
        motor_open_loop_drive(0.0f, 0.0f);

        // 关键：将辨识出的真实电机参数同步推送给 FOC 电流环结构！
        // 安全校验：pole_pairs 不能为 0，否则 FOC 角度计算永久失效

        if (g_identified_params.uvw_dir == 0) {
            g_identified_params.uvw_dir = 1;
        }

        float elec_align_angle = MOTOR_OPEN_LOOP_ALIGN_ANGLE + MOTOR_OPEN_LOOP_VECTOR_SHIFT;
        float mech_zero_offset =
            elec_align_angle / ((float)g_identified_params.pole_pairs * (float)g_identified_params.uvw_dir);
        g_identified_params.zero_angle_offset = motor_identify_normalize_angle(align_mech_angle - mech_zero_offset);

        motor_current_loop_set_motor_identity_params(
            g_identified_params.pole_pairs, g_identified_params.zero_angle_offset, g_identified_params.uvw_dir);

        g_identify_timer = 0;
        // 到这一步，所有所需参数都已获取，辨识流程全部结束
        g_identify_state = IDENTIFY_STATE_DONE;
    }
}

// 4. 同时识别 UVW 相序方向与极对数
// 目的1：判断接线方向(相序)。知道给正向电压时，编码器的读数是变大还是变小。
// 目的2：计算极对数(Pole Pairs)。知道转子上有几块磁铁，这是电角度和机械角度相互转换的核心参数。
static float g_accumulated_mech_angle = 0.0f; // 累加的机械角度
static uint16_t g_last_raw_angle = 0;         // 上一次循环的编码器角度

// 推进开环旋转辨识 UVW 方向和极对数，保留锁定、转动和采样判定次序。
static void identify_uvw_and_poles(void)
{
    // 按原较低电角速度转过三个电周期，
    // 在限制机械运动范围的同时累积极对数估计。
    const float target_elec_angle = MOTOR_IDENTIFY_UVW_CYCLES * MOTOR_TWO_PI;
    const uint32_t lock_time = MOTOR_IDENTIFY_UVW_LOCK_MS;

    // ================= 阶段 1：静态预对齐 =================
    if (g_identify_timer < lock_time) {
        // 锁定在电角度 0 的位置，让转子物理对齐
        motor_open_loop_drive(0.0f, 200.0f);

        // 在锁定即将结束的前一刻，清零累加器，并记录此时真正的起始机械角度
        if (g_identify_timer == lock_time - 1U) {
            g_uvw_elec_angle = 0.0f;
            g_accumulated_mech_angle = 0.0f;
            g_last_raw_angle = motor_encoder_get_raw_angle();
        }
        g_identify_timer++;
        return; // 预对齐阶段直接返回
    }

    // ================= 阶段 2：开环拖动与积分 =================
    // 每次循环让电角度略微往前推进 (相当于给定一个固定的开环速度)
    g_uvw_elec_angle += MOTOR_IDENTIFY_UVW_STEP_RAD;
    motor_open_loop_drive(fmodf(g_uvw_elec_angle, 2.0f * 3.1415926f), 200.0f);

    // 1. 获取当前最新角度
    uint16_t current_angle = motor_encoder_get_raw_angle();

    // 2. 计算这 1ms 内发生的微小位移
    int32_t step_delta = (int32_t)current_angle - (int32_t)g_last_raw_angle;

    // 3. 处理15位编码器单步跨零点。
    if (step_delta > (int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        step_delta -= (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    } else if (step_delta < -(int32_t)MOTOR_ENCODER_HALF_REV_U32) {
        step_delta += (int32_t)MOTOR_ENCODER_COUNTS_PER_REV_U32;
    }

    // 4. 将微小位移积分到全局累加器中，并更新历史值
    g_accumulated_mech_angle += (float)step_delta;
    g_last_raw_angle = current_angle;

    g_identify_timer++;

    // ================= 阶段 3：结算数据 =================
    if (g_uvw_elec_angle >= target_elec_angle) {
        // 1. 判断相序方向 (累加的角度是正还是负一目了然)
        g_identified_params.uvw_dir = (g_accumulated_mech_angle >= 0.0f) ? 1 : -1;

        // 2. 计算极对数（累计机械计数/32768=机械圈数）。
        float mech_turns = fabsf(g_accumulated_mech_angle) / MOTOR_ENCODER_COUNTS_PER_REV_F;

        // 电周期数除以机械转数得到极对数。
        if (mech_turns < 0.01f) {
            motor_identify_fail();
            return;
        }

        /* 极对数使用 GB4310 规格的 14；开环运动仅用于确认相序方向。 */
        g_identified_params.pole_pairs = MOTOR_POLE_PAIRS;

        // 停止输出，状态流转
        motor_open_loop_drive(0.0f, 0.0f);
        g_identify_timer = 0;
        g_identify_state = IDENTIFY_STATE_ALIGN;
    }
}

// 辨识状态机轮询任务
void motor_identify_task(void)
{
    switch (g_identify_state) {
    case IDENTIFY_STATE_IDLE:
        // 等待启动指令
        break;

    case IDENTIFY_STATE_MEASURE_R:
        identify_measure_r();
        break;

    case IDENTIFY_STATE_MEASURE_L:
        identify_measure_l();
        break;

    case IDENTIFY_STATE_ALIGN:
        identify_align();
        break;

    case IDENTIFY_STATE_UVW_AND_POLES:
        identify_uvw_and_poles();
        break;

    case IDENTIFY_STATE_DONE:
        // 辨识出结果并保存，等待应用层进入 RUN 状态
        break;

    case IDENTIFY_STATE_ERROR:
        // 若监测到电流超标或未连接电机，在此切断输出停止
        break;
    }
}

#include "motor_identify.h"
#include "as5600.h"
#include "motor_system.h"
#include "motor_current_loop.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>
// #include "motor_current_loop.h" // 后续可能需要调用开环输出电压/电流的接口

static MotorIdentifyState g_identify_state = IDENTIFY_STATE_IDLE;
MotorIdentifiedParams g_identified_params = {0};

// 状态机等待计时器
static uint32_t g_identify_timer = 0;
static float g_uvw_elec_angle = 0.0f;
static float g_r_current_sum = 0.0f;
static uint32_t g_r_current_count = 0U;
static float g_align_sin_sum = 0.0f;
static float g_align_cos_sum = 0.0f;
static uint32_t g_align_sample_count = 0U;

#define MOTOR_TWO_PI                 6.2832f
#define MOTOR_HALF_PI                1.5708f
#define MOTOR_OPEN_LOOP_ALIGN_ANGLE  0.0f
/*
 * Motor_OpenLoop_Drive(theta) generates phase voltages with sin(theta).
 * In alpha-beta coordinates that voltage vector is theta - pi/2, while the
 * closed-loop Park/InvPark convention uses theta = 0 on the +alpha axis.
 */
#define MOTOR_OPEN_LOOP_VECTOR_SHIFT (-MOTOR_HALF_PI)

#define MOTOR_IDENTIFY_R_SETTLE_MS       400U
#define MOTOR_IDENTIFY_R_SAMPLE_MS       600U
#define MOTOR_IDENTIFY_L_COOLDOWN_MS     100U
#define MOTOR_IDENTIFY_UVW_LOCK_MS       1000U
#define MOTOR_IDENTIFY_UVW_CYCLES        3.0f
#define MOTOR_IDENTIFY_UVW_STEP_RAD      0.025f
#define MOTOR_IDENTIFY_ALIGN_SETTLE_MS   2000U
#define MOTOR_IDENTIFY_ALIGN_SAMPLE_MS   500U
#define MOTOR_IDENTIFY_MAX_POLE_PAIRS    64U

static float Motor_Identify_NormalizeAngle(float angle)
{
    while (angle >= MOTOR_TWO_PI)
    {
        angle -= MOTOR_TWO_PI;
    }

    while (angle < 0.0f)
    {
        angle += MOTOR_TWO_PI;
    }

    return angle;
}

static void Motor_Identify_Fail(void)
{
    Motor_OpenLoop_Drive(0.0f, 0.0f);
    g_identify_timer = 0U;
    g_identify_state = IDENTIFY_STATE_ERROR;
}

void Motor_OpenLoop_Drive(float elec_angle, float amplitude)
{
    // 幅值限幅 (中心点为500)
    if (amplitude > 500.0f) amplitude = 500.0f;
    if (amplitude < 0.0f)   amplitude = 0.0f;

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
void Motor_Identify_Start(void)
{
    if (g_identify_state == IDENTIFY_STATE_IDLE ||
        g_identify_state == IDENTIFY_STATE_DONE ||
        g_identify_state == IDENTIFY_STATE_ERROR)
    {
        g_identified_params = (MotorIdentifiedParams){0};
        g_identify_timer = 0U;
        g_uvw_elec_angle = 0.0f;
        g_r_current_sum = 0.0f;
        g_r_current_count = 0U;
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
//   - IDENTIFY_STATE_DONE        : 辨识完成（此时可以调用 Motor_Identify_GetResult() 取结果）
//   - IDENTIFY_STATE_ERROR       : 辨识失败或异常
// 用途：主程序可以轮询此函数来了解校准进度，在OLED上显示"正在测电阻..."等提示。
MotorIdentifyState Motor_Identify_GetState(void)
{
    return g_identify_state;
}

// 获取辨识出来的电机参数结果
// 返回值：MotorIdentifiedParams 结构体，包含：
//   - resistance       : 相电阻 (单位：欧姆Ω)，用于计算电机的热损耗和驱动电压限制
//   - inductance       : 相电感 (单位：亨利H)，用于设计电流环的PI控制器参数
//   - pole_pairs       : 极对数(整数，如7、11、14)，用于电角度和机械角度的相互转换
//   - uvw_dir          : UVW相序方向(1=正向接线，-1=反向接线)，影响电机的旋转方向
//   - zero_angle_offset: 磁编码器零点偏置(单位：弧度)，使FOC算法知道磁场零点在哪里
// 注意：只有在 Motor_Identify_GetState() 返回 IDENTIFY_STATE_DONE 时，
//      这里返回的数据才是有效的、经过完整测试的。
// 用途：辨识完成后调用此函数，把参数保存到Flash或直接应用到FOC闭环控制中。
MotorIdentifiedParams Motor_Identify_GetResult(void)
{
    return g_identified_params;
}

void Motor_Identify_UseResult(const MotorIdentifiedParams *params)
{
    if (params == NULL) {
        return;
    }

    g_identified_params = *params;
    Motor_CurrentLoop_SetMotorIdentityParams(params->pole_pairs,
                                             params->zero_angle_offset,
                                             params->uvw_dir);
    g_identify_timer = 0U;
    g_identify_state = IDENTIFY_STATE_DONE;
}

// =========================================================
// 状态机具体实现：云台电机参数辨识
// 由于云台电机内阻大、电感小、极对数多，且容易发热，
// 必须遵循：测电阻 -> 测电感 -> 测极对数和相序 -> 静止对齐零点 的安全顺序。
// =========================================================

// 1. 测量相电阻 (欧姆定律: R = U/I)
// 目的：算出电机的真实电阻。因为只有知道了电阻，后续强拖时才知道给多大的电压是安全的，避免电机烧毁。
static void Identify_MeasureR(void)
{
    // 云台电机内阻大，这里施加约2.4V的相电压测试 (假设母线是SYSTEM_BUS_VOLTAGE)
    // 幅值 200 对应占空比 200/1000
    const float test_amplitude = 200.0f; 
    const float bus_voltage = SYSTEM_BUS_VOLTAGE; 

    // 电压加载在 Alpha 轴 (也就是 U 相)，角度设为 MOTOR_HALF_PI (90度)
    // 此时 U 相占空比最高，V/W 相占空比相等且较低。
    Motor_OpenLoop_Drive(MOTOR_HALF_PI, test_amplitude);

    g_identify_timer++;
    
    // Wait 400 ms, then average 600 current samples at 1 kHz.
    if ((g_identify_timer > MOTOR_IDENTIFY_R_SETTLE_MS) &&
        (g_identify_timer <= (MOTOR_IDENTIFY_R_SETTLE_MS +
                              MOTOR_IDENTIFY_R_SAMPLE_MS)))
    {
        // 累加稳态下的 U 相电流大小 (绝对值)
        g_r_current_sum += fabsf(g_foc_state.sample.iu_a);
        g_r_current_count++;
    }
    else if (g_identify_timer > (MOTOR_IDENTIFY_R_SETTLE_MS +
                                 MOTOR_IDENTIFY_R_SAMPLE_MS))
    {
        Motor_OpenLoop_Drive(0.0f, 0.0f); // 测试完毕，关闭输出
        
        if (g_r_current_count == 0U) {
            Motor_Identify_Fail();
            return;
        }

        float current_avg = g_r_current_sum / (float)g_r_current_count;
        if (current_avg < 0.01f) {
            Motor_Identify_Fail();
            return;
        }
        
        // 计算 U 相实际施加的相电压 (与中心点的压差)
        float test_voltage = (test_amplitude / 1000.0f) * bus_voltage;
        
        // 根据欧姆定律 R = U / I 计算相电阻
        g_identified_params.resistance = test_voltage / current_avg;

        // 清零静态变量，准备进入下个状态
        g_r_current_sum = 0.0f;
        g_r_current_count = 0U;
        g_identify_timer = 0U;
        
        // 测完电阻后，进入测电感状态
        g_identify_state = IDENTIFY_STATE_MEASURE_L; 
    }
}

// 2. 测量相电感 
// 目的：测出电感(L)大小，主要是为了后面的“电流环”能自动算出 PI 控制器的参数。
static void Identify_MeasureL(void)
{
    // 由于云台电机时间常数极窄（<1ms），很难在 1ms 调度周期内完成斜率抓取。
    // 在工程中对于这种电机，最好的策略是测准电阻后，电感直接给厂家的标称值。
    // 商家给的线间电感为 1.2mH，FOC所需要的相电感 = 1.2 / 2 = 0.6mH = 0.0006H

    g_identify_timer++;

    // Wait 100 ms so the resistance-test current has fully decayed.
    if (g_identify_timer <= MOTOR_IDENTIFY_L_COOLDOWN_MS)
    {
        Motor_OpenLoop_Drive(0.0f, 0.0f);
        return;
    }

    // 强行赋理论相电感值 0.6mH
    g_identified_params.inductance = 0.0006f;

    g_identify_timer = 0;
    g_identify_state = IDENTIFY_STATE_UVW_AND_POLES; // 进入测相序极对数
}

// 3. 对齐转子到电角度零点 (获取电角度偏差 Offset)
// 目的：FOC算法需要知道磁场（电角度）的零点对应编码器的哪个物理位置。
static void Identify_Align(void)
{
    // 调用开环驱动函数，电气角度固定在 0 (0.0f)，幅值使用 200.0f
    // 这相当于用磁力把转子强行“吸附”在位置0，不让它动。
    Motor_OpenLoop_Drive(MOTOR_OPEN_LOOP_ALIGN_ANGLE, 200.0f);
    
    g_identify_timer++;
    if ((g_identify_timer > MOTOR_IDENTIFY_ALIGN_SETTLE_MS) &&
        (g_identify_timer <= (MOTOR_IDENTIFY_ALIGN_SETTLE_MS +
                              MOTOR_IDENTIFY_ALIGN_SAMPLE_MS))) {
        float sample_angle = (float)AS5600_ReadRawAngle() *
                             (MOTOR_TWO_PI / 4096.0f);
        g_align_sin_sum += sinf(sample_angle);
        g_align_cos_sum += cosf(sample_angle);
        g_align_sample_count++;
    }

    // Settle for 2 s, then circular-average the encoder for 500 ms.
    if (g_identify_timer > (MOTOR_IDENTIFY_ALIGN_SETTLE_MS +
                            MOTOR_IDENTIFY_ALIGN_SAMPLE_MS))
    {
        // 极点吸固后，读取此刻的磁编码器角度作为机械零点
        // AS5600 读出的原始值是 0~4095，这里将其换算成国际标准单位：弧度 (0 ~ 2π)
        float align_mech_angle;

        if (g_align_sample_count == 0U) {
            Motor_Identify_Fail();
            return;
        }

        align_mech_angle = atan2f(g_align_sin_sum, g_align_cos_sum);
        align_mech_angle = Motor_Identify_NormalizeAngle(align_mech_angle);
        
        // 记录完零点后，关闭电机输出电压
        Motor_OpenLoop_Drive(0.0f, 0.0f);
        
        // 关键：将辨识出的真实电机参数同步推送给 FOC 电流环结构！
        // 安全校验：pole_pairs 不能为 0，否则 FOC 角度计算永久失效

        if (g_identified_params.uvw_dir == 0)
        {
            g_identified_params.uvw_dir = 1;
        }

        float elec_align_angle = MOTOR_OPEN_LOOP_ALIGN_ANGLE + MOTOR_OPEN_LOOP_VECTOR_SHIFT;
        float mech_zero_offset = elec_align_angle /
                                 ((float)g_identified_params.pole_pairs *
                                  (float)g_identified_params.uvw_dir);
        g_identified_params.zero_angle_offset = 
            Motor_Identify_NormalizeAngle(align_mech_angle - mech_zero_offset);

        Motor_CurrentLoop_SetMotorIdentityParams(
            g_identified_params.pole_pairs,
            g_identified_params.zero_angle_offset,
            g_identified_params.uvw_dir
        );
        
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

static void Identify_UvwAndPoles(void)
{
    // Traverse three electrical cycles at a slower rate for a more stable
    // pole-pair estimate while keeping mechanical motion limited.
    const float target_elec_angle = MOTOR_IDENTIFY_UVW_CYCLES * MOTOR_TWO_PI;
    const uint32_t lock_time = MOTOR_IDENTIFY_UVW_LOCK_MS;
    
    // ================= 阶段 1：静态预对齐 =================
    if (g_identify_timer < lock_time)
    {
        // 锁定在电角度 0 的位置，让转子物理对齐
        Motor_OpenLoop_Drive(0.0f, 200.0f);
        
        // 在锁定即将结束的前一刻，清零累加器，并记录此时真正的起始机械角度
        if (g_identify_timer == lock_time - 1U)
        {
            g_uvw_elec_angle = 0.0f;
            g_accumulated_mech_angle = 0.0f;
            g_last_raw_angle = AS5600_ReadRawAngle();
        }
        g_identify_timer++;
        return; // 预对齐阶段直接返回
    }

    // ================= 阶段 2：开环拖动与积分 =================
    // 每次循环让电角度略微往前推进 (相当于给定一个固定的开环速度)
    g_uvw_elec_angle += MOTOR_IDENTIFY_UVW_STEP_RAD;
    Motor_OpenLoop_Drive(fmodf(g_uvw_elec_angle, 2.0f * 3.1415926f), 200.0f);

    // 1. 获取当前最新角度
    uint16_t current_angle = AS5600_ReadRawAngle();
    
    // 2. 计算这 1ms 内发生的微小位移
    int32_t step_delta = (int32_t)current_angle - (int32_t)g_last_raw_angle;
    
    // 3. 处理单步的跨零点 (因为是 1ms 的微小位移，绝不可能超过 2048，此处逻辑变得100%安全)
    if (step_delta > 2048) step_delta -= 4096;
    else if (step_delta < -2048) step_delta += 4096;
    
    // 4. 将微小位移积分到全局累加器中，并更新历史值
    g_accumulated_mech_angle += (float)step_delta;
    g_last_raw_angle = current_angle;

    g_identify_timer++;
    
    // ================= 阶段 3：结算数据 =================
    if (g_uvw_elec_angle >= target_elec_angle)
    {
        // 1. 判断相序方向 (累加的角度是正还是负一目了然)
        g_identified_params.uvw_dir = (g_accumulated_mech_angle >= 0.0f) ? 1 : -1;
        
        // 2. 计算极对数 (累积的机械角度 / 4096 = 机械圈数)
        float mech_turns = fabsf(g_accumulated_mech_angle) / 4096.0f; 
        
        // Electrical cycles divided by mechanical turns gives pole pairs.
        if (mech_turns < 0.01f) {
            Motor_Identify_Fail();
            return;
        }

        g_identified_params.pole_pairs =
            (uint16_t)roundf(MOTOR_IDENTIFY_UVW_CYCLES / mech_turns);

        if ((g_identified_params.pole_pairs == 0U) ||
            (g_identified_params.pole_pairs > MOTOR_IDENTIFY_MAX_POLE_PAIRS)) {
            Motor_Identify_Fail();
            return;
        }
   

        // 停止输出，状态流转
        Motor_OpenLoop_Drive(0.0f, 0.0f);
        g_identify_timer = 0;
        g_identify_state = IDENTIFY_STATE_ALIGN; 
    }
}

// 辨识状态机轮询任务
void Motor_Identify_Task(void)
{
    switch (g_identify_state)
    {
        case IDENTIFY_STATE_IDLE:
            // 等待启动指令
            break;
            
        case IDENTIFY_STATE_MEASURE_R:
            Identify_MeasureR();
            break;
            
        case IDENTIFY_STATE_MEASURE_L:
            Identify_MeasureL();
            break;

        case IDENTIFY_STATE_ALIGN:
            Identify_Align();
            break;
            
        case IDENTIFY_STATE_UVW_AND_POLES:
            Identify_UvwAndPoles();
            break;
            
        case IDENTIFY_STATE_DONE:
            // 辨识出结果并保存，等待应用层进入 RUN 状态
            break;
            
        case IDENTIFY_STATE_ERROR:
            // 若监测到电流超标或未连接电机，在此切断输出停止
            break;
    }
}

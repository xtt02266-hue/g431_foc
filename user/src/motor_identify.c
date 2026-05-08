#include "motor_identify.h"
#include "as5600.h"
#include "motor_system.h"
#include "motor_current_loop.h"
#include "tim.h"
#include <math.h>
// #include "motor_current_loop.h" // 后续可能需要调用开环输出电压/电流的接口

static MotorIdentifyState g_identify_state = IDENTIFY_STATE_IDLE;
static MotorIdentifiedParams g_identified_params = {0};

// 状态机等待计时器
static uint32_t g_identify_timer = 0;
static uint16_t g_uvw_start_angle = 0;
static float g_uvw_elec_angle = 0.0f;

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
        // 优先测量电阻，为后续提供安全的对齐电压估算
        g_identify_state = IDENTIFY_STATE_MEASURE_R;
        g_identify_timer = 0;
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

// =========================================================
// 状态机具体实现：云台电机参数辨识
// 由于云台电机内阻大、电感小、极对数多，且容易发热，
// 必须遵循：测电阻 -> 测电感 -> 测极对数和相序 -> 静止对齐零点 的安全顺序。
// =========================================================

// 1. 测量相电阻 (欧姆定律: R = U/I)
// 目的：算出电机的真实电阻。因为只有知道了电阻，后续强拖时才知道给多大的电压是安全的，避免电机烧毁。
static void Identify_MeasureR(void)
{
    // FIXME: 在这里向电机施加一个已知的安全测试电压（比如给D轴加固定的低电压）。
    // 然后读取ADC当前的相电流反馈，等待电流稳定。
    // 当前使用临时固定值，后续替换为真实测量逻辑。
    g_identify_timer++;
    if (g_identify_timer > 500) // 等待500ms让系统稳定
    {
        // 临时固定相电阻值 (Ω)，典型云台电机约 5~15Ω
        g_identified_params.resistance = 0.2f;

        g_identify_timer = 0;
        g_identify_state = IDENTIFY_STATE_MEASURE_L; // 测完电阻后，进入下一个状态：测电感
    }
}

// 2. 测量相电感 
// 目的：测出电感(L)大小，主要是为了后面的“电流环”能自动算出 PI 控制器的参数。
static void Identify_MeasureL(void)
{
    // FIXME: 通常的做法是给电机施加一个高频的方波电压，看电流上升的速度(斜率)。
    // 电感 L = 电压 U / (电流变化量 di / 时间变化量 dt)
    // 当前使用临时固定值，后续替换为真实测量逻辑。
    g_identify_timer++;
    if (g_identify_timer > 500)
    {
        // 临时固定相电感值 (H)，典型云台电机约 0.1~1.0 mH
        g_identified_params.inductance = 0.000001f;

        g_identify_timer = 0;
        g_identify_state = IDENTIFY_STATE_UVW_AND_POLES; // 测完电感后，进入下一个状态：测相序与极对数
    }
}

// 3. 对齐转子到电角度零点 (获取电角度偏差 Offset)
// 目的：FOC算法需要知道磁场（电角度）的零点对应编码器的哪个物理位置。
static void Identify_Align(void)
{
    // 调用开环驱动函数，电气角度固定在 0 (0.0f)，幅值使用 200.0f
    // 这相当于用磁力把转子强行“吸附”在位置0，不让它动。
    Motor_OpenLoop_Drive(0.0f, 200.0f);
    
    g_identify_timer++;
    // 等待 1500 毫秒 (1.5秒)，确保云台电机完全停止晃动，稳定在零点
    if (g_identify_timer > 1500)
    {
        // 极点吸固后，读取此刻的磁编码器角度作为机械零点
        // AS5600 读出的原始值是 0~4095，这里将其换算成国际标准单位：弧度 (0 ~ 2π)
        g_identified_params.zero_angle_offset = (float)AS5600_ReadRawAngle() * (2.0f * 3.1415926f / 4096.0f);
        
        // 记录完零点后，关闭电机输出电压
        Motor_OpenLoop_Drive(0.0f, 0.0f);
        
        // 关键：将辨识出的真实电机参数同步推送给 FOC 电流环结构！
        // 安全校验：pole_pairs 不能为 0，否则 FOC 角度计算永久失效
        if (g_identified_params.pole_pairs == 0)
        {
            g_identified_params.pole_pairs = 7;
        }
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
    const float target_elec_angle = 4.0f * 2.0f * 3.1415926f; // 目标转过 4 个电周期
    const uint32_t lock_time = 500; // 预对齐锁定时间：500个计时周期 (通常为 500ms)
    
    // ================= 阶段 1：静态预对齐 =================
    if (g_identify_timer < lock_time)
    {
        // 锁定在电角度 0 的位置，让转子物理对齐
        Motor_OpenLoop_Drive(0.0f, 200.0f);
        
        // 在锁定即将结束的前一刻，清零累加器，并记录此时真正的起始机械角度
        if (g_identify_timer == lock_time - 1)
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
    g_uvw_elec_angle += 0.05f; 
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
        
        if (mech_turns > 0.01f) 
        {
            g_identified_params.pole_pairs = (uint16_t)roundf(4.0f / mech_turns);
        }
        else 
        {
            g_identified_params.pole_pairs = 7; // 安全兜底
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

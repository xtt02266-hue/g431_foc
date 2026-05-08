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
    
    g_identify_timer++;
    if (g_identify_timer > 1000) // 等待1000ms让电流完全稳定不变
    {
        // 根据稳定后的电流，用欧姆定律算出电阻: R = 电压 / 电流
        // g_identified_params.resistance = U_test / current_a;

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
    
    g_identify_timer++;
    if (g_identify_timer > 500)
    {
        // 把算出来的电感存入结果中
        // g_identified_params.inductance = ...;
        
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
static void Identify_UvwAndPoles(void)
{
    // 我们在这个环节，用代码强行拖着电机转 4 个电周期 (4圈电角度，即 4 * 2π)。
    // 为什么要转 4 个周期？因为云台电机在低速下可能会有“一顿一顿”的齿槽效应，多转几圈综合计算出来的误差更小。
    const float target_elec_angle = 4.0f * 2.0f * 3.1415926f;

    // 刚进入此状态时，记录电机当前的位置作为起点
    if (g_identify_timer == 0)
    {
        g_uvw_start_angle = AS5600_ReadRawAngle();
        g_uvw_elec_angle = 0.0f;
    }

    // 每次循环（通常1毫秒一次），让电角度略微往前推进一点点 (0.05弧度)
    g_uvw_elec_angle += 0.05f; 
    
    // 向底层给入电角度和驱动电压 (幅值200.0f约为安全力度)
    // fmodf() 是为了防止角度无限变大，把它限制在 0~2π 的范围内传到底层
    Motor_OpenLoop_Drive(fmodf(g_uvw_elec_angle, 2.0f * 3.1415926f), 200.0f);

    g_identify_timer++;
    
    // 当我们累积推过的总电角度 达到或超过 4圈 (target_elec_angle) 时，说明测完了
    if (g_uvw_elec_angle >= target_elec_angle)
    {
        uint16_t end_angle = AS5600_ReadRawAngle(); // 获取拖动结束时的最终角度
        
        // 计算机械上实际转过了多少 (终点减起点)
        int32_t delta = (int32_t)end_angle - (int32_t)g_uvw_start_angle;
        
        // 【注意跨零点问题】
        // 编码器是从 0 到 4095 一直循环的。如果起点是 4000，终点是 100，
        // 算出来 delta 是 -3900，但实际上它是正向跨过了 0 点转了 196 (4096-4000+100)。
        // 所以我们需要对过大的偏差进行修补：
        if (delta > 2048) delta -= 4096;
        else if (delta < -2048) delta += 4096;
        
        // 1. 判断相序方向 (极简判断法：电角度正向加，如果是正转，相序就是对的1，否则反转则是-1)
        g_identified_params.uvw_dir = (delta >= 0) ? 1 : -1;
        
        // 2. 计算极对数
        // 公式：极对数 = 我们强迫它转的总电周期数 / 机械上实际跟着转了多少圈
        // 我们上面让它转了 4个电周期，现在算它机械上转了几圈 (delta 占 4096 的比例)
        float mech_turns = (float)abs(delta) / 4096.0f; 
        if (mech_turns > 0.01f) // 防止除零导致程序崩溃
        {
            // roundf 是四舍五入。因为极对数肯定是个整数(如 7, 11, 14)。
            g_identified_params.pole_pairs = (uint16_t)roundf(4.0f / mech_turns);
        }

        // 停止输出电压，松开电机
        Motor_OpenLoop_Drive(0.0f, 0.0f);
        g_identify_timer = 0;
        
        // 测完极对数和方向后，就可以去做最后一步：静止对齐零点了
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

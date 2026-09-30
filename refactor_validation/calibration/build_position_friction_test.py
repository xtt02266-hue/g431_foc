from pathlib import Path
root=Path(__file__).resolve().parents[2]
def function(path,signature):
    s=(root/path).read_text(encoding='utf-8');a=s.index(signature);b=s.index('{',a)+1;depth=1
    while depth:depth+=(s[b]=='{')-(s[b]=='}');b+=1
    return s[a:b]
s='''#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "pid.h"
#include "motor_feedforward.h"
#define MOTOR_ENCODER_HALF_REV_F 16384.0f
#define MOTOR_ENCODER_COUNTS_PER_REV_F 32768.0f
#define MOTOR_SPEED_PID_OUT_MAX .1f
#define MOTOR_SPEED_PID_OUT_MIN -.1f
#define MOTOR_TORQUE_CURRENT_LIMIT_A 1.5f
static uint8_t g_cal_saved_valid,point;
static float g_host_iq_limit_a=.02f;
static float g_debug_position_target_counts,g_debug_position_actual_counts,g_debug_position_error_counts;
static float g_debug_position_target_speed_rpm,g_debug_friction_iq_a,g_debug_speed_loop_iq_a,g_debug_cogging_iq_a;
static struct { float target_q,target_d; } g_foc_state;
static MotorFrictionConfig g_friction_config={1,.035f,.00001f,20,.045f};
static MotorFrictionConfig g_cal_saved_friction={1,.035f,.00001f,20,.045f};
static unsigned assist;
static float desired_speed,pi_output;
static int8_t direction=-1;
static PID_Controller speed_pid;
static unsigned Motor_Encoder_GetRawAngle(void) { return 0; }
static float Motor_PositionLoop_Run(float t,float a) { (void)t;(void)a;return desired_speed; }
static struct identified { int8_t uvw_dir; } Motor_Identify_GetResult(void) { return (struct identified){direction}; }
static unsigned Motor_Calibration_IsPointMode(void) { return point; }
static unsigned Motor_Calibration_PointAssist(void) { return assist; }
static void Motor_SpeedLoop_SetTarget(float target) { speed_pid.target=target; }
static float Motor_SpeedLoop_Update(float current) { (void)current;return fmaxf(speed_pid.out_min,fminf(speed_pid.out_max,pi_output)); }
'''
s+=function('user/src/motor_feedforward.c','static float Motor_Feedforward_ClampFloat')+'\n'
s+=function('user/src/motor_feedforward.c','uint8_t Motor_Feedforward_IsFrictionConfigValid')+'\n'
s+=function('user/src/motor_feedforward.c','float Motor_Feedforward_FrictionWithConfig')+'\n'
s+=function('user/src/motor_feedforward.c','float Motor_Feedforward_FrictionCompensation')+'\n'
s+=function('user/src/motor_system.c','static void Motor_System_RunPositionMode')+'\n'
s+='''int main(void) {
 desired_speed=60;pi_output=0;Motor_System_RunPositionMode(100,0);
 assert(speed_pid.target==-60 && g_debug_friction_iq_a<-.03f);
 assert(fabsf(g_foc_state.target_q-g_debug_friction_iq_a)<1e-7f);
 desired_speed=-60;Motor_System_RunPositionMode(100,0);assert(g_debug_friction_iq_a>.03f);
 direction=1;Motor_System_RunPositionMode(100,0);assert(g_debug_friction_iq_a<-.03f);
 desired_speed=0;Motor_System_RunPositionMode(100,0);assert(g_debug_friction_iq_a==0);
 g_friction_config.enabled=0;desired_speed=60;Motor_System_RunPositionMode(100,0);assert(g_debug_friction_iq_a==0);
 g_friction_config.enabled=1;pi_output=10;Motor_System_RunPositionMode(100,0);
 assert(g_foc_state.target_q<=.100001f && speed_pid.out_max<.1f);
 g_cal_saved_valid=1;point=1;pi_output=0;Motor_System_RunPositionMode(100,0);
 assert(g_debug_friction_iq_a==0 && speed_pid.out_max==g_host_iq_limit_a);
 assist=1;g_friction_config.enabled=0;Motor_System_RunPositionMode(100,0);
 assert(g_debug_friction_iq_a>0 && g_debug_friction_iq_a<=g_host_iq_limit_a);
 assist=0;Motor_System_RunPositionMode(100,0);assert(g_debug_friction_iq_a==0);
 pi_output=10;Motor_System_RunPositionMode(100,0);assert(g_foc_state.target_q<=.020001f);
 puts("PASS: position friction uses target speed, UVW signs, zero speed, disabled, total limit and point exclusion");
}
'''
(root/'.codex_tmp/position_friction_test.c').write_text(s,encoding='utf-8')

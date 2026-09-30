from pathlib import Path
root=Path(__file__).resolve().parents[2]
def extract(path,name):
    s=(root/path).read_text(encoding='utf-8');start=s.index(name);start=s.rfind('\n',0,start)+1
    first=s.index('{',start);end=first+1;depth=1
    while depth:
        depth+=(s[end]=='{')-(s[end]=='}');end+=1
    return s[start:end]
source='''#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "pid.h"
#include <stdint.h>
typedef int MotorCommandResult;
enum { MOTOR_CMD_OK, MOTOR_CMD_NOT_OWNER, MOTOR_CMD_BUSY, MOTOR_CMD_MUST_STOP_FIRST,
 MOTOR_CMD_INVALID_COMBINATION, MOTOR_CMD_INVALID_VALUE };
enum { MOTOR_OWNER_HOST=1,MOTOR_STATE_STOPPED=0,MOTOR_STATE_SENSORED_RUN=3,
 MOTOR_CONTROL_SPEED=7,MOTOR_INPUT_HOST=1 };
PID_Controller speed_pid;
static unsigned active,g_config_revision,g_control_owner=1,g_control_mode=7,g_input_source=1;
static struct { unsigned state; } g_motor_system;
static unsigned Motor_Calibration_IsActive(void) { return active; }
static unsigned __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
void PID_Reset(PID_Controller *p) { p->integral=0;p->output=0;p->prev_error=0;p->prev_measure=p->measure; }
'''
source+=extract('user/src/motor_speed_loop.c','uint8_t Motor_SpeedLoop_SetPI')+'\n'
source+=extract('user/src/motor_system.c','MotorCommandResult Motor_System_HostSetSpeedPI')+'\n'
source+='''int main(void) {
 speed_pid=(PID_Controller){.kp=.0003f,.ki=.006f,.target=60,.measure=55,.integral=.01f,.out_min=-.2f,.out_max=.2f};
 float old=speed_pid.kp*5+speed_pid.integral;
 g_motor_system.state=3;
 assert(Motor_System_HostSetSpeedPI(.0005f,.003f)==MOTOR_CMD_OK);
 assert(fabsf(old-(speed_pid.kp*5+speed_pid.integral))<1e-7f);
 assert(g_config_revision==1 && speed_pid.ki==.003f);
 assert(Motor_System_HostSetSpeedPI(NAN,.1f)==MOTOR_CMD_INVALID_VALUE);
 assert(g_config_revision==1);
 g_control_owner=0;assert(Motor_System_HostSetSpeedPI(.001f,.01f)==MOTOR_CMD_NOT_OWNER);g_control_owner=1;
 active=1;assert(Motor_System_HostSetSpeedPI(.001f,.01f)==MOTOR_CMD_BUSY);active=0;
 g_control_mode=2;assert(Motor_System_HostSetSpeedPI(.001f,.01f)==MOTOR_CMD_INVALID_COMBINATION);
 g_control_mode=7;g_input_source=0;assert(Motor_System_HostSetSpeedPI(.001f,.01f)==MOTOR_CMD_INVALID_COMBINATION);
 g_motor_system.state=1;assert(Motor_System_HostSetSpeedPI(.001f,.01f)==MOTOR_CMD_MUST_STOP_FIRST);
 g_motor_system.state=0;assert(Motor_System_HostSetSpeedPI(.0003f,.00001f)==MOTOR_CMD_OK);
 assert(speed_pid.integral==0 && speed_pid.output==0);
 g_input_source=1;g_motor_system.state=3;speed_pid.integral=.199f;speed_pid.target=0;speed_pid.measure=100;
 assert(Motor_System_HostSetSpeedPI(.1f,1)==MOTOR_CMD_OK && speed_pid.integral<=speed_pid.out_max);
 puts("PASS: speed PI output continuity, clamp, stopped reset, bounds, owner, mode, calibration freeze");
}
'''
(root/'.codex_tmp/speed_pi_test.c').write_text(source,encoding='utf-8')
print('Generated test from actual speed PI setter and motor command guard.')

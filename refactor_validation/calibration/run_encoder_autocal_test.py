from pathlib import Path
import subprocess
import argparse
import shutil
root=Path(__file__).resolve().parents[2]
shim=root/'.codex_tmp/encoder_cal_shim'
shim.mkdir(parents=True,exist_ok=True)
(shim/'motor_system.h').write_text('''#include <stdint.h>
enum {MOTOR_OWNER_LOCAL, MOTOR_OWNER_HOST};
enum {MOTOR_STATE_STOPPED, MOTOR_STATE_WAIT_PARAMETERS, MOTOR_STATE_IDENTIFYING, MOTOR_STATE_SENSORED_RUN, MOTOR_STATE_MUSIC, MOTOR_STATE_FAULT};
typedef struct {uint8_t state,owner,run_requested;float target_speed_rpm;} MotorControlSnapshot;
typedef struct {struct {float speed_rpm;} run_data;uint8_t state;} MotorSystem;
extern MotorSystem g_motor_system;
void Motor_System_GetControlSnapshot(MotorControlSnapshot*);
void Motor_System_StopControl(void);
''',encoding='utf-8')
(shim/'motor_encoder.h').write_text('''#include <stdint.h>
uint8_t Motor_Encoder_IsDataFresh(uint32_t);
uint16_t Motor_Encoder_GetRawAngle(void);
''',encoding='utf-8')
(shim/'stm32g4xx_hal.h').write_text('''#include <stdint.h>
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
''',encoding='utf-8')
driver=root/'refactor_validation/calibration/test_encoder_autocal.c'
parser=argparse.ArgumentParser(description='Run assertions against the production encoder implementation using Zig C')
parser.add_argument('--zig', default=shutil.which('zig'))
args=parser.parse_args()
if not args.zig: parser.error('Zig is required: add zig to PATH or pass --zig /path/to/zig')
zig=args.zig
exe=root/'.codex_tmp/encoder_autocal_test.exe'
subprocess.run([zig,'cc','-std=c11','-O1','-UNDEBUG','-I'+str(shim),'-I'+str(root/'user/inc'),str(root/'user/src/motor_encoder_calibration.c'),str(driver),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)

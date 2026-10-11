"""Compare post-refactor ARM execution against commit 2f6fb3d over multiple 1 ms ticks.

Only time, raw sensor inputs, parameter readiness and physical output boundaries
are stubbed. The actual speed estimator, PI, friction/cogging models, mode logic,
heartbeat and stop/restore logic execute. This is not interrupt timing or hardware acceptance.
"""
from pathlib import Path
import itertools
import json
import struct
import sys

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
TEMP=ROOT/'.codex_tmp/architecture_validation'
sys.path.insert(0,str(ROOT/'refactor_validation/style_20261011'))
from verify_commands import Firmware, WATCHED, arm


class ObservedFirmware(Firmware):
    def reset(self,*args,**kwargs):
        super().reset(*args,**kwargs)
        self.outputs=[]

    def hook(self,uc,address,size,user):
        if address in self.stubs:
            name,_=self.stubs[address]
            if name=='motor_open_loop_drive':
                self.outputs.append((name,uc.reg_read(arm.UC_ARM_REG_S0),uc.reg_read(arm.UC_ARM_REG_S1)))
            elif name in ['motor_current_loop_enable','svpwm_enable','svpwm_disable']:
                self.outputs.append((name,uc.reg_read(arm.UC_ARM_REG_R0) if name=='motor_current_loop_enable' else 0))
        super().hook(uc,address,size,user)


WATCH=[name for name in WATCHED if name!='g_sensorless']+['g_fault_latched','g_host_last_heartbeat_ms','g_force_drag_active','g_force_drag_phase_rad',
              'g_force_drag_align_ticks','g_continuous_angle_rad','g_haptic_center_rad','g_last_angle_counts',
              'g_angle_initialized','g_pot_filter_initialized','g_pot_target_filtered','speed_est',
              'g_debug_position_target_counts','g_debug_position_actual_counts','g_debug_position_error_counts',
              'g_debug_position_target_speed_rpm','g_cal_saved_valid','g_cal_saved_cogging','g_cal_saved_friction',
              'g_cal_saved_limit','g_cal_saved_speed_slew','g_cal_saved_mode','g_cal_saved_source',
              'g_cal_saved_pos_max','g_cal_saved_pos_min','g_cal_control_flags','g_cal_last_crc','g_cal_last_transfer']


def main():
    baseline=ObservedFirmware(TEMP/'baseline.elf',True,{})
    current=ObservedFirmware(TEMP/'current.elf',True,{})
    pair=[baseline,current]
    cases=[]

    def prepare(mode=1,source=1,direction=1,owner=1):
        for f in pair:
            f.reset(3,owner)
            for name,value in [('HAL_GetTick',100),('motor_encoder_is_data_fresh',1),
                ('motor_encoder_get_raw_angle',0),('pot_read_raw',2048),('motor_parameters_is_busy',0),
                ('motor_parameters_is_ready',1),('motor_parameters_get_status',1),
                ('motor_encoder_cal_is_active',0),('motor_encoder_cal_requires_power_cycle',0),
                ('motor_encoder_cal_notify_stop',0),('motor_current_loop_is_enabled',1),
                ('motor_current_loop_enable',0),('svpwm_is_enabled',1),('svpwm_enable',0),
                ('svpwm_disable',0),('motor_open_loop_drive',0),('mt6826s_get_status',0),
                ('mt6826s_get_crc_error_count',0),('mt6826s_get_transfer_error_count',0),
                ('motor_encoder_get_sample_age_ms',0)]:f.stub(name,value)
            f.call('motor_system_init')
            f.call('motor_identify_use_result',[('float',2.),('float',.001),('uint16_t',14),('float',.3),('int8_t',direction)])
            for name,value in [('g_motor_system',3),('g_run_requested',1),('g_control_mode',mode),
                ('g_input_source',source),('g_control_owner',owner),('g_host_last_heartbeat_ms',100),
                ('g_host_iq_limit_a',.1),('g_torque_current_a',.08),('g_host_iq_slew_a_per_s',1.),
                ('g_host_speed_target_rpm',250.),('g_host_speed_slew_rpm_per_s',1000.)]:f.write(name,value)
            f.call('motor_feedforward_set_friction_config',
                   [('uint8_t',1),('float',.02),('float',.00001),('float',20.),('float',.03)])
            f.outputs=[]

    def same(label):
        a,b=[f.snapshot(WATCH) for f in pair]
        differences=[name for name in a if a[name]!=b[name]]
        assert not differences,(label,differences)
        assert baseline.outputs==current.outputs,(label,'hardware-boundary calls')
        cases.append(label)

    for mode,source,direction,step in itertools.product(range(8),range(3),[-1,1],[-180,-1,0,1,180]):
        prepare(mode,source,direction)
        for tick in range(32):
            angle=(32760+step*tick)%32768
            for f in pair:
                f.stub('HAL_GetTick',100+tick)
                f.stub('motor_encoder_get_raw_angle',angle)
                f.stub('pot_read_raw',[0,2048,4095][(tick//8)%3])
                if tick==16:f.write('g_host_speed_target_rpm',-250.)
                f.call('motor_system_task')
            same(f'mode={mode}/source={source}/direction={direction}/step={step}/tick={tick}')

    # Heartbeat transitions, including uint32 wrap and an already stale encoder.
    for elapsed,age,mode in itertools.product([0,499,500,501,1000,0xFFFFFFF0],range(2),[1,7]):
        prepare(mode)
        for f in pair:
            f.stub('HAL_GetTick',(100+elapsed)&0xFFFFFFFF)
            f.stub('motor_encoder_is_data_fresh',age)
            f.call('motor_system_task')
        same(f'heartbeat={elapsed}/fresh={age}/mode={mode}')

    # Real lifecycle state evaluation with different parameter and sensor outcomes.
    for parameter_status,busy,fresh,run,fault,powercycle,force in itertools.product(
            range(5),range(2),range(2),range(2),range(2),range(2),range(2)):
        prepare()
        for f in pair:
            f.stub('motor_parameters_get_status',parameter_status)
            f.stub('motor_parameters_is_busy',busy)
            f.stub('motor_encoder_is_data_fresh',fresh)
            f.stub('motor_encoder_cal_requires_power_cycle',powercycle)
            f.write('g_run_requested',run);f.write('g_fault_latched',fault);f.write('g_force_drag_active',force)
            # Original state helper is local; ELF symbols retain it for comparison.
            f.call('motor_control_update_state' if f is current else 'motor_system_update_operating_state')
        same(f'state={parameter_status}/{busy}/{fresh}/{run}/{fault}/{powercycle}/{force}')

    # Save/restore real calibration configuration through the command and stop paths.
    for mode,owner in itertools.product(range(6),range(2)):
        prepare(owner=owner)
        statuses=[]
        for f in pair:
            f.write('g_motor_system',0);f.write('g_run_requested',0)
            statuses.append(f.call('motor_system_host_start_calibration',
                [('uint8_t',mode),('uint16_t',16),('uint8_t',1),('uint8_t',0),('uint8_t',0)]))
        assert statuses[0]==statuses[1],(mode,owner,statuses)
        same(f'calibration-start={mode}/{owner}')
        for tick in range(6):
            for f in pair:
                f.stub('HAL_GetTick',100+tick)
                f.call('motor_system_task')
            same(f'calibration-task={mode}/{owner}/{tick}')
        for f in pair:f.call('motor_system_stop_control')
        same(f'calibration-stop-restore={mode}/{owner}')

    # Forced drag: observe the open-loop angle/amplitude without touching timer registers.
    prepare()
    for f in pair:
        f.write('g_force_drag_active',1);f.write('g_force_drag_align_ticks',2)
    for tick in range(12):
        for f in pair:f.call('motor_system_task')
        same(f'force-drag={tick}')

    result={'passed':True,'baseline_commit':'2f6fb3d022037055b2bc99174480761adcf9c91a',
            'case_count':len(cases),'observations':len(WATCH),'cases':cases,
            'limitations':['No asynchronous interrupts or hardware timing',
                           'Raw sensors, hardware outputs, time and parameter readiness use deterministic stubs']}
    (HERE/'runtime_equivalence.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(f'PASS: {len(cases)} ARM control/state/calibration steps against 2f6fb3d.')


if __name__=='__main__':main()

"""Exercise valid calibration sessions and restoration against the pre-refactor ARM firmware."""
from pathlib import Path
import json
import struct
import sys

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
TEMP=ROOT/'.codex_tmp/architecture_validation'
sys.path.insert(0,str(HERE))
from verify_runtime import ObservedFirmware, WATCH

pair=[ObservedFirmware(TEMP/'baseline.elf',True,{}),ObservedFirmware(TEMP/'current.elf',True,{})]
cases=[]
for mode in [0,1,4,5]:
    for direction in [-1,1]:
        for f in pair:
            f.reset()
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
            f.write('g_control_owner',1);f.write('g_motor_system',0)
            f.call('motor_feedforward_set_friction_config',
                   [('uint8_t',1),('float',.02),('float',.00001),('float',20.),('float',.03)])
            status=f.call('motor_system_host_start_calibration',
                [('uint8_t',mode),('uint16_t',128),('uint8_t',3),('uint8_t',0),('uint8_t',0)])
            assert status==0,(mode,direction,status)
            assert f.uc.mem_read(f.address('g_cal_saved_valid'),1)==b'\x01'
        assert pair[0].snapshot(WATCH)==pair[1].snapshot(WATCH),(mode,direction,'start')
        for tick in range(12):
            for f in pair:
                f.write('g_motor_system',3)
                f.stub('HAL_GetTick',100+tick)
                f.stub('motor_encoder_get_raw_angle',(32760+tick*direction)%32768)
                f.call('motor_system_task')
            a,b=[f.snapshot(WATCH) for f in pair]
            assert a==b,(mode,direction,tick,[name for name in a if a[name]!=b[name]])
            assert pair[0].outputs==pair[1].outputs
            cases.append(f'session={mode}/direction={direction}/tick={tick}')
        for f in pair:
            f.call('motor_system_stop_control')
            assert f.uc.mem_read(f.address('g_cal_saved_valid'),1)==b'\x00'
            assert f.uc.mem_read(f.address('g_friction_config'),1)==b'\x01'
        assert pair[0].snapshot(WATCH)==pair[1].snapshot(WATCH),(mode,direction,'restore')
        cases.append(f'session={mode}/direction={direction}/stop-and-restore')
result={'passed':True,'case_count':len(cases),'valid_sessions':8,'cases':cases}
(HERE/'calibration_session_equivalence.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(f'PASS: {len(cases)} checks across 8 valid calibration sessions and stop/restore paths.')

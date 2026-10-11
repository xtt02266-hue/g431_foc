"""Require all object-code differences to stay within the enumerated interface migration sites."""
from pathlib import Path
import json
from verify_units import object_contents

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).parent
objects=ROOT/'.codex_tmp/style_validation/objects'
mappings=json.loads((HERE/'all_renames.json').read_text())
ALLOWED={
    'motor_cogging.c': ['Motor_Cogging_SetConfig','Motor_Cogging_TableBegin','motor_cogging_get_transaction_id'],
    'motor_current_loop.c': ['Motor_CurrentLoop_SetParams'],
    'motor_feedforward.c': ['Motor_Feedforward_SetFrictionConfig'],
    'motor_identify.c': ['Motor_Identify_UseResult'],
    'motor_music.c': ['Motor_Music_Start','Motor_Music_StartTimes','Motor_Music_StartDemo','Motor_Music_StartDemoTimes'],
    'motor_parameters.c': ['Motor_Parameters_Init'],
    'motor_sensorless.c': ['Motor_Sensorless_Configure','Motor_Sensorless_ConfigureMotor'],
    'motor_songs.c': ['motor_songs_register_score','motor_songs_get_score','s_custom_note_count','s_custom_score'],
    'motor_system.c': ['Motor_System_HostStartCalibration','Motor_System_HostStartEncoderCalibration',
                       'Motor_System_RestoreCalibration','Motor_System_HostSetHapticParams',
                       'Motor_System_HostSetFrictionConfig','Motor_System_HostSaveFrictionConfig',
                       'Motor_System_HostSetCoggingConfig','Motor_System_PlaySong','Motor_System_PlaySongLoop'],
    'pc_protocol.c': ['handle_command'],
    'oled.c': ['I2C_Delay','OLED_W_SCL','OLED_W_SDA','OLED_I2C_Init','OLED_I2C_Start',
               'OLED_I2C_Stop','OLED_I2C_SendByte','OLED_WriteCommand','OLED_WriteData','OLED_SetCursor','OLED_Pow'],
}
ALLOWED['motor_system.c'] += ['Motor_System_HostSetIq','Motor_System_HostSetSpeed','Motor_System_HostBeginCoggingTable',
                            'motor_system_get_last_accepted_iq','motor_system_get_last_accepted_speed',
                            's_last_accepted_iq_a','s_last_accepted_speed_rpm']
results=[]; preserved=0
for source in sorted((ROOT/'user/src').glob('*.c')):
    for profile in (0,1):
        left=object_contents(objects/f'{source.stem}_{profile}_baseline.o',{})
        right=object_contents(objects/f'{source.stem}_{profile}_current.o',
                              {new:old for old,new in mappings[source.name].items()})
        changes={}
        for category in ('sections','relocations','public_symbols'):
            changed=sorted(name for name in left[category].keys()|right[category].keys()
                           if left[category].get(name)!=right[category].get(name))
            changes[category]=changed
            for name in changed:
                symbol=name.removeprefix('.text.').removeprefix('.bss.')
                assert symbol in ALLOWED.get(source.name,[]),(source.name,profile,category,name)
        preserved += sum(name.startswith('.text.') and name not in changes['sections']
                         and name not in changes['relocations'] for name in left['sections'])
        results.append({'source':source.name,'profile_sensorless':profile,'passed':True,'intentional_changes':changes})
compile_report=json.loads((HERE/'interfaces_units_comparison.json').read_text())
assert all(not entry['warnings'] for entry in compile_report['units'])
assert all(entry['passed'] for entry in compile_report['headers'])
report={'passed':True,'units':results,'preserved_function_profile_pairs':preserved,
        'note':'Every other function retains identical instruction/data sections and symbolic relocations.'}
(HERE/'control_code_equivalence.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(f'PASS: all 54 objects; {preserved} function/profile pairs unchanged; only enumerated interface sites differ.')

"""Run baseline and migrated ARM firmware commands in Unicorn, comparing results and state.

Hardware time/status and Flash-save results are deterministic boundary stubs.
This verifies firmware logic, not peripheral timing, interrupts or physical motor behavior.
"""
from pathlib import Path
import argparse
import itertools
import json
import math
import re
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.codex_tmp/style_validation/python_modules'))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
import unicorn.arm_const as arm
from elftools.elf.elffile import ELFFile

SCRATCH = 0x10000000
RETURN = SCRATCH + 0xF000
STACK = 0x20008000
REGISTERS = [arm.UC_ARM_REG_R0, arm.UC_ARM_REG_R1, arm.UC_ARM_REG_R2, arm.UC_ARM_REG_R3]
FLOAT_REGISTERS = [getattr(arm, f'UC_ARM_REG_S{i}') for i in range(16)]
FORMATS = {'float': 'f', 'uint8_t': 'B', 'int8_t': 'b', 'uint16_t': 'H', 'int16_t': 'h', 'uint32_t': 'I'}


def pack_config(fields, values):
    data = bytearray()
    for (kind, _), value in zip(fields, values):
        size = struct.calcsize(FORMATS[kind])
        data.extend(bytes((-len(data)) % size))
        data.extend(struct.pack('<' + FORMATS[kind], value))
    data.extend(bytes((-len(data)) % 4))
    return bytes(data)


class Firmware:
    def __init__(self, path, current, renames):
        self.current = current
        self.renames = renames
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.ctl_set_cpu_model(arm.UC_CPU_ARM_CORTEX_M4)
        self.uc.mem_map(0x08000000, 0x20000)
        self.uc.mem_map(0x20000000, 0x8000)
        self.uc.mem_map(SCRATCH, 0x10000)
        self.uc.mem_map(0xE000E000, 0x2000)
        self.uc.reg_write(arm.UC_ARM_REG_C1_C0_2, 0xF << 20)
        self.uc.reg_write(arm.UC_ARM_REG_FPEXC, 0x40000000)
        self.uc.mem_write(0xE000ED88, struct.pack('<I', 0xF << 20))
        self.symbols = {}
        with path.open('rb') as file:
            elf = ELFFile(file)
            for section in elf.iter_sections():
                if section['sh_flags'] & 2 and section['sh_size'] and section['sh_type'] != 'SHT_NOBITS':
                    self.uc.mem_write(section['sh_addr'], section.data())
            for symbol in elf.get_section_by_name('.symtab').iter_symbols():
                if symbol.name and symbol['st_shndx'] != 'SHN_UNDEF':
                    self.symbols[symbol.name] = (symbol['st_value'], symbol['st_size'])
        self.initial_ram = bytes(self.uc.mem_read(0x20000000, 0x8000))
        self.initial_context = self.uc.context_save()
        self.stubs = {}
        self.saved_configs = []
        self.queued_frames = []
        self.uc.hook_add(UC_HOOK_CODE, self.hook)

    def name(self, name):
        return self.renames.get(name, name) if self.current else name

    def address(self, name):
        return self.symbols[self.name(name)][0]

    def reset(self, state=0, owner=1):
        self.uc.context_restore(self.initial_context)
        self.uc.mem_write(0x20000000, self.initial_ram)
        self.uc.mem_write(SCRATCH, bytes(0x10000))
        self.saved_configs = []
        self.queued_frames = []
        self.stubs = {}
        self.write('g_motor_system', state)
        self.write('g_control_owner', owner)
        self.stub('HAL_GetTick', 12345)

    def write(self, name, value):
        address, size = self.symbols[self.name(name)]
        if isinstance(value, bytes):
            self.uc.mem_write(address, value)
        else:
            self.uc.mem_write(address, struct.pack('<f' if isinstance(value, float) else '<I', value)[:size])

    def stub(self, name, result):
        translated = self.name(name)
        if translated in self.symbols:
            self.stubs[self.symbols[translated][0] & ~1] = (name, result)

    def hook(self, uc, address, size, _):
        if address not in self.stubs:
            return
        name, result = self.stubs[address]
        if name == 'Motor_FrictionStorage_Save':
            self.saved_configs.append(bytes(uc.mem_read(uc.reg_read(arm.UC_ARM_REG_R0), 20)))
        if name == 'queue_frame':
            length = uc.reg_read(arm.UC_ARM_REG_R3)
            self.queued_frames.append([uc.reg_read(arm.UC_ARM_REG_R0),uc.reg_read(arm.UC_ARM_REG_R1),
                                       bytes(uc.mem_read(uc.reg_read(arm.UC_ARM_REG_R2),length)).hex()])
        uc.reg_write(arm.UC_ARM_REG_R0, result)
        uc.reg_write(arm.UC_ARM_REG_PC, uc.reg_read(arm.UC_ARM_REG_LR))

    def call(self, name, parameters=(), float_result=False):
        # AAPCS hard-float: scalar floats use S registers, integer/pointer scalars use R registers.
        integer_index = float_index = 0
        stack = bytearray()
        for kind, value in parameters:
            if kind == 'float' and float_index < 16:
                self.uc.reg_write(FLOAT_REGISTERS[float_index], struct.unpack('<I', struct.pack('<f', value))[0])
                float_index += 1
            elif kind != 'float' and integer_index < 4:
                self.uc.reg_write(REGISTERS[integer_index], int(value) & 0xFFFFFFFF)
                integer_index += 1
            else:
                stack.extend(struct.pack('<f' if kind == 'float' else '<I', value))
        stack_address = STACK - ((len(stack) + 7) & ~7)
        if stack:
            self.uc.mem_write(stack_address, bytes(stack))
        self.uc.reg_write(arm.UC_ARM_REG_SP, stack_address)
        self.uc.reg_write(arm.UC_ARM_REG_LR, RETURN | 1)
        self.uc.emu_start(self.address(name) | 1, RETURN, count=200000)
        if self.uc.reg_read(arm.UC_ARM_REG_PC) != RETURN:
            raise RuntimeError(f'{name}: did not return within instruction budget')
        return self.uc.reg_read(arm.UC_ARM_REG_S0 if float_result else arm.UC_ARM_REG_R0)

    def snapshot(self, names):
        result = {}
        for name in names:
            translated = self.name(name)
            if translated in self.symbols:
                address, size = self.symbols[translated]
                contents = bytearray(self.uc.mem_read(address, size))
                # C struct assignment does not specify padding bytes. Observe every actual field.
                padding = {'g_haptic_params': [(22,24)], 'g_friction_config': [(1,4)],
                           'g_config': [(1,4),(22,24)], 'g_sensorless': [(14,16),(45,48)],
                           'g_identified_params': [(10,12),(17,20)]}.get(name, [])
                for start,end in padding:
                    contents[start:end] = bytes(end-start)
                result[name] = contents.hex()
        result['saved_configs'] = [(value[:1]+bytes(3)+value[4:]).hex() for value in self.saved_configs]
        result['queued_frames'] = self.queued_frames
        return result


WATCHED = ['g_motor_system', 'g_control_mode', 'g_input_source', 'g_control_owner', 'g_run_requested',
           'g_torque_current_a', 'g_host_iq_limit_a', 'g_host_iq_applied_a', 'g_host_iq_slew_a_per_s',
           'g_host_speed_target_rpm', 'g_host_speed_applied_rpm', 'g_host_speed_slew_rpm_per_s',
           'g_config_revision', 'g_haptic_params', 'g_friction_config', 'g_config', 'g_persisted',
           'g_effective_gain', 'g_ramp_elapsed_ms', 'speed_pid', 'g_pi_pos', 'g_foc_state',
           'g_debug_speed_loop_iq_a', 'g_debug_friction_iq_a', 'g_debug_cogging_iq_a',
           'g_song', 'g_note_count', 'g_note_index', 'g_note_remaining_ms', 'g_gap_start_ms',
           'g_plays_remaining', 'g_repeat_forever', 'g_playing', 'g_phase_reset_requested',
           'g_phase_step', 'g_amplitude_a', 'g_phase_accumulator',
           'g_sensorless', 'g_identified_params', 'g_identify_state', 'g_identify_timer']
WATCHED += ['g_transaction_counter','g_active_transaction','g_transaction_last_ms','g_expected_crc','g_received']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--current', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    renames = json.loads((Path(__file__).parent / 'public_renames.json').read_text())
    baseline = Firmware(args.baseline, False, renames)
    current = Firmware(args.current, True, renames)
    results = []

    def check(label, original_name, baseline_args, current_args, state=0, owner=1, setup=None,
              compare_return=True):
        returned = []
        snapshots = []
        for firmware, arguments in [(baseline, baseline_args), (current, current_args)]:
            firmware.reset(state, owner)
            if setup:
                setup(firmware)
            returned.append(firmware.call(original_name, arguments))
            snapshots.append(firmware.snapshot(WATCHED))
        differences = [key for key in snapshots[0] if snapshots[0][key] != snapshots[1][key]]
        assert (not compare_return or returned[0] == returned[1]) and not differences, (label, returned, differences)
        results.append({'case': label, 'status': returned[0], 'passed': True})

    # Modes use uint8_t, matching the original ARM enum width and protocol byte domain.
    for mode, source in itertools.product(range(256), range(5)):
        values = [('uint8_t', mode), ('uint8_t', source)]
        check(f'mode:{mode}/{source}', 'Motor_System_HostSetMode', values, values)
    for state, owner in itertools.product(range(6), range(2)):
        values = [('uint8_t', 255), ('uint8_t', 255)]
        check(f'mode:priority:{state}/{owner}', 'Motor_System_HostSetMode', values, values, state, owner)
    for mode, state, owner in itertools.product(list(range(9))+[255], range(6), range(2)):
        values = [('uint8_t', mode)]
        check(f'local-mode:{mode}/{state}/{owner}', 'Motor_System_SetControlMode', values, values, state, owner)

    manifest = json.loads((Path(__file__).parent / 'scalar_migration.json').read_text())
    originals = {new: old for old, new in renames.items()}
    defaults = {
        'motor_system_host_set_haptic_params': [0.03, 0.002, 0.05, 0.05, 90.0, 24],
        'motor_system_host_set_friction_config': [1, 0.02, 0.00001, 20.0, 0.03],
        'motor_system_host_save_friction_config': [1, 0.02, 0.00001, 20.0, 0.03],
        'motor_system_host_set_cogging_config': [1, 1.0, 0.03, 50.0, 100.0, 0],
    }
    edges = [-math.inf, math.inf, math.nan, -1.0, -0.0, 0.0, 0.01, 0.02, 0.5, 1.0, 5.0,
             10.0, 20.0, 50.0, 90.0, 100.0, 180.0, 200.0, 5000.0]
    for name, defaults_values in defaults.items():
        fields = manifest[name]['fields']
        cases = [defaults_values]
        for index, (kind, _) in enumerate(fields):
            variants = edges if kind == 'float' else ([0, 1, 2, 127, 128, 129, 255] if kind == 'uint8_t' else [0, 1, 128, 129, 32767, 65535])
            for value in variants:
                candidate = list(defaults_values)
                candidate[index] = value
                cases.append(candidate)
        for index, values in enumerate(cases):
            packed = pack_config(fields, values)
            def setup(firmware, data=packed):
                firmware.uc.mem_write(SCRATCH, data)
                firmware.stub('Motor_Calibration_IsActive', 0)
                firmware.stub('Motor_Calibration_HasSamples', 0)
                firmware.stub('Motor_Parameters_IsBusy', 0)
                firmware.stub('Motor_FrictionStorage_Save', 1)
            check(f'{name}:{index}', originals[name], [('pointer', SCRATCH)],
                  [(kind, value) for (kind, _), value in zip(fields, values)], setup=setup)
        for state, owner in itertools.product(range(6), range(2)):
            bad_values = list(defaults_values); bad_values[1] = math.nan
            packed = pack_config(fields, bad_values)
            def setup(firmware, data=packed):
                firmware.uc.mem_write(SCRATCH, data)
                firmware.stub('Motor_Calibration_IsActive', 0)
                firmware.stub('Motor_Calibration_HasSamples', 0)
                firmware.stub('Motor_Parameters_IsBusy', 0)
                firmware.stub('Motor_FrictionStorage_Save', 1)
            check(f'{name}:priority:{state}/{owner}', originals[name], [('pointer', SCRATCH)],
                  [(kind, value) for (kind, _), value in zip(fields, bad_values)], state, owner, setup)

    # Persistence outcomes and busy precedence remain visible, rather than checking success alone.
    name = 'motor_system_host_save_friction_config'
    fields = manifest[name]['fields']; values = defaults[name]
    for active, samples, busy, saved in itertools.product(range(2), repeat=4):
        def setup(firmware):
            firmware.uc.mem_write(SCRATCH, pack_config(fields, values))
            firmware.stub('Motor_Calibration_IsActive', active)
            firmware.stub('Motor_Calibration_HasSamples', samples)
            firmware.stub('Motor_Parameters_IsBusy', busy)
            firmware.stub('Motor_FrictionStorage_Save', saved)
        check(f'friction-save:{active}/{samples}/{busy}/{saved}', originals[name], [('pointer', SCRATCH)],
              [(kind, value) for (kind, _), value in zip(fields, values)], setup=setup)

    # Pointer input vs scalar arguments for the full sampling configuration (unused production API).
    fields = manifest['motor_current_loop_set_params']['fields']
    for values in [[3.3,1.65,1.65,.016,20.,4095.,14,.4,-1],
                   [5.,2.,1.,.01,10.,1023.,1,-.2,1]]:
        packed = pack_config(fields, values)
        words = struct.unpack('<'+'I'*(len(packed)//4), packed)
        check('current-loop-config:'+str(values[-1]), 'Motor_CurrentLoop_SetParams',
              [('uint32_t', word) for word in words],
              [(kind, value) for (kind, _), value in zip(fields, values)], compare_return=False)

    # Replay arbitrary external songs through the existing sequencer and actual 20 kHz DDS logic.
    for name, defaults_values in [
        ('motor_sensorless_configure',[.00005,1.,.0006,1,.05,.4,250.,12000.,8000.,.30,100,250,4]),
        ('motor_identify_use_result',[2.,.001,14,.3,-1]),
    ]:
        fields=manifest[name]['fields']
        cases=[defaults_values]
        for index,(kind,_) in enumerate(fields):
            variants=edges if kind=='float' else ([0,1,2,255] if kind=='uint8_t' else
                     [-1,0,1,127] if kind=='int8_t' else [0,1,14,65535])
            for value in variants:
                values=list(defaults_values);values[index]=value;cases.append(values)
        for index,values in enumerate(cases):
            def setup(firmware):
                firmware.uc.mem_write(SCRATCH,pack_config(fields,values))
            check(f'{name}:{index}',originals[name],[('pointer',SCRATCH)],
                  [(kind,value) for (kind,_),value in zip(fields,values)],setup=setup,
                  compare_return=name!='motor_identify_use_result')

    # Replay arbitrary external songs through the existing sequencer and actual 20 kHz DDS logic.
    for count, plays, loop in itertools.product([0,1,3], [0,1,2], range(2)):
        for firmware in (baseline, current):
            firmware.reset()
            firmware.uc.mem_write(SCRATCH, struct.pack('<fHxxfHxxfHxx',440.,0,0.,2,987.77,3))
            if firmware.current:
                firmware.call('motor_songs_register_score',[('pointer',SCRATCH),('uint16_t',count)])
            if loop:
                original='Motor_Music_Start'
                arguments = [('uint8_t',1),('uint8_t',plays)] if firmware.current else \
                            [('pointer',SCRATCH),('uint16_t',count),('uint8_t',plays)]
            else:
                original='Motor_Music_StartTimes'
                arguments = [('uint8_t',1),('uint16_t',plays)] if firmware.current else \
                            [('pointer',SCRATCH),('uint16_t',count),('uint16_t',plays)]
            firmware.call(original,arguments)
        for tick in range(25):
            for firmware in (baseline,current):
                firmware.call('Motor_Music_Task1ms')
            for sample in range(2):
                values = [firmware.call('Motor_Music_ProcessIqTarget',[('float',.037)],float_result=True)
                          for firmware in (baseline,current)]
                assert values[0]==values[1], ('music DDS', count,plays,loop,tick,sample,values)
            assert baseline.snapshot(WATCHED)==current.snapshot(WATCHED), ('music state',count,plays,loop,tick)
        results.append({'case':f'music:{count}/{plays}/{loop}:25ms+50DDS','passed':True})

    for ready, enabled, active, power_cycle, count, plays in itertools.product(range(2),range(2),range(2),range(2),[0,3],[0,2]):
        def setup(firmware):
            firmware.uc.mem_write(SCRATCH,struct.pack('<fHxxfHxxfHxx',440.,1,0.,2,987.77,3))
            firmware.stub('Motor_Parameters_IsReady',ready)
            firmware.stub('Motor_CurrentLoop_IsEnabled',enabled)
            firmware.stub('Motor_EncoderCal_IsActive',active)
            firmware.stub('Motor_EncoderCal_RequiresPowerCycle',power_cycle)
            if firmware.current:
                firmware.call('motor_songs_register_score',[('pointer',SCRATCH),('uint16_t',count)])
        check(f'system-song:{ready}/{enabled}/{active}/{power_cycle}/{count}/{plays}',
              'Motor_System_PlaySong',[('pointer',SCRATCH),('uint16_t',count),('uint16_t',plays)],
              [('uint8_t',1),('uint16_t',plays)],setup=setup)

    # Execute the actual protocol dispatcher and response serializer; replace only DMA queue handoff.
    protocol = (ROOT/'user/src/pc_protocol.c').read_text(encoding='utf-8')
    commands = {name:int(value,0) for name,value in re.findall(r'\b(CMD_\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)',protocol)}
    bodies = {
        'CMD_SET_MODE': bytes([1,1]),
        'CMD_SET_HAPTIC': struct.pack('<5fH',.03,.002,.05,.05,90.,24),
        'CMD_SET_FRICTION': struct.pack('<B4f',1,.02,.00001,20.,.03),
        'CMD_SAVE_FRICTION': struct.pack('<B4f',1,.02,.00001,20.,.03),
        'CMD_SET_COGGING_CONFIG': struct.pack('<B4fH',1,1.,.03,50.,100.,0),
        'CMD_SET_IQ': struct.pack('<ff',.5,1.),
        'CMD_SET_SPEED': struct.pack('<ff',250.,100.),
        'CMD_COGGING_TABLE_BEGIN': struct.pack('<HI',512,0x12345678),
    }
    for command, body in bodies.items():
        variants = [body, body[:-1], body+b'\0', bytes(len(body))]
        if command=='CMD_SET_MODE':
            variants += [bytes([mode,source]) for mode,source in itertools.product(range(9),range(4))]
        for index, candidate in enumerate(variants):
            for state,owner,token in itertools.product([0,3],[0,1],[0x12345678,0]):
                packet=bytes([commands[command]])+struct.pack('<I',token)+candidate
                def setup(firmware):
                    firmware.write('s_token',0x12345678)
                    firmware.write('g_control_mode',7 if command=='CMD_SET_SPEED' else 1)
                    firmware.write('g_input_source',1)
                    firmware.uc.mem_write(SCRATCH,packet)
                    firmware.stub('queue_frame',1)
                    firmware.stub('Motor_Calibration_IsActive',0)
                    firmware.stub('Motor_Calibration_HasSamples',0)
                    firmware.stub('Motor_Parameters_IsBusy',0)
                    firmware.stub('Motor_FrictionStorage_Save',1)
                parameters=[('uint16_t',42),('pointer',SCRATCH),('uint16_t',len(packet))]
                check(f'protocol:{command}:{index}/{state}/{owner}/{token}', 'handle_command',
                      parameters,parameters,state,owner,setup,compare_return=False)

    # Scalar command + captured-result getter retains the previous pointer-output contract.
    for name,getter,mode,target in [('Motor_System_HostSetIq','motor_system_get_last_accepted_iq',1,'g_torque_current_a'),
                                    ('Motor_System_HostSetSpeed','motor_system_get_last_accepted_speed',7,'g_host_speed_target_rpm')]:
        for value,slew,state,owner in itertools.product(edges,[0.,1.,math.nan,-1.,1e9],[0,3],[0,1]):
            returns=[];accepted=[];snapshots=[]
            for firmware in (baseline,current):
                firmware.reset(state,owner)
                firmware.write('g_control_mode',mode);firmware.write('g_input_source',1)
                firmware.uc.mem_write(SCRATCH,struct.pack('<f',-99.))
                arguments=[('float',value),('float',slew)]
                if not firmware.current:arguments += [('pointer',SCRATCH)]
                status=firmware.call(name,arguments);returns.append(status)
                if status==0:
                    # Model an ISR clearing the running target between command and protocol result read.
                    firmware.write(target,0.)
                    accepted.append(firmware.call(getter,float_result=True) if firmware.current else
                                    struct.unpack('<I',firmware.uc.mem_read(SCRATCH,4))[0])
                snapshots.append(firmware.snapshot(WATCHED))
            assert returns[0]==returns[1] and snapshots[0]==snapshots[1],(name,value,slew,state,owner)
            assert not accepted or accepted[0]==accepted[1],('accepted result',name,value,accepted)
            results.append({'case':f'{name}:scalar-result:{value}/{slew}/{state}/{owner}','passed':True})
    for count,counter,state,owner in itertools.product([0,511,512,513],[0,19,65535],[0,3],[0,1]):
        returns=[];ids=[];snapshots=[]
        for firmware in (baseline,current):
            firmware.reset(state,owner);firmware.write('g_transaction_counter',counter)
            arguments=[('uint16_t',count),('uint32_t',0x12345678)]
            if not firmware.current:arguments += [('pointer',SCRATCH)]
            status=firmware.call('Motor_System_HostBeginCoggingTable',arguments);returns.append(status)
            if status==0:
                ids.append(firmware.call('motor_cogging_get_transaction_id') if firmware.current else
                           struct.unpack('<H',firmware.uc.mem_read(SCRATCH,2))[0])
            snapshots.append(firmware.snapshot(WATCHED))
        assert returns[0]==returns[1] and snapshots[0]==snapshots[1] and (not ids or ids[0]==ids[1]),(count,counter,state,owner)
        results.append({'case':f'cogging-begin:{count}/{counter}/{state}/{owner}','passed':True})
    result = {'passed': True, 'case_count': len(results), 'cases': results,
              'limitations': ['No asynchronous interrupts or hardware peripherals',
                              'Flash save is a deterministic stub; storage implementation unchanged',
                              'Named configuration padding bytes excluded; all member bytes compared',
                              'Null struct input removed by scalar API; old null call is no longer representable']}
    args.report.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(f'PASS: {len(results)} ARM command cases; statuses and {len(WATCHED)} state observations match.')


if __name__ == '__main__':
    main()

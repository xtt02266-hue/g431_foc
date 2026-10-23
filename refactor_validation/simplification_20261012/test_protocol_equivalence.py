"""Compare every command, replay handling, and calibration batch consumption in ARM."""
from pathlib import Path
import re
import struct
import sys
import itertools

ROOT = Path(__file__).resolve().parents[2]
TEMP = ROOT / '.codex_tmp/simplification_20261012'
sys.path.insert(0, str(ROOT / 'refactor_validation/style_20261011'))
from verify_commands import Firmware, WATCHED, SCRATCH


def main():
    pair = [Firmware(TEMP / name / 'g431_foc.elf', True, {}) for name in ['baseline', 'current']]
    watched = [n for n in WATCHED if n != 'g_sensorless'] + ['state', 'head', 'tail', 's_token', 's_last_request_seq']
    commands = re.findall(r'\b(CMD_\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)',
                          (ROOT / 'firmware/communication/pc_protocol.c').read_text(encoding='utf-8'))
    cases = 0
    def prepare(f):
        f.reset(0, 1)
        f.stub('motor_encoder_is_data_fresh', 1)
        f.stub('motor_parameters_is_ready', 1)
        f.stub('motor_parameters_get_status', 1)
        f.stub('motor_parameters_is_busy', 0)
        f.stub('motor_encoder_cal_is_active', 0)
        f.stub('motor_encoder_cal_requires_power_cycle', 0)
        f.stub('motor_encoder_cal_notify_stop', 0)
        f.stub('motor_open_loop_drive', 0)
        f.stub('motor_current_loop_is_enabled', 0)
        f.stub('motor_current_loop_enable', 0)
        f.stub('svpwm_is_enabled', 0)
        f.stub('svpwm_enable', 0)
        f.stub('svpwm_disable', 0)
        f.stub('motor_friction_storage_save', 1)
        f.stub('motor_cogging_save_to_flash', 1)
        for i in range(3):
            f.stub('HAL_GetUIDw' + str(i), 123 + i)
        f.call('pc_protocol_init')
        f.write('s_token', 0x12345678)
        f.stub('queue_frame', 1)
    def equal(label):
        a, b = [f.snapshot(watched) for f in pair]
        assert a == b, (label, [k for k in a if a[k] != b[k]])
    for (name, value), length, gate in itertools.product(commands, [0, 1, 2, 3, 4, 5, 6, 8, 9, 17, 19, 22], range(4)):
        packet = bytes([int(value, 0)]) + struct.pack('<I', 0 if gate == 1 else 0x12345678) + bytes(length)
        for f in pair:
            prepare(f)
            if gate == 2:
                f.stub('motor_encoder_cal_is_active', 1)
            if gate == 3:
                f.stub('motor_calibration_is_active', 1)
            f.uc.mem_write(SCRATCH, packet)
            try:
                f.call('handle_command', [('uint16_t', 42), ('pointer', SCRATCH), ('uint16_t', len(packet))])
            except Exception as error:
                raise RuntimeError((name, length, gate)) from error
        equal((name, length, gate))
        cases += 1
    # A repeated request must reuse ACK bytes; a same-sequence conflict must not execute.
    for f in pair:
        prepare(f)
    for packet in [b'\x20' + struct.pack('<I', 0x12345678) + b'\x01\x01'] * 2 + [b'\x20' + struct.pack('<I', 0x12345678) + b'\x02\x01']:
        for f in pair:
            f.uc.mem_write(SCRATCH, packet)
            f.call('handle_command', [('uint16_t', 43), ('pointer', SCRATCH), ('uint16_t', len(packet))])
        equal('replay/conflict')
        cases += 1
    for point, count, queued, active in itertools.product(range(2), [2, 3], range(2), range(2)):
        for f in pair:
            prepare(f)
            assert f.call('motor_calibration_begin', [('uint8_t', 5 if point else 1)]) == 1
            for i in range(count):
                sample = bytearray(64)
                struct.pack_into('<I', sample, 4, 100 + i)
                struct.pack_into('<H', sample, 8, 32760 + i)
                struct.pack_into('<f', sample, 12, .125 + i)
                f.uc.mem_write(SCRATCH, bytes(sample))
                f.call('motor_calibration_push', [('pointer', SCRATCH)])
            # Actual replay uses the saved ring records and must leave the consumer tail unchanged.
            packet = b'\x38' + struct.pack('<III', 0x12345678, 1, 1) + bytes([count])
            f.uc.mem_write(SCRATCH, packet)
            f.call('handle_command', [('uint16_t', 44), ('pointer', SCRATCH), ('uint16_t', len(packet))])
            if not active:
                f.call('motor_calibration_abort', [('uint8_t', 1)])
            f.stub('HAL_GetTick', 50)
            f.write('s_last_status_ms', 50)
            f.write('s_last_telemetry_ms', 50)
            f.stub('queue_frame', queued)
            before = bytes(f.uc.mem_read(f.address('tail'), 4))
            f.call('pc_protocol_task')
            after = bytes(f.uc.mem_read(f.address('tail'), 4))
            assert (before != after) == bool(queued and (count == 3 or not active))
        equal(('calibration-batch', point, count, queued, active))
        cases += 1
    print(f'PASS: {cases} all-command/gate/length/replay/calibration-stream comparisons.')


if __name__ == '__main__':
    main()

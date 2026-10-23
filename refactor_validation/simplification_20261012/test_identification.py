"""Run actual ARM cancellation/lifecycle code with deterministic output/Flash boundaries."""
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'refactor_validation/style_20261011'))
from verify_commands import Firmware, arm

ELF = ROOT / '.codex_tmp/simplification_20261012/current/g431_foc.elf'


class IdentificationFirmware(Firmware):
    def hook(self, uc, address, size, user):
        if address in self.stubs:
            name, result = self.stubs[address]
            if name == 'motor_open_loop_drive':
                self.amplitudes.append(struct.unpack('<f', struct.pack('<I', uc.reg_read(arm.UC_ARM_REG_S1)))[0])
            if name == 'motor_parameters_save' and self.cancel_during_save:
                context = uc.context_save()
                self.call('motor_system_stop_control')
                uc.context_restore(context)
        super().hook(uc, address, size, user)

    def prepare(self, ready):
        self.reset(0, 1)
        self.amplitudes = []
        self.cancel_during_save = False
        for name, value in [('motor_encoder_is_data_fresh', 1), ('motor_open_loop_drive', 0),
                            ('motor_current_loop_enable', 0), ('motor_current_loop_is_enabled', 0),
                            ('svpwm_disable', 0), ('svpwm_is_enabled', 0), ('motor_encoder_cal_notify_stop', 0),
                            ('motor_encoder_cal_is_active', 0), ('motor_encoder_cal_requires_power_cycle', 0)]:
            self.stub(name, value)
        self.call('motor_identify_use_result', [('float', 2.0), ('float', .001), ('uint16_t', 14),
                                              ('float', .3), ('int8_t', -1)])
        self.write('g_parameters_status', 1 if ready else 0)
        self.write('g_has_stored_data', ready)
        self.old_params = self.snapshot(['g_identified_params'])


def main():
    f = IdentificationFirmware(ELF, True, {})
    checks = 0
    for ready in [0, 1]:
        for phase in [1, 2, 3, 4, 5]:
            f.prepare(ready)
            assert f.call('motor_system_identify_and_save') == 1
            assert f.call('motor_system_start_control') == 0
            f.write('g_identify_state', phase)
            if phase == 5:
                f.write('g_parameters_status', 3)
            f.call('motor_system_stop_control')
            assert f.call('motor_parameters_get_status') == ready
            assert f.call('motor_parameters_is_busy') == 0
            if ready:
                assert f.snapshot(['g_identified_params']) == f.old_params
            f.amplitudes = []
            for _ in range(8):
                f.call('motor_parameters_control_task1ms')
                f.call('motor_identify_task')
                f.call('motor_control_update_state')
            assert not any(f.amplitudes)
            assert f.call('motor_system_get_state') == (0 if ready else 1)
            f.call('motor_system_stop_control')
            assert f.call('motor_parameters_get_status') == ready
            checks += 1
    for phase in [1, 2, 3, 4]:
        f.prepare(1)
        assert f.call('motor_system_identify_and_save') == 1
        f.write('g_identify_state', phase)
        f.stub('motor_encoder_is_data_fresh', 0)
        f.amplitudes = []
        f.call('motor_parameters_control_task1ms')
        assert f.call('motor_parameters_get_status') == 4
        assert not any(f.amplitudes)
        f.call('motor_control_update_state')
        assert f.call('motor_system_get_state') == 5
        assert bytes(f.uc.mem_read(f.address('g_run_requested'), 1)) == b'\0'
        checks += 1
    f.prepare(1)
    f.stub('motor_encoder_is_data_fresh', 0)
    assert f.call('motor_parameters_identify_and_save') == 0
    assert f.call('motor_system_identify_and_save') == 0
    f.call('motor_identify_start')
    assert f.call('motor_identify_get_state') == 6
    assert not any(f.amplitudes)
    checks += 1
    for saved in [0, 1]:
        f.prepare(1)
        f.call('motor_system_identify_and_save')
        f.write('g_parameters_status', 3)
        f.cancel_during_save = True
        f.stub('motor_parameters_save', saved)
        f.call('motor_parameters_background_task')
        assert f.call('motor_parameters_get_status') == 1
        assert f.call('motor_parameters_has_stored_data') == saved
        assert f.snapshot(['g_identified_params']) == f.old_params
        checks += 1
    f.prepare(1)
    f.call('motor_system_identify_and_save')
    f.write('g_fault_latched', 1)
    f.call('motor_control_update_state')
    assert f.call('motor_parameters_is_busy') == 0
    assert f.call('motor_system_get_state') == 5
    checks += 1
    f.prepare(1)
    f.call('motor_system_identify_and_save')
    f.write('g_host_last_heartbeat_ms', 0)
    f.call('motor_system_task')
    assert f.call('motor_parameters_is_busy') == 0
    assert f.call('motor_system_get_control_owner') == 0
    checks += 1
    f.prepare(0)
    f.write('g_auto_identify_pending', 1)
    f.call('motor_system_stop_control')
    f.call('motor_parameters_background_task')
    assert f.call('motor_parameters_is_busy') == 0
    checks += 1
    f.prepare(1)
    f.call('motor_identify_start')
    f.call('motor_system_stop_control')
    f.amplitudes = []
    f.call('motor_identify_task')
    assert not any(f.amplitudes)
    assert f.call('motor_parameters_is_ready') == 0
    checks += 1
    print(f'PASS: {checks} ARM identification cancellation/sensor/save/heartbeat scenarios.')


if __name__ == '__main__':
    main()

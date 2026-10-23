"""运行真实 ARM ELF，验证所有正常模式的补偿叠加、方向、限幅及标定隔离；不连接设备。"""
from pathlib import Path
import argparse
import csv
import hashlib
import itertools
import json
import math
import struct

from verify_builtin import CoggingFirmware, FLASH_PAGE, FLASH_PAGE_SIZE, CSV_PATH, arm

ROOT = Path(__file__).resolve().parents[2]
MODES = ('POSITION', 'TORQUE', 'FREE', 'DAMPING', 'SPRING', 'DETENT', 'LIMIT', 'SPEED')
PID_FIELDS = ('kp', 'ki', 'kd', 'dt', 'target', 'measure', 'output', 'integral',
              'prev_error', 'prev_measure', 'out_max', 'out_min')
TOLERANCE = 2e-6
COUNTS_PER_REV = 32768
RAD_PER_COUNT = 2. * math.pi / COUNTS_PER_REV


def angle_counts(angle_rad):
    return round(angle_rad / RAD_PER_COUNT)


def quantized_angle(angle_rad):
    return angle_counts(angle_rad) * RAD_PER_COUNT


def clamp(value, limit):
    return max(-limit, min(limit, value))


class ModeFirmware(CoggingFirmware):
    """仅替换原始反馈/时间/物理输出，真实模式、PID、补偿和标定仍由 ELF 执行。"""

    def __init__(self, path):
        self.float_stubs = {}
        self.entries = {}
        self.templates = {}
        super().__init__(path, True, {})
        assert self.symbols['speed_pid'][1] == 48, 'PID 布局改变，需更新测试字段映射'
        assert self.symbols['g_motor_system'][1] == 16, '系统状态布局改变，需更新测试字段映射'
        assert self.symbols['g_foc_state'][1] == 184, 'FOC 布局改变，需更新测试字段映射'
        for name in ('motor_feedforward_friction_compensation', 'motor_cogging_compensation',
                     'motor_open_loop_drive', 'motor_haptic_task', 'motor_system_run_torque_mode',
                     'motor_system_run_position_mode', 'motor_system_run_speed_mode'):
            self.entries[self.address(name) & ~1] = name
        self.calls = {}

    def hook(self, uc, address, size, user):
        if address in self.entries:
            name = self.entries[address]
            self.calls[name] = self.calls.get(name, 0) + 1
        if address in self.float_stubs:
            uc.reg_write(arm.UC_ARM_REG_S0, struct.unpack('<I', struct.pack('<f', self.float_stubs[address]))[0])
            uc.reg_write(arm.UC_ARM_REG_PC, uc.reg_read(arm.UC_ARM_REG_LR))
            return
        super().hook(uc, address, size, user)

    def float_stub(self, name, value):
        self.float_stubs[self.address(name) & ~1] = value

    def read_float(self, name, offset=0):
        return struct.unpack('<f', bytes(self.uc.mem_read(self.address(name) + offset, 4)))[0]

    def write_float(self, name, offset, value):
        self.uc.mem_write(self.address(name) + offset, struct.pack('<f', value))

    def read_int64(self, name):
        return struct.unpack('<q', bytes(self.uc.mem_read(self.address(name), 8)))[0]

    def inject_position(self, counts):
        """保持真实累计位置、单圈传感器及近似遥测一致；不使用float角作为控制输入。"""
        if 's_continuous_angle_counts' in self.symbols:
            self.uc.mem_write(self.address('s_continuous_angle_counts'), struct.pack('<q', counts))
            self.write('g_angle_initialized', 1)
            self.write('g_last_angle_counts', counts % COUNTS_PER_REV)
        self.stub('motor_encoder_get_raw_angle', counts % COUNTS_PER_REV)
        self.write('g_continuous_angle_rad', counts * RAD_PER_COUNT)

    def integer_result(self, name):
        self.call(name)
        value = self.uc.reg_read(arm.UC_ARM_REG_R0) | (self.uc.reg_read(arm.UC_ARM_REG_R1) << 32)
        return value if value < (1 << 63) else value - (1 << 64)

    def position_counts(self):
        return self.integer_result('motor_runtime_get_position_counts')

    def relative_counts(self):
        return self.integer_result('motor_haptic_get_relative_counts')

    def detent_center_units(self, count):
        return self.relative_counts() * count - self.read_int64('s_detent_error_units')

    def read_byte(self, name):
        return bytes(self.uc.mem_read(self.address(name), 1))[0]

    def pid(self):
        return dict(zip(PID_FIELDS, struct.unpack('<12f', bytes(self.uc.mem_read(self.address('speed_pid'), 48)))))

    def initialize_pid(self, name, kp=0., ki=0., limit=.9):
        self.call('pid_init', [('uint32_t', self.address(name)), ('float', kp), ('float', ki),
                              ('float', 0.), ('float', limit), ('float', -limit), ('float', .001)])

    def install_boundaries(self, speed=0., angle=64):
        for name, value in [('HAL_GetTick', 100), ('motor_encoder_is_data_fresh', 1),
                            ('motor_encoder_get_raw_angle', angle), ('pot_read_raw', 2048),
                            ('motor_parameters_is_busy', 0), ('motor_parameters_is_ready', 1),
                            ('motor_parameters_get_status', 1), ('motor_encoder_cal_is_active', 0),
                            ('motor_encoder_cal_requires_power_cycle', 0), ('motor_encoder_cal_notify_stop', 0),
                            ('motor_current_loop_is_enabled', 1), ('motor_current_loop_enable', 0),
                            ('svpwm_is_enabled', 1), ('svpwm_enable', 0), ('svpwm_disable', 0),
                            ('motor_open_loop_drive', 0), ('mt6826s_get_status', 0),
                            ('mt6826s_get_crc_error_count', 0), ('mt6826s_get_transfer_error_count', 0),
                            ('motor_encoder_get_sample_age_ms', 0)]:
            self.stub(name, value)
        self.float_stub('motor_speed_estimator_update_adaptive', speed)

    def prepare(self, mode=1, direction=1, speed=40., angle=64, friction=True, cogging=True, state=3,
                custom_haptic=True):
        key = (friction, cogging)
        if key not in self.templates:
            self.float_stubs = {}
            self.reset()
            self.uc.mem_write(FLASH_PAGE, bytes([255]) * FLASH_PAGE_SIZE)
            self.install_boundaries()
            self.call('motor_control_init')
            self.call('motor_feedforward_set_friction_config', [('uint8_t', int(friction)), ('float', .02),
                      ('float', .00001), ('float', 20.), ('float', .03)])
            self.set_config(enabled=int(cogging))
            for _ in range(500):
                self.call('motor_cogging_task1ms', [('uint8_t', 1)])
            self.templates[key] = (bytes(self.uc.mem_read(0x20000000, 0x8000)), self.uc.context_save())
        ram, context = self.templates[key]
        self.uc.context_restore(context)
        self.uc.mem_write(0x20000000, ram)
        self.stubs = {}
        self.float_stubs = {}
        self.calls = {}
        self.flash_operations = []
        self.install_boundaries(speed, angle)
        self.call('motor_identify_use_result', [('float', 2.), ('float', .001), ('uint16_t', 14),
                  ('float', .3), ('int8_t', direction)])
        for name, value in [('g_motor_system', state), ('g_run_requested', int(state == 3)),
                            ('g_control_mode', mode), ('g_input_source', 1), ('g_control_owner', 1),
                            ('g_host_last_heartbeat_ms', 100), ('g_host_iq_limit_a', .2),
                            ('g_host_iq_slew_a_per_s', 0.), ('g_host_speed_slew_rpm_per_s', 0.),
                            ('g_host_speed_target_rpm', -15. if speed < 0 else 15.),
                            ('g_torque_current_a', .01), ('g_continuous_angle_rad', .07),
                            ('g_haptic_center_rad', 0.)]:
            self.write(name, value)
        self.write_float('g_motor_system', 4, speed)
        if custom_haptic:
            self.write('g_haptic_params', struct.pack('<5fH2x', .03, .002, .05, .05, 1., 24))
        self.initialize_pid('speed_pid')
        self.initialize_pid('g_pi_pos', kp=.5, limit=350.)
        if 4 <= mode <= 6:
            # profile先以真实0计数建立参考，再设置编码器能实际表示的相对角。
            self.inject_position(0)
            self.call('motor_haptic_task', [('int8_t', direction), ('uint16_t', 0)])
            self.inject_position(angle_counts(.07))
        else:
            self.inject_position(angle)
        self.calls = {}

    def run_mode(self, mode, speed, direction, angle=64):
        if mode == 0:
            self.call('motor_system_run_position_mode', [('float', float(angle + 20)), ('float', speed * direction)])
        elif mode == 1:
            self.call('motor_system_run_torque_mode', [('float', speed), ('int8_t', direction), ('uint16_t', angle)])
        elif mode == 7:
            self.call('motor_system_run_speed_mode', [('float', speed), ('int8_t', direction), ('uint16_t', angle)])
        else:
            if 4 <= mode <= 6:
                angle = self.position_counts() % COUNTS_PER_REV
            self.call('motor_haptic_task', [('int8_t', direction), ('uint16_t', angle)])

    def output(self):
        return {'iq_a': self.read_float('g_foc_state', 164), 'id_a': self.read_float('g_foc_state', 160),
                'friction_a': self.read_float('g_debug_friction_iq_a'),
                'cogging_a': self.read_float('g_debug_cogging_iq_a'),
                'pi_a': self.read_float('g_debug_speed_loop_iq_a')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', type=Path, default=ROOT / 'build/Debug/g431_foc.elf')
    parser.add_argument('--baseline', type=Path, default=ROOT /
                        '.codex_tmp/all_modes_compensation_20261022/baseline/build/Debug/g431_foc.elf')
    parser.add_argument('--report', type=Path, default=ROOT /
                        '.codex_tmp/all_modes_compensation_20261022/mode_verification.json')
    parser.add_argument('--collect-failures', action='store_true', help='收集全部断言失败后以非零状态退出')
    args = parser.parse_args()
    results = []

    def check(label, condition, **evidence):
        results.append({'assertion': label, 'passed': bool(condition), **evidence})
        if not args.collect_failures:
            assert condition, (label, evidence)

    def close(actual, expected):
        return math.isclose(actual, expected, abs_tol=TOLERANCE, rel_tol=1e-5)

    with CSV_PATH.open(encoding='utf-8-sig', newline='') as stream:
        table = [round(float(row['iq_a']) / .0001) for row in csv.DictReader(stream)]

    def friction_model(speed, free=False):
        value = clamp(.02 * math.tanh(speed / 20.) + .00001 * speed, .03)
        if free:
            value = clamp(value - clamp(.0001 * speed * 2. * math.pi / 60., .01), .03)
        return value

    def cogging_model(angle, speed):
        idx, fraction = angle // 64, angle % 64 / 64.
        value = (table[idx] + (table[(idx + 1) % 512] - table[idx]) * fraction) * .0001 * .9
        fade = clamp((200. - abs(speed)) / 100., 1.) if abs(speed) > 100. else 1.
        return clamp(value * max(0., fade), .08)

    def remove_deadband(value, width):
        return math.copysign(max(0., abs(value) - width), value)

    def detent_center(relative_counts, previous_units, count):
        # Python整数独立描述格点与55%迟滞；阈值相邻真实编码器count也能直接覆盖。
        delta, snap = relative_counts * count - previous_units, 55 * COUNTS_PER_REV // 100
        if delta > snap:
            return previous_units + ((delta - snap + COUNTS_PER_REV - 1) // COUNTS_PER_REV) * COUNTS_PER_REV
        if delta < -snap:
            return previous_units - ((-delta - snap + COUNTS_PER_REV - 1) // COUNTS_PER_REV) * COUNTS_PER_REV
        return previous_units

    def haptic_model(mode, speed):
        omega, relative = speed * 2. * math.pi / 60., quantized_angle(.07)
        if mode == 2:
            return 0.
        if mode == 3:
            return -.002 * omega
        if mode == 4:
            error = remove_deadband(relative, math.radians(.15))
            gain = 1. + min(abs(error) / (math.pi / 2.), 1.) ** 2
            return -.03 * error * gain - (.002 + .03 * .025) * omega
        if mode == 5:
            spacing = 2. * math.pi / 24.
            center = detent_center(angle_counts(.07), 0, 24)
            error = remove_deadband((angle_counts(.07) * 24 - center) * RAD_PER_COUNT / 24,
                                    min(spacing * .02, math.radians(.15)))
            gain = 1. + min(abs(error) / (spacing / 2.), 1.) ** 2
            return -.05 * error * gain - (.002 + .05 * .025) * omega
        penetration = relative - math.pi / 180.
        gain = 1. + min(penetration / math.radians(10.), 2.) ** 2
        return -.05 * penetration * gain - (.002 + .05 * .025) * max(0., omega) * min(
            penetration / math.radians(.5), 1.)

    f = ModeFirmware(args.elf)
    f.prepare(custom_haptic=False)
    defaults = struct.unpack('<5fH2x', bytes(f.uc.mem_read(f.address('g_haptic_params'), 24)))
    check('触觉默认K与共享B/单位保持约定', all(close(actual, expected) for actual, expected in
          zip(defaults, (.12, .002, .4, .8, 90., 24))), defaults=defaults)
    # 每种补偿分别启闭，避免两种前馈相互抵消后掩盖漏接；独立公式检查而非只比较自身输出。
    for mode, direction, speed, enabled in itertools.product(range(8), (-1, 1), (-40., 0., 40.),
                                                             ((False, False), (True, False), (False, True), (True, True))):
        friction_on, cogging_on = enabled
        f.prepare(mode, direction, speed, friction=friction_on, cogging=cogging_on)
        model_speed = 10. if mode == 0 else (-15. if speed < 0 else 15.) if mode == 7 else speed
        friction = friction_model(model_speed, mode == 2) * direction if friction_on else 0.
        actual_angle = angle_counts(.07) % COUNTS_PER_REV if 4 <= mode <= 6 else 64
        cogging = cogging_model(actual_angle, speed) if cogging_on else 0.
        base = .01 if mode == 1 else haptic_model(mode, speed) * direction if 2 <= mode <= 6 else 0.
        expected = base + friction + cogging
        # 将斜率历史放在独立计算的稳态点；单独的阶跃测试覆盖实际斜率限制。
        f.write('g_host_iq_applied_a', expected)
        f.write_float('g_foc_state', 164, expected)
        f.run_mode(mode, speed, direction)
        actual = f.output()
        label = f'{MODES[mode]}/dir={direction}/rpm={speed:g}/friction={friction_on}/cogging={cogging_on}'
        check(label + '/独立模型与实际输出', close(actual['iq_a'], expected) and
              close(actual['friction_a'], friction) and close(actual['cogging_a'], cogging) and actual['id_a'] == 0.,
              expected_iq_a=expected, **actual)
        if mode in (0, 7):
            limit = .9 if mode == 0 else .2
            pid = f.pid()
            check(label + '/前馈预留PI余量', close(pid['out_max'], limit - friction - cogging) and
                  close(pid['out_min'], -limit - friction - cogging), pi_max=pid['out_max'], pi_min=pid['out_min'])
        if mode == 1:
            check(label + '/命令和力矩遥测不混入补偿', close(f.read_float('g_host_iq_applied_a'), .01) and
                  close(f.read_float('g_motor_system', 12), .01 * .23))

    # 现有 FREE/SPEED 在相同实际输入、PID、补偿配置和斜率历史下与修改前 ELF 对照。
    baseline = ModeFirmware(args.baseline)
    for mode, direction, speed, angle in itertools.product((2, 7), (-1, 1), (-40., 0., 40., 150., 200.), (0, 64, 32767)):
        observed = []
        for candidate in (baseline, f):
            candidate.prepare(mode, direction, speed, angle)
            candidate.run_mode(mode, speed, direction, angle)
            observed.append(candidate.output())
        check(f'原有{MODES[mode]}/dir={direction}/rpm={speed:g}/angle={angle}/保持',
              all(close(observed[0][key], observed[1][key]) for key in observed[0]),
              baseline=observed[0], current=observed[1])

    # 双向持续饱和时积分不得继续增长；前馈仍计入总电流和 PI 上下界。
    for mode, direction, sign in itertools.product((0, 7), (-1, 1), (-1, 1)):
        f.prepare(mode, direction, 0.)
        f.initialize_pid('speed_pid', kp=.02, ki=.0032)
        f.initialize_pid('g_pi_pos', kp=1., limit=350.)
        f.write('g_host_iq_limit_a', .04)
        f.write('g_host_speed_target_rpm', sign * 150.)
        limit = .9 if mode == 0 else .04
        for _ in range(200):
            if mode == 0:
                f.call('motor_system_run_position_mode', [('float', float(64 + sign * 150)), ('float', 0.)])
            else:
                f.run_mode(mode, 0., direction)
        actual, pid = f.output(), f.pid()
        check(f'{MODES[mode]}/dir={direction}/sign={sign}/饱和与条件积分',
              close(actual['iq_a'], sign * direction * limit) and abs(pid['integral']) < TOLERANCE and
              abs(actual['iq_a']) <= limit + TOLERANCE and
              close(pid['out_max'], limit - actual['friction_a'] - actual['cogging_a']) and
              close(pid['out_min'], -limit - actual['friction_a'] - actual['cogging_a']),
              integral_a=pid['integral'], **actual)

    # TORQUE 总输出补偿不能绕过配置的斜率；命令渐变历史与最终电流历史分开观测。
    for direction, sign in itertools.product((-1, 1), (-1, 1)):
        f.prepare(1, direction, sign * 40.)
        f.write('g_torque_current_a', sign * .1)
        f.write('g_host_iq_slew_a_per_s', 1.)
        f.write('g_host_iq_applied_a', 0.)
        f.write_float('g_foc_state', 164, 0.)
        previous = 0.
        for tick in range(1, 51):
            f.run_mode(1, sign * 40., direction)
            actual = f.output()['iq_a']
            check(f'TORQUE/dir={direction}/sign={sign}/tick={tick}/总输出斜率',
                  abs(actual - previous) <= .001 + TOLERANCE and abs(actual) <= .2 + TOLERANCE,
                  previous_a=previous, actual_a=actual)
            previous = actual
        f.write('g_host_iq_slew_a_per_s', 0.)
        f.write('g_torque_current_a', sign * .04)
        settled = []
        for _ in range(120):
            f.run_mode(1, sign * 40., direction)
            settled.append(f.output()['iq_a'])
        expected = sign * .04 + friction_model(sign * 40.) * direction + cogging_model(64, sign * 40.)
        check(f'TORQUE/dir={direction}/sign={sign}/补偿不累加到基础命令',
              all(close(value, expected) for value in settled) and
              close(f.read_float('g_host_iq_applied_a'), sign * .04),
              expected_a=expected, first_a=settled[0], final_a=settled[-1])

    for sign in (-1, 1):
        angle = 64 if sign > 0 else 0
        f.prepare(1, speed=sign * 40., angle=angle)
        f.write('g_input_source', 0)
        f.write('g_pot_target_filtered', 4095. if sign > 0 else 0.)
        f.run_mode(1, sign * 40., 1, angle)
        check(f'TORQUE/POT/sign={sign}/补偿后保持0.2A原边界',
              close(f.output()['iq_a'], sign * .2) and
              abs(f.output()['iq_a']) <= .2 + TOLERANCE, output=f.output())

    for mode, sign in itertools.product(range(1, 7), (-1, 1)):
        angle = 64 if sign > 0 else 0
        # 用实际速度建立明确超限的模型/阻尼目标；任意小位移与补偿可能抵消，不能假定必饱和。
        f.prepare(mode, speed=sign * 400., angle=angle)
        f.write('g_host_iq_limit_a', .01)
        f.write('g_torque_current_a', sign * 1.)
        if 4 <= mode <= 6:
            f.inject_position(angle_counts(sign * 10.))
        for _ in range(100):
            f.run_mode(mode, sign * 400., 1, angle)
        check(f'{MODES[mode]}/sign={sign}/补偿后总电流限幅',
              close(abs(f.output()['iq_a']), .01), output=f.output())

    # profile轨迹关闭真实寄生补偿，隔离虚拟力模型；补偿叠加另由上面的192组矩阵检查。
    def profile_prepare(mode, direction=1):
        f.prepare(mode, direction, 0., friction=False, cogging=False, custom_haptic=False)
        f.write('g_host_iq_limit_a', 1.)
        f.write('g_host_iq_applied_a', 0.)
        f.write_float('g_foc_state', 164, 0.)
        f.inject_position(0)

    def profile_counts_point(mode, counts, speed=0., direction=1, ticks=60):
        f.inject_position(counts)
        f.write_float('g_motor_system', 4, speed)
        for _ in range(ticks):
            f.call('motor_haptic_task', [('int8_t', direction), ('uint16_t', counts % COUNTS_PER_REV)])
        return f.output()['iq_a'] * direction

    def profile_point(mode, angle, speed=0., direction=1, ticks=60):
        return profile_counts_point(mode, angle_counts(angle), speed, direction, ticks)

    for direction in (-1, 1):
        profile_prepare(4, direction)
        for degrees in (-.1, 0., .1):
            force = profile_point(4, math.radians(degrees), direction=direction)
            check(f'SPRING/dir={direction}/中心死区{degrees}deg', abs(force) <= TOLERANCE, mechanical_iq_a=force)
        values = {}
        for degrees in (-60., -10., 10., 60.):
            force = profile_point(4, math.radians(degrees), direction=direction)
            error = remove_deadband(quantized_angle(math.radians(degrees)), math.radians(.15))
            expected = -.12 * error * (1. + min(abs(error) / (math.pi / 2.), 1.) ** 2)
            check(f'SPRING/dir={direction}/{degrees}deg/回中及独立模型', close(force, expected) and force * degrees < 0.,
                  expected_a=expected, mechanical_iq_a=force)
            values[degrees] = force
        check(f'SPRING/dir={direction}/大偏角渐硬',
              abs(values[60.]) / math.radians(59.85) > 1.2 * abs(values[10.]) / math.radians(9.85))
        for speed in (-10., 10.):
            force = profile_point(4, 0., speed, direction)
            expected = -(.002 + .12 * .025) * speed * 2. * math.pi / 60.
            check(f'SPRING/dir={direction}/阻尼反向{speed}rpm', close(force, expected) and force * speed < 0.)

        profile_prepare(5, direction)
        step = 2. * math.pi / 24.
        for degrees in (-.1, .1):
            force = profile_point(5, math.radians(degrees), direction=direction)
            check(f'DETENT/dir={direction}/档心静区{degrees}deg', abs(force) <= TOLERANCE)
        before = profile_point(5, .54 * step, direction=direction)
        after = profile_point(5, .56 * step, direction=direction)
        check(f'DETENT/dir={direction}/0.55步阈值跨档', before < -TOLERANCE and after > TOLERANCE,
              before_a=before, after_a=after)
        for fraction in (.52, .48, .50, .54, .46):
            force = profile_point(5, fraction * step, direction=direction)
            check(f'DETENT/dir={direction}/半步附近{fraction}迟滞防抖', force > TOLERANCE, mechanical_iq_a=force)
        returned = profile_point(5, .44 * step, direction=direction)
        check(f'DETENT/dir={direction}/反向越过阈值才回档', returned < -TOLERANCE)
        expected_center = 0
        for relative_steps in (6.3, -5.2):
            relative_counts = angle_counts(relative_steps * step)
            expected_center = detent_center(relative_counts, expected_center, 24)
            force = profile_point(5, relative_steps * step, direction=direction)
            error = remove_deadband((relative_counts * 24 - expected_center) * RAD_PER_COUNT / 24,
                                    min(step * .02, math.radians(.15)))
            expected = -.4 * error * (1. + min(abs(error) / (step / 2.), 1.) ** 2)
            check(f'DETENT/dir={direction}/一次跨多档{relative_steps}', close(force, expected),
                  expected_a=expected, mechanical_iq_a=force)
            aligned = profile_point(5, round(relative_steps) * step, direction=direction)
            check(f'DETENT/dir={direction}/跨多档后中心为整步', abs(aligned) <= TOLERANCE)

        boundary = (COUNTS_PER_REV + 55 * COUNTS_PER_REV // 100) // 24
        for sign, offset in itertools.product((-1, 1), (-1, 0, 1)):
            profile_prepare(5, direction)
            angle = sign * (boundary + offset) * RAD_PER_COUNT
            force = profile_point(5, angle, direction=direction)
            # 真实编码器count覆盖阈值两侧；原先±1e-5rad小于1count，不能作为不同传感器输入。
            expected_sign = -sign if offset <= 0. else sign
            check(f'DETENT/dir={direction}/sign={sign}/多档阈值offset_counts={offset}/重复拍稳定',
                  force * expected_sign > TOLERANCE, angle_rad=angle, mechanical_iq_a=force)

        boundary_cases = ((24, -1.1911888122558594), (24, 1.1911888122558594),
                          (24, -128.1638946533203), (24, 128.1638946533203),
                          (32, -32.11296844482422), (32, 32.11296844482422))
        for count, angle in boundary_cases:
            profile_prepare(5, direction)
            f.uc.mem_write(f.address('g_haptic_params') + 20, struct.pack('<H', count))
            profile_point(5, angle, direction=direction, ticks=1)
            first_center = f.detent_center_units(count)
            centers = []
            for _ in range(60):
                force = profile_point(5, angle, direction=direction, ticks=1)
                centers.append(f.detent_center_units(count))
            # 亚编码器计数误差窗口允许首拍选择相邻任一档；位置不动时不能在后续拍继续跳档。
            case_step = 2. * math.pi / count
            error = remove_deadband((angle_counts(angle) * count - first_center) * RAD_PER_COUNT / count,
                                    min(case_step * .02, math.radians(.15)))
            expected = -.4 * error * (1. + min(abs(error) / (case_step / 2.), 1.) ** 2)
            check(f'DETENT/dir={direction}/count={count}/shifted_threshold={angle}/首拍决定后档心与恢复力稳定',
                  all(center == first_center for center in centers) and close(force, expected),
                  first_center_units=first_center, last_center_units=centers[-1],
                  expected_iq_a=expected, mechanical_iq_a=force)

        for first, second in ((32760, 8), (8, 32760)):
            f.prepare(5, direction, 0., angle=first, friction=False, cogging=False, custom_haptic=False)
            f.call('motor_haptic_reset')
            f.inject_position(first)
            f.call('motor_control_task')
            reference = f.read_int64('s_reference_counts')
            f.stub('motor_encoder_get_raw_angle', second)
            f.call('motor_control_task')
            delta = (16. if second == 8 else -16.) * 2. * math.pi / 32768.
            error = remove_deadband(delta, math.radians(.15))
            expected = -.4 * error * (1. + (abs(error) / (step / 2.)) ** 2)
            check(f'DETENT/dir={direction}/真实反馈跨零{first}->{second}',
                  f.position_counts() - reference == (16 if second == 8 else -16) and
                  close(f.output()['iq_a'], expected * direction), expected_a=expected * direction, output=f.output())

        profile_prepare(6, direction)
        for side, speed in itertools.product((-1, 1), (-40., 0., 40.)):
            force = profile_point(6, side * math.radians(89.), speed, direction)
            check(f'LIMIT/dir={direction}/墙内side={side}/rpm={speed}/无虚拟力', abs(force) <= TOLERANCE)
        for side, penetration_deg in itertools.product((-1, 1), (.1, 1., 5., 10.)):
            angle = side * math.radians(90. + penetration_deg)
            idle = profile_point(6, angle, 0., direction)
            returning = profile_point(6, angle, -side * 40., direction)
            outward = profile_point(6, angle, side * 40., direction)
            penetration = abs(quantized_angle(angle)) - math.pi / 2.
            expected_elastic = -side * .8 * penetration * (1. + min(penetration / math.radians(10.), 2.) ** 2)
            expected_outward = expected_elastic - side * (.002 + .8 * .025) * (40. * 2. * math.pi / 60.) * min(
                penetration / math.radians(.5), 1.)
            check(f'LIMIT/dir={direction}/side={side}/penetration={penetration_deg}deg/向内墙力与返回无阻尼',
                  close(idle, expected_elastic) and close(returning, idle) and close(outward, expected_outward) and
                  idle * side < 0. and returning * side < 0. and outward * side < 0.,
                  idle_a=idle, returning_a=returning, outward_a=outward)

        for mode, amplitude in ((4, math.radians(30.)), (5, .4 * step), (6, math.radians(120.))):
            profile_prepare(mode, direction)
            f.write('g_host_iq_limit_a', .05)
            previous = 0.
            for sign in (1, -1):
                for tick in range(8):
                    force = profile_point(mode, sign * amplitude, direction=direction, ticks=1) * direction
                    check(f'{MODES[mode]}/dir={direction}/sign={sign}/tick={tick}/20A每秒与总限幅',
                          abs(force - previous) <= .020 + TOLERANCE and abs(force) <= .05 + TOLERANCE,
                          previous_a=previous, iq_a=force)
                    previous = force
                check(f'{MODES[mode]}/dir={direction}/sign={sign}/达到受限墙或恢复力', close(abs(force), .05))

        for mode in (4, 5, 6):
            profile_prepare(mode, direction)
            relative = math.radians(10.) if mode == 4 else .25 * step if mode == 5 else math.radians(100.)
            before = profile_point(mode, relative, direction=direction)
            center = f.read_int64('s_reference_counts')
            statuses = [f.call('motor_system_host_start') for _ in range(3)]
            after = profile_point(mode, relative, direction=direction)
            check(f'{MODES[mode]}/dir={direction}/重复START不重锁中心', statuses == [0, 0, 0] and
                  f.read_int64('s_reference_counts') == center and close(before, after) and abs(after) > TOLERANCE)
            f.call('motor_system_stop_control')
            f.inject_position(angle_counts(.8))
            status = f.call('motor_system_host_start')
            f.write('g_motor_system', 3)
            restarted = profile_point(mode, .8, direction=direction)
            check(f'{MODES[mode]}/dir={direction}/停机重启在新位置捕获', status == 0 and
                  f.read_int64('s_reference_counts') == angle_counts(.8) and abs(restarted) <= TOLERANCE)
        profile_prepare(4, direction)
        for mode, angle in ((5, .3), (6, .7), (4, 1.2)):
            f.write('g_control_mode', mode)
            force = profile_point(mode, angle, direction=direction)
            check(f'profile切换到{MODES[mode]}/dir={direction}/重捕获中心',
                  f.read_int64('s_reference_counts') == angle_counts(angle) and abs(force) <= TOLERANCE)

    for mode in (2, 3):
        f.prepare(mode, speed=40., friction=True, cogging=False)
        f.write('g_host_iq_applied_a', 0.)
        f.run_mode(mode, 40., 1)
        check(f'{MODES[mode]}/仍保持原1A每秒斜率', close(abs(f.output()['iq_a']), .001))

    # 运行入口必须在每种模式推进渐入；跨模式不停机仍连续，停机后重新由零启动。
    for mode in range(8):
        f.prepare(mode, speed=0.)
        f.call('motor_cogging_runtime_start')
        f.call('motor_control_task')
        check(f'{MODES[mode]}/运行入口首tick渐入', close(f.floating('motor_cogging_get_effective_gain'), .9 / 500))
        f.write('g_control_mode', (mode + 1) % 8)
        f.call('motor_control_task')
        check(f'{MODES[mode]}/切换模式继续渐入', close(f.floating('motor_cogging_get_effective_gain'), .9 * 2 / 500))
        f.write('g_motor_system', 0)
        f.call('motor_control_task')
        check(f'{MODES[mode]}/停机清零渐入与输出', f.floating('motor_cogging_get_effective_gain') == 0. and
              f.output()['iq_a'] == 0.)
        f.write('g_motor_system', 3)
        f.call('motor_control_task')
        check(f'{MODES[mode]}/重启重新渐入', close(f.floating('motor_cogging_get_effective_gain'), .9 / 500))

    # 特殊状态保持输出所有权；不调用普通补偿。强拖只观察开环边界，不执行寄存器写入。
    for state in (0, 1, 2, 4, 5):
        f.prepare(1, state=state)
        f.call('motor_control_task')
        check(f'特殊状态{state}/补偿不产生电流', f.output()['iq_a'] == 0. and
              f.floating('motor_cogging_get_effective_gain') == 0. and
              not f.calls.get('motor_feedforward_friction_compensation', 0) and
              not f.calls.get('motor_cogging_compensation', 0), output=f.output())
    f.prepare(1)
    f.write('g_force_drag_active', 1)
    f.write('g_force_drag_align_ticks', 2)
    f.call('motor_control_task')
    check('强拖开环接管且不叠加补偿', f.output()['iq_a'] == 0. and f.output()['friction_a'] == 0. and
          f.output()['cogging_a'] == 0. and f.calls.get('motor_open_loop_drive', 0) == 1 and
          not f.calls.get('motor_cogging_compensation', 0))

    # 启动真实标定会话，检查配置门控及停机恢复；模式3是已有的补偿验证专用例外。
    for cal_mode in range(6):
        f.prepare(7, state=0)
        f.set_config(limit=.01)
        original = f.config()
        status = f.call('motor_system_host_start_calibration', [('uint8_t', cal_mode), ('uint16_t', 512),
                        ('uint8_t', 3), ('uint8_t', 0), ('uint8_t', 0)])
        check(f'标定{cal_mode}/真实会话启动', status == 0, status=status)
        check(f'标定{cal_mode}/补偿配置隔离', f.config()['enabled'] == int(cal_mode == 3) and
              f.read_byte('g_friction_config') == int(cal_mode < 4))
        f.write('g_motor_system', 3)
        f.write('g_host_speed_slew_rpm_per_s', 0.)
        f.write('g_host_speed_target_rpm', 15.)
        f.float_stub('motor_speed_estimator_update_adaptive', 0.)
        for _ in range(4):
            f.call('motor_control_task')
        output = f.output()
        if cal_mode != 3:
            check(f'标定{cal_mode}/齿槽不污染采集', output['cogging_a'] == 0., output=output)
        if cal_mode == 4:
            check('摩擦标定/摩擦项为零', output['friction_a'] == 0., output=output)
        if cal_mode == 5:
            # 真实状态机到位后先撤销已保存的摩擦助力，再计算稳定窗口。
            check('逐点移动阶段保留原摩擦助力', f.call('motor_calibration_point_assist') == 1 and
                  abs(output['friction_a']) > 1e-6 and output['cogging_a'] == 0., output=output)
            f.initialize_pid('g_pi_pos', kp=.5, limit=30.)
            for _ in range(2100):
                # CAL_ZERO稳定后还会转到首个采集点，反馈跟随实际会话目标而非固定预置角。
                target = round(f.floating('motor_calibration_point_target'))
                f.stub('motor_encoder_get_raw_angle', target)
                f.call('motor_control_task')
            f.call('motor_calibration_get_status', [('uint32_t', 0x10004000)])
            phase = bytes(f.uc.mem_read(0x10004000 + 12, 1))[0]
            check('逐点正式采集阶段摩擦及齿槽均为零', phase == 2 and
                  not f.call('motor_calibration_point_assist') and
                  f.output()['friction_a'] == 0. and f.output()['cogging_a'] == 0., phase=phase, output=f.output())
        f.call('motor_system_stop_control')
        restored = f.config()
        check(f'标定{cal_mode}/停机恢复原配置', restored == original and f.read_byte('g_friction_config') == 1 and
              f.read_byte('g_cal_saved_valid') == 0, restored=restored)

    # 快进到远大于4096rad的绝对累计位置，再用真实原始编码器推进1count；控制与相对遥测须保持分辨率。
    for mode, direction, sign in itertools.product((4, 5, 6), (-1, 1), (-1, 1)):
        profile_prepare(mode, direction)
        f.call('motor_haptic_reset')
        base = sign * ((1 << 40) + 12345)
        f.inject_position(base)
        f.call('motor_control_task')
        check(f'{MODES[mode]}/dir={direction}/base={base}/大绝对位置重新锁中心',
              f.position_counts() == base and f.read_int64('s_reference_counts') == base and
              f.relative_counts() == 0 and abs(f.output()['iq_a']) <= TOLERANCE)
        f.stub('motor_encoder_get_raw_angle', (base + sign) % COUNTS_PER_REV)
        f.call('motor_control_task')
        f.call('motor_system_get_control_snapshot', [('uint32_t', 0x10005000)])
        relative_rad = struct.unpack('<f', bytes(f.uc.mem_read(0x10005000 + 16, 4)))[0]
        check(f'{MODES[mode]}/dir={direction}/base={base}/一计数与相对角不丢失',
              f.position_counts() == base + sign and f.relative_counts() == sign and
              relative_rad * sign > 0. and close(relative_rad, sign * RAD_PER_COUNT) and
              abs(f.output()['iq_a']) <= TOLERANCE, relative_rad=relative_rad)
        offset = 8192 + 32 if mode == 6 else 32
        force_a = profile_counts_point(mode, base + sign * offset, direction=direction)
        force_b = profile_counts_point(mode, base + sign * (offset + 1), direction=direction)
        check(f'{MODES[mode]}/dir={direction}/base={base}/死区或墙外一计数仍改变恢复力',
              (force_b - force_a) * sign < -1e-5 and
              f.relative_counts() == sign * (offset + 1), force_a=force_a, force_b=force_b)

    # 连续1024圈的真实解回绕与整数档内相位；每拍位移小于半圈，正反向都不积累档位浮点误差。
    for direction, sign in itertools.product((-1, 1), (-1, 1)):
        profile_prepare(5, direction)
        for tick in range(1, 4097):
            f.stub('motor_encoder_get_raw_angle', (sign * tick * 8192) % COUNTS_PER_REV)
            f.call('motor_control_task')
        total = sign * 1024 * COUNTS_PER_REV
        check(f'DETENT/dir={direction}/sign={sign}/1024圈累计与档内相位精确',
              f.position_counts() == total and f.relative_counts() == total and
              f.read_int64('s_detent_error_units') == 0, counts=f.position_counts())
        before = profile_counts_point(5, total + sign * 32, direction=direction)
        after = profile_counts_point(5, total + sign * 33, direction=direction)
        check(f'DETENT/dir={direction}/sign={sign}/1024圈后档内一计数仍有效',
              (after - before) * sign < -5e-5 and f.read_int64('s_detent_error_units') == sign * 33 * 24,
              before_a=before, after_a=after, phase_units=f.read_int64('s_detent_error_units'))

    # 实际滚动窗函数跨INT32正负界限和UINT32回绕；只替换原始编码器，不替换测速计算。
    for seed, sign, step_counts in itertools.product(
            (0x7FFFFFF5, 0x8000000A, 0xFFFFFFF5, 10), (-1, 1), (3, 27, 55)):
        window = {3: 20, 27: 10, 55: 5}[step_counts]
        f.prepare(1, speed=0., friction=False, cogging=False)
        f.float_stubs.pop(f.address('motor_speed_estimator_update_adaptive') & ~1, None)
        f.call('motor_speed_estimator_init', [('float', 1.)])
        f.stub('motor_encoder_get_raw_angle', seed % COUNTS_PER_REV)
        f.floating('motor_speed_estimator_update_adaptive', [('float', 0.)])
        f.uc.mem_write(f.address('g_speed_unwrapped_counts'), struct.pack('<I', seed))
        f.uc.mem_write(f.address('g_speed_position_history'), struct.pack('<20I', *([seed] * 20)))
        expected = sign * step_counts * 60. / (COUNTS_PER_REV * .001)
        observations = []
        for tick in range(1, 41):
            f.stub('motor_encoder_get_raw_angle', (seed + sign * step_counts * tick) % COUNTS_PER_REV)
            observations.append(f.floating('motor_speed_estimator_update_adaptive', [('float', expected)]))
        actual_position = struct.unpack('<I', bytes(f.uc.mem_read(f.address('g_speed_unwrapped_counts'), 4)))[0]
        check(f'滚动测速/seed={seed:#x}/sign={sign}/step={step_counts}/跨界窗口{window}ms',
              all(math.isclose(value, expected, abs_tol=2e-4, rel_tol=1e-6) for value in observations) and
              f.call('motor_speed_estimator_get_window_ticks') == window and
              actual_position == (seed + sign * step_counts * 40) % (1 << 32),
              expected_rpm=expected, first_rpm=observations[0], final_rpm=observations[-1], position=actual_position)

    failures = [item for item in results if not item['passed']]
    report = {'status': 'failed_offline' if failures else 'passed_offline',
              'assertion_count': len(results), 'failure_count': len(failures), 'elf': str(args.elf.resolve()),
              'elf_sha256': hashlib.sha256(args.elf.read_bytes()).hexdigest(),
              'baseline_elf': str(args.baseline.resolve()),
              'baseline_sha256': hashlib.sha256(args.baseline.read_bytes()).hexdigest(),
              'assertions': results,
              'method': 'Unicorn执行真实ARM模式、PID、补偿及标定；传感器原始输入、时间和物理输出使用边界桩。',
              'limitations': ['未烧录或连接MCU，未验证ADC/PWM/中断时序、物理力矩与各模式手感。',
                              '测试不验证摩擦模型参数与实机的一致性，也不能替代触觉模式稳定性和能量验证。']}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'{"FAIL" if failures else "PASS"}: {len(results)} assertions, {len(failures)} failures; {args.report}')
    if failures:
        raise AssertionError('; '.join(item['assertion'] for item in failures))


if __name__ == '__main__':
    main()

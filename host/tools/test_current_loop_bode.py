"""验证离散频响与独立逐拍仿真、极点判稳及输入/报告边界。"""

import csv
from contextlib import contextmanager
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

import current_loop_bode as bode


@contextmanager
def workspace_temporary_directory():
    """临时文件放在工作区；清理前确认最终绝对路径仍在指定临时目录内。"""
    parent = (bode.ROOT / "tmp").resolve()
    parent.mkdir(exist_ok=True)
    temporary = tempfile.TemporaryDirectory(dir=parent)
    target = Path(temporary.name).resolve()
    if not target.is_relative_to(parent):
        raise RuntimeError("临时目录超出预期工作区")
    try:
        yield target
    finally:
        if target.resolve().is_relative_to(parent):
            temporary.cleanup()


class CurrentLoopBodeTests(unittest.TestCase):
    def test_frequency_response_matches_time_simulation(self):
        """独立递推包含电机、反馈滤波、积分次序及 0/1/2 拍额外延迟。"""
        resistance, inductance, fs, alpha = 5.16, 0.00238, 20000.0, 0.25
        kp = inductance * 2 * math.pi * 150
        ki = resistance * 2 * math.pi * 150 * 0.8
        a = math.exp(-resistance / (fs * inductance))
        b = (1 - a) / resistance
        count, window = 12000, 4000
        for delay in (0, 1, 2):
            for frequency in (50.0, 400.0):
                with self.subTest(delay=delay, frequency=frequency):
                    reference = np.cos(2 * np.pi * frequency * np.arange(count) / fs)
                    voltage, current, feedback = np.zeros((3, count))
                    integral = 0.0
                    for k in range(count):
                        source = k - 1 - delay
                        previous_current = current[k - 1] if k else 0.0
                        previous_feedback = feedback[k - 1] if k else 0.0
                        current[k] = a * previous_current + b * (voltage[source] if source >= 0 else 0.0)
                        feedback[k] = previous_feedback + alpha * (current[k] - previous_feedback)
                        error = reference[k] - feedback[k]
                        integral += ki * error / fs
                        voltage[k] = kp * error + integral
                    phasor = np.exp(-2j * np.pi * frequency * np.arange(count - window, count) / fs)
                    reference_phasor = np.dot(reference[-window:], phasor)
                    _, predicted_feedback, predicted_current, _ = bode.response(
                        np.array([frequency]), resistance, inductance, fs, alpha, kp, ki, delay)
                    for signal, predicted in ((feedback, predicted_feedback), (current, predicted_current)):
                        observed = np.dot(signal[-window:], phasor) / reference_phasor
                        self.assertAlmostEqual(abs(observed - predicted[0]), 0.0, places=9)

    def test_poles_reject_unstable_high_bandwidth(self):
        """判稳不能仅凭出现一个看似正常的幅频交越。"""
        f = np.geomspace(0.1, 9999, 12000)
        for bandwidth, delay, expected in ((75, 1, True), (2000, 1, True), (2000, 2, False)):
            loop, feedback, _, radius = bode.response(
                f, 5.16, 0.00238, 20000, 0.25,
                0.00238 * 2 * math.pi * bandwidth, 5.16 * 2 * math.pi * bandwidth * 0.8, delay)
            result = bode.summarize(f, loop, feedback, radius)
            self.assertEqual(result["stable"], expected)
            if not expected:
                self.assertIsNone(result["measured_bandwidth_hz"])

    def test_ideal_rl_pi_limit(self):
        """弱离散化影响下，未滤波且精确对消的结果趋于一阶闭环。"""
        frequency = np.array([10.0, 100.0, 500.0])
        wc = 2 * math.pi * 100
        _, actual, _, _ = bode.response(frequency, 5.16, 0.00238, 2e8, 1.0,
                                      0.00238 * wc, 5.16 * wc, 0)
        expected = wc / (2j * math.pi * frequency + wc)
        np.testing.assert_allclose(actual, expected, rtol=2e-5)

    def test_measured_csv_validation(self):
        """拒绝重复频率、非有限值和普通遥测字段。"""
        with workspace_temporary_directory() as directory:
            path = Path(directory) / "measured.csv"
            for content in ("time_s,iq_actual_a\n0,1\n", "frequency_hz,gain_db,phase_deg\n10,0,0\n10,-1,-5\n",
                            "frequency_hz,gain_db,phase_deg\n10,nan,0\n20,-1,-5\n"):
                path.write_text(content, encoding="utf-8")
                with self.assertRaises(ValueError):
                    bode.load_measured(path)
            path.write_text("frequency_hz,gain_db,phase_deg\n20,-1,-5\n10,0,0\n", encoding="utf-8")
            self.assertEqual(bode.load_measured(path)[:, 0].tolist(), [10, 20])

    def test_cli_and_report(self):
        """验证真实命令入口、中文报告、SVG 格式与闭环 CSV 叠加。"""
        with workspace_temporary_directory() as directory:
            output = Path(directory) / "report"
            measured = Path(directory) / "measured.csv"
            measured.write_text("frequency_hz,gain_db,phase_deg\n10,-0.1,-10\n100,-5,-60\n", encoding="utf-8")
            command = [sys.executable, str(Path(bode.__file__)), "--points", "256", "--output", str(output),
                       "--measured", str(measured)]
            result = subprocess.run(command, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("离线模型预测", (output / "report.html").read_text(encoding="utf-8"))
            for path in output.glob("*.svg"):
                ET.parse(path)
            self.assertEqual(len(list(output.glob("*.svg"))), 4)
            with (output / "model_response.csv").open(encoding="utf-8-sig", newline="") as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 3 * 256)
            for option in (["--alpha", "nan"], ["--delays", "0.5"], ["--fmax", "10000"],
                           ["--bandwidths", "49"], ["--inductance", "0"]):
                rejected = subprocess.run(command + option, capture_output=True)
                self.assertNotEqual(rejected.returncode, 0)


if __name__ == "__main__":
    unittest.main()

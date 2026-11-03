"""离线电流环频响分析；不连接串口、不修改固件、不启动电机。

依赖 NumPy。运行说明与实测 CSV 格式见 host/README.md。
"""

import argparse
import csv
import html
import json
import math
from pathlib import Path
import re
import webbrowser

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
COLORS = ("#2563eb", "#ea580c", "#059669", "#9333ea", "#dc2626", "#0891b2")


def read_defaults():
    """从当前仓库读取整定输入，避免把历史默认值当作当前固件配置。"""
    config = (ROOT / "firmware/config/motor_config.h").read_text(encoding="utf-8-sig")
    context = (ROOT / "firmware/app/private/motor_context.h").read_text(encoding="utf-8-sig")
    loop = (ROOT / "firmware/control/motor_current_loop.c").read_text(encoding="utf-8-sig")

    def macro(source, name):
        match = re.search(r"^\s*#define\s+" + name + r"\s+([0-9.eE+-]+)f?\b", source, re.M)
        if match is None:
            raise ValueError(f"无法读取 {name}，请核查配置或更新脚本")
        return float(match[1])

    # 单相模型按当前源码的相间值乘 0.5；转换规则变化时拒绝静默使用旧假设。
    for name in ("RESISTANCE_OHM", "INDUCTANCE_H"):
        expression = f"(MOTOR_INTERPHASE_{name} * 0.5f)"
        if expression not in config:
            raise ValueError("固件的相间/单相换算规则已变化，请更新脚本")
    tune = loop.split("void motor_current_loop_auto_tune_pid_with_bandwidth(", 1)[1]
    dt = re.search(r"float\s+dt\s*=\s*([0-9.eE+-]+)f", tune)
    if dt is None:
        raise ValueError("无法读取整定函数中的 dt")
    return dict(
        resistance=macro(config, "MOTOR_INTERPHASE_RESISTANCE_OHM") * 0.5,
        inductance=macro(config, "MOTOR_INTERPHASE_INDUCTANCE_H") * 0.5,
        sample_rate=1.0 / float(dt[1]),
        alpha=macro(loop, "ADC_FILTER_ALPHA"),
        kp_scale=macro(loop, "KP_DAMPING"),
        ki_scale=macro(loop, "KI_DAMPING"),
        bandwidth=macro(context, "MOTOR_CURRENT_LOOP_CONTROL_BW_HZ"),
    )


def number_list(value):
    """解析逗号分隔的有限数值。"""
    try:
        result = [float(item) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("请使用逗号分隔的数值") from exc
    if not result or not all(math.isfinite(item) for item in result):
        raise argparse.ArgumentTypeError("参数必须是有限数值")
    return list(dict.fromkeys(result))


def response(frequency, resistance, inductance, sample_rate, alpha, kp, ki, delay):
    """静止转子单轴线性模型；ZOH 已含在 RL 模型内，delay 只表示额外整拍延迟。"""
    ts = 1.0 / sample_rate
    q = np.exp(-2j * np.pi * frequency * ts)
    a = math.exp(-resistance * ts / inductance)
    b = -math.expm1(-resistance * ts / inductance) / resistance
    plant = b * q / (1.0 - a * q)
    # 固件先更新本拍积分，再计算本拍输出；仅适用于未饱和的小信号路径。
    controller = kp + ki * ts / (1.0 - q)
    feedback = alpha / (1.0 - (1.0 - alpha) * q)
    forward = controller * plant * q**delay
    loop = forward * feedback
    measured = loop / (1.0 + loop)
    physical = forward / (1.0 + loop)

    # 按 q=z^-1 排列的特征多项式，可直接作为 z 的降幂系数求根。
    denominator = np.polynomial.polynomial.polymul([1, -1], [1, -a])
    denominator = np.polynomial.polynomial.polymul(denominator, [1, -(1.0 - alpha)])
    characteristic = np.zeros(max(len(denominator), delay + 3))
    characteristic[:len(denominator)] = denominator
    characteristic[delay + 1] += b * alpha * (kp + ki * ts)
    characteristic[delay + 2] -= b * alpha * kp
    radius = float(np.max(np.abs(np.roots(characteristic))))
    return loop, measured, physical, radius


def magnitude_db(values):
    """转换为幅值 dB，避免零幅值产生无穷数。"""
    return 20.0 * np.log10(np.maximum(np.abs(values), 1e-300))


def phase_deg(values):
    """沿频率轴展开相位，单位度。"""
    return np.rad2deg(np.unwrap(np.angle(values)))


def crossings(frequency, values, level):
    """在对数频率轴上线性插值所有穿越点，不只选择第一个。"""
    result = []
    for index in np.flatnonzero((values[:-1] - level) * (values[1:] - level) <= 0):
        if values[index + 1] == values[index]:
            continue
        weight = (level - values[index]) / (values[index + 1] - values[index])
        point = math.exp(np.log(frequency[index]) * (1 - weight)
                         + np.log(frequency[index + 1]) * weight)
        if not result or abs(point - result[-1]) > point * 1e-9:
            result.append(point)
    return result


def summarize(frequency, loop, measured, radius):
    """裕度来自开环；闭环带宽仅在极点稳定时报告。未找到穿越点不等于无限裕度。"""
    logf = np.log(frequency)
    db, phase = magnitude_db(loop), phase_deg(loop)
    gain_crossings = crossings(frequency, db, 0.0)
    margins = [dict(frequency_hz=f, phase_margin_deg=float(180 + np.interp(np.log(f), logf, phase)))
               for f in gain_crossings]
    gain_margins = []
    for level in np.arange(-180.0, min(-180.0, float(phase.min())) - 1.0, -360.0):
        for f in crossings(frequency, phase, level):
            gain_margins.append(dict(frequency_hz=f,
                                     gain_margin_db=float(-np.interp(np.log(f), logf, db))))
    closed_db = magnitude_db(measured)
    cutoff = crossings(frequency, closed_db, -10 * math.log10(2))
    stable = radius < 1.0 - 1e-9
    return dict(stable=stable, max_pole_radius=radius, gain_crossings=margins,
                phase_crossings=gain_margins,
                measured_bandwidth_hz=cutoff[0] if stable and cutoff else None,
                measured_peak_db=float(closed_db.max()) if stable else None)


def load_measured(path):
    """读取已完成幅相提取的实测闭环数据，不把普通遥测或模型结果当作扫频测量。"""
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = ("frequency_hz", "gain_db", "phase_deg")
        if not reader.fieldnames or not set(required).issubset(reader.fieldnames):
            raise ValueError("实测 CSV 必须包含 frequency_hz,gain_db,phase_deg")
        try:
            rows = [[float(row[key]) for key in required] for row in reader]
        except (TypeError, ValueError) as exc:
            raise ValueError("实测 CSV 存在缺失或非数值字段") from exc
    data = np.asarray(rows, dtype=float)
    if data.ndim != 2 or len(data) < 2 or not np.isfinite(data).all():
        raise ValueError("实测 CSV 至少需要两个有效频点，且不能有 NaN/Inf")
    data = data[np.argsort(data[:, 0])]
    if np.any(data[:, 0] <= 0) or np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError("实测频率必须为正，且不能重复")
    return data


def svg_panel(series, title, ylabel, horizontal=None):
    """使用标准 SVG 绘制对数频率图，无需 Matplotlib 或网络资源。"""
    width, height = 960, 350
    left, top, right, bottom = 85, 42, 25, 55
    plot_w, plot_h = width - left - right, height - top - bottom
    xmin = min(float(np.log10(x).min()) for _, x, _, _ in series)
    xmax = max(float(np.log10(x).max()) for _, x, _, _ in series)
    ymin = min(float(y.min()) for _, _, y, _ in series)
    ymax = max(float(y.max()) for _, _, y, _ in series)
    if horizontal is not None:
        ymin, ymax = min(ymin, horizontal), max(ymax, horizontal)
    padding = max(3.0, (ymax - ymin) * 0.08)
    ymin, ymax = ymin - padding, ymax + padding

    def px(value):
        return left + (np.log10(value) - xmin) / (xmax - xmin) * plot_w

    def py(value):
        return top + (ymax - value) / (ymax - ymin) * plot_h

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
             f'role="img" aria-label="{html.escape(title, quote=True)}">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<g font-family="Segoe UI,Microsoft YaHei,sans-serif" font-size="13" fill="#334155">',
             f'<text x="{left}" y="24" font-size="18">{html.escape(title)}</text>']
    for exponent in range(math.floor(xmin), math.ceil(xmax) + 1):
        for multiplier in (1, 2, 5):
            value = multiplier * 10.0**exponent
            if xmin <= math.log10(value) <= xmax:
                x = px(value)
                parts.extend([f'<path d="M{x:.2f},{top} v{plot_h}" stroke="#e2e8f0"/>',
                              f'<text x="{x:.2f}" y="{height - bottom + 22}" '
                              f'text-anchor="middle">{value:g}</text>'])
    for value in np.linspace(ymin, ymax, 6):
        y = py(value)
        parts.extend([f'<path d="M{left},{y:.2f} h{plot_w}" stroke="#e2e8f0"/>',
                      f'<text x="{left - 9}" y="{y + 4:.2f}" text-anchor="end">{value:.1f}</text>'])
    if horizontal is not None:
        parts.append(f'<path d="M{left},{py(horizontal):.2f} h{plot_w}" '
                     'stroke="#64748b" stroke-dasharray="6 5"/>')
    for label, frequency, values, color in series:
        points = " ".join(f"{px(x):.2f},{py(y):.2f}" for x, y in zip(frequency, values))
        parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.8">'
                     f'<title>{html.escape(label)}</title></polyline>')
    parts.extend([f'<text x="{left + plot_w / 2}" y="{height - 10}" text-anchor="middle">频率 / Hz（对数轴）</text>',
                  f'<text transform="translate(18,{top + plot_h / 2}) rotate(-90)" '
                  f'text-anchor="middle">{html.escape(ylabel)}</text>', '</g></svg>'])
    return "\n".join(parts)


def write_report(output, args, defaults, cases, measured):
    """生成独立 HTML、SVG、频响 CSV 和参数/指标 JSON。"""
    output.mkdir(parents=True, exist_ok=True)
    specs = [("open_magnitude", "模型开环环路增益", "幅值 / dB", "loop", magnitude_db, 0),
             ("open_phase", "模型开环相位", "相位 / °", "loop", phase_deg, -180),
             ("closed_magnitude", "闭环：PI 目标 → 滤波后反馈电流", "幅值 / dB", "measured", magnitude_db, -10 * math.log10(2)),
             ("closed_phase", "闭环：PI 目标 → 滤波后反馈电流", "相位 / °", "measured", phase_deg, None)]
    panels = []
    for filename, title, ylabel, key, transform, reference in specs:
        series = [(case["label"], case["frequency"], transform(case[key]), case["color"]) for case in cases]
        if measured is not None and key == "measured":
            column = 1 if filename.endswith("magnitude") else 2
            values = measured[:, column] if column == 1 else np.rad2deg(np.unwrap(np.deg2rad(measured[:, 2])))
            series.append(("实测闭环 CSV", measured[:, 0], values, "#111827"))
        svg = svg_panel(series, title, ylabel, reference)
        (output / f"{filename}.svg").write_text(svg, encoding="utf-8")
        panels.append(svg)

    def display(value):
        return "未获得" if value is None else f"{value:.2f}"

    rows, legends = [], []
    summaries = []
    with (output / "model_response.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["tuning_hz", "extra_delay_samples", "frequency_hz", "loop_gain_db", "loop_phase_deg",
                         "feedback_gain_db", "feedback_phase_deg", "physical_gain_db", "physical_phase_deg"])
        for case in cases:
            summary = case["summary"]
            pm = summary["gain_crossings"]
            gm = summary["phase_crossings"]
            rows.append("<tr>" + "".join(f"<td>{html.escape(str(value))}</td>" for value in (
                case["label"], "稳定" if summary["stable"] else "不稳定/临界",
                f'{summary["max_pole_radius"]:.6f}',
                "；".join(f'{p["frequency_hz"]:.1f} Hz / {p["phase_margin_deg"]:.1f}°' for p in pm) or "未找到",
                "；".join(f'{p["gain_margin_db"]:.1f} dB' for p in gm) or "未找到",
                display(summary["measured_bandwidth_hz"]))) + "</tr>")
            legends.append(f'<span style="color:{case["color"]}">● {html.escape(case["label"])}</span>')
            summaries.append(dict(tuning_hz=case["bandwidth"], extra_delay_samples=case["delay"],
                                  kp=case["kp"], ki=case["ki"], **summary))
            arrays = [case["frequency"]]
            for key in ("loop", "measured", "physical"):
                arrays.extend((magnitude_db(case[key]), phase_deg(case[key])))
            writer.writerows((case["bandwidth"], case["delay"], *row) for row in zip(*arrays))
    if measured is not None:
        legends.append('<span>● 实测闭环 CSV（黑色；不是开环裕度）</span>')
    parameters = {key: getattr(args, key) for key in ("resistance", "inductance", "sample_rate", "alpha")}
    metadata = dict(kind="offline_linear_model_prediction", firmware_defaults=defaults,
                    parameters=parameters, cases=summaries,
                    measured_csv=str(args.measured.resolve()) if args.measured else None,
                    assumptions=["静止转子单轴；无 dq 耦合、反电动势变化、机械运动或电压补偿",
                                 "未饱和；忽略 ADC 量化、噪声及电流过零非线性",
                                 "RL 精确 ZOH 已包含；额外延迟为未经上板确认的整拍假设",
                                 "闭环输出是滤波后反馈；物理电流响应另列于 CSV",
                                 "极点判稳；裕度仅搜索显示频段，未找到不表示无限裕度"])
    (output / "summary.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    document = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>电流环 Bode 分析</title>
<style>body{{font:15px/1.65 "Segoe UI","Microsoft YaHei",sans-serif;max-width:1100px;margin:30px auto;padding:0 18px;color:#172033;background:#f8fafc}}
h1{{font-size:27px}}svg{{width:100%;height:auto}}.notice,section{{background:white;padding:18px;border:1px solid #dbe3ed;border-radius:9px;margin:18px 0}}
table{{border-collapse:collapse;width:100%;font-size:13px}}td,th{{padding:9px;border-bottom:1px solid #ddd;text-align:left}}.table{{overflow-x:auto}}.legend{{display:flex;gap:15px;flex-wrap:wrap}}</style>
<h1>电流环 Bode 分析 · 离线模型预测</h1>
<div class="notice"><b>本程序不连接硬件。模型图不是实测结果，整定参数也不等于实际闭环带宽。</b>
<p>R = {args.resistance:g} Ω；L = {args.inductance:g} H；采样率 = {args.sample_rate:g} Hz；反馈 α = {args.alpha:g}。
PI：Kp = L·2πf·{defaults['kp_scale']:g}，Ki = R·2πf·{defaults['ki_scale']:g}。</p>
<p>RL 使用精确 ZOH 离散模型；额外延迟以采样周期为单位，不再重复添加半拍 ZOH。
这些延迟是假设，实际 PWM 生效时刻仍需测量。不稳定模型的闭环曲线仅是代数频响，不能视为可实现的稳态跟踪。</p>
<p>假设转子静止、单轴、未饱和；未包含机械动态、角度误差、dq 耦合及功率级非线性。R/L 默认来自规格配置。
反馈滤波在相电流域实现；静止角度下才可等效为此单轴模型。</p></div>
<div class="legend">{''.join(legends)}</div>
<section class="table"><table><tr><th>模型案例</th><th>极点判稳</th><th>最大极点模</th><th>0 dB 交越 / PM</th><th>GM</th><th>反馈 −3 dB / Hz</th></tr>{''.join(rows)}</table>
<p>裕度搜索范围：{args.fmin:g}～{args.fmax:g} Hz。“未找到”不表示无限裕度；全部交越点另见 JSON。−3 dB 相对单位 DC 增益。</p></section>
{''.join('<section>' + panel + '</section>' for panel in panels)}
<p>同目录包含独立 SVG、model_response.csv 与 summary.json。物理电流与滤波后反馈电流的响应不同，均已导出至 CSV。
实测 CSV 应为与图中相同定义的闭环幅相；普通遥测文件不能直接代替它。</p></html>'''
    report = output / "report.html"
    report.write_text(document, encoding="utf-8")
    return report


def main():
    """校验输入并执行离线分析，错误输入不生成部分报告。"""
    defaults = read_defaults()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bandwidths", type=number_list, default=[defaults["bandwidth"], 150.0, 300.0],
                        help="PI 整定参数 Hz，逗号分隔；固件支持 50～2000")
    parser.add_argument("--delays", type=number_list, default=[1.0], help="额外整拍延迟假设，如 0,1,2；默认 1")
    for name in ("resistance", "inductance", "sample_rate", "alpha"):
        parser.add_argument("--" + name.replace("_", "-"), type=float, default=defaults[name])
    parser.add_argument("--fmin", type=float, default=1.0)
    parser.add_argument("--fmax", type=float, default=None, help="默认采样率的 0.45 倍")
    parser.add_argument("--points", type=int, default=4096)
    parser.add_argument("--measured", type=Path, help="可选实测闭环幅相 CSV")
    parser.add_argument("--output", type=Path, default=ROOT / "tmp/current_loop_bode")
    parser.add_argument("--show", action="store_true", help="生成后在默认浏览器打开报告")
    args = parser.parse_args()
    if args.fmax is None:
        args.fmax = 0.45 * args.sample_rate
    values = [args.resistance, args.inductance, args.sample_rate, args.alpha, args.fmin, args.fmax]
    if not all(math.isfinite(v) and v > 0 for v in values):
        parser.error("R、L、采样率、alpha 和频率必须为有限正数")
    if args.alpha > 1 or not args.fmin < args.fmax < args.sample_rate / 2:
        parser.error("要求 alpha≤1，且 0<fmin<fmax<采样率/2")
    if not 128 <= args.points <= 100000:
        parser.error("points 必须在 128～100000 之间")
    if any(not 50 <= bw <= 2000 for bw in args.bandwidths):
        parser.error("整定参数必须在固件支持的 50～2000 Hz 内")
    if any(delay < 0 or delay > 10 or delay != int(delay) for delay in args.delays):
        parser.error("额外延迟必须为 0～10 之间的整数采样周期")
    if len(args.bandwidths) * len(args.delays) > 24:
        parser.error("最多同时比较 24 个模型案例")
    measured = load_measured(args.measured) if args.measured else None
    frequency = np.geomspace(args.fmin, args.fmax, args.points)
    cases = []
    for bandwidth in args.bandwidths:
        for delay in args.delays:
            kp = args.inductance * 2 * math.pi * bandwidth * defaults["kp_scale"]
            ki = args.resistance * 2 * math.pi * bandwidth * defaults["ki_scale"]
            loop, feedback, physical, radius = response(frequency, args.resistance, args.inductance,
                                                       args.sample_rate, args.alpha, kp, ki, int(delay))
            summary = summarize(frequency, loop, feedback, radius)
            label = f"整定 {bandwidth:g} Hz / 额外延迟 {int(delay)} 拍"
            case_count = len(args.bandwidths) * len(args.delays)
            color = (COLORS[len(cases)] if case_count <= len(COLORS)
                     else f"hsl({len(cases) * 360 / case_count:.1f},70%,38%)")
            cases.append(dict(label=label, bandwidth=bandwidth, delay=int(delay), kp=kp, ki=ki,
                              color=color, frequency=frequency,
                              loop=loop, measured=feedback, physical=physical, summary=summary))
            pm = ", ".join(f'{p["phase_margin_deg"]:.1f} deg' for p in summary["gain_crossings"]) or "N/A"
            print(f"tuning={bandwidth:g}Hz extra_delay={int(delay)}Ts Kp={kp:.6g} Ki={ki:.6g} "
                  f"stable={summary['stable']} PM={pm} feedback_BW={summary['measured_bandwidth_hz']}")
    report = write_report(args.output, args, defaults, cases, measured)
    print(f"OFFLINE MODEL ONLY / 离线模型，不代表实机验证\nReport: {report.resolve()}")
    if args.show:
        webbrowser.open(report.resolve().as_uri())


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, IndexError) as error:
        raise SystemExit(f"分析失败：{error}") from error

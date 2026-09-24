"""Read-only motor log analysis; outputs reproducible numerical summaries.

Usage: python analyze_motor_history.py --log-root D:/ --output <directory>
The inputs are never modified. Channel identities are resolved by full suffix,
not bare CH numbers, because the historical telemetry schemas differ.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd


def channel(d, suffix):
    candidates = [c for c in d.columns if c.startswith('CH') and
                  c.split('_', 1)[1] == suffix]
    return d[candidates[0]].to_numpy(float) if len(candidates) == 1 else None


def describe(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if not len(x):
        return None
    return dict(n=len(x), mean=float(x.mean()), median=float(np.median(x)), std=float(x.std()),
                min=float(x.min()), p01=float(np.quantile(x, .01)),
                p99=float(np.quantile(x, .99)), max=float(x.max()))


def metrics(d):
    result = {'rows': len(d), 'start_s': float(d.time_s.iloc[0]),
              'end_s': float(d.time_s.iloc[-1])}
    for suffix in ['mechanical_speed_rpm', 'iq_actual_A', 'id_actual_A',
                   'iq_pi_target_A', 'iq_applied_target_A', 'speed_loop_iq_A',
                   'uq_pi_output_V', 'target_speed_rpm', 'friction_iq_A',
                   'cogging_iq_A']:
        x = channel(d, suffix)
        if x is not None:
            result[suffix] = describe(x)
    raw = channel(d, 'encoder_raw_count_count')
    theta = channel(d, 'mechanical_angle_rad')
    if raw is None and theta is not None:
        raw = theta * (32768 / (2 * np.pi))
        result['angle_source'] = 'CH6 scaled to equivalent counts, not raw ADC/SPI'
    elif raw is not None:
        result['angle_source'] = 'encoder raw telemetry'
    if raw is not None:
        delta = (np.diff(raw) + 16384) % 32768 - 16384
        result['angle_delta_counts'] = describe(delta)
        result['angle_zero_delta_count'] = int(np.sum(np.abs(delta) < .05))
        result['signed_revolutions'] = float(np.sum(delta) / 32768)
    # Position-domain harmonic amplitudes: joint least-squares fit h=1..14.
    # Do not interpolate sparsely occupied bins and present them as measured.
    speed = channel(d, 'mechanical_speed_rpm')
    if theta is not None and speed is not None and len(d) >= 100:
        phase = theta % (2 * np.pi)
        bins = np.floor(phase * 72 / (2 * np.pi)).astype(int)
        result['angle_coverage_72_bins'] = int(len(np.unique(bins)))
        design = np.column_stack([np.ones(len(d))] +
                                 [f(h * phase) for h in range(1, 15)
                                  for f in (np.cos, np.sin)])
        beta, _, rank, _ = np.linalg.lstsq(design, speed, rcond=None)
        result['harmonic_fit_rank'] = int(rank)
        result['harmonic_fit_condition'] = float(np.linalg.cond(design))
        result['speed_harmonic_amplitude_rpm'] = {
            str(h): float(np.hypot(beta[2*h-1], beta[2*h]))
            for h in range(1, 15)}
    return result


def analyze(path):
    d = pd.read_csv(path)
    t = d.time_s.to_numpy(float)
    target = channel(d, 'target_speed_rpm')
    out = {'file': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
           'columns': list(d.columns), 'rows': len(d),
           'duration_s': float(t[-1] - t[0]), 'dt_s': describe(np.diff(t)),
           'nonfinite_cells': int((~np.isfinite(d.select_dtypes('number'))).sum().sum()),
           'nonpositive_dt': int(np.sum(np.diff(t) <= 0)), 'whole_log': metrics(d)}
    for suffix in ['encoder_status_', 'encoder_crc_errors_count',
                   'encoder_transfer_errors_count']:
        x = channel(d, suffix)
        out[suffix] = describe(x) if x is not None else None
    if target is not None:
        v, n = np.unique(np.round(target, 3), return_counts=True)
        out['targets_top_counts'] = sorted(zip(v.tolist(), n.tolist()),
                                         key=lambda q: q[1], reverse=True)[:12]
        changes = np.r_[0, np.flatnonzero(np.abs(np.diff(target)) > 1e-4)+1, len(d)]
        out['constant_target_segments'] = []
        for a,b in zip(changes[:-1], changes[1:]):
            if t[b-1] - t[a] < 1.5:
                continue
            keep = d.iloc[a:b]
            keep = keep[keep.time_s >= t[a] + 1]
            if len(keep) >= 50:
                out['constant_target_segments'].append(metrics(keep))
    return d, out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--log-root', type=Path, default=Path('D:/'))
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    frames, records = {}, []
    for path in sorted(args.log_root.glob('haptic_knob_log_202609*.csv')):
        d, record = analyze(path)
        frames[path.stem] = d
        records.append(record)
    # Explicit windows are fixed and reported, never chosen by speed threshold.
    windows = {'163103': {'open': (1,7), 'closed': (12,15.8)},
               '164345': {'open': (3.7,7.4), 'closed': (15.2,19.4)}}
    comparison = {}
    for suffix, modes in windows.items():
        d = frames['haptic_knob_log_20260922_' + suffix]
        comparison[suffix] = {}
        for mode, (a,b) in modes.items():
            z = d[(d.time_s >= a) & (d.time_s <= b)]
            item = metrics(z)
            th = channel(z, 'electrical_angle_rad')
            iq, id_ = channel(z, 'iq_actual_A'), channel(z, 'id_actual_A')
            alpha = id_ * np.cos(th) - iq * np.sin(th)
            beta = id_ * np.sin(th) + iq * np.cos(th)
            fit = np.column_stack([np.ones(len(z)), np.cos(th), np.sin(th)])
            item['reconstructed_phase_fit'] = {}
            for label, vals in [('u', alpha), ('w', -.5*alpha-np.sqrt(3)/2*beta)]:
                coef = np.linalg.lstsq(fit, vals, rcond=None)[0]
                item['reconstructed_phase_fit'][label] = {
                    'dc_A': float(coef[0]),
                    'fundamental_amplitude_A': float(np.hypot(coef[1],coef[2]))}
            pre = channel(z, 'speed_loop_iq_A') + channel(z, 'friction_iq_A') + channel(z, 'cogging_iq_A')
            applied = channel(z, 'iq_applied_target_A')
            item['pre_slew_minus_applied_A'] = describe(pre-applied)
            item['pre_slew_difference_above_20mA_fraction'] = float(
                np.mean(np.abs(pre-applied) > .02))
            peak_pos = int(np.argmax(channel(z, 'mechanical_speed_rpm')))
            item['maximum_speed_row'] = {
                name: float(value) for name, value in z.iloc[peak_pos].items()}
            comparison[suffix][mode] = item
    result = {'method': 'population std; contiguous windows; joint spatial LS h=1..14',
              'logs': records, 'comparison': comparison}
    (args.output / 'history_analysis.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    for r in records:
        s = r['whole_log'].get('mechanical_speed_rpm')
        print(Path(r['file']).name, 'rows', r['rows'], 'duration', round(r['duration_s'],3),
              'cols', len(r['columns']), 'dt_p99_ms', round(r['dt_s']['p99']*1000,3),
              'speed', {k:round(s[k],3) for k in ['mean','std','min','max']} if s else None,
              'targets', r.get('targets_top_counts'))
        for q in r.get('constant_target_segments', []):
            s=q.get('mechanical_speed_rpm', {})
            print(' segment', round(q['start_s'],3),round(q['end_s'],3),
                  'target',round(q['target_speed_rpm']['mean'],2),
                  'speed',[round(s[k],2) for k in ['mean','std','min','max']],
                  'zero_deltas',q.get('angle_zero_delta_count'))
    for suffix,modes in comparison.items():
        for mode,item in modes.items():
            print('comparison',suffix,mode,item['mechanical_speed_rpm'],
                  'h7',item['speed_harmonic_amplitude_rpm']['7'],
                  'h14',item['speed_harmonic_amplitude_rpm']['14'])


if __name__ == '__main__':
    main()

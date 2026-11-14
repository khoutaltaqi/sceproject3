"""Plot saved paths, corrections, tracking deviations, and independent final errors."""
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from .common import folder, home, save


def number(value):
    return isinstance(value, (float, int)) and math.isfinite(value)


def column(rows, name):
    return np.array([float(r[name]) if r.get(name) else np.nan for r in rows])


def export(fig, dest, name):
    for suffix in ['png', 'pdf']:
        fig.savefig(dest/f'{name}.{suffix}', dpi=200)
    plt.close(fig)


def rms(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(values**2))) if values.size else None


def trial_plots(rows, s, dest, i):
    name = f'trial_{i:02d}'
    t = column(rows, 'time_s')
    fig, ax = plt.subplots(figsize=(7, 4), constrained_layout=True)
    for col, label in [('odom_distance_m', 'Odometry only'), ('camera_distance_m', 'Camera'),
                       ('estimated_distance_m', 'EKF corrected')]:
        ax.plot(t, column(rows, col), label=label)
    ax.axhline(s['stop_target_m'], color='black', ls='--', label='Target')
    ax.set(xlabel='Trial time (s)', ylabel='Distance (m)', title=f'Trial {i}: {s["status"]}')
    ax.legend(); ax.grid(alpha=0.25)
    export(fig, dest, name+'_distance')
    metrics = {'status': s['status'], 'trajectory_available': False,
        'correction_diagnostics_available': False,
        'tracking_definition': 'estimated position minus cubic reference, during DRIVE',
        'ground_truth': 'Independent final tape distance only. Paths are estimates.'}
    if 'x_ref_m' in rows[0] and np.isfinite(column(rows, 'plan_time_s')).any():
        path = [r for r in rows if r.get('plan_time_s')]
        tp = column(path, 'plan_time_s')
        fig, axes = plt.subplots(1, 3, figsize=(12, 3.5), constrained_layout=True)
        curves = [('x_ref_m', 'y_ref_m', 'Cubic reference', 'black', '--'),
                  ('odom_px_m', 'odom_py_m', 'Odometry only', '0.45', '-'),
                  ('px_m', 'py_m', 'EKF corrected', 'tab:blue', '-')]
        for xc, yc, label, color, style in curves:
            x, y = column(path, xc), column(path, yc)
            for ax, values in zip(axes[:2], [x, y]):
                ax.plot(tp, values, label=label, color=color, ls=style)
            axes[2].plot(x, y, label=label, color=color, ls=style)
            valid = np.flatnonzero(np.isfinite(x) & np.isfinite(y))
            if xc != 'x_ref_m' and valid.size:
                axes[2].plot(x[valid[-1]], y[valid[-1]], 'x', color=color, ms=7)
        axes[0].set(xlabel='Plan time (s)', ylabel='x (m)')
        axes[1].set(xlabel='Plan time (s)', ylabel='y (m)')
        axes[2].set(xlabel='x (m)', ylabel='y (m)', title='x-y path (last sample marked)')
        for ax in axes:
            ax.grid(alpha=0.25); ax.legend(fontsize=8)
        export(fig, dest, name+'_trajectory')
        drive = [r for r in path if r['state'] == 'DRIVE']
        fig, axes = plt.subplots(1, 3, figsize=(12, 3.5), constrained_layout=True)
        for xc, yc, label, color, _ in curves[1:]:
            ex = column(path, xc)-column(path, 'x_ref_m')
            ey = column(path, yc)-column(path, 'y_ref_m')
            for ax, values in zip(axes, [ex, ey, np.hypot(ex, ey)]):
                ax.plot(tp, values, color=color, label=label)
            dx = column(drive, xc)-column(drive, 'x_ref_m')
            dy = column(drive, yc)-column(drive, 'y_ref_m')
            metrics['odom' if xc == 'odom_px_m' else 'ekf'] = {
                'tracking_rmse_x_m': rms(dx), 'tracking_rmse_y_m': rms(dy),
                'tracking_rmse_xy_m': rms(np.hypot(dx, dy))}
        for ax, label in zip(axes, ['x - x_ref (m)', 'y - y_ref (m)', 'Distance to reference (m)']):
            ax.axhline(0, color='black', ls='--', lw=0.8)
            ax.set(xlabel='Plan time (s)', ylabel=label)
            ax.grid(alpha=0.25); ax.legend(fontsize=8)
        export(fig, dest, name+'_tracking_error')
        metrics['trajectory_available'] = True
    if 'tag_correction_x_m' in rows[0]:
        used = column(rows, 'vision_used') == 1
        attempted = np.isfinite(column(rows, 'vision_nis'))
        fig, axes = plt.subplots(3, 2, figsize=(10, 9), constrained_layout=True)
        for ax, pre, post, scale, label in [
            (axes[0, 0], 'range_innovation_m', 'range_residual_m', 1000, 'Range residual (mm)'),
            (axes[0, 1], 'bearing_innovation_rad', 'bearing_residual_rad', 180/np.pi, 'Bearing residual (deg)')]:
            before, after = column(rows, pre)*scale, column(rows, post)*scale
            ax.plot(t, np.where(used, before, np.nan), '.-', label='Before tag update')
            ax.plot(t, np.where(used, after, np.nan), '.-', label='After tag update')
            ax.scatter(t[attempted & ~used], before[attempted & ~used], marker='x', color='tab:red', label='Rejected')
            ax.axhline(0, color='black', lw=0.6)
            ax.set(ylabel=label)
        for sensor, style in [('odom', '--'), ('tag', '-')]:
            for coord, color in [('x', 'tab:blue'), ('y', 'tab:orange')]:
                axes[1, 0].plot(t, column(rows, f'{sensor}_correction_{coord}_m')*1000,
                    ls=style, color=color, label=f'{sensor}: delta {coord}')
        axes[1, 0].set(ylabel='Applied position correction (mm)')
        axes[1, 1].plot(t, column(rows, 'vision_nis'), '.', label='Tag NIS')
        axes[1, 1].axhline(9.21, color='tab:red', ls='--', label='Gate 9.21')
        axes[1, 1].set(ylabel='NIS')
        for ax, coord in zip(axes[2], ['x', 'y']):
            ax.plot(t, column(rows, f'pred_sigma_{coord}_m')*1000, ls='--', label='Before measurements')
            ax.plot(t, column(rows, f'sigma_{coord}_m')*1000, label='After odom + tag')
            ax.set(ylabel=f'Model sigma {coord} (mm)')
        for ax in axes.flat:
            ax.set(xlabel='Trial time (s)'); ax.grid(alpha=0.25); ax.legend(fontsize=8)
        export(fig, dest, name+'_correction')
        metrics.update(correction_diagnostics_available=True,
            tag_attempted=int(attempted.sum()), tag_accepted=int(used.sum()),
            tag_rejected=int((attempted & ~used).sum()),
            range_innovation_rms_m=rms(column(rows, 'range_innovation_m')[used]),
            range_posterior_residual_rms_m=rms(column(rows, 'range_residual_m')[used]),
            residual_definition='Same camera measurement minus predicted/posterior measurement. Not independent accuracy.')
    actual, odom, corrected = (s.get(k) for k in ['actual_final_m', 'odom_final_m', 'estimated_final_m'])
    if all(number(v) for v in [actual, odom, corrected]) and actual > 0:
        metrics['independent_final_comparison'] = {
            'odom_error_m': odom-actual, 'corrected_error_m': corrected-actual,
            'absolute_error_reduction_m': abs(odom-actual)-abs(corrected-actual)}
    save(dest/(name+'_metrics.json'), metrics)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('path', nargs='?', type=Path, default=home()/'results')
    a = p.parse_args()
    files = [a.path/'summary.json'] if (a.path/'summary.json').is_file() else sorted(a.path.rglob('summary.json'))
    groups = defaultdict(list)
    for path in files:
        s = json.loads(path.read_text())
        if s.get('kind') == 'rm_phase3_trial':
            groups[s['experiment_id']].append((path.parent, s))
    if not groups:
        raise SystemExit('No trial summary.json files found')
    out = folder('analysis')
    for key, trials in groups.items():
        dest = out/key
        dest.mkdir()
        pairs, baseline = [], []
        with (dest/'trials.csv').open('w', newline='') as f:
            table = csv.writer(f)
            table.writerow(['trial', 'folder', 'status', 'start_actual_m', 'estimated_final_m',
                            'actual_final_m', 'error_m', 'included_in_statistics',
                            'odom_final_m', 'odom_error_m', 'absolute_error_reduction_m'])
            for i, (path, s) in enumerate(trials, 1):
                estimated, actual = s.get('estimated_final_m'), s.get('actual_final_m')
                paired = number(estimated) and number(actual) and actual > 0
                error = estimated-actual if paired else None
                included = paired and s['status'] == 'ARRIVED'
                odom = s.get('odom_final_m')
                odom_error = odom-actual if paired and number(odom) else None
                reduction = None if odom_error is None else abs(odom_error)-abs(error)
                table.writerow([i, str(path), s['status'], s['start_actual_m'], estimated,
                                actual, error, int(included), odom, odom_error, reduction])
                if included:
                    pairs.append([i, estimated, actual, error])
                    if odom_error is not None:
                        baseline.append([i, odom, odom_error, error])
                trace = path/'trace.csv'
                if trace.is_file():
                    with trace.open() as stream:
                        rows = list(csv.DictReader(stream))
                    if rows:
                        trial_plots(rows, s, dest, i)
        stats = {'experiment_id': key, 'total_attempts': len(trials),
                 'completed_paired_trials': len(pairs), 'required_8_to_10_trials': 8 <= len(pairs) <= 10,
                 'error_definition': 'estimated_final_m - actual_final_m',
                 'mean_absolute_error_m': None, 'min_error_m': None, 'max_error_m': None,
                 'error_span_m': None, 'sample_std_error_m': None,
                 'baseline_paired_trials': len(baseline), 'odom_mae_m': None,
                 'corrected_mae_same_subset_m': None, 'correction_mae_reduction_m': None}
        if pairs:
            values = np.array(pairs)
            e = values[:, 3]
            stats.update(mean_absolute_error_m=float(np.mean(abs(e))), min_error_m=float(e.min()),
                         max_error_m=float(e.max()), error_span_m=float(np.ptp(e)),
                         sample_std_error_m=float(np.std(e, ddof=1)) if len(e) > 1 else None)
            fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
            axes[0].scatter(values[:, 0], values[:, 1], marker='o', label='EKF corrected')
            axes[0].scatter(values[:, 0], values[:, 2], marker='x', label='Independent tape')
            axes[0].axhline(trials[0][1]['stop_target_m'], color='black', ls='--', label='Target')
            axes[0].set(xlabel='Trial number', ylabel='Final distance (m)', title='Final distances')
            axes[0].legend()
            axes[1].scatter(values[:, 0], e*1000)
            if baseline:
                b = np.array(baseline)
                odom_mae, corrected_mae = float(np.mean(abs(b[:, 2]))), float(np.mean(abs(b[:, 3])))
                stats.update(odom_mae_m=odom_mae, corrected_mae_same_subset_m=corrected_mae,
                             correction_mae_reduction_m=odom_mae-corrected_mae)
                axes[0].scatter(b[:, 0], b[:, 1], marker='+', label='Odometry only')
                axes[0].legend()
                axes[1].scatter(b[:, 0], b[:, 2]*1000, marker='+', label='Odometry only')
            axes[1].collections[0].set_label('EKF corrected')
            axes[1].legend()
            axes[1].axhline(0, color='black', ls='--')
            axes[1].set(xlabel='Trial number', ylabel='Estimated - measured (mm)', title='Estimation errors')
            for ax in axes:
                ax.grid(alpha=0.25)
                ax.set_xticks(values[:, 0])
            export(fig, dest, 'scatter')
            print(f'{key}: n={len(pairs)}, MAE={stats["mean_absolute_error_m"]:.4f} m, '
                  f'range=[{e.min():+.4f}, {e.max():+.4f}] m')
        else:
            print(f'{key}: no completed trials with independent final measurements. No scatter/errors invented.')
        save(dest/'statistics.json', stats)
    print('Saved:', out)


if __name__ == '__main__':
    main()

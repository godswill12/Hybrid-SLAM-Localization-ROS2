#!/usr/bin/env python3
"""
Comparison Analysis
===================
Reads the CSV output from plotter.py and produces:
  1. Full statistical comparison table
  2. RTAB-Map raw vs EKF(RTAB corr) dedicated analysis
  3. All methods comparison
  4. Percentage improvement table
  5. Error distribution analysis

Run after experiment:
  python3 compare_analysis.py /ros_ws/csv_files/data_TIMESTAMP.csv
"""

import sys
import csv
import math
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime


def load_csv(path):
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['time_s'] == '' or row['time_s'].startswith('='):
                break
            try:
                rows.append({k: float(v) if v != '' else None
                             for k, v in row.items()})
            except ValueError:
                break
    return rows


def extract(rows, key):
    return [r[key] for r in rows if r.get(key) is not None]


def stats(values):
    if not values:
        return dict(avg=0,std=0,med=0,min=0,max=0,rmse=0,p95=0)
    v = np.array(values)
    return {
        'avg':  float(np.mean(v)),
        'std':  float(np.std(v)),
        'med':  float(np.median(v)),
        'min':  float(np.min(v)),
        'max':  float(np.max(v)),
        'rmse': float(np.sqrt(np.mean(v**2))),
        'p95':  float(np.percentile(v, 95)),
    }


def pct_below(values, thr):
    if not values:
        return 0.0
    return 100.0 * sum(1 for v in values if v < thr) / len(values)


def improvement(baseline, improved):
    if baseline == 0:
        return 0.0
    return 100.0 * (baseline - improved) / baseline


def print_divider(char='=', width=72):
    print(char * width)


def analyse(csv_path):
    print(f'\nLoading: {csv_path}')
    rows = load_csv(csv_path)
    if not rows:
        print('ERROR: No data found in CSV')
        return

    n = len(rows)
    duration = rows[-1]['time_s'] - rows[0]['time_s'] if n > 1 else 0
    print(f'Samples: {n}   Duration: {duration:.1f}s\n')

    # Extract all error series
    methods = {
        'AMCL':             extract(rows, 'amcl_err'),
        'EKF (AMCL corr)':  extract(rows, 'ekf_err'),
        'EIF (AMCL corr)':  extract(rows, 'eif_err'),
        'RTAB-Map':         extract(rows, 'rtab_err'),
        'Fused (EKF+RTAB)': extract(rows, 'fused_err'),
        'EKF (RTAB corr)':  extract(rows, 'ekfr_err'),
    }

    # Remove methods with no data
    methods = {k: v for k, v in methods.items() if len(v) > 0 and max(v) > 0}

    # ── Table 1: Full statistical comparison ─────────────────────────────
    print_divider()
    print('  TABLE 1 — FULL STATISTICAL COMPARISON (metres)')
    print_divider()
    print(f'{"Method":<22} {"Avg":>7} {"Std":>7} {"Med":>7} '
          f'{"Min":>7} {"Max":>7} {"RMSE":>7} {"P95":>7} {"<5cm":>7}')
    print_divider('-')
    for name, values in methods.items():
        s = stats(values)
        pb = pct_below(values, 0.05)
        print(f'{name:<22} '
              f'{s["avg"]:>7.4f} '
              f'{s["std"]:>7.4f} '
              f'{s["med"]:>7.4f} '
              f'{s["min"]:>7.4f} '
              f'{s["max"]:>7.4f} '
              f'{s["rmse"]:>7.4f} '
              f'{s["p95"]:>7.4f} '
              f'{pb:>6.1f}%')
    print_divider()

    # ── Table 2: RTAB-Map vs EKF(RTAB) dedicated comparison ──────────────
    if 'RTAB-Map' in methods and 'EKF (RTAB corr)' in methods:
        print('\n')
        print_divider()
        print('  TABLE 2 — RTAB-Map RAW vs EKF WITH RTAB CORRECTION')
        print('  (How much does EKF smoothing improve RTAB-Map pose?)')
        print_divider()

        rtab  = stats(methods['RTAB-Map'])
        ekfr  = stats(methods['EKF (RTAB corr)'])
        names = ['RTAB-Map raw', 'EKF (RTAB corr)', 'Improvement (%)']

        metrics = [
            ('Avg error (m)',   rtab['avg'],  ekfr['avg']),
            ('Std dev (m)',     rtab['std'],  ekfr['std']),
            ('Median (m)',      rtab['med'],  ekfr['med']),
            ('Max error (m)',   rtab['max'],  ekfr['max']),
            ('RMSE (m)',        rtab['rmse'], ekfr['rmse']),
            ('95th pct (m)',    rtab['p95'],  ekfr['p95']),
            ('% below 5cm',
             pct_below(methods['RTAB-Map'],0.05),
             pct_below(methods['EKF (RTAB corr)'],0.05)),
        ]

        print(f'{"Metric":<20} {"RTAB-Map raw":>14} {"EKF(RTAB corr)":>16} {"Improvement":>13}')
        print_divider('-')
        for label, r_val, e_val in metrics:
            if label == '% below 5cm':
                imp = e_val - r_val  # higher is better here
                imp_str = f'+{imp:.1f}%' if imp >= 0 else f'{imp:.1f}%'
                print(f'{label:<20} {r_val:>13.1f}% {e_val:>15.1f}% {imp_str:>13}')
            else:
                imp = improvement(r_val, e_val)
                imp_str = f'{imp:+.1f}%'
                print(f'{label:<20} {r_val:>14.4f} {e_val:>16.4f} {imp_str:>13}')
        print_divider()

    # ── Table 3: All methods improvement vs AMCL baseline ────────────────
    if 'AMCL' in methods:
        print('\n')
        print_divider()
        print('  TABLE 3 — ALL METHODS vs AMCL BASELINE')
        print('  (How much does each method improve over the standard baseline?)')
        print_divider()

        amcl_s = stats(methods['AMCL'])
        print(f'{"Method":<22} {"Avg err":>9} {"vs AMCL":>9} '
              f'{"RMSE":>9} {"vs AMCL":>9} {"Max":>9} {"vs AMCL":>9}')
        print_divider('-')

        for name, values in methods.items():
            s = stats(values)
            avg_imp  = improvement(amcl_s['avg'],  s['avg'])
            rmse_imp = improvement(amcl_s['rmse'], s['rmse'])
            max_imp  = improvement(amcl_s['max'],  s['max'])
            marker = ' ◄ baseline' if name == 'AMCL' else ''
            print(f'{name:<22} '
                  f'{s["avg"]:>9.4f} '
                  f'{avg_imp:>+8.1f}% '
                  f'{s["rmse"]:>9.4f} '
                  f'{rmse_imp:>+8.1f}% '
                  f'{s["max"]:>9.4f} '
                  f'{max_imp:>+8.1f}%'
                  f'{marker}')
        print_divider()

    # ── Table 4: Error distribution bins ─────────────────────────────────
    print('\n')
    print_divider()
    print('  TABLE 4 — ERROR DISTRIBUTION (% of time in each error band)')
    print_divider()

    bands = [
        ('< 2 cm',  0.00, 0.02),
        ('2-5 cm',  0.02, 0.05),
        ('5-10 cm', 0.05, 0.10),
        ('10-20cm', 0.10, 0.20),
        ('> 20 cm', 0.20, 9999),
    ]

    header = f'{"Method":<22}'
    for label, _, _ in bands:
        header += f' {label:>9}'
    print(header)
    print_divider('-')

    for name, values in methods.items():
        if not values:
            continue
        row_str = f'{name:<22}'
        for label, lo, hi in bands:
            count = sum(1 for v in values if lo <= v < hi)
            pct   = 100.0 * count / len(values)
            row_str += f' {pct:>8.1f}%'
        print(row_str)
    print_divider()

    # ── Generate comparison plots ─────────────────────────────────────────
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir   = os.path.dirname(csv_path)

    # Plot 1: Box plot of error distributions
    fig, ax = plt.subplots(figsize=(12, 6))
    plot_data  = []
    plot_labels = []
    colors = {
        'AMCL':            '#E24B4A',
        'EKF (AMCL corr)': '#378ADD',
        'EIF (AMCL corr)': '#1D9E75',
        'RTAB-Map':        '#BA7517',
        'Fused (EKF+RTAB)':'#3B6D11',
        'EKF (RTAB corr)': '#9B30FF',
    }
    for name, values in methods.items():
        if values:
            plot_data.append(values)
            plot_labels.append(name.replace(' ', '\n'))

    bp = ax.boxplot(plot_data, patch_artist=True, notch=False,
                    medianprops=dict(color='black', linewidth=2))

    for patch, name in zip(bp['boxes'], methods.keys()):
        patch.set_facecolor(colors.get(name, '#888888'))
        patch.set_alpha(0.7)

    ax.set_xticklabels(plot_labels, fontsize=9)
    ax.set_ylabel('Position error (m)', fontsize=12)
    ax.set_title('Error distribution by method — box plot', fontsize=13)
    ax.axhline(y=0.05, color='green', linestyle=':', linewidth=1,
               alpha=0.7, label='5 cm threshold')
    ax.legend(fontsize=10)
    ax.grid(True, axis='y', alpha=0.3)
    ax.set_ylim(bottom=0)
    fig.tight_layout()
    boxplot_path = f'{out_dir}/boxplot_{timestamp}.png'
    fig.savefig(boxplot_path, dpi=150)
    plt.close(fig)
    print(f'\nBox plot saved: {boxplot_path}')

    # Plot 2: RTAB-Map vs EKF(RTAB) direct comparison
    if 'RTAB-Map' in methods and 'EKF (RTAB corr)' in methods:
        times = [r['time_s'] for r in rows]
        rtab_errs = [r.get('rtab_err') for r in rows]
        ekfr_errs = [r.get('ekfr_err') for r in rows]

        def valid_pairs(times, errs):
            pairs = [(t,e) for t,e in zip(times,errs) if e is not None]
            if not pairs: return [],[]
            t,e = zip(*pairs); return list(t),list(e)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

        tr, er = valid_pairs(times, rtab_errs)
        te, ee = valid_pairs(times, ekfr_errs)

        ax1.plot(tr, er, color='#BA7517', linewidth=1.5,
                 linestyle=(0,(10,3,3,3)), label='RTAB-Map raw')
        ax1.axhline(y=0.05, color='green', linestyle=':', linewidth=1, alpha=0.7)
        ax1.set_ylabel('Error (m)', fontsize=11)
        ax1.set_title('RTAB-Map raw pose error', fontsize=12)
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(bottom=0)

        ax2.plot(te, ee, color='#9B30FF', linewidth=1.5,
                 linestyle=(0,(8,2,2,2)), label='EKF (RTAB corr)')
        ax2.axhline(y=0.05, color='green', linestyle=':', linewidth=1, alpha=0.7)
        ax2.set_xlabel('Time (s)', fontsize=11)
        ax2.set_ylabel('Error (m)', fontsize=11)
        ax2.set_title('EKF with RTAB-Map correction — same data smoothed', fontsize=12)
        ax2.legend(fontsize=10)
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim(bottom=0)

        # Shared y axis for fair comparison
        max_y = max(max(er) if er else 0, max(ee) if ee else 0) * 1.1
        ax1.set_ylim(0, max_y)
        ax2.set_ylim(0, max_y)

        fig.suptitle('RTAB-Map raw vs EKF with RTAB correction\n'
                     '(same RTAB-Map input, EKF adds odometry smoothing)',
                     fontsize=13)
        fig.tight_layout()
        compare_path = f'{out_dir}/rtab_vs_ekfrtab_{timestamp}.png'
        fig.savefig(compare_path, dpi=150)
        plt.close(fig)
        print(f'Comparison plot saved: {compare_path}')

    # Plot 3: Bar chart of key metrics
    if len(methods) >= 2:
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))

        names  = list(methods.keys())
        clrs   = [colors.get(n, '#888888') for n in names]
        avgs   = [stats(methods[n])['avg']  for n in names]
        maxes  = [stats(methods[n])['max']  for n in names]
        rmses  = [stats(methods[n])['rmse'] for n in names]

        short_names = [n.replace(' (AMCL corr)','').replace(' (RTAB corr)','\n(RTAB)').replace('Fused ','Fused\n') for n in names]

        for ax, values, title, ylabel in [
            (axes[0], avgs,  'Average error',  'Avg error (m)'),
            (axes[1], maxes, 'Maximum error',  'Max error (m)'),
            (axes[2], rmses, 'RMSE',           'RMSE (m)'),
        ]:
            bars = ax.bar(range(len(names)), values,
                          color=clrs, alpha=0.8,
                          edgecolor='white', linewidth=0.5)
            ax.set_xticks(range(len(names)))
            ax.set_xticklabels(short_names, fontsize=8)
            ax.set_ylabel(ylabel, fontsize=10)
            ax.set_title(title, fontsize=11)
            ax.axhline(y=0.05, color='green', linestyle=':',
                       linewidth=1, alpha=0.7)
            ax.grid(True, axis='y', alpha=0.3)
            ax.set_ylim(bottom=0)
            for bar, val in zip(bars, values):
                ax.text(bar.get_x()+bar.get_width()/2,
                        bar.get_height()+0.002,
                        f'{val:.3f}', ha='center', va='bottom', fontsize=8)

        fig.suptitle('Key error metrics — all methods comparison', fontsize=13)
        fig.tight_layout()
        bar_path = f'{out_dir}/metrics_bar_{timestamp}.png'
        fig.savefig(bar_path, dpi=150)
        plt.close(fig)
        print(f'Bar chart saved:       {bar_path}')

    print('\nAnalysis complete.')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 compare_analysis.py <path_to_data_csv>')
        print('Example: python3 compare_analysis.py /ros_ws/csv_files/data_20260511_153105.csv')
        sys.exit(1)
    analyse(sys.argv[1])

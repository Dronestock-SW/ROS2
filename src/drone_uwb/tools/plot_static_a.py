#!/usr/bin/env python3
"""Render recorded A static-reference results without recalculating positions."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    summary = json.loads((args.run/'summary.json').read_text(encoding='utf-8'))
    with (args.run/'positions.csv').open(encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
    colors = {'without_bias': '#777777', 'with_candidate_bias': '#006cba'}
    labels = {'without_bias': 'Before bias subtraction', 'with_candidate_bias': 'After bias subtraction'}
    for variant in colors:
        subset = [r for r in rows if r['variant'] == variant]
        time = [float(r['t_rel_s']) for r in subset]
        diff = [float(r['reference_difference_m'])*100 for r in subset]
        axes[0].plot(time, diff, color=colors[variant], lw=.8, alpha=.8, label=labels[variant])
    shaded = [float(r['t_rel_s']) for r in rows if r['in_reported_span'] == 'True']
    axes[0].axvspan(min(shaded), max(shaded), color='#e38a1f', alpha=.20, label='Reported excursion span')
    axes[0].set(xlabel='Time since first UWB cycle (s)', ylabel='Difference from reported XY (cm)',
                title='Separate 60-second evaluation capture; all 2,540 cycles retained')
    groups = [('all', 'All samples'), ('outside_reported_span', 'Outside reported span'),
              ('reported_excursion_span', 'Reported span')]
    x = np.arange(len(groups))
    for shift, variant in [(-.18, 'without_bias'), (.18, 'with_candidate_bias')]:
        values = [summary['groups'][key]['models'][variant]['rmse_m']*100 for key, _ in groups]
        bars = axes[1].bar(x+shift, values, width=.36, color=colors[variant], label=labels[variant])
        axes[1].bar_label(bars, fmt='%.2f', padding=3, fontsize=9)
    axes[1].set_xticks(x, [label for _, label in groups])
    axes[1].set(ylabel='RMSE relative to reported XY (cm)', ylim=(0, 59),
                title='Single-point calibration trial; reference measurements are not independently verified')
    for ax in axes:
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
        ax.legend(frameon=False, loc='upper right', fontsize=9)
    fig.suptitle('Baseline A on measured UWB: calibration 16:16, evaluation 17:09 (2026-09-06)', fontsize=12)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for ext in ('png', 'pdf'):
        output = args.output_dir/('static_a_comparison.'+ext)
        if output.exists():
            raise FileExistsError(output)
        fig.savefig(output, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()

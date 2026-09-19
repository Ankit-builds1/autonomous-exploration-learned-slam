#!/usr/bin/env python3
"""Rebuild the PPO training curve from the real TensorBoard logs."""

import os, glob
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

LOGS = os.path.expanduser('~/ros2_ws/src/explore_nav/models/tb_logs')
OUT  = os.path.expanduser('~/ros2_ws/src/explore_nav/results')
os.makedirs(OUT, exist_ok=True)

TAG = 'rollout/ep_rew_mean'
runs = {}

print(f'{"run":10s} {"points":>7s} {"final steps":>12s} {"final reward":>13s}')
print('-' * 46)
for d in sorted(glob.glob(os.path.join(LOGS, 'PPO_*'))):
    try:
        ea = EventAccumulator(d); ea.Reload()
        if TAG not in ea.Tags()['scalars']:
            continue
        ev = ea.Scalars(TAG)
        steps = [e.step for e in ev]
        vals  = [e.value for e in ev]
        runs[os.path.basename(d)] = (steps, vals)
        print(f'{os.path.basename(d):10s} {len(steps):7d} {steps[-1]:12d} {vals[-1]:13.2f}')
    except Exception as e:
        print(os.path.basename(d), 'skipped:', e)

# longest run = the 250k final training
best = sorted(runs, key=lambda k: int(k.split('_')[1]))[-1]
steps, vals = runs[best]
print(f'\nPlotting {best}  ({len(steps)} points, {steps[-1]} timesteps)')

plt.rcParams.update({'font.size': 11, 'axes.spines.top': False,
                     'axes.spines.right': False, 'figure.dpi': 130})
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(steps, vals, color='#DD8452', lw=2, label='PPO (mean episode reward)')
ax.axhline(-32.60, ls='--', color='#8C8C8C', lw=1.4, label='Random baseline (-32.60)')
ax.axhline(60.21,  ls='--', color='#4C72B0', lw=1.4, label='Hand-coded heuristic (60.21)')
ax.set_xlabel('Training timesteps')
ax.set_ylabel('Mean episode reward')
ax.set_title(f'PPO training progression ({best}, single run)', fontweight='bold')
ax.grid(alpha=0.25); ax.set_axisbelow(True)
ax.legend(frameon=False, fontsize=10, loc='lower right')
fig.tight_layout()
fig.savefig(f'{OUT}/fig3_training_curve.png', bbox_inches='tight')
print('fig3 regenerated ->', OUT)

#!/usr/bin/env python3
"""Generate the comparison figures for the project report."""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = os.path.expanduser('~/ros2_ws/src/explore_nav/results')
MAPS = os.path.expanduser('~/maps')
os.makedirs(OUT, exist_ok=True)

C_HEUR, C_RL, C_NEAR, C_RAND = '#4C72B0', '#DD8452', '#55A868', '#8C8C8C'
plt.rcParams.update({'font.size': 11, 'axes.spines.top': False,
                     'axes.spines.right': False, 'figure.dpi': 130})


def label_bars(ax, bars, fmt='{:.1f}'):
    for b in bars:
        ax.text(b.get_x() + b.get_width()/2, b.get_height(),
                fmt.format(b.get_height()), ha='center', va='bottom',
                fontsize=10, fontweight='bold')


# ---------- Figure 1: Gazebo head-to-head ----------
metrics = [('Time to complete (s)', 134.5, 120.1, '{:.1f}'),
           ('Path length (m)',       14.6,  11.4, '{:.1f}'),
           ('Goals dispatched',      18,    16,   '{:.0f}'),
           ('Coverage (% cells)',    74.5,  74.3, '{:.1f}')]

fig, axes = plt.subplots(1, 4, figsize=(13, 3.8))
for ax, (title, h, r, fmt) in zip(axes, metrics):
    bars = ax.bar(['Heuristic', 'RL (PPO)'], [h, r], color=[C_HEUR, C_RL], width=0.6)
    label_bars(ax, bars, fmt)
    ax.set_title(title, fontsize=11)
    ax.set_ylim(0, max(h, r) * 1.25)
    ax.grid(axis='y', alpha=0.25)
    ax.set_axisbelow(True)
fig.suptitle('Gazebo: learned policy vs hand-coded heuristic (turtlebot3_world)',
             fontsize=13, fontweight='bold')
fig.tight_layout()
fig.savefig(f'{OUT}/fig1_gazebo_comparison.png', bbox_inches='tight')
print('fig1 saved')


# ---------- Figure 2: simulator benchmark, 50 maps ----------
pol = ['Random', 'Nearest', 'Heuristic', 'PPO (ours)']
rew = [-32.60, 59.75, 60.21, 59.47]
cov = [65.7, 98.3, 98.2, 98.4]
cols = [C_RAND, C_NEAR, C_HEUR, C_RL]

fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4))
b1 = a1.bar(pol, rew, color=cols, width=0.62)
label_bars(a1, b1, '{:.2f}')
a1.axhline(0, color='black', lw=0.8)
a1.set_title('Mean episode reward')
a1.grid(axis='y', alpha=0.25); a1.set_axisbelow(True)

b2 = a2.bar(pol, cov, color=cols, width=0.62)
label_bars(a2, b2, '{:.1f}')
a2.set_ylim(0, 108)
a2.set_title('Mean coverage (%)')
a2.grid(axis='y', alpha=0.25); a2.set_axisbelow(True)

fig.suptitle('Simulator benchmark - 50 identical randomised maze maps per policy',
             fontsize=13, fontweight='bold')
fig.tight_layout()
fig.savefig(f'{OUT}/fig2_benchmark_50maps.png', bbox_inches='tight')
print('fig2 saved')


# ---------- Figure 3: PPO training progression ----------
steps = [2048, 6144, 12288, 34816, 73728, 251904]
reward = [-10.1, 8.04, 38.8, 54.5, 68.1, 58.1]
eplen = [57.5, 54.0, 42.3, 39.7, 26.8, 36.3]

fig, ax = plt.subplots(figsize=(7.5, 4.2))
ax.plot(steps, reward, marker='o', color=C_RL, lw=2, label='Mean episode reward')
ax.axhline(-32.60, ls='--', color=C_RAND, lw=1.4, label='Random baseline')
ax.axhline(60.21, ls='--', color=C_HEUR, lw=1.4, label='Hand-coded heuristic')
ax.set_xlabel('Training timesteps')
ax.set_ylabel('Mean episode reward')
ax.set_title('PPO training progression', fontweight='bold')
ax.grid(alpha=0.25); ax.set_axisbelow(True)
ax.legend(frameon=False, fontsize=10)
fig.tight_layout()
fig.savefig(f'{OUT}/fig3_training_curve.png', bbox_inches='tight')
print('fig3 saved')


# ---------- Figure 4: the two resulting maps ----------
try:
    from PIL import Image
    fig, (m1, m2) = plt.subplots(1, 2, figsize=(10, 5.5))
    for ax, name, title, col in [
            (m1, 'run_heuristic.pgm', 'Heuristic - 134.5 s, 14.6 m', C_HEUR),
            (m2, 'run_rl.pgm',        'RL (PPO) - 120.1 s, 11.4 m', C_RL)]:
        img = np.array(Image.open(os.path.join(MAPS, name)))
        ax.imshow(img, cmap='gray', origin='lower')
        ax.set_title(title, color=col, fontweight='bold')
        ax.axis('off')
    fig.suptitle('Final occupancy grids produced by each policy',
                 fontsize=13, fontweight='bold')
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig4_maps.png', bbox_inches='tight')
    print('fig4 saved')
except Exception as e:
    print('fig4 skipped:', e)

print('\nAll figures written to', OUT)

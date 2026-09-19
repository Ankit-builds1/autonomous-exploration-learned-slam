#!/bin/bash
# Pre-demo verification. Does not need Gazebo running.
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=============================================="
echo " PROJECT VERIFICATION"
echo "=============================================="

echo
echo "-- 1. source files present"
for f in frontier_explorer exploration_env train_rl_policy llm_task_planner task_executor; do
  p=~/ros2_ws/src/explore_nav/explore_nav/$f.py
  if [ -f "$p" ]; then echo "   OK   $f.py ($(wc -l < $p) lines)"; else echo "   MISSING $f.py"; fi
done

echo
echo "-- 2. trained model + stats"
for f in ppo_explorer.zip vecnormalize.pkl; do
  p=~/ros2_ws/src/explore_nav/models/$f
  if [ -f "$p" ]; then echo "   OK   $f ($(du -h $p | cut -f1))"; else echo "   MISSING $f"; fi
done

echo
echo "-- 3. registered ROS2 executables"
ros2 pkg executables explore_nav

echo
echo "-- 4. python imports"
python3 - <<'PY'
import importlib, sys
mods = ['explore_nav.frontier_explorer',
        'explore_nav.exploration_env',
        'explore_nav.llm_task_planner',
        'explore_nav.task_executor']
ok = True
for m in mods:
    try:
        importlib.import_module(m)
        print('   OK  ', m)
    except Exception as e:
        ok = False
        print('   FAIL', m, '->', e)
sys.exit(0 if ok else 1)
PY

echo
echo "-- 5. model loads"
python3 - <<'PY'
import os, pickle
from stable_baselines3 import PPO
base = os.path.expanduser('~/ros2_ws/src/explore_nav/models')
m = PPO.load(os.path.join(base, 'ppo_explorer.zip'), device='cpu')
print('   OK   PPO model loaded, obs space', m.observation_space.shape,
      'actions', m.action_space.n)
with open(os.path.join(base, 'vecnormalize.pkl'), 'rb') as f:
    vn = pickle.load(f)
print('   OK   normalisation stats, mean shape', vn.obs_rms.mean.shape)
PY

echo
echo "-- 6. planner decomposes an instruction"
python3 - <<'PY'
from explore_nav.llm_task_planner import make_plan
tasks, backend = make_plan(
    'explore the upper left first, avoid the lower right, '
    'stop once you have mapped 85 percent, then come back home')
print('   backend:', backend)
for i, t in enumerate(tasks, 1):
    print('   ', i, t)
PY

echo
echo "-- 7. saved maps"
ls -1 ~/maps/*.pgm | while read f; do echo "   OK   $(basename $f)"; done

echo
echo "=============================================="
echo " VERIFICATION COMPLETE"
echo "=============================================="

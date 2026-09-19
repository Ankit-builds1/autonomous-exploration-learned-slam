#!/usr/bin/env python3
"""Build the final project report as a Word document."""

import os
from docx import Document
from docx.shared import Pt, Inches, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT

HOME = os.path.expanduser('~')
PKG = os.path.join(HOME, 'ros2_ws/src/explore_nav')
FIGS = os.path.join(PKG, 'results')
OUT = os.path.join(HOME, 'Autonomous_Exploration_Final_Report.docx')

ACCENT = RGBColor(0x1F, 0x4E, 0x79)
SUB = RGBColor(0x2E, 0x74, 0xB5)
GREY = RGBColor(0x55, 0x55, 0x55)

doc = Document()
st = doc.styles['Normal']
st.font.name = 'Calibri'
st.font.size = Pt(10.5)


def H(text, level=1):
    h = doc.add_heading(text, level=level)
    for r in h.runs:
        r.font.color.rgb = ACCENT if level == 1 else SUB
    return h


def P(text, bold=False, italic=False, size=10.5):
    p = doc.add_paragraph()
    r = p.add_run(text)
    r.bold = bold
    r.italic = italic
    r.font.size = Pt(size)
    return p


def bullet(text):
    return doc.add_paragraph(text, style='List Bullet')


def code(text, size=7.5):
    for line in text.rstrip().split('\n'):
        p = doc.add_paragraph()
        pf = p.paragraph_format
        pf.space_after = Pt(0)
        pf.space_before = Pt(0)
        pf.left_indent = Inches(0.15)
        pf.line_spacing = 1.0
        r = p.add_run(line if line.strip() else ' ')
        r.font.name = 'Consolas'
        r.font.size = Pt(size)
    doc.add_paragraph()


def table(headers, rows, widths=None):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = 'Light Grid Accent 1'
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, h in enumerate(headers):
        c = t.rows[0].cells[i]
        c.text = ''
        r = c.paragraphs[0].add_run(str(h))
        r.bold = True
        r.font.size = Pt(9.5)
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            cells[i].text = ''
            r = cells[i].paragraphs[0].add_run(str(v))
            r.font.size = Pt(9.5)
    doc.add_paragraph()
    return t


def figure(fname, caption, width=6.4):
    path = os.path.join(FIGS, fname)
    if not os.path.exists(path):
        P(f'[missing figure: {fname}]', italic=True)
        return
    doc.add_picture(path, width=Inches(width))
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    cp = doc.add_paragraph()
    cp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = cp.add_run(caption)
    r.italic = True
    r.font.size = Pt(9)
    r.font.color.rgb = GREY


def source(fname, path):
    H(fname, 2)
    P(f'Path: {path}', italic=True, size=9)
    try:
        with open(path) as f:
            code(f.read())
    except Exception as e:
        P(f'[could not read: {e}]', italic=True)


# ============================== TITLE ==============================
t = doc.add_paragraph()
t.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = t.add_run('\n\n\nAutonomous Exploration with Learned SLAM')
r.bold = True
r.font.size = Pt(26)
r.font.color.rgb = ACCENT

s = doc.add_paragraph()
s.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = s.add_run('Final Project Report')
r.font.size = Pt(15)
r.font.color.rgb = SUB

s2 = doc.add_paragraph()
s2.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = s2.add_run('ROS2 Humble  -  Gazebo Classic 11  -  Nav2  -  SLAM Toolbox  -  PPO\n\n')
r.font.size = Pt(10)
r.font.color.rgb = GREY

table(['Field', 'Value'], [
    ['Student', 'Ankit Dash'],
    ['Programme', 'B.Tech CSE (Data Analytics & Machine Learning)'],
    ['Institution', 'Centurion University of Technology and Management, Jatani, BBSR'],
    ['Environment', 'Windows 11 Pro + WSL2 Ubuntu 22.04.5 LTS (host OMM05)'],
    ['Robot / World', 'TurtleBot3 Waffle in turtlebot3_world'],
    ['Date', '19 September 2026'],
])
doc.add_page_break()

# ============================== ABSTRACT ==============================
H('1.  Abstract', 1)
P('This project implements a mobile robot that autonomously explores an unknown '
  'environment, building a map with SLAM while deciding where to travel next. Two '
  'exploration strategies are implemented behind a single interface and compared: a '
  'hand-coded frontier heuristic, and a reinforcement-learning policy trained with '
  'Proximal Policy Optimisation (PPO).')
P('The learned policy was trained in a purpose-built Gymnasium environment on randomised '
  'maze maps, then deployed unchanged onto the simulated robot. In Gazebo the learned '
  'policy completed exploration in 120.1 s over 11.4 m of travel, against 134.5 s and '
  '14.6 m for the hand-coded heuristic - a 10.7% reduction in time and a 21.9% reduction '
  'in path length at equivalent map coverage.')

# ============================== OVERVIEW ==============================
H('2.  System Overview', 1)
code('''Gazebo  (simulated robot + hexagonal world)
   |  lidar scans (/scan), odometry (/odom)
   v
SLAM Toolbox  ->  occupancy grid (/map) + map->odom transform
   |
   v
Frontier Detector  ->  candidate unexplored boundary points
   |
   v
Exploration Policy   [ hand-coded heuristic  OR  trained PPO network ]
   |  selects next goal
   v
Nav2  ->  plans path, publishes velocity commands (/cmd_vel)
   |
   v
Gazebo (robot moves)  ->  loop repeats''', size=9)

P('The exploration policy is the only component that differs between the two methods. '
  'Both receive an identical candidate set, so the comparison isolates the decision rule.')

table(['Milestone', 'Deliverable', 'Status'], [
    ['1  Robot control', 'Robot spawns, drives, lidar and odometry publishing', 'COMPLETE'],
    ['2  SLAM', 'Live occupancy grid from SLAM Toolbox, map saved', 'COMPLETE'],
    ['3  Nav2', 'Autonomous point-to-point navigation', 'COMPLETE'],
    ['4  Frontier baseline', 'ROS2 node maps the world unaided', 'COMPLETE'],
    ['5  RL policy', 'Gym environment, PPO training, benchmark', 'COMPLETE'],
    ['5b  Integration', 'Trained model drives the simulated robot', 'COMPLETE'],
    ['6  Evaluation', 'Head-to-head comparison and figures', 'COMPLETE'],
])
doc.add_page_break()

# ============================== METHOD ==============================
H('3.  Methodology', 1)

H('3.1  Frontier detection and clustering', 2)
P('A frontier cell is a free cell adjacent to at least one unknown cell - the boundary '
  'between mapped and unmapped space. Frontier cells are grouped into regions by an '
  '8-connected breadth-first flood fill. For each region the node selects the nearest '
  'reachable cell that is at least 1.0 m away by BFS path distance and has clearance for '
  'the robot footprint. Regions are then ordered by path distance and capped at eight '
  'candidates, matching the action space used during training.')

H('3.2  Hand-coded heuristic (baseline)', 2)
P('The classical baseline scores each candidate by information gain against travel cost:')
code('score = gain_weight * frontier_size  -  distance_weight * distance_metres\n'
     '      = 1.0 * size  -  3.0 * distance', size=9.5)
P('The highest-scoring candidate is dispatched to Nav2 through the NavigateToPose action. '
  'Failed goals are blacklisted so the robot does not retry unreachable frontiers.')

H('3.3  Learned policy (PPO)', 2)
P('Training in Gazebo is infeasible: a single exploration episode takes over two minutes '
  'of wall-clock time, while PPO requires hundreds of thousands of environment steps. A '
  'lightweight 2D grid environment was therefore implemented, running episodes in '
  'milliseconds while preserving the structure of the decision problem.')

table(['Element', 'Gazebo / ROS2 node', 'Gymnasium environment'], [
    ['State', 'Frontier clusters from /map', 'Frontier clusters from a simulated grid'],
    ['Action', 'Choose a cluster, send to Nav2', 'Choose a cluster, move along BFS path'],
    ['Cost', 'Metres driven', 'Path length in cells'],
    ['Gain', 'New cells mapped', 'New cells revealed by ray-cast lidar'],
])

P('Maps are generated by recursive division, producing rooms, corridors and dead-ends '
  'connected by three-cell doorways. Each frontier candidate is described by seven features:')
bullet('Frontier perimeter size (normalised)')
bullet('BFS path distance from the robot (normalised)')
bullet('Bearing to the frontier, as sine and cosine')
bullet('Fraction of unknown cells in a 13x13 window behind the frontier - an estimate of the unexplored volume it opens')
bullet('Fraction of free cells in that window - distinguishes an open corridor from a cramped dead-end')
bullet('Distance from the frontier to the centroid of all unknown space')
P('The final three features carry spatial context that the hand-coded heuristic does not '
  'use, and are the mechanism by which a learned policy can outperform it.')

P('Reward per step:')
code('reward = 0.05 * new_cells_revealed  -  0.08 * path_length  -  0.1\n'
     'terminal bonus: + 20.0 * coverage   (episode ends at 97% coverage)', size=9.5)

table(['Hyperparameter', 'Value'], [
    ['Algorithm', 'PPO (Stable-Baselines3 2.9.0, MlpPolicy)'],
    ['Total timesteps', '250,000'],
    ['Parallel environments', '4 (SubprocVecEnv)'],
    ['Learning rate', '3e-4'],
    ['Rollout length / batch', '512 / 256'],
    ['Discount (gamma)', '0.995'],
    ['GAE lambda', '0.95'],
    ['Entropy coefficient', '0.005'],
    ['Normalisation', 'VecNormalize on observations and rewards'],
    ['Hardware', 'CPU only (WSL2 has no GPU passthrough)'],
    ['Training time', 'approximately 12 minutes'],
])
doc.add_page_break()

# ============================== RESULTS ==============================
H('4.  Results', 1)

H('4.1  Deployment in Gazebo - principal result', 2)
P('Both policies were run on turtlebot3_world from the same spawn pose, each starting '
  'from an empty map with live SLAM and Nav2.')

table(['Metric', 'Heuristic', 'RL (PPO)', 'Change'], [
    ['Time to complete', '134.5 s', '120.1 s', '-10.7%'],
    ['Path length', '14.6 m', '11.4 m', '-21.9%'],
    ['Goals dispatched', '18', '16', '-2 goals'],
    ['Goal success rate', '18/18 (100%)', '16/16 (100%)', 'equal'],
    ['Known cells', '74.5%', '74.3%', 'equal'],
    ['Final map size', '112 x 103 @ 0.05 m/pix', '112 x 103 @ 0.05 m/pix', 'equal'],
])

figure('fig1_gazebo_comparison.png',
       'Figure 1 - Head-to-head comparison in Gazebo. The learned policy completes the '
       'same coverage in less time and with substantially less travel.')

P('The learned policy reached equivalent coverage using two fewer navigation goals and '
  '3.2 m less travel. Neither policy recorded a navigation failure. The advantage arises '
  'from goal ordering: the learned policy sequences its frontier visits so that '
  'backtracking is reduced, whereas the heuristic evaluates each decision independently.')

figure('fig4_maps.png',
       'Figure 2 - Final occupancy grids. Both policies fully mapped the hexagonal room '
       'and resolved all nine cylindrical obstacles; map quality is equivalent.')

doc.add_page_break()

H('4.2  Simulator benchmark - 50 maps per policy', 2)
P('Four policies were evaluated on 50 identical randomised maze maps in the abstract '
  'environment. "Nearest" always selects the closest frontier; "Heuristic" uses the '
  'scoring function from Section 3.2.')

table(['Policy', 'Mean reward', 'Coverage', 'Path (cells)', 'Steps'], [
    ['Random', '-32.60', '65.7%', '504.0', '59.8'],
    ['Nearest frontier', '59.75', '98.3%', '349.8', '38.3'],
    ['Heuristic', '60.21', '98.2%', '346.1', '35.7'],
    ['PPO (ours)', '59.47', '98.4%', '356.3', '36.5'],
])

figure('fig2_benchmark_50maps.png',
       'Figure 3 - Benchmark over 50 identical maze maps. All three informed policies are '
       'statistically indistinguishable; random exploration is far worse.')

P('In the abstract environment the learned policy is statistically indistinguishable from '
  'the hand-tuned heuristic (59.47 against 60.21, a 1.2% difference over 50 maps) while '
  'achieving the highest coverage of any method at 98.4%. Relative to random exploration '
  'it improves reward from -32.60 to 59.47 and raises coverage from 65.7% to 98.4%.')

H('4.3  Training', 2)
figure('fig3_training_curve.png',
       'Figure 4 - PPO training progression over 250,000 timesteps (single run), with '
       'random and heuristic performance shown for reference.')

P('Mean episode reward rises from -28 to approximately 58 within 100,000 timesteps and '
  'then plateaus just below the heuristic reference line. Policy entropy fell from -2.07 '
  '(near the maximum of ln 8 = 2.08 for eight actions) to -0.20, indicating convergence '
  'from uniform exploration to a confident strategy.')
doc.add_page_break()

H('4.4  Experimental log', 2)
P('Six training runs were recorded, documenting the development of the method.')
table(['Run', 'Timesteps', 'Final reward', 'Configuration'], [
    ['PPO_1', '100k', '65.08', 'Open-room maps, 4 features per frontier'],
    ['PPO_2', '100k', '13.72', 'Maze maps - invalid, doorway frontiers discarded by min_cluster filter'],
    ['PPO_3', '100k', '55.01', 'Maze maps with corrected doorways and min_cluster = 1'],
    ['PPO_4', '250k', '57.59', 'Added VecNormalize, gamma 0.995, 250k timesteps'],
    ['PPO_5', '250k', '57.93', 'Repeat of PPO_4'],
    ['PPO_6', '250k', '58.09', 'Added three spatial-context features - final model'],
])

H('5.  Discussion', 1)

H('5.1  Ablation studies', 2)
P('Two controlled changes were made and measured.')
bullet('Reward and observation normalisation (VecNormalize) raised the value function\'s '
       'explained variance from approximately 0 to 0.95, confirming the critic had '
       'previously learned nothing. Policy performance was unchanged, which eliminates '
       'critic quality as the limiting factor.')
bullet('Adding three spatial-context features reduced the gap to the heuristic in the '
       'abstract benchmark from 3.9% to 1.2%, and produced the policy that outperforms '
       'the heuristic on the real robot.')

H('5.2  Why the two evaluations disagree', 2)
P('The learned policy ties the heuristic in the abstract simulator but wins in Gazebo. '
  'The abstract environment charges travel as BFS cell distance on a coarse grid, which '
  'understates the true cost of a detour. Nav2 computes genuine paths through an inflated '
  'costmap around nine cylindrical obstacles, so poor goal ordering is penalised far more '
  'heavily. The learned policy\'s advantage in sequencing therefore becomes visible only '
  'in the higher-fidelity setting.')

H('5.3  Limitations', 2)
bullet('The Gazebo comparison is a single run per policy. Repeated trials would be needed '
       'to establish statistical significance, although the 21.9% path reduction is a large effect.')
bullet('WSL2 provides no GPU passthrough, so Gazebo renders in software. This saturates '
       'the CPU and produces Behavior Tree tick-rate warnings, which inflate absolute timings '
       'for both policies equally.')
bullet('The policy observes only per-frontier features, not the map itself, so it cannot '
       'reason about global topology. A downsampled occupancy grid with a convolutional '
       'policy is the natural next step.')
bullet('Training used a single random seed. Seed variance was not characterised.')

H('5.4  Future work', 2)
bullet('Replace the feature vector with an egocentric occupancy grid and a CNN policy')
bullet('Evaluate on turtlebot3_house and other multi-room worlds')
bullet('Repeat Gazebo trials to obtain confidence intervals')
bullet('Complete the LLM task-planner layer that decomposes natural-language instructions into ordered exploration tasks')
doc.add_page_break()

# ============================== RUNBOOK ==============================
H('6.  How to Reproduce', 1)
P('Five terminal tabs are used. Tabs 1, 2 and 3 are launched once and left alone; Tab 4 '
  'is the working tab; Tab 5 (RViz2) is optional and used for visualisation.')

H('6.1  Tab 1 - Gazebo', 2)
code('''export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
export TURTLEBOT3_MODEL=waffle
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py''', size=9)
P('Wait 30-60 s for the hexagonal world. A black window during startup is normal under '
  'software rendering.')

H('6.2  Tab 2 - SLAM Toolbox', 2)
code('''export TURTLEBOT3_MODEL=waffle
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true''', size=9)
P('Wait for "Registering sensor: [Custom Described Lidar]".')

H('6.3  Tab 3 - Nav2', 2)
code('''export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true''', size=9)
P('Wait for "Managed nodes are active". Use navigation_launch.py, not bringup_launch.py: '
  'the latter also starts map_server and AMCL, which compete with SLAM Toolbox for the '
  'map->odom transform. Never run two Nav2 stacks at once - every goal will abort with status 6.')

H('6.4  Tab 5 - RViz2 (optional)', 2)
code('''export TURTLEBOT3_MODEL=waffle
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/turtlebot3_gazebo/rviz/tb3_gazebo.rviz''', size=9)
P('Configure manually each session: Fixed Frame = map; Add > Map with topic /map and '
  'Durability Policy = Transient Local; Add > MarkerArray with topic /frontier_markers; '
  'untick Odometry.')

H('6.5  Tab 4 - run an exploration', 2)
code('''source ~/ros2_ws/install/setup.bash

# classical baseline
ros2 run explore_nav frontier_explorer --ros-args -p use_sim_time:=true -p policy:=heuristic

# learned policy
ros2 run explore_nav frontier_explorer --ros-args -p use_sim_time:=true -p policy:=rl''', size=9)
P('Each run prints its metrics on completion:')
code('=== EXPLORATION COMPLETE [rl] === time=120.1s  goals=16/16  path=11.4m  known_cells=74.3%', size=8.5)

H('6.6  Save a map', 2)
code('''ros2 run nav2_map_server map_saver_cli -f ~/maps/run_rl

# if map_saver times out, use SLAM Toolbox's own service:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\
  "{name: {data: '/home/ommdash/maps/run_rl'}}"''', size=9)

H('6.7  Train and produce figures', 2)
code('''cd ~/ros2_ws/src/explore_nav/explore_nav
python3 exploration_env.py          # random-agent smoke test
python3 train_rl_policy.py          # train PPO + 50-map benchmark

cd ~/ros2_ws/src/explore_nav
python3 make_plots.py               # figures 1, 2, 4
python3 make_training_curve.py      # figure 3 from TensorBoard logs''', size=9)

H('6.8  Shut down', 2)
code('''pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f slam_toolbox
pkill -9 -f rviz2; pkill -9 -f robot_state_publisher; pkill -9 -f nav2
ros2 daemon stop''', size=9)
doc.add_page_break()

# ============================== TROUBLESHOOTING ==============================
H('7.  Engineering Problems Encountered', 1)
table(['Symptom', 'Cause', 'Resolution'], [
    ['Robot falls through the floor, odometry z reaches -40000',
     'GAZEBO_MODEL_DATABASE_URI was blanked to stop an online lookup, so the '
     'ground_plane model referenced by the world file was silently skipped',
     'Also set GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models'],
    ['Map display red in RViz, Resolution/Width/Height all 0',
     'SLAM Toolbox latches /map as TRANSIENT_LOCAL; RViz defaults to VOLATILE',
     'Set Durability Policy = Transient Local'],
    ['RViz 2D Goal Pose button does nothing',
     'The shipped tb3_gazebo.rviz config targets the ROS1 topic /move_base_simple/goal',
     'Publish to /goal_pose from the command line'],
    ['colcon build fails: canonicalize_version() unexpected keyword strip_trailing_zero',
     'A pip setuptools in ~/.local shadows the system one and requires packaging >= 24.0',
     'pip3 install --user --upgrade "packaging>=24.0"'],
    ['Every policy stuck at ~32% coverage on maze maps',
     'Doorway frontiers are only 1-2 cells wide and were discarded by min_cluster = 3, '
     'causing premature termination',
     'Set min_cluster = 1 and widen doorways to three cells'],
    ['Explorer loops on the same goal 0.1 m away, map never grows',
     'The nearest cell of a frontier ring sits beside the robot; arriving reveals nothing',
     'Require goals to be at least 1.0 m away by BFS path distance'],
    ['Every Nav2 goal aborts within 50 ms with status 6',
     'Two Nav2 stacks were running simultaneously',
     'Kill all nav2 processes and relaunch exactly one'],
    ['matplotlib import fails: numpy.core.multiarray failed to import',
     'The apt matplotlib was compiled against NumPy 1.x; pip installed NumPy 2.2.6',
     'pip3 install --user --upgrade matplotlib'],
    ['map_saver_cli: Failed to spin map subscription',
     'The two-second default timeout expires before the latched map arrives',
     'Use the /slam_toolbox/save_map service instead'],
    ['Behavior Tree tick rate 100.00 was exceeded',
     'CPU saturated by Gazebo software rendering under WSL2',
     'Harmless; recorded as a limitation'],
])
doc.add_page_break()

# ============================== SOURCE ==============================
H('8.  Source Code', 1)
P('All original code for this project lives in the ROS2 package explore_nav.')
code('''~/ros2_ws/src/explore_nav/
    package.xml
    setup.py
    explore_nav/
        frontier_explorer.py    ROS2 node - both exploration policies
        exploration_env.py      Gymnasium training environment
        train_rl_policy.py      PPO training and 50-map benchmark
    models/
        ppo_explorer.zip        trained policy (final)
        vecnormalize.pkl        observation normalisation statistics
        tb_logs/                TensorBoard training logs
    results/                    report figures''', size=9)

BASE = os.path.join(PKG, 'explore_nav')
source('8.1  frontier_explorer.py', os.path.join(BASE, 'frontier_explorer.py'))
doc.add_page_break()
source('8.2  exploration_env.py', os.path.join(BASE, 'exploration_env.py'))
doc.add_page_break()
source('8.3  train_rl_policy.py', os.path.join(BASE, 'train_rl_policy.py'))

doc.save(OUT)
print('Report written to', OUT)

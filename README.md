# Autonomous Exploration with Learned SLAM

A mobile robot that explores an unknown environment on its own — building a map
with SLAM while a **reinforcement learning policy decides where to go next**,
and a natural-language planner that turns one English sentence into an ordered
mission.

Built on **ROS2 Humble · Gazebo Classic 11 · Nav2 · SLAM Toolbox · PPO**

---

## Headline result

The learned policy explores the same area **10.7% faster** using **21.9% less
travel** than a hand-tuned frontier heuristic — on the real simulated robot,
with identical coverage and a 100% navigation success rate.

| Metric | Heuristic | RL (PPO) | Change |
|---|---|---|---|
| Time to complete | 134.5 s | **120.1 s** | **−10.7%** |
| Path length | 14.6 m | **11.4 m** | **−21.9%** |
| Goals dispatched | 18 | 16 | −2 |
| Goal success rate | 18/18 | 16/16 | both 100% |
| Coverage | 74.5% | 74.3% | equal |

![Gazebo comparison](04_Figures/fig1_gazebo_comparison.png)

Both policies produce an equivalent map — the learned one just gets there with
less driving:

![Final maps](04_Figures/fig4_maps.png)

---

## Architecture

```
Gazebo  (simulated robot + hexagonal world)
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
Gazebo (robot moves)  ->  loop repeats
```

The exploration policy is the **only** component that differs between methods,
so the comparison isolates the decision rule.

---

## How the learned policy works

A frontier is a free cell adjacent to an unknown cell — the boundary of the
map. Frontiers are clustered by flood fill, and the agent picks **which cluster
to drive to** (a macro-action, not a velocity command).

Each candidate is described by seven features:

| Feature | What it captures |
|---|---|
| Frontier size | Perimeter of the unexplored region |
| BFS path distance | True travel cost, not straight-line |
| Bearing (sin, cos) | Direction relative to the robot |
| Unknown fraction behind it | How much space it actually opens up |
| Local free-space ratio | Open corridor vs. cramped dead-end |
| Distance to unknown centroid | Points toward the unexplored bulk |

The last three carry spatial context the hand-coded heuristic does **not** have
— the mechanism by which a learned policy can beat it.

**Reward:** `0.05 × new_cells − 0.08 × path_length − 0.1`, plus
`20 × coverage` on completion.

**Training:** PPO, 250k timesteps, 4 parallel envs, CPU only, ~12 minutes.
Trained in a fast 2D grid simulator (Gazebo runs at 1× real time — one episode
takes over two minutes, making direct training infeasible), then deployed
unchanged onto the simulated robot.

![Training curve](04_Figures/fig3_training_curve.png)

---

## Benchmark — 50 maps per policy

![Benchmark](04_Figures/fig2_benchmark_50maps.png)

| Policy | Mean reward | Coverage | Path | Steps |
|---|---|---|---|---|
| Random | −32.60 | 65.7% | 504.0 | 59.8 |
| Nearest frontier | 59.75 | 98.3% | 349.8 | 38.3 |
| Heuristic | **60.21** | 98.2% | **346.1** | **35.7** |
| PPO (ours) | 59.47 | **98.4%** | 356.3 | 36.5 |

In the abstract simulator PPO *ties* the heuristic while achieving the highest
coverage of any method. On the real robot it *wins*. That gap is the most
interesting finding in the project: the abstract environment charges travel as
coarse cell distance, understating detour cost, while Nav2 computes genuine
paths through an inflated costmap — so poor goal ordering is punished far more
heavily, and the learned policy's sequencing advantage only becomes visible at
higher fidelity.

---

## Natural-language agentic layer

One English instruction becomes an ordered, executable mission:

```
"explore the upper left first, avoid the lower right,
 stop once you have mapped 85 percent, then come back home"
```

```json
[{"action": "explore_region",  "region": "top_left"},
 {"action": "avoid_region",    "region": "bottom_right"},
 {"action": "explore_until",   "coverage": 0.85},
 {"action": "return_to_start"},
 {"action": "stop"}]
```

Result: **10/10 goals**, 122.7 s, 11.7 m, all five tasks executed in order.

The planner has two backends — the Anthropic API when `ANTHROPIC_API_KEY` is
set, and a deterministic keyword parser otherwise, so it never fails for lack
of a network. The executor subclasses the exploration node, so the agentic
layer composes with **either** policy.

---

## Repository layout

| Folder | Contents |
|---|---|
| `01_Source_Code` | ROS2 package `explore_nav` — all original code |
| `02_Trained_Models` | PPO models, normalisation stats, TensorBoard logs (6 runs) |
| `03_Maps` | Maps produced by each strategy (.pgm/.yaml + .png previews) |
| `04_Figures` | Report figures |
| `05_Report` | 27-page project report |
| `06_Build_Scripts` | Figure/report generation and a pre-demo verification script |
| `07_Documentation` | How to run, full results, troubleshooting |

---

## Running it

Full step-by-step instructions: **[07_Documentation/01_HOW_TO_RUN.md](07_Documentation/01_HOW_TO_RUN.md)**

```bash
# classical baseline
ros2 run explore_nav frontier_explorer --ros-args -p use_sim_time:=true -p policy:=heuristic

# learned policy
ros2 run explore_nav frontier_explorer --ros-args -p use_sim_time:=true -p policy:=rl

# natural-language mission
ros2 run explore_nav task_executor --ros-args -p use_sim_time:=true -p policy:=rl
ros2 run explore_nav llm_task_planner "explore the left side, stop at 80 percent, then return to start"
```

---

## Limitations

- The Gazebo comparison is a single run per policy; repeated trials would be
  needed for statistical significance, though a 21.9% path reduction is a large
  effect.
- WSL2 has no GPU passthrough, so Gazebo renders in software — this inflates
  absolute timings equally for both policies.
- The policy observes per-frontier features, not the map itself, so it cannot
  reason about global topology. A downsampled occupancy grid with a CNN policy
  is the natural next step.
- Training used a single random seed; seed variance was not characterised.

---

**Ankit Dash** — B.Tech CSE (Data Analytics & Machine Learning), 2027
Centurion University of Technology and Management, Jatani, BBSR

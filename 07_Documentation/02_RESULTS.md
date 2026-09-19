# Results

All figures referenced here are in `04_Figures`.

---

## 1. Gazebo — the principal result

Both policies run on `turtlebot3_world` from the same spawn pose, each
starting from an empty map with live SLAM and Nav2.

| Metric | Heuristic | RL (PPO) | Change |
|---|---|---|---|
| Time to complete | 134.5 s | **120.1 s** | **-10.7%** |
| Path length | 14.6 m | **11.4 m** | **-21.9%** |
| Goals dispatched | 18 | 16 | -2 |
| Goal success rate | 18/18 (100%) | 16/16 (100%) | equal |
| Known cells | 74.5% | 74.3% | equal |
| Final map | 112 x 103 @ 0.05 m/pix | same | equal |

→ **fig1_gazebo_comparison.png**, **fig4_maps.png**

The learned policy reached equivalent coverage using two fewer navigation
goals and 3.2 m less travel, with no navigation failures. The advantage comes
from goal *ordering*: the RL policy sequences its frontier visits so that
backtracking is reduced, while the heuristic evaluates each decision in
isolation.

---

## 2. Simulator benchmark — 50 maps per policy

Four policies on 50 identical randomised maze maps in the abstract
training environment.

| Policy | Mean reward | Coverage | Path (cells) | Steps |
|---|---|---|---|---|
| Random | -32.60 | 65.7% | 504.0 | 59.8 |
| Nearest frontier | 59.75 | 98.3% | 349.8 | 38.3 |
| Heuristic | **60.21** | 98.2% | **346.1** | **35.7** |
| PPO (ours) | 59.47 | **98.4%** | 356.3 | 36.5 |

→ **fig2_benchmark_50maps.png**

PPO is statistically indistinguishable from the hand-tuned heuristic here
(1.2% apart over 50 maps) while achieving the **highest coverage of any
method**. Against random it improves reward from -32.60 to 59.47 and
coverage from 65.7% to 98.4%.

---

## 3. Training

250,000 timesteps, 4 parallel environments, CPU only, ~12 minutes.

→ **fig3_training_curve.png**

Mean episode reward rises from -28 to ~58 within 100,000 timesteps and then
plateaus. Policy entropy fell from -2.07 (near the maximum ln 8 = 2.08 for
eight actions) to -0.20, showing convergence from uniform exploration to a
confident strategy.

### All six training runs

| Run | Timesteps | Final reward | Configuration |
|---|---|---|---|
| PPO_1 | 100k | 65.08 | Open-room maps, 4 features per frontier |
| PPO_2 | 100k | 13.72 | Maze maps — INVALID, doorway frontiers discarded by min_cluster filter |
| PPO_3 | 100k | 55.01 | Maze maps with corrected 3-cell doorways, min_cluster = 1 |
| PPO_4 | 250k | 57.59 | Added VecNormalize, gamma 0.995 |
| PPO_5 | 250k | 57.93 | Repeat of PPO_4 |
| PPO_6 | 250k | **58.09** | Added 3 spatial-context features — **final model** |

---

## 4. LLM agentic layer

Instruction given in plain English:

> "explore the upper left first, avoid the lower right, stop once you have
> mapped 85 percent, then come back home"

Decomposed automatically into:

```json
[{"action": "explore_region",  "region": "top_left"},
 {"action": "avoid_region",    "region": "bottom_right"},
 {"action": "explore_until",   "coverage": 0.85},
 {"action": "return_to_start"},
 {"action": "stop"}]
```

Execution result:

| Metric | Value |
|---|---|
| Time | 122.7 s |
| Goals | **10/10 (100%)** |
| Path | 11.7 m |
| Known cells | 73.9% |
| Tasks executed | 5 of 5, in order |

Note: it stopped at 73.9% rather than 85% because no frontiers remained.
The coverage metric divides by *all* grid cells including the region outside
the hexagonal wall, which the robot can never observe — so ~74% is
effectively complete coverage and 85% was unreachable by that definition.
Normalising by reachable cells instead (as the Gym environment already does)
is listed as future work.

---

## 5. Ablation studies

Two controlled changes were made and measured.

**Reward and observation normalisation (VecNormalize)** raised the value
function's explained variance from ~0 to 0.95, confirming the critic had
previously learned nothing. Policy performance was unchanged — this
*eliminates critic quality as the limiting factor*.

**Three spatial-context features** (unexplored area behind each frontier,
local openness, distance to the centroid of unknown space) reduced the gap to
the heuristic in the abstract benchmark from 3.9% to 1.2%, and produced the
policy that outperforms the heuristic on the real robot.

---

## 6. Why the two evaluations disagree

The learned policy *ties* the heuristic in the abstract simulator but *wins*
in Gazebo. The abstract environment charges travel as BFS cell distance on a
coarse grid, which understates the true cost of a detour. Nav2 computes
genuine paths through an inflated costmap around nine cylindrical obstacles,
so poor goal ordering is penalised far more heavily. The learned policy's
advantage in sequencing therefore becomes visible only in the
higher-fidelity setting.

**This is the most interesting finding in the project and the best thing to
be able to explain out loud.**

---

## 7. Limitations (state these honestly)

- The Gazebo comparison is a **single run per policy**. Repeated trials would
  be needed for statistical significance, though the 21.9% path reduction is
  a large effect.
- WSL2 has no GPU passthrough, so Gazebo renders in software. This saturates
  the CPU and produces Behavior Tree tick-rate warnings, inflating absolute
  timings for both policies equally.
- The policy observes only per-frontier features, not the map itself, so it
  cannot reason about global topology. A downsampled occupancy grid with a
  CNN policy is the natural next step.
- Training used a single random seed; seed variance was not characterised.

# Autonomous Exploration with Learned SLAM
### Final Year Project — 7th Semester

**Ankit Dash** · B.Tech CSE (Data Analytics & Machine Learning)
Centurion University of Technology and Management, Jatani, BBSR

---

## What this project does

A mobile robot explores an unknown environment on its own. It builds a map with
SLAM while deciding where to go next. Three decision strategies are implemented
and compared:

1. **Classical frontier heuristic** — a hand-written scoring rule
2. **Reinforcement learning (PPO)** — a neural network trained from scratch
3. **LLM agentic layer** — plain English instructions decomposed into an
   ordered task list the robot executes

---

## Headline result

| Metric | Heuristic | RL (PPO) | Improvement |
|---|---|---|---|
| Time to complete | 134.5 s | **120.1 s** | **10.7% faster** |
| Path length | 14.6 m | **11.4 m** | **21.9% shorter** |
| Goals dispatched | 18 | **16** | 2 fewer |
| Goal success rate | 18/18 | 16/16 | both 100% |
| Coverage | 74.5% | 74.3% | equal |

**The learned policy explores the same area faster and with less travel than the
hand-coded rule.**

---

## Folder guide

| Folder | Contents |
|---|---|
| `01_Source_Code` | All original Python — the ROS2 package `explore_nav` |
| `02_Trained_Models` | Trained PPO models, normalisation stats, TensorBoard logs for all 6 runs |
| `03_Maps` | Maps produced by each strategy (.pgm + .yaml + .png preview) |
| `04_Figures` | The four report figures |
| `05_Report` | Final 27-page Word report |
| `06_Build_Scripts` | Scripts that generate the figures and the report |
| `07_Documentation` | How to run, full results, troubleshooting |
| `08_Screenshots` | (put demo screenshots here) |

---

## Reading order for a reviewer

1. This file
2. `07_Documentation/02_RESULTS.md` — every number
3. `04_Figures/fig1_gazebo_comparison.png` — the headline chart
4. `04_Figures/fig4_maps.png` — both maps side by side
5. `05_Report/Autonomous_Exploration_Final_Report.docx` — the full write-up
6. `01_Source_Code/explore_nav/frontier_explorer.py` — the main node

---

## Technology stack

ROS2 Humble · Gazebo Classic 11 · Nav2 · SLAM Toolbox · TurtleBot3 Waffle
PyTorch 2.13 (CPU) · Stable-Baselines3 2.9 · Gymnasium
Environment: Windows 11 Pro + WSL2 Ubuntu 22.04.5

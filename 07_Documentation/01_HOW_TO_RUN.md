# How to run the demo

Five Windows Terminal tabs. Open a tab with the `+` button, then type `wsl`.

**Rule: Tabs 1, 2 and 3 are launched once and then left alone.
Tab 4 and 5 are where you type.**

---

## STEP 0 — always start clean

Run this in any tab before a demo. Two Nav2 stacks running at once makes
every goal abort, so never skip it.

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f slam_toolbox
pkill -9 -f rviz2; pkill -9 -f robot_state_publisher; pkill -9 -f nav2
pkill -9 -f controller_server; pkill -9 -f planner_server
pkill -9 -f bt_navigator; pkill -9 -f lifecycle_manager
ros2 daemon stop
```

Then `ros2 node list` should print nothing (or "daemon is not running").

---

## TAB 1 — Gazebo

```bash
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
export TURTLEBOT3_MODEL=waffle
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Wait 30-60 s for the hexagonal room. A black window at first is normal -
WSL2 has no GPU so Gazebo renders in software.

**Do not close or type in this tab again.**

---

## TAB 2 — SLAM

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

Wait for `Registering sensor: [Custom Described Lidar]`.
The two laser-range warnings are normal.

---

## TAB 3 — Nav2

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

Wait for `Managed nodes are active`.

Use `navigation_launch.py`, NOT `bringup_launch.py` — the latter also starts
map_server and AMCL, which fight SLAM Toolbox for the map->odom transform.

---

## TAB 4 — RViz2 (visualisation)

```bash
export TURTLEBOT3_MODEL=waffle
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/turtlebot3_gazebo/rviz/tb3_gazebo.rviz
```

Configure by hand — this must be redone every session:

1. **Fixed Frame** -> `map`
2. **Add** -> **Map** -> **OK** -> Topic `/map` -> **Durability Policy = Transient Local**
3. **Add** -> **MarkerArray** -> **OK** -> Topic `/frontier_markers`
4. **Untick Odometry**

If the Map display is red with Resolution 0, the Durability Policy is wrong.

---

## TAB 5 — run the robot

Source the workspace first, every time:

```bash
source ~/ros2_ws/install/setup.bash
```

### Demo A — classical heuristic
```bash
ros2 run explore_nav frontier_explorer --ros-args -p use_sim_time:=true -p policy:=heuristic
```

### Demo B — learned RL policy
```bash
ros2 run explore_nav frontier_explorer --ros-args -p use_sim_time:=true -p policy:=rl
```

First three lines must say:
```
Loaded PPO model from .../ppo_explorer.zip
Loaded observation normalisation stats
Frontier explorer started. POLICY = RL
```

Each run ends with its metrics:
```
=== EXPLORATION COMPLETE [rl] === time=120.1s goals=16/16 path=11.4m known_cells=74.3%
```

### Demo C — LLM agentic layer (two tabs)

**Tab 5** — start the executor, it waits for a plan:
```bash
ros2 run explore_nav task_executor --ros-args -p use_sim_time:=true -p policy:=rl
```

**Tab 6** — send one English instruction:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run explore_nav llm_task_planner "explore the upper left first, avoid the lower right, stop once you have mapped 85 percent, then come back home"
```

The planner decomposes it into 5 ordered tasks and the executor runs them.

---

## Saving a map

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/ommdash/maps/my_run'}}"
```

Use this rather than `map_saver_cli` — the CLI tool has a 2 second timeout
that often expires before the latched map arrives.

---

## Shutting down

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f slam_toolbox
pkill -9 -f rviz2; pkill -9 -f robot_state_publisher; pkill -9 -f nav2
ros2 daemon stop
```

---

## If the code needs rebuilding

```bash
cd ~/ros2_ws
colcon build --packages-select explore_nav --symlink-install
source install/setup.bash
```

`--symlink-install` means Python edits take effect without rebuilding.

# Troubleshooting

Every problem encountered during development, with cause and fix.
**Read this before a demo.**

---

## Demo-day checklist

1. Run the STEP 0 kill block — never start with an old stack running
2. Launch in order: Gazebo -> SLAM -> Nav2 -> RViz2 -> node
3. Wait for each tab's ready line before starting the next
4. Re-add the RViz displays (they are not saved)
5. `source ~/ros2_ws/install/setup.bash` in the tab you type in

---

## Problems and fixes

### Robot falls through the floor, odometry z reaches -40000
**Cause:** `GAZEBO_MODEL_DATABASE_URI` was blanked to stop an online lookup,
so the `ground_plane` model referenced by the world file was silently skipped.
**Fix:** also set `GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models`.
**Verify:** `ros2 service call /get_model_list gazebo_msgs/srv/GetModelList`
must list `ground_plane`, and odom z should sit at about 0.008-0.010.

---

### Map display red in RViz, Resolution/Width/Height all 0
**Cause:** SLAM Toolbox latches `/map` as TRANSIENT_LOCAL; RViz defaults to
VOLATILE, so the two never connect.
**Fix:** set the Map display's **Durability Policy = Transient Local**.

---

### RViz "2D Goal Pose" button does nothing
**Cause:** the shipped `tb3_gazebo.rviz` config targets the ROS1-era topic
`/move_base_simple/goal`.
**Fix:** publish to `/goal_pose` from the command line instead.

---

### Every Nav2 goal aborts within 50 ms with status 6
**Cause:** two Nav2 stacks running at once.
**Fix:** kill all nav2 processes and relaunch exactly one.
**This is the most likely demo-day failure — always run STEP 0 first.**

---

### Explorer loops on the same goal 0.1 m away, map never grows
**Cause:** the nearest cell of a frontier ring sits right beside the robot;
arriving there reveals nothing.
**Fix:** goals must be at least 1.0 m away by BFS path distance
(`min_cells` in `build_candidates`).

---

### Agentic plan finishes instantly with 0 goals
**Cause:** when the requested region had no frontiers, the executor released
the restriction *and* ended the task in the same step.
**Fix:** release the restriction and keep the task running
(`return` instead of `self.active = None`).

---

### colcon build: `canonicalize_version() got an unexpected keyword argument`
**Cause:** a pip-installed setuptools in `~/.local` shadows the system one and
needs `packaging >= 24.0`.
**Fix:** `pip3 install --user --upgrade "packaging>=24.0"`

---

### matplotlib: `numpy.core.multiarray failed to import`
**Cause:** the apt matplotlib was compiled against NumPy 1.x; pip installed
NumPy 2.2.6.
**Fix:** `pip3 install --user --upgrade matplotlib`
(Do NOT downgrade NumPy — the trained model pipeline depends on NumPy 2.)

---

### map_saver_cli: `Failed to spin map subscription`
**Cause:** its 2 second default timeout expires before the latched map arrives.
**Fix:** use SLAM Toolbox's own service:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/ommdash/maps/NAME'}}"
```
A response of `result=0` means success.

---

### All policies stuck at ~32% coverage on maze maps
**Cause:** doorway frontiers are only 1-2 cells wide and were discarded by the
`min_cluster = 3` filter, causing premature termination.
**Fix:** `min_cluster = 1` and widen generated doorways to 3 cells.

---

### Robot does not spawn (spawn_entity timeout)
Known intermittent TurtleBot3 launch race. Spawn manually:
```bash
ros2 run gazebo_ros spawn_entity.py -entity waffle \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf \
  -x -2.0 -y -0.5 -z 0.01
```

---

## Harmless messages — ignore these

| Message | Why it appears |
|---|---|
| `Behavior Tree tick rate 100.00 was exceeded` | CPU saturated by software rendering under WSL2 |
| `Message Filter dropping message: frame 'base_scan'` | Scans arrived before the transform buffer filled; startup only |
| `minimum/maximum laser range setting exceeds capabilities` | SLAM defaults are wider than the TurtleBot3's 0.12-3.5 m lidar; it clips automatically |
| `No goal checker was specified` | Nav2 falls back to the only loaded plugin |
| `Unable to import Axes3D` | Two matplotlib versions installed; only 3D plotting affected, which is not used |

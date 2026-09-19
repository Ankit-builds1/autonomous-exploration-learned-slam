#!/usr/bin/env python3
"""
Autonomous frontier exploration for TurtleBot3 + Nav2 + SLAM Toolbox.

Two interchangeable decision policies, selected with the `policy` parameter:

  policy:=heuristic   score = gain_weight*size - distance_weight*distance
  policy:=rl          a trained PPO network chooses among the candidates

Both see identical candidates, so the comparison is fair.
"""

import math
import os
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener

UNKNOWN = -1
FREE_MAX = 25
OCC_MIN = 65
K = 8            # max candidates - must match the training environment
R_CTX = 6        # context window radius - must match training


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('policy', 'heuristic')
        self.declare_parameter('model_path', os.path.expanduser(
            '~/ros2_ws/src/explore_nav/models/ppo_explorer.zip'))
        self.declare_parameter('vecnorm_path', os.path.expanduser(
            '~/ros2_ws/src/explore_nav/models/vecnormalize.pkl'))
        self.declare_parameter('min_frontier_size', 1)
        self.declare_parameter('robot_radius_cells', 4)
        self.declare_parameter('gain_weight', 1.0)
        self.declare_parameter('distance_weight', 3.0)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('planning_period', 2.0)

        self.policy = self.get_parameter('policy').value
        self.min_size = self.get_parameter('min_frontier_size').value
        self.radius = self.get_parameter('robot_radius_cells').value
        self.w_gain = self.get_parameter('gain_weight').value
        self.w_dist = self.get_parameter('distance_weight').value
        self.bl_radius = self.get_parameter('blacklist_radius').value
        period = self.get_parameter('planning_period').value

        self.model = None
        self.obs_mean = None
        self.obs_var = None
        if self.policy == 'rl':
            self._load_model()

        self.grid = None
        self.navigating = False
        self.blacklist = []
        self.goals_sent = 0
        self.goals_done = 0
        self.path_len = 0.0
        self.last_pos = None
        self.start_time = None

        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/frontier_markers', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(period, self.plan_cycle)
        self.get_logger().info(
            f'Frontier explorer started. POLICY = {self.policy.upper()}')

    # ======================================================
    # Model loading
    # ======================================================

    def _load_model(self):
        try:
            import pickle
            from stable_baselines3 import PPO
            mp = self.get_parameter('model_path').value
            self.model = PPO.load(mp, device='cpu')
            self.get_logger().info(f'Loaded PPO model from {mp}')

            vp = self.get_parameter('vecnorm_path').value
            if os.path.exists(vp):
                with open(vp, 'rb') as f:
                    vn = pickle.load(f)
                self.obs_mean = vn.obs_rms.mean
                self.obs_var = vn.obs_rms.var
                self.get_logger().info('Loaded observation normalisation stats')
            else:
                self.get_logger().warn(
                    'vecnormalize.pkl not found - running without obs '
                    'normalisation (performance will be degraded)')
        except Exception as e:
            self.get_logger().error(
                f'Could not load RL model ({e}) - falling back to heuristic')
            self.policy = 'heuristic'
            self.model = None

    # ======================================================
    # Callbacks / helpers
    # ======================================================

    def map_cb(self, msg):
        self.grid = msg

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def at(self, mx, my):
        g = self.grid
        if mx < 0 or my < 0 or mx >= g.info.width or my >= g.info.height:
            return UNKNOWN
        return g.data[my * g.info.width + mx]

    def to_world(self, mx, my):
        g = self.grid
        return (g.info.origin.position.x + (mx + 0.5) * g.info.resolution,
                g.info.origin.position.y + (my + 0.5) * g.info.resolution)

    def to_cell(self, wx, wy):
        g = self.grid
        return (int((wx - g.info.origin.position.x) / g.info.resolution),
                int((wy - g.info.origin.position.y) / g.info.resolution))

    def is_clear(self, mx, my):
        for dy in range(-self.radius, self.radius + 1):
            for dx in range(-self.radius, self.radius + 1):
                if self.at(mx + dx, my + dy) >= OCC_MIN:
                    return False
        return True

    # ======================================================
    # Frontier detection and clustering
    # ======================================================

    def find_frontier_cells(self):
        g = self.grid
        cells = set()
        W, H = g.info.width, g.info.height
        for my in range(H):
            row = my * W
            for mx in range(W):
                v = g.data[row + mx]
                if v < 0 or v > FREE_MAX:
                    continue
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    if self.at(mx + dx, my + dy) == UNKNOWN:
                        cells.add((mx, my))
                        break
        return cells

    def cluster(self, cells):
        clusters = []
        unvisited = set(cells)
        while unvisited:
            seed = unvisited.pop()
            group = [seed]
            q = deque([seed])
            while q:
                cx, cy = q.popleft()
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        n = (cx + dx, cy + dy)
                        if n in unvisited:
                            unvisited.remove(n)
                            group.append(n)
                            q.append(n)
            if len(group) >= self.min_size:
                clusters.append(group)
        return clusters

    def bfs_distances(self, start_cell):
        """Path length in cells from the robot to every reachable free cell."""
        g = self.grid
        W, H = g.info.width, g.info.height
        dist = np.full(W * H, -1, dtype=np.int32)
        sx, sy = start_cell
        if not (0 <= sx < W and 0 <= sy < H):
            return dist
        dist[sy * W + sx] = 0
        q = deque([(sx, sy)])
        while q:
            cx, cy = q.popleft()
            d = dist[cy * W + cx]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < W and 0 <= ny < H:
                    idx = ny * W + nx
                    if dist[idx] == -1:
                        v = g.data[idx]
                        if 0 <= v <= FREE_MAX:
                            dist[idx] = d + 1
                            q.append((nx, ny))
        return dist

    # ======================================================
    # Candidate construction (must mirror the Gym environment)
    # ======================================================

    def build_candidates(self, rx, ry):
        g = self.grid
        W, H = g.info.width, g.info.height
        clusters = self.cluster(self.find_frontier_cells())
        if not clusters:
            return []

        dist = self.bfs_distances(self.to_cell(rx, ry))

        known = np.array(g.data, dtype=np.int16).reshape(H, W)
        unknown_idx = np.argwhere(known == UNKNOWN)
        if len(unknown_idx):
            centroid = unknown_idx.mean(axis=0)      # (row, col) = (my, mx)
        else:
            centroid = np.array([H / 2.0, W / 2.0])

        cands = []
        # a goal must be far enough away to actually reveal new area
        min_cells = max(4, int(1.0 / g.info.resolution))
        for grp in clusters:
            # nearest reachable cell that is still far enough to reveal
            # new territory - a frontier ring's centroid sits on the robot,
            # so centroid-aiming makes no progress
            best = None
            for (mx, my) in grp:
                d = dist[my * W + mx]
                if d >= min_cells and self.is_clear(mx, my):
                    if best is None or d < best[0]:
                        best = (d, mx, my)
            if best is None:
                continue
            d, mx, my = best

            wx, wy = self.to_world(mx, my)
            if self.blacklisted(wx, wy):
                continue

            y0, y1 = max(0, my - R_CTX), min(H, my + R_CTX + 1)
            x0, x1 = max(0, mx - R_CTX), min(W, mx + R_CTX + 1)
            win = known[y0:y1, x0:x1]
            n = max(win.size, 1)

            cands.append({
                'd': int(d),
                'cell': (mx, my),
                'world': (wx, wy),
                'size': len(grp),
                'unk': float(np.count_nonzero(win == UNKNOWN)) / n,
                'free': float(np.count_nonzero((win >= 0) & (win <= FREE_MAX))) / n,
                'dcent': math.hypot(my - centroid[0], mx - centroid[1]),
            })

        cands.sort(key=lambda c: c['d'])
        return cands[:K]

    def build_obs(self, cands, rx, ry):
        """The same 7-features-per-candidate vector used during training."""
        g = self.grid
        W, H = g.info.width, g.info.height
        rcx, rcy = self.to_cell(rx, ry)
        max_d = float(W + H)
        diag = float(math.hypot(W, H))

        v = np.zeros(K * 7 + 2, dtype=np.float32)
        for i, cd in enumerate(cands):
            mx, my = cd['cell']
            ang = math.atan2(my - rcy, mx - rcx)
            b = i * 7
            v[b + 0] = min(cd['size'] / 40.0, 1.0)
            v[b + 1] = min(cd['d'] / max_d, 1.0)
            v[b + 2] = math.sin(ang)
            v[b + 3] = math.cos(ang)
            v[b + 4] = cd['unk']
            v[b + 5] = cd['free']
            v[b + 6] = min(cd['dcent'] / diag, 1.0)

        known = np.array(g.data, dtype=np.int16)
        v[-2] = float(np.count_nonzero(known != UNKNOWN)) / float(known.size)
        v[-1] = min(len(cands) / float(K), 1.0)
        return v

    # ======================================================
    # The two policies
    # ======================================================

    def choose_heuristic(self, cands):
        best_i, best_s = 0, -1e9
        for i, cd in enumerate(cands):
            dist_m = cd['d'] * self.grid.info.resolution
            s = self.w_gain * cd['size'] - self.w_dist * dist_m
            if s > best_s:
                best_s, best_i = s, i
        return best_i

    def choose_rl(self, cands, rx, ry):
        obs = self.build_obs(cands, rx, ry)
        if self.obs_mean is not None:
            obs = (obs - self.obs_mean) / np.sqrt(self.obs_var + 1e-8)
            obs = np.clip(obs, -10.0, 10.0).astype(np.float32)
        action, _ = self.model.predict(obs, deterministic=True)
        a = int(action)
        return a if a < len(cands) else 0       # invalid slot -> nearest

    # ======================================================
    # Nav2 goal dispatch
    # ======================================================

    def blacklisted(self, wx, wy):
        return any(math.hypot(wx - bx, wy - by) < self.bl_radius
                   for bx, by in self.blacklist)

    def send_goal(self, wx, wy, rx, ry):
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = wx
        p.pose.position.y = wy
        yaw = math.atan2(wy - ry, wx - rx)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = p

        self.navigating = True
        self.goals_sent += 1
        self.current_goal = (wx, wy)
        self.nav_client.send_goal_async(goal).add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.navigating = False
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        if status == 4:
            self.goals_done += 1
            self.get_logger().info(
                f'Reached frontier ({self.goals_done}/{self.goals_sent}).')
        else:
            self.get_logger().warn(
                f'Navigation failed (status {status}) - blacklisting.')
            self.blacklist.append(self.current_goal)
        self.navigating = False

    # ======================================================
    # Markers
    # ======================================================

    def publish_markers(self, cands, chosen):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, cd in enumerate(cands):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'frontiers', i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y = cd['world']
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.b, m.color.a = 1.0, 0.8
            arr.markers.append(m)

        if chosen is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'selected', 0, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y = cands[chosen]['world']
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.g, m.color.a = 1.0, 1.0
            arr.markers.append(m)

        self.marker_pub.publish(arr)

    # ======================================================
    # Main loop
    # ======================================================

    def plan_cycle(self):
        if self.grid is None or self.navigating:
            return

        pos = self.robot_xy()
        if pos is None:
            self.get_logger().info('Waiting for map->base_link transform...',
                                   throttle_duration_sec=5.0)
            return
        rx, ry = pos

        if self.start_time is None:
            self.start_time = time.time()
            self.last_pos = pos
        else:
            self.path_len += math.hypot(rx - self.last_pos[0],
                                        ry - self.last_pos[1])
            self.last_pos = pos

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available yet.')
            return

        cands = self.build_candidates(rx, ry)
        if not cands:
            elapsed = time.time() - self.start_time
            known = np.array(self.grid.data, dtype=np.int16)
            cov = 100.0 * np.count_nonzero(known != UNKNOWN) / known.size
            self.get_logger().info(
                f'=== EXPLORATION COMPLETE [{self.policy}] === '
                f'time={elapsed:.1f}s  goals={self.goals_done}/{self.goals_sent}  '
                f'path={self.path_len:.1f}m  known_cells={cov:.1f}%')
            return

        if self.policy == 'rl' and self.model is not None:
            idx = self.choose_rl(cands, rx, ry)
        else:
            idx = self.choose_heuristic(cands)

        self.publish_markers(cands, idx)
        cd = cands[idx]
        wx, wy = cd['world']
        self.get_logger().info(
            f'[{self.policy}] {len(cands)} cands | goal ({wx:.2f}, {wy:.2f}) '
            f'| size={cd["size"]} d={cd["d"]}cells unk={cd["unk"]:.2f}')
        self.send_goal(wx, wy, rx, ry)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

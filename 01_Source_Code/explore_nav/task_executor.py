#!/usr/bin/env python3
"""
Task executor: runs the ordered task list produced by llm_task_planner.py.

Subclasses FrontierExplorer, so frontier detection, clustering, scoring and
both exploration policies (heuristic / rl) are reused unchanged. This node
adds only the agentic layer:

  explore_region   restrict goal selection to one part of the map
  avoid_region     never send goals into that part
  explore_until    run until a coverage fraction is reached
  explore_all      run until no frontiers remain
  return_to_start  drive back to the pose recorded at startup
  stop             halt
"""

import json
import math
import time

import numpy as np

import rclpy
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from std_msgs.msg import String

from explore_nav.frontier_explorer import FrontierExplorer, UNKNOWN


class TaskExecutor(FrontierExplorer):

    def __init__(self):
        super().__init__()
        self.declare_parameter('instruction', '')

        self.tasks = []
        self.task_i = 0
        self.active = None
        self.region_filter = None     # explore only here
        self.avoid_regions = []       # never go here
        self.start_pose = None
        self.finished = False
        self.task_started = None

        qos = QoSProfile(depth=1,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/exploration_tasks',
                                 self.tasks_cb, qos)

        inline = self.get_parameter('instruction').value
        if inline:
            from explore_nav.llm_task_planner import make_plan
            tasks, backend = make_plan(inline)
            self.get_logger().info(f'Planned inline via "{backend}" backend.')
            self.load_tasks(tasks)
        else:
            self.get_logger().info(
                'Waiting for a plan on /exploration_tasks ...')

    # ==================================================
    # Plan handling
    # ==================================================

    def tasks_cb(self, msg):
        if self.tasks:
            return
        try:
            self.load_tasks(json.loads(msg.data))
        except Exception as e:
            self.get_logger().error(f'Bad task list: {e}')

    def load_tasks(self, tasks):
        self.tasks = tasks
        self.task_i = 0
        self.active = None
        self.get_logger().info(f'Received {len(tasks)} tasks:')
        for i, t in enumerate(tasks, 1):
            self.get_logger().info(f'  {i}. {json.dumps(t)}')

    def next_task(self):
        if self.task_i >= len(self.tasks):
            self.active = None
            return
        self.active = self.tasks[self.task_i]
        self.task_i += 1
        self.task_started = time.time()
        a = self.active['action']
        self.get_logger().info(
            f'--- TASK {self.task_i}/{len(self.tasks)}: {json.dumps(self.active)}')

        if a == 'explore_region':
            self.region_filter = self.active['region']
            if self.region_filter == 'all':
                self.region_filter = None
            self.active = None                 # constraint, not a step
            self.next_task()
        elif a == 'avoid_region':
            self.avoid_regions.append(self.active['region'])
            self.active = None
            self.next_task()

    # ==================================================
    # Region geometry
    # ==================================================

    def region_bounds(self):
        """Centre of the current map in world coordinates."""
        g = self.grid
        cx = g.info.origin.position.x + (g.info.width * g.info.resolution) / 2.0
        cy = g.info.origin.position.y + (g.info.height * g.info.resolution) / 2.0
        return cx, cy

    def in_region(self, wx, wy, region):
        cx, cy = self.region_bounds()
        left, right = wx < cx, wx >= cx
        bottom, top = wy < cy, wy >= cy
        return {
            'left': left, 'right': right, 'top': top, 'bottom': bottom,
            'top_left': top and left, 'top_right': top and right,
            'bottom_left': bottom and left, 'bottom_right': bottom and right,
            'all': True,
        }.get(region, True)

    def coverage(self):
        known = np.array(self.grid.data, dtype=np.int16)
        return float(np.count_nonzero(known != UNKNOWN)) / float(known.size)

    # ==================================================
    # Candidate filtering - the agentic constraint layer
    # ==================================================

    def build_candidates(self, rx, ry):
        cands = super().build_candidates(rx, ry)
        kept = []
        for cd in cands:
            wx, wy = cd['world']
            if self.region_filter and not self.in_region(wx, wy, self.region_filter):
                continue
            if any(self.in_region(wx, wy, r) for r in self.avoid_regions):
                continue
            kept.append(cd)
        return kept

    # ==================================================
    # Return to start
    # ==================================================

    def go_home(self, rx, ry):
        hx, hy = self.start_pose
        if math.hypot(rx - hx, ry - hy) < 0.35:
            self.get_logger().info('Back at the starting pose.')
            return True
        if not self.navigating:
            self.get_logger().info(f'Returning to start ({hx:.2f}, {hy:.2f})')
            self.send_goal(hx, hy, rx, ry)
        return False

    # ==================================================
    # Main loop
    # ==================================================

    def plan_cycle(self):
        if self.grid is None or self.finished:
            return

        pos = self.robot_xy()
        if pos is None:
            self.get_logger().info('Waiting for map->base_link transform...',
                                   throttle_duration_sec=5.0)
            return
        rx, ry = pos

        if self.start_pose is None:
            self.start_pose = (rx, ry)
            self.start_time = time.time()
            self.last_pos = pos
            self.get_logger().info(
                f'Start pose recorded: ({rx:.2f}, {ry:.2f})')
        else:
            self.path_len += math.hypot(rx - self.last_pos[0],
                                        ry - self.last_pos[1])
            self.last_pos = pos

        if not self.tasks:
            return
        if self.active is None:
            self.next_task()
            if self.active is None:
                self.report('all tasks complete')
                return

        action = self.active['action']

        if action == 'stop':
            self.report('stop requested by plan')
            return

        if action == 'return_to_start':
            if self.go_home(rx, ry):
                self.active = None
            return

        if self.navigating:
            return
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available yet.')
            return

        # explore_until / explore_all
        if action == 'explore_until':
            target = self.active['coverage']
            cov = self.coverage()
            if cov >= target:
                self.get_logger().info(
                    f'Coverage {cov*100:.1f}% reached target {target*100:.0f}% '
                    f'in {time.time()-self.task_started:.1f}s')
                self.active = None
                return

        cands = self.build_candidates(rx, ry)
        if not cands:
            where = self.region_filter or 'the whole map'
            self.get_logger().info(f'No frontiers left in {where}.')
            if self.region_filter:
                # the preferred region is exhausted (or was never reachable) -
                # widen to the whole map and KEEP the current task running
                self.get_logger().info(
                    'Releasing region restriction; continuing on the full map.')
                self.region_filter = None
                return
            self.active = None
            return

        idx = (self.choose_rl(cands, rx, ry)
               if self.policy == 'rl' and self.model is not None
               else self.choose_heuristic(cands))
        self.publish_markers(cands, idx)
        cd = cands[idx]
        wx, wy = cd['world']
        self.get_logger().info(
            f'[{self.policy}|{action}] {len(cands)} cands | goal '
            f'({wx:.2f}, {wy:.2f}) | size={cd["size"]} d={cd["d"]}cells')
        self.send_goal(wx, wy, rx, ry)

    def report(self, why):
        self.finished = True
        cov = self.coverage() * 100.0
        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f'=== PLAN COMPLETE ({why}) === policy={self.policy} '
            f'time={elapsed:.1f}s goals={self.goals_done}/{self.goals_sent} '
            f'path={self.path_len:.1f}m known_cells={cov:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutor()
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

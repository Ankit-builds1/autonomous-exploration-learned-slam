#!/usr/bin/env python3
"""
LLM task planner: one compound natural-language instruction -> ordered JSON task list.

    "explore the left side first, avoid the top-right corner,
     stop once you have mapped 90 percent, then return to start"

becomes

    [{"action": "explore_region",  "region": "left"},
     {"action": "avoid_region",    "region": "top_right"},
     {"action": "explore_until",   "coverage": 0.90},
     {"action": "return_to_start"}]

Two backends:
  llm    - Anthropic API (set ANTHROPIC_API_KEY); handles free-form phrasing
  rules  - deterministic keyword parser; no network, always available

The plan is published on /exploration_tasks (latched) for task_executor.py.
"""

import json
import os
import re
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                       QoSReliabilityPolicy, QoSHistoryPolicy)
from std_msgs.msg import String

REGIONS = ['left', 'right', 'top', 'bottom',
           'top_left', 'top_right', 'bottom_left', 'bottom_right', 'all']

SCHEMA = """Return ONLY a JSON array. Each element is one of:
  {"action": "explore_region", "region": R}   explore only that part of the map
  {"action": "avoid_region",   "region": R}   never send goals into that part
  {"action": "explore_until",  "coverage": F} explore until coverage fraction F (0-1)
  {"action": "explore_all"}                   explore whatever remains
  {"action": "return_to_start"}               drive back to the starting pose
  {"action": "stop"}                          halt
R must be one of: left, right, top, bottom, top_left, top_right,
bottom_left, bottom_right, all.
Order the array so it matches the order the user asked for.
No prose, no markdown fences - JSON only."""


# ======================================================
# Backend 1: rule-based parser (no network required)
# ======================================================

def plan_with_rules(text):
    t = text.lower()
    tasks = []

    def region_after(keyword):
        """Find the region word that follows a keyword."""
        m = re.search(keyword + r'[^.,;]{0,40}', t)
        if not m:
            return None
        chunk = m.group(0)
        for r in ('top left', 'top-left', 'upper left'):
            if r in chunk:
                return 'top_left'
        for r in ('top right', 'top-right', 'upper right'):
            if r in chunk:
                return 'top_right'
        for r in ('bottom left', 'bottom-left', 'lower left'):
            if r in chunk:
                return 'bottom_left'
        for r in ('bottom right', 'bottom-right', 'lower right'):
            if r in chunk:
                return 'bottom_right'
        if 'left' in chunk:
            return 'left'
        if 'right' in chunk:
            return 'right'
        if 'top' in chunk or 'north' in chunk or 'upper' in chunk:
            return 'top'
        if 'bottom' in chunk or 'south' in chunk or 'lower' in chunk:
            return 'bottom'
        return None

    first = region_after(r'(explore|start with|begin with|go to|map)')
    if first:
        tasks.append({'action': 'explore_region', 'region': first})

    avoid = region_after(r'(avoid|skip|stay out of|do not enter|don\'t enter)')
    if avoid:
        tasks.append({'action': 'avoid_region', 'region': avoid})

    m = re.search(r'(\d{1,3})\s*(?:%|percent)', t)
    if m:
        tasks.append({'action': 'explore_until',
                      'coverage': min(int(m.group(1)), 100) / 100.0})
    elif 'until' in t and ('done' in t or 'complete' in t or 'finish' in t):
        tasks.append({'action': 'explore_all'})

    if not any(x['action'] in ('explore_until', 'explore_all') for x in tasks):
        tasks.append({'action': 'explore_all'})

    if re.search(r'(return|come back|go back|back to start|home)', t):
        tasks.append({'action': 'return_to_start'})

    tasks.append({'action': 'stop'})
    return tasks


# ======================================================
# Backend 2: Anthropic API
# ======================================================

def plan_with_llm(text):
    """Ask Claude to decompose the instruction. Returns None if unavailable."""
    key = os.environ.get('ANTHROPIC_API_KEY')
    if not key:
        return None
    try:
        import anthropic
    except ImportError:
        print('[planner] anthropic SDK not installed; using rule-based backend')
        return None

    try:
        client = anthropic.Anthropic(api_key=key)
        msg = client.messages.create(
            model='claude-sonnet-4-5',
            max_tokens=512,
            system=('You decompose a robot exploration instruction into an '
                    'ordered task list.\n' + SCHEMA),
            messages=[{'role': 'user', 'content': text}],
        )
        raw = msg.content[0].text.strip()
        raw = re.sub(r'^```(?:json)?|```$', '', raw, flags=re.MULTILINE).strip()
        tasks = json.loads(raw)
        if isinstance(tasks, dict) and 'tasks' in tasks:
            tasks = tasks['tasks']
        return validate(tasks)
    except Exception as e:
        print(f'[planner] LLM backend failed ({e}); using rule-based backend')
        return None


def validate(tasks):
    """Drop anything the executor cannot run."""
    ok = []
    for t in tasks:
        a = t.get('action')
        if a in ('explore_region', 'avoid_region'):
            if t.get('region') in REGIONS:
                ok.append({'action': a, 'region': t['region']})
        elif a == 'explore_until':
            c = float(t.get('coverage', 0.9))
            ok.append({'action': a, 'coverage': max(0.05, min(c, 1.0))})
        elif a in ('explore_all', 'return_to_start', 'stop'):
            ok.append({'action': a})
    if not ok:
        raise ValueError('no valid tasks produced')
    if ok[-1]['action'] != 'stop':
        ok.append({'action': 'stop'})
    return ok


def make_plan(text):
    tasks = plan_with_llm(text)
    backend = 'llm'
    if tasks is None:
        tasks = validate(plan_with_rules(text))
        backend = 'rules'
    return tasks, backend


# ======================================================
# ROS2 node
# ======================================================

class TaskPlanner(Node):

    def __init__(self, instruction):
        super().__init__('llm_task_planner')
        qos = QoSProfile(depth=1,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, '/exploration_tasks', qos)

        tasks, backend = make_plan(instruction)
        self.get_logger().info(f'Instruction: "{instruction}"')
        self.get_logger().info(f'Backend: {backend}')
        self.get_logger().info('Plan:')
        for i, t in enumerate(tasks, 1):
            self.get_logger().info(f'  {i}. {json.dumps(t)}')

        msg = String()
        msg.data = json.dumps(tasks)
        self.pub.publish(msg)
        self.get_logger().info('Published on /exploration_tasks (latched).')


def main(args=None):
    argv = [a for a in (args or sys.argv[1:]) if not a.startswith('--ros-args')]
    instruction = ' '.join(argv).strip()
    if not instruction:
        instruction = ('explore the left side first, avoid the top right corner, '
                       'stop once you have mapped 90 percent, then return to start')
        print(f'[planner] no instruction given, using demo:\n  "{instruction}"')

    rclpy.init()
    node = TaskPlanner(instruction)
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
        import time
        time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

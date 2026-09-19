#!/usr/bin/env python3
"""
Fast 2D grid environment for learning an exploration policy.

Abstracts the Gazebo/Nav2 loop: the agent sees a set of frontier
candidates and chooses which one to visit. Reward trades information
gained against distance travelled - the same trade-off the hand-coded
heuristic in frontier_explorer.py makes with fixed weights.
"""

import math
from collections import deque

import numpy as np
import gymnasium as gym
from gymnasium import spaces

UNKNOWN, FREE, OCC = -1, 0, 1


class ExplorationEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, size=40, max_candidates=8, sensor_range=8.0,
                 n_rays=120, max_steps=60, min_cluster=1, seed=None):
        super().__init__()
        self.size = size
        self.K = max_candidates
        self.sensor_range = sensor_range
        self.n_rays = n_rays
        self.max_steps = max_steps
        self.min_cluster = min_cluster

        # action: which of the K frontier candidates to drive to
        self.action_space = spaces.Discrete(self.K)

        # observation: K candidates x 4 features, plus 2 global features
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.K * 7 + 2,), dtype=np.float32)

        self.rng = np.random.default_rng(seed)
        self.candidates = []

    # ==================================================
    # Map generation
    # ==================================================

    def _random_map(self):
        """Ground truth: 0 = free, 1 = occupied.

        Recursive division: repeatedly split the space with a wall that
        has a single doorway. Produces rooms, corridors and dead-ends -
        topology where greedy nearest-frontier selection is myopic.
        """
        g = np.zeros((self.size, self.size), dtype=np.int8)
        g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = OCC
        self._divide(g, 1, 1, self.size - 2, self.size - 2, 0)
        return g

    def _divide(self, g, r0, c0, r1, c1, depth):
        h, w = r1 - r0 + 1, c1 - c0 + 1
        if h < 7 or w < 7 or depth > 4:
            return
        horizontal = h > w if h != w else bool(self.rng.random() < 0.5)
        if horizontal:
            wr = int(self.rng.integers(r0 + 3, r1 - 2))
            g[wr, c0:c1 + 1] = OCC
            door = int(self.rng.integers(c0, c1 + 1))
            g[wr, max(c0, door - 1):min(c1 + 1, door + 2)] = 0
            self._divide(g, r0, c0, wr - 1, c1, depth + 1)
            self._divide(g, wr + 1, c0, r1, c1, depth + 1)
        else:
            wc = int(self.rng.integers(c0 + 3, c1 - 2))
            g[r0:r1 + 1, wc] = OCC
            door = int(self.rng.integers(r0, r1 + 1))
            g[max(r0, door - 1):min(r1 + 1, door + 2), wc] = 0
            self._divide(g, r0, c0, r1, wc - 1, depth + 1)
            self._divide(g, r0, wc + 1, r1, c1, depth + 1)

    def _free_start(self):
        while True:
            r = int(self.rng.integers(1, self.size - 1))
            c = int(self.rng.integers(1, self.size - 1))
            if self.gt[r, c] == 0:
                return (r, c)

    # ==================================================
    # Simulated lidar
    # ==================================================

    def _reveal(self):
        """Ray-cast 360 degrees from the robot, marking cells known."""
        new = 0
        r0, c0 = self.pos[0] + 0.5, self.pos[1] + 0.5
        for i in range(self.n_rays):
            a = 2.0 * math.pi * i / self.n_rays
            dr, dc = math.sin(a), math.cos(a)
            t = 0.0
            while t < self.sensor_range:
                t += 0.4
                r, c = int(r0 + t * dr), int(c0 + t * dc)
                if r < 0 or c < 0 or r >= self.size or c >= self.size:
                    break
                if self.gt[r, c] == OCC:
                    if self.known[r, c] == UNKNOWN:
                        new += 1
                    self.known[r, c] = OCC
                    break
                if self.known[r, c] == UNKNOWN:
                    self.known[r, c] = FREE
                    new += 1
        return new

    # ==================================================
    # Frontier detection + clustering (mirrors the ROS node)
    # ==================================================

    def _frontier_cells(self):
        cells = set()
        k = self.known
        for r in range(1, self.size - 1):
            for c in range(1, self.size - 1):
                if k[r, c] != FREE:
                    continue
                if (k[r-1, c] == UNKNOWN or k[r+1, c] == UNKNOWN or
                        k[r, c-1] == UNKNOWN or k[r, c+1] == UNKNOWN):
                    cells.add((r, c))
        return cells

    def _cluster(self, cells):
        groups, unvisited = [], set(cells)
        while unvisited:
            seed = unvisited.pop()
            g, q = [seed], deque([seed])
            while q:
                r, c = q.popleft()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        n = (r + dr, c + dc)
                        if n in unvisited:
                            unvisited.remove(n)
                            g.append(n)
                            q.append(n)
            if len(g) >= self.min_cluster:
                groups.append(g)
        return groups

    def _bfs_distances(self):
        """Path length from the robot to every reachable known-free cell."""
        dist = np.full((self.size, self.size), -1, dtype=np.int32)
        dist[self.pos] = 0
        q = deque([self.pos])
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.size and 0 <= nc < self.size:
                    if dist[nr, nc] == -1 and self.known[nr, nc] == FREE:
                        dist[nr, nc] = dist[r, c] + 1
                        q.append((nr, nc))
        return dist

    def _build_candidates(self):
        """Reachable frontier clusters, with spatial-context features."""
        groups = self._cluster(self._frontier_cells())
        dist = self._bfs_distances()

        unk_idx = np.argwhere(self.known == UNKNOWN)
        if len(unk_idx):
            centroid = unk_idx.mean(axis=0)
        else:
            centroid = np.array(self.pos, dtype=float)

        R = 6
        cands = []
        for g in groups:
            best = None
            for (r, c) in g:
                d = dist[r, c]
                if d >= 0 and (best is None or d < best[0]):
                    best = (d, r, c)
            if best is None:
                continue
            d, r, c = best

            r0, r1 = max(0, r - R), min(self.size, r + R + 1)
            c0, c1 = max(0, c - R), min(self.size, c + R + 1)
            win = self.known[r0:r1, c0:c1]
            n = max(win.size, 1)

            cands.append({
                'd': int(d),
                'rc': (r, c),
                'size': len(g),
                'unk': float(np.count_nonzero(win == UNKNOWN)) / n,
                'free': float(np.count_nonzero(win == FREE)) / n,
                'dcent': math.hypot(r - centroid[0], c - centroid[1]),
            })
        cands.sort(key=lambda x: x['d'])
        return cands[:self.K]

    def _obs(self):
        v = np.zeros(self.K * 7 + 2, dtype=np.float32)
        max_d = float(self.size * 2)
        diag = float(self.size * 1.4142)
        for i, cd in enumerate(self.candidates):
            dr = cd['rc'][0] - self.pos[0]
            dc = cd['rc'][1] - self.pos[1]
            ang = math.atan2(dr, dc)
            b = i * 7
            v[b + 0] = min(cd['size'] / 40.0, 1.0)
            v[b + 1] = min(cd['d'] / max_d, 1.0)
            v[b + 2] = math.sin(ang)
            v[b + 3] = math.cos(ang)
            v[b + 4] = cd['unk']
            v[b + 5] = cd['free']
            v[b + 6] = min(cd['dcent'] / diag, 1.0)
        v[-2] = self.coverage()
        v[-1] = min(len(self.candidates) / float(self.K), 1.0)
        return v

    def coverage(self):
        known = np.count_nonzero(self.known != UNKNOWN)
        return float(known) / float(self.reachable_total)

    # ==================================================
    # Gym API
    # ==================================================

    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.gt = self._random_map()
        self.known = np.full((self.size, self.size), UNKNOWN, dtype=np.int8)
        self.pos = self._free_start()
        self.steps = 0
        self.path_len = 0

        # how much of the map is reachable at all (denominator for coverage)
        self.known[self.pos] = FREE
        seen = np.zeros_like(self.gt, dtype=bool)
        q = deque([self.pos])
        seen[self.pos] = True
        n = 1
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.size and 0 <= nc < self.size and not seen[nr, nc]:
                    seen[nr, nc] = True
                    if self.gt[nr, nc] == 0:
                        n += 1
                        q.append((nr, nc))
                    else:
                        n += 1          # walls we can see also count as known
        self.reachable_total = max(n, 1)

        self._reveal()
        self.candidates = self._build_candidates()
        return self._obs(), {}

    def step(self, action):
        self.steps += 1
        terminated = truncated = False

        # invalid action: chose a slot with no candidate in it
        if action >= len(self.candidates):
            reward = -1.0
            if self.steps >= self.max_steps:
                truncated = True
            return self._obs(), reward, terminated, truncated, {"coverage": self.coverage(), "path_len": self.path_len}

        cd = self.candidates[action]
        self.pos = cd['rc']
        self.path_len += cd['d']

        new_cells = self._reveal()
        self.candidates = self._build_candidates()

        # information gained, minus distance travelled, minus time
        reward = 0.05 * new_cells - 0.08 * cd['d'] - 0.1

        if not self.candidates or self.coverage() >= 0.97:
            terminated = True
            reward += 20.0 * self.coverage()      # finish bonus, scaled by how much was mapped
        elif self.steps >= self.max_steps:
            truncated = True

        info = {'coverage': self.coverage(), 'path_len': self.path_len}
        return self._obs(), float(reward), terminated, truncated, info


# ------------------------------------------------------
# Smoke test: random agent, 3 episodes
# ------------------------------------------------------
if __name__ == '__main__':
    env = ExplorationEnv(seed=0)
    for ep in range(3):
        obs, _ = env.reset()
        total, done = 0.0, False
        while not done:
            a = env.action_space.sample()
            obs, r, term, trunc, info = env.step(a)
            total += r
            done = term or trunc
        print(f'episode {ep}: reward={total:7.2f} '
              f'coverage={info["coverage"]*100:5.1f}% '
              f'path={info["path_len"]} steps={env.steps}')

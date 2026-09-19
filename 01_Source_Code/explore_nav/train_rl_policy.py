#!/usr/bin/env python3
"""
Train a PPO exploration policy on randomised maps.

The policy replaces the hand-coded scoring line in frontier_explorer.py:
    score = w_gain * size - w_dist * distance
with a learned function of the same inputs.
"""

import os
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy

from exploration_env import ExplorationEnv

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(os.path.dirname(HERE), 'models')
MODEL_PATH = os.path.join(MODEL_DIR, 'ppo_explorer')
LOG_DIR = os.path.join(MODEL_DIR, 'tb_logs')

TOTAL_TIMESTEPS = 250_000
N_ENVS = 4


def make_env(rank):
    def _init():
        return Monitor(ExplorationEnv(seed=1000 + rank))
    return _init


def benchmark(env, policy, n=50, label=''):
    """Run n episodes and report the metrics used in the report."""
    rewards, covs, paths, steps = [], [], [], []
    for i in range(n):
        obs, _ = env.reset(seed=5000 + i)
        total, done, info = 0.0, False, {}
        while not done:
            a = policy(obs, env)
            obs, r, term, trunc, info = env.step(a)
            total += r
            done = term or trunc
        rewards.append(total)
        covs.append(info.get('coverage', 0.0))
        paths.append(info.get('path_len', 0))
        steps.append(env.steps)
    print(f'{label:12s} reward={np.mean(rewards):7.2f} '
          f'coverage={np.mean(covs)*100:5.1f}% '
          f'path={np.mean(paths):6.1f} steps={np.mean(steps):5.1f}')
    return dict(reward=np.mean(rewards), coverage=np.mean(covs),
                path=np.mean(paths), steps=np.mean(steps))


def random_policy(obs, env):
    return env.action_space.sample()


def greedy_policy(obs, env):
    """The Milestone 4 heuristic, reimplemented here for comparison."""
    best, best_i = -1e9, 0
    for i, cd in enumerate(env.candidates):
        s = 1.0 * cd['size'] - 3.0 * cd['d']
        if s > best:
            best, best_i = s, i
    return best_i


def nearest_policy(obs, env):
    return 0          # candidates are sorted nearest-first


if __name__ == '__main__':
    os.makedirs(MODEL_DIR, exist_ok=True)

    print(f'Training PPO for {TOTAL_TIMESTEPS} timesteps on {N_ENVS} envs...')
    vec = SubprocVecEnv([make_env(i) for i in range(N_ENVS)])
    vec = VecNormalize(vec, norm_obs=True, norm_reward=True, clip_obs=10.0)

    model = PPO('MlpPolicy', vec,
                learning_rate=3e-4,
                n_steps=512,
                batch_size=256,
                gamma=0.995,
                gae_lambda=0.95,
                ent_coef=0.005,
                verbose=1,
                tensorboard_log=LOG_DIR)

    model.learn(total_timesteps=TOTAL_TIMESTEPS, progress_bar=False)
    model.save(MODEL_PATH)
    vec.save(os.path.join(MODEL_DIR, 'vecnormalize.pkl'))
    obs_rms = vec.obs_rms
    vec.close()
    print(f'\nModel saved to {MODEL_PATH}.zip\n')

    # ---- head-to-head comparison on identical maps ----
    print('=' * 62)
    print('BENCHMARK - 50 identical randomised maps per policy')
    print('=' * 62)

    env = ExplorationEnv()
    benchmark(env, random_policy, label='random')
    benchmark(env, nearest_policy, label='nearest')
    benchmark(env, greedy_policy, label='heuristic')

    def ppo_policy(obs, env):
        # apply the same observation normalisation used during training
        o = (obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8)
        o = np.clip(o, -10.0, 10.0).astype(np.float32)
        a, _ = model.predict(o, deterministic=True)
        return int(a)

    benchmark(env, ppo_policy, label='PPO (ours)')

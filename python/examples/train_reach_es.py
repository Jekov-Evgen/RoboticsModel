"""Train a small neural policy on RoboticsModel/ArmReach-v0 with evolution strategies (numpy only, no RL library).

    pip install gymnasium
    python python/examples/train_reach_es.py [--generations 60]

Prints the mean final distance to the goal of the current policy on a fixed validation set, next to two
reference lines: a policy that stands still and the closed-form IK "oracle" (the best possible).
"""

import argparse
import time

import gymnasium as gym
import numpy as np

import robotics_model.envs  # noqa: F401  (registers the environment ids)
from robotics_model.envs import arm

HIDDEN = 32
STEPS = 60  # control steps per training episode (2 s at 30 Hz)
N_PARAMS = 15 * HIDDEN + HIDDEN + HIDDEN * 3 + 3


def unpack(theta):
    i = 0
    w1 = theta[i:i + 15 * HIDDEN].reshape(15, HIDDEN)
    i += 15 * HIDDEN
    b1 = theta[i:i + HIDDEN]
    i += HIDDEN
    w2 = theta[i:i + HIDDEN * 3].reshape(HIDDEN, 3)
    i += HIDDEN * 3
    return w1, b1, w2, theta[i:i + 3]


def policy(theta, obs):
    w1, b1, w2, b2 = unpack(theta)
    return np.tanh(np.tanh(obs @ w1 + b1) @ w2 + b2)


def rollout(env, theta, seed, steps=STEPS):
    obs, info = env.reset(seed=seed)
    total = 0.0
    for _ in range(steps):
        obs, reward, _, _, info = env.step(policy(theta, obs))
        total += reward
    return total, info["distance"]


def evaluate(env, theta, seeds):
    return float(np.mean([rollout(env, theta, s)[1] for s in seeds]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--generations", type=int, default=60)
    ap.add_argument("--pairs", type=int, default=16, help="antithetic pairs per generation")
    ap.add_argument("--sigma", type=float, default=0.1)
    ap.add_argument("--lr", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    env = gym.make("RoboticsModel/ArmReach-v0")
    rng = np.random.default_rng(args.seed)
    theta = rng.normal(0, 0.1, N_PARAMS)
    validation = list(range(1000, 1010))

    still = np.mean([rollout(env, np.zeros(N_PARAMS), s)[1] for s in validation])  # all-zero policy: no motion
    oracle = []
    for s in validation:
        obs, info = env.reset(seed=s)
        for _ in range(STEPS):
            q = arm.inverse_kinematics(obs[9:12].astype(float))
            obs, _, _, _, info = env.step(np.clip((q - env.unwrapped.targets) / env.unwrapped.action_scale, -1, 1))
        oracle.append(info["distance"])
    print(f"reference: standing still {still:.3f} m, IK oracle {np.mean(oracle):.3f} m (mean final distance)")

    start = time.time()
    for gen in range(args.generations + 1):
        if gen % 5 == 0:
            print(f"gen {gen:3d}  validation distance {evaluate(env, theta, validation):.3f} m   ({time.time() - start:5.1f} s)")
        if gen == args.generations:
            break
        eps = rng.normal(0, 1, (args.pairs, N_PARAMS))
        seeds = rng.integers(0, 10**6, 3)  # common random numbers: every candidate sees the same goals
        scores = np.zeros((args.pairs, 2))
        for k in range(args.pairs):
            for j, sign in enumerate((1, -1)):
                scores[k, j] = np.mean([rollout(env, theta + sign * args.sigma * eps[k], int(s))[0] for s in seeds])
        # rank-based fitness shaping, then the antithetic ES gradient
        flat = scores.ravel()
        ranks = np.empty_like(flat)
        ranks[flat.argsort()] = np.linspace(-0.5, 0.5, flat.size)
        ranks = ranks.reshape(scores.shape)
        grad = ((ranks[:, 0] - ranks[:, 1])[:, None] * eps).sum(0) / (2 * args.pairs * args.sigma)
        theta = theta + args.lr * grad


if __name__ == "__main__":
    main()

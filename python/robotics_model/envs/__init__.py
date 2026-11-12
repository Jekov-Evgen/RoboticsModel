"""Gymnasium environments built on the Robotics Model engine.

    import gymnasium as gym
    import robotics_model.envs  # registers the ids below

    env = gym.make("RoboticsModel/ArmReach-v0")
    obs, info = env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())

Requires ``pip install gymnasium`` (extra: ``pip install "robotics-model[gym]"``).
"""

from .arm import ArmReachEnv, make_arm_world, tool_position
from .base import RoboticsEnv

__all__ = ["ArmReachEnv", "RoboticsEnv", "make_arm_world", "tool_position", "register_envs"]


def register_envs():
    """Registers the environment ids with gymnasium (idempotent)."""
    import gymnasium as gym

    if "RoboticsModel/ArmReach-v0" not in gym.registry:
        gym.register(
            id="RoboticsModel/ArmReach-v0",
            entry_point="robotics_model.envs:ArmReachEnv",
            max_episode_steps=200,
        )


register_envs()

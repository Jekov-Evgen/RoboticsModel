"""Reach task for the 3-DOF manipulator of ``examples/manipulator_3dof.robot``."""

from __future__ import annotations

import math
from typing import Any, Optional

import gymnasium as gym
import numpy as np

import robotics_model as rm

from .base import RoboticsEnv

# Geometry of the arm (metres, kilograms).
PIVOT_HEIGHT = 0.6  # shoulder height above the floor
UPPER = 0.8  # upper-arm length
FORE = 0.7  # forearm length (the tool sphere sits at its end)

JOINT_NAMES = ("yaw", "shoulder", "elbow")
# Joint ranges in radians (hard limits in the World).
LIMITS = np.radians([[-170.0, 170.0], [-45.0, 90.0], [-90.0, 120.0]])
MAX_EFFORT = (100.0, 150.0, 60.0)
INERTIA = (4.5308, 4.49854, 0.574313)  # kg m^2 about each axis in the home pose
BANDWIDTH = (8.0, 8.0, 16.0)  # rad/s of the PID design (damping 1, Ti = 8 / wn)


def make_arm_world() -> rm.World:
    """The course-project manipulator with a PID controller on every joint (targets start at 0)."""
    w = rm.World()
    w.iterations = 32
    base = w.add_body("box", size=(0.7, 0.2, 0.7), mass=0, position=(0, 0.1, 0), name="base")
    turret = w.add_body("box", size=(0.22, 0.4, 0.22), mass=4, position=(0, 0.4, 0), name="turret")
    yaw = w.connect(base, turret, "hinge", anchor=(0, 0.2, 0), axis=(0, 1, 0))
    upper = w.add_body("box", size=(UPPER, 0.13, 0.13), mass=3, position=(UPPER / 2, PIVOT_HEIGHT, 0), name="upper_arm")
    shoulder = w.connect(turret, upper, "hinge", anchor=(0, PIVOT_HEIGHT, 0), axis=(0, 0, 1))
    fore = w.add_body("box", size=(FORE, 0.1, 0.1), mass=2, position=(UPPER + FORE / 2, PIVOT_HEIGHT, 0), name="forearm")
    elbow = w.connect(upper, fore, "hinge", anchor=(UPPER, PIVOT_HEIGHT, 0), axis=(0, 0, 1))
    tool = w.add_body("sphere", size=(0.14, 0.14, 0.14), mass=0.5, position=(UPPER + FORE, PIVOT_HEIGHT, 0), name="tool")
    w.connect(fore, tool, "fixed", anchor=(UPPER + FORE, PIVOT_HEIGHT, 0))
    for joint, effort, inertia, wn, (lo, hi) in zip((yaw, shoulder, elbow), MAX_EFFORT, INERTIA, BANDWIDTH, LIMITS):
        joint.max_effort = effort
        joint.set_limits(float(lo), float(hi))
        kp = inertia * wn * wn
        joint.set_controller("pid", kp=kp, ki=kp * wn / 8, kd=2 * wn * inertia, integral_limit=0.9 * effort)
        joint.target = 0.0
    return w


def tool_position(q) -> np.ndarray:
    """Forward kinematics: tool position for joint angles (yaw, shoulder, elbow) in radians."""
    yaw, shoulder, elbow = (float(v) for v in q)
    reach = UPPER * math.cos(shoulder) + FORE * math.cos(shoulder + elbow)
    height = PIVOT_HEIGHT + UPPER * math.sin(shoulder) + FORE * math.sin(shoulder + elbow)
    return np.array([reach * math.cos(yaw), height, -reach * math.sin(yaw)])


def inverse_kinematics(target) -> Optional[np.ndarray]:
    """Closed-form IK (elbow up) for a target tool position, or None when it is out of reach or the limits."""
    x, y, z = (float(v) for v in target)
    yaw = math.atan2(-z, x)
    r = math.hypot(x, z)
    h = y - PIVOT_HEIGHT
    d2 = r * r + h * h
    c = (d2 - UPPER**2 - FORE**2) / (2 * UPPER * FORE)
    if abs(c) > 1:
        return None
    best = None
    for sign in (1.0, -1.0):
        elbow = sign * math.acos(c)
        shoulder = math.atan2(h, r) - math.atan2(FORE * math.sin(elbow), UPPER + FORE * math.cos(elbow))
        q = np.array([yaw, shoulder, elbow])
        if np.all(q >= LIMITS[:, 0]) and np.all(q <= LIMITS[:, 1]):
            if best is None or q[2] > best[2]:  # prefer the elbow-up solution
                best = q
    return best


class ArmReachEnv(RoboticsEnv):
    """Move the tool to a random goal point.

    Action:      3 numbers in [-1, 1]: change of the joint-angle targets, scaled by ``action_scale`` rad per step.
                 The PID controllers of the joints track the targets (so gravity is already compensated).
    Observation: joint angles (3), joint speeds (3), tool position (3), goal (3), goal - tool (3).
    Reward:      minus the distance to the goal (m); ``info['is_success']`` when it is below ``success_radius``.
    Goals:       tool positions of random reachable joint configurations, so every goal is reachable.
    """

    def __init__(self, action_scale: float = 0.08, success_radius: float = 0.05, frame_skip: int = 8, render_mode: Optional[str] = None):
        self.action_scale = action_scale
        self.success_radius = success_radius
        super().__init__(frame_skip=frame_skip, render_mode=render_mode)
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-100.0, 100.0, shape=(15,), dtype=np.float32)
        self.goal = np.zeros(3)
        self._targets = np.zeros(3)

    def _make_world(self) -> rm.World:
        return make_arm_world()

    def _tool(self) -> np.ndarray:
        return np.array(self.world.body("tool").position)

    def _reset_episode(self, options: Optional[dict]) -> None:
        self._targets = np.zeros(3)
        if options and "goal" in options:
            self.goal = np.asarray(options["goal"], dtype=np.float64)
            return
        q = np.array([
            self.np_random.uniform(np.radians(-90), np.radians(90)),
            self.np_random.uniform(np.radians(0), np.radians(60)),
            self.np_random.uniform(np.radians(20), np.radians(100)),
        ])
        self.goal = tool_position(q)

    def _apply_action(self, action: np.ndarray) -> None:
        self._targets = np.clip(self._targets + self.action_scale * action, LIMITS[:, 0], LIMITS[:, 1])
        self.world.set_joint_targets(self._targets)

    def _observe(self) -> np.ndarray:
        tool = self._tool()
        return np.concatenate([self.world.joint_angles(), self.world.joint_speeds(), tool, self.goal, self.goal - tool]).astype(np.float32)

    def _reward(self, action: np.ndarray) -> tuple[float, bool, dict[str, Any]]:
        distance = float(np.linalg.norm(self.goal - self._tool()))
        return -distance, False, {"distance": distance, "is_success": distance < self.success_radius}

    def _info(self) -> dict[str, Any]:
        info = super()._info()
        info["distance"] = float(np.linalg.norm(self.goal - self._tool()))
        info["is_success"] = info["distance"] < self.success_radius
        return info

    @property
    def targets(self) -> np.ndarray:
        """Current joint-angle targets commanded to the controllers (rad)."""
        return self._targets.copy()

"""ROS-independent half of the ROS 2 bridge.

Everything here works on plain Python/numpy data, so it is unit-tested without ROS. ``node.py`` turns these
dictionaries into ROS messages.

Conventions
-----------
* Frames: the engine is Y-up, ROS is Z-up (REP 103). With ``ros_frame=True`` (default) positions are converted
  ``(x, y, z) -> (x, -z, y)`` and orientations accordingly. URDF robots imported with ``z_up_to_y_up`` therefore come
  out in their original URDF axes.
* A joint is named after its child body (the engine has no joint names), e.g. ``wheel_front_left``.
* Time is simulation time: ``World.time``.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence

import numpy as np

import robotics_model as rm

_QC = np.array([math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0])  # +90 deg about X: engine -> ROS axes


def to_ros_vector(v) -> np.ndarray:
    """Engine (Y-up) vector -> ROS (Z-up) vector."""
    x, y, z = (float(c) for c in v)
    return np.array([x, -z, y])


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def to_ros_quaternion(q) -> np.ndarray:
    """Engine orientation (w, x, y, z) -> ROS-axes orientation (w, x, y, z)."""
    qc_inv = _QC * np.array([1, -1, -1, -1])
    return _qmul(_qmul(_QC, np.asarray(q, dtype=float)), qc_inv)


def joint_name(joint: rm.Joint) -> str:
    return joint.child.name


class SimulationBridge:
    """Wraps a World and exposes it the way a ROS node needs it."""

    def __init__(self, world: rm.World, *, ros_frame: bool = True, real_time_factor: float = 1.0,
                 wheel_radius: Optional[float] = None, track_width: Optional[float] = None, turn_scale: float = 1.0):
        if real_time_factor <= 0:
            raise ValueError("real_time_factor must be positive")
        self.world = world
        self.ros_frame = ros_frame
        self.real_time_factor = real_time_factor
        self.turn_scale = turn_scale
        self._initial = world.clone()
        self._carry = 0.0
        self.paused = False
        self.wheel_radius, self.track_width = wheel_radius, track_width
        if wheel_radius is None or track_width is None:
            radius, track = self._estimate_wheels()
            self.wheel_radius = wheel_radius or radius
            self.track_width = track_width or track
        names = [joint_name(self.world.joint(i)) for i in self.world.actuated_joints]
        if len(set(names)) != len(names):
            raise ValueError(f"joint names must be unique, got {names}")

    # ---- time -------------------------------------------------------------------------------------------
    @property
    def time(self) -> float:
        return self.world.time

    def advance(self, wall_dt: float) -> int:
        """Advances the simulation by ``wall_dt * real_time_factor`` seconds (whole physics steps; the remainder is
        carried over). Returns the number of steps taken."""
        if self.paused:
            return 0
        self._carry += wall_dt * self.real_time_factor
        steps = int(self._carry // rm.DEFAULT_TIME_STEP)
        if steps > 0:
            self._carry -= steps * rm.DEFAULT_TIME_STEP
            self.world.step(steps)
        return steps

    def reset(self) -> None:
        """Restores the scene as it was when the bridge was created."""
        self.world.copy_state_from(self._initial)
        self._carry = 0.0

    # ---- joints -----------------------------------------------------------------------------------------
    def joint_state(self) -> dict:
        ids = self.world.actuated_joints
        return {
            "name": [joint_name(self.world.joint(i)) for i in ids],
            "position": self.world.joint_angles().tolist(),
            "velocity": self.world.joint_speeds().tolist(),
            "effort": self.world.joint_efforts().tolist(),
        }

    def command_joints(self, names: Sequence[str], positions: Sequence[float]) -> list[str]:
        """Sets the position setpoints of the named joints (their PID controllers must be active).
        Returns the names that were not found."""
        if len(names) != len(positions):
            raise ValueError("names and positions must have the same length")
        by_name = {joint_name(self.world.joint(i)): self.world.joint(i) for i in self.world.actuated_joints}
        unknown = []
        for name, value in zip(names, positions):
            joint = by_name.get(name)
            if joint is None or not math.isfinite(value):
                unknown.append(name)
                continue
            joint.target = float(value)
        return unknown

    # ---- driving ----------------------------------------------------------------------------------------
    def _estimate_wheels(self):
        radii, zs = [], []
        for i in range(self.world.num_joints):
            joint = self.world.joint(i)
            child = joint.child
            if joint.type.name == "HINGE" and child.shape.name == "CYLINDER":
                radii.append(child.size[0] / 2)
                zs.append(child.position[2])
        if not radii:
            return None, None
        return float(np.mean(radii)), float(max(zs) - min(zs))

    def command_velocity(self, linear_x: float, angular_z: float) -> int:
        """Twist command for a differential-drive robot (ROS convention: +x forward, +z counter-clockwise).
        Open loop: the wheel speeds follow the ideal kinematics, so on a skid-steer rover the real yaw rate is
        lower (tune ``turn_scale``). Returns the number of wheels commanded."""
        if self.wheel_radius is None:
            return 0
        forward = linear_x / self.wheel_radius
        turn = angular_z * (self.track_width / 2) / self.wheel_radius * self.turn_scale
        return rm.demos.drive_rover(self.world, forward, turn)

    # ---- poses and sensors --------------------------------------------------------------------------------
    def _vec(self, v):
        return to_ros_vector(v) if self.ros_frame else np.asarray(v, dtype=float)

    def _quat(self, q):
        return to_ros_quaternion(q) if self.ros_frame else np.asarray(q, dtype=float)

    def body_poses(self) -> list[dict]:
        """One entry per body: frame name, translation (3) and orientation (w, x, y, z) relative to 'world'."""
        return [{"frame": b.name, "parent": "world", "translation": self._vec(b.position).tolist(),
                 "rotation": self._quat(b.orientation).tolist()} for b in self.world.bodies]

    def imu(self, sensor: rm.Sensor) -> dict:
        v = sensor.value
        if len(v) != 10:
            raise ValueError("not an IMU sensor")
        return {"frame": sensor.body.name,
                "linear_acceleration": self._vec(v[0:3]).tolist(),
                "angular_velocity": self._vec(v[3:6]).tolist(),
                "orientation": self._quat(v[6:10]).tolist()}

    def laser_scan(self, sensor: rm.Sensor) -> dict:
        angles = sensor.lidar_angles
        if len(angles) == 0:
            raise ValueError("not a lidar sensor")
        step = float(angles[1] - angles[0]) if len(angles) > 1 else 0.0
        return {"frame": sensor.body.name, "angle_min": float(angles[0]), "angle_max": float(angles[-1]), "angle_increment": step,
                "range_min": sensor.min_range, "range_max": sensor.max_range, "ranges": sensor.value.tolist()}

    def sensors_of(self, kind: str) -> list[rm.Sensor]:
        wanted = {"imu": rm.SensorType.IMU, "lidar": rm.SensorType.LIDAR2D}[kind]
        return [s for s in self.world.sensors if s.type == wanted]

"""Robotics Model: a 3D rigid-body robot simulator with its own physics core.

    import robotics_model as rm

    world = rm.World()
    base = world.add_body(size=(0.3, 1.0, 0.3), mass=0, position=(0, 0.5, 0))
    arm = world.add_body(size=(1.2, 0.16, 0.16), position=(0.6, 1.0, 0))
    servo = world.connect(base, arm, "hinge", anchor=(0, 1, 0), axis=(0, 1, 0))
    servo.max_effort = 8
    servo.set_controller("pid", kp=14, ki=3, kd=4)
    servo.target = 1.0                 # rad
    world.run(5.0)                     # simulated seconds
    print(servo.angle)
"""

from ._core import (
    DEFAULT_TIME_STEP,
    Body,
    Chain,
    Contact,
    ControllerType,
    ControlVariable,
    IkResult,
    Joint,
    JointType,
    MeshCollision,
    ReferenceSignal,
    RoboticsError,
    Sensor,
    SensorType,
    Shape,
    Trajectory,
    UrdfRobot,
    World,
    __version__,
    demos,
    load_scene,
    load_urdf,
    min_jerk_duration,
    plan_point_to_point,
    plan_waypoints,
    save_scene,
)
from .control import follow_trajectory

__all__ = [
    "DEFAULT_TIME_STEP",
    "Body",
    "Chain",
    "Contact",
    "ControlVariable",
    "ControllerType",
    "IkResult",
    "Joint",
    "JointType",
    "MeshCollision",
    "ReferenceSignal",
    "RoboticsError",
    "Sensor",
    "SensorType",
    "Shape",
    "Trajectory",
    "UrdfRobot",
    "World",
    "demos",
    "follow_trajectory",
    "load_scene",
    "load_urdf",
    "min_jerk_duration",
    "plan_point_to_point",
    "plan_waypoints",
    "save_scene",
]

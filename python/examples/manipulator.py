"""Load the 3-DOF manipulator, command joint targets from Python and read the tool position.

    python python/examples/manipulator.py
"""

import math
import pathlib

import numpy as np

import robotics_model as rm

root = pathlib.Path(__file__).resolve().parents[2]
world = rm.load_scene(str(root / "examples" / "manipulator_3dof.robot"))
tool = world.body("tool")

# Replace the scene's built-in step references by targets we set ourselves.
route_deg = [(0, 0, 0), (-40, 30, 40), (45, 30, 40), (0, 0, 0)]
for yaw, shoulder, elbow in route_deg:
    world.set_joint_targets(np.radians([yaw, shoulder, elbow]))
    world.run(3.0)
    print(f"targets {yaw:4d} {shoulder:4d} {elbow:4d} deg -> angles {np.degrees(world.joint_angles()).round(1)}, tool at {tool.position.round(3)}")

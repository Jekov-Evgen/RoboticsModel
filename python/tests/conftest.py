import pathlib

import pytest

import robotics_model as rm

ROOT = pathlib.Path(__file__).resolve().parents[2]


@pytest.fixture
def examples():
    return ROOT / "examples"


@pytest.fixture
def pendulum_world():
    """A 2 kg, 1 m horizontal link on a hinge about Z: gravity torque 9.81 N·m at zero angle."""
    w = rm.World()
    base = w.add_body("box", size=(0.2, 0.2, 0.2), mass=0, position=(0, 1, 0))
    link = w.add_body("box", size=(1.0, 0.1, 0.1), mass=2, position=(0.5, 1, 0))
    joint = w.connect(base, link, "hinge", anchor=(0, 1, 0), axis=(0, 0, 1))
    joint.max_effort = 100
    return w, joint

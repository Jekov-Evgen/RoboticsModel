import math

import numpy as np
import pytest

import robotics_model as rm


def make_pile():
    w = rm.World()
    for i in range(5):
        w.add_body("box", size=(0.4, 0.4, 0.4), mass=1, position=(0.01 * i, 0.2 + 0.41 * i, 0))
    w.add_body("sphere", size=(0.3, 0.3, 0.3), mass=1, position=(0.3, 3, 0.1))
    return w


def test_free_fall_matches_kinematics():
    w = rm.World()
    ball = w.add_body("sphere", size=(0.2, 0.2, 0.2), position=(0, 100, 0))
    w.run(1.0)
    assert abs((100 - ball.position[1]) - 0.5 * 9.81) < 0.05
    assert abs(ball.velocity[1] + 9.81) < 0.05


def test_object_rests_on_the_ground_and_reports_contacts():
    w = rm.World()
    box = w.add_body("box", size=(1, 1, 1), position=(0, 2, 0))
    w.run(3.0)
    assert abs(box.position[1] - 0.5) < 0.01
    contacts = [c for c in w.contacts() if c.body_b == -1]
    assert contacts and all(c.normal_impulse >= 0 for c in contacts)
    assert abs(sum(c.normal_impulse for c in contacts) / rm.DEFAULT_TIME_STEP - 9.81) < 0.3  # supports its weight


def test_runs_are_bit_identical():
    def run():
        w = make_pile()
        w.run(2.0)
        return w.positions(), w.velocities()

    p1, v1 = run()
    p2, v2 = run()
    assert np.array_equal(p1, p2) and np.array_equal(v1, v2)


def test_clone_and_copy_state_from():
    w = make_pile()
    w.run(0.5)
    snapshot = w.clone()
    w.run(1.0)
    moved = w.positions().copy()
    w.copy_state_from(snapshot)
    assert abs(w.time - 0.5) < 1e-9
    w.run(1.0)
    assert np.array_equal(w.positions(), moved)  # replaying from the snapshot reproduces the run


def test_handles_are_live_views_and_survive_growth():
    w = rm.World()
    first = w.add_body("box", mass=0, position=(1, 2, 3), name="first")
    for _ in range(50):  # forces the underlying vector to reallocate
        w.add_body("sphere", size=(0.1, 0.1, 0.1))
    assert first.name == "first" and np.allclose(first.position, (1, 2, 3))
    first.position = (4, 5, 6)
    assert np.allclose(w.positions()[0], (4, 5, 6))
    assert w.body("first").index == 0 and len(w.bodies) == 51


def test_joint_speed_agrees_with_finite_differences(pendulum_world):
    w, joint = pendulum_world
    joint.set_controller("speed_motor")
    joint.motor_speed = 0.7
    joint.max_effort = 100
    w.gravity = (0, 0, 0)
    w.run(1.0)
    a0 = joint.angle
    w.step()
    assert abs(joint.speed - (joint.angle - a0) * 240) < 0.01
    assert abs(joint.speed - 0.7) < 0.01


def test_bulk_arrays(pendulum_world):
    w, joint = pendulum_world
    assert w.actuated_joints == [0]
    joint.set_controller("pid", kp=100, ki=100, kd=20, integral_limit=30)
    w.set_joint_targets([0.3])
    w.run(6.0)
    assert w.joint_angles().shape == (1,) and abs(w.joint_angles()[0] - 0.3) < 0.01
    assert w.joint_speeds().shape == (1,) and w.orientations().shape == (2, 4)
    with pytest.raises(ValueError):
        w.set_joint_targets([0.0, 1.0])


def test_dumps_loads_round_trip(pendulum_world):
    w, joint = pendulum_world
    joint.set_controller("pid", kp=11, ki=2, kd=3)
    text = w.dumps()
    again = rm.World.loads(text)
    assert again.dumps() == text
    assert again.joint(0).kp == 11


def test_raycast():
    w = rm.World()
    box = w.add_body("box", size=(1, 1, 1), mass=0, position=(5, 1, 0))
    hit = w.raycast((0, 1, 0), (1, 0, 0), 20)
    assert hit["body"] == box.index and abs(hit["distance"] - 4.5) < 1e-9
    assert w.raycast((0, 1, 0), (-1, 0, 0), 20) is None
    ground = w.raycast((0, 3, 0), (0, -1, 0), 20)
    assert ground["body"] == -1 and abs(ground["distance"] - 3) < 1e-9


def test_errors_are_python_exceptions():
    w = rm.World()
    a = w.add_body()
    with pytest.raises(ValueError, match="unknown Shape"):
        w.add_body("cube")
    with pytest.raises(ValueError, match="sequence of 3"):
        a.position = (1, 2)
    with pytest.raises(rm.RoboticsError):
        w.add_body(mass=-1)
    with pytest.raises(IndexError):
        w.body(7)
    with pytest.raises(KeyError):
        w.sensor("missing")
    other = rm.World()
    b = other.add_body()
    with pytest.raises(ValueError, match="different World"):
        w.connect(a, b)


def test_gil_is_released_while_stepping():
    import threading
    import time

    w = make_pile()
    ticks = []
    stop = threading.Event()

    def ticker():
        while not stop.is_set():
            ticks.append(time.perf_counter())
            time.sleep(0.001)

    t = threading.Thread(target=ticker)
    t.start()
    w.run(20.0)
    stop.set()
    t.join()
    assert len(ticks) > 10  # the ticker kept running during the simulation

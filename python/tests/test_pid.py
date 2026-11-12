import math

import robotics_model as rm


def test_readme_example_matches_the_cpp_core():
    """The same scene as the C++ example in the README: 1.0180 rad after 5 s."""
    w = rm.World()
    base = w.add_body(size=(0.3, 1.0, 0.3), mass=0, position=(0, 0.5, 0))
    arm = w.add_body(size=(1.2, 0.16, 0.16), position=(0.6, 1.0, 0))
    servo = w.connect(base, arm, "hinge", anchor=(0, 1, 0), axis=(0, 1, 0))
    servo.max_effort = 8
    servo.set_controller("pid", kp=14, ki=3, kd=4)
    servo.target = 1.0
    w.run(5.0)
    assert abs(servo.angle - 1.018) < 0.002


def test_pd_leaves_the_gravity_sag_and_pid_removes_it(pendulum_world):
    """e = g(q)/Kp for P/PD, zero for PID: the classic reason to use the integral term."""
    w, joint = pendulum_world
    kp = 100.0
    joint.set_controller("pd", kp=kp, kd=20)
    joint.target = 0.0
    w.run(8.0)
    expected_sag = 2 * 9.81 * 0.5 / kp  # m g (L/2) / Kp
    # Within 6 %: cos(angle) trims the gravity torque, and the speed sampled at the end of a step is not exactly zero.
    assert abs(joint.angle + expected_sag) < 0.06 * expected_sag



def test_pid_converges_under_gravity(pendulum_world):
    w, joint = pendulum_world
    joint.set_controller("pid", kp=100, ki=100, kd=20, integral_limit=30)
    joint.target = 0.0
    w.run(10.0)
    assert abs(joint.angle) < 1e-3
    # The integral term carries the whole gravity torque.
    assert abs(joint.effort - 9.81) < 0.05


def test_step_response_overshoot_matches_second_order_theory():
    """Double integrator J q'' = M with PD: overshoot exp(-pi z / sqrt(1 - z^2)). The engine must agree."""
    w = rm.World()
    base = w.add_body(size=(0.3, 1.0, 0.3), mass=0, position=(0, 0.5, 0))
    arm = w.add_body(size=(1.2, 0.16, 0.16), position=(0.6, 1.0, 0))
    joint = w.connect(base, arm, "hinge", anchor=(0, 1, 0), axis=(0, 1, 0))  # vertical axis: no gravity torque
    inertia = 1.0 * (1.2**2 + 0.16**2) / 12 + 1.0 * 0.6**2
    wn, zeta = 5.0, 0.3
    joint.max_effort = 1000
    joint.set_controller("pd", kp=inertia * wn**2, kd=2 * zeta * wn * inertia, derivative_filter=0.001)
    joint.set_step(0.0, math.radians(30), start_time=0.5)
    peak = 0.0
    for _ in range(int(5 * 240)):
        w.step()
        peak = max(peak, joint.angle)
    overshoot = (peak / math.radians(30) - 1) * 100
    predicted = math.exp(-math.pi * zeta / math.sqrt(1 - zeta**2)) * 100
    assert abs(overshoot - predicted) < 1.5


def test_speed_motor_and_limits(pendulum_world):
    w, joint = pendulum_world
    w.gravity = (0, 0, 0)
    joint.set_controller("speed_motor")
    joint.motor_speed = 1.0
    joint.max_effort = 50
    joint.set_limits(-0.5, 0.8)
    w.run(3.0)
    assert abs(joint.angle - 0.8) < 0.01  # the hard limit holds against the motor
    assert joint.limits == (-0.5, 0.8)


def test_controller_validation(pendulum_world):
    _, joint = pendulum_world
    import pytest

    with pytest.raises(ValueError):
        joint.set_controller("pid", kp=-1)
    with pytest.raises(ValueError):
        joint.set_controller("bang-bang")

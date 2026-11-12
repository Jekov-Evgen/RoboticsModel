import math

import numpy as np
import pytest

import robotics_model as rm


def test_manipulator_example_reaches_its_step_targets(examples):
    """examples/manipulator_3dof.robot: yaw +50°, shoulder +35°, elbow +55° with PID joints."""
    w = rm.load_scene(str(examples / "manipulator_3dof.robot"))
    w.run(8.0)
    angles = np.degrees(w.joint_angles())
    assert np.allclose(angles, [50, 35, 55], atol=0.3)
    assert w.max_anchor_error() < 1e-4


def test_scene_round_trip_through_a_file(tmp_path, examples):
    w = rm.load_scene(str(examples / "spatial_arm.robot"))
    path = tmp_path / "copy.robot"
    rm.save_scene(w, str(path))
    assert rm.load_scene(str(path)).dumps() == w.dumps()


def test_missing_scene_raises():
    with pytest.raises(rm.RoboticsError):
        rm.load_scene("/nonexistent/scene.robot")


def test_urdf_import_and_drive(examples):
    robot = rm.load_urdf(str(examples / "urdf" / "diff_drive.urdf"))
    w = robot.world
    assert robot.name and w.num_bodies >= 3
    assert rm.demos.drive_rover(w, 3.0, 0.0) >= 2
    x0 = w.positions()[0][0]
    w.run(3.0)
    assert abs(w.positions()[0][0] - x0) > 0.5  # it actually drives


def test_urdf_arm_fixed_base(examples):
    robot = rm.load_urdf(str(examples / "urdf" / "two_link_arm.urdf"), fix_base=True)
    assert robot.world.body(0).is_static
    robot.world.run(1.0)


@pytest.mark.parametrize("name", ["manipulator", "pid_servo", "linear_actuator", "rover", "sensor_rover", "noisy_pid"])
def test_demos_run(name):
    w = getattr(rm.demos, name)()
    w.run(1.0)
    assert math.isfinite(w.max_anchor_error()) and w.max_anchor_error() < 1e-2


def test_rover_drives_forward():
    w = rm.demos.rover()
    assert rm.demos.drive_rover(w, 3.0, 0.0) == 4
    chassis = w.body(0)
    x0 = chassis.position[0]
    w.run(4.0)
    assert chassis.position[0] - x0 > 1.0

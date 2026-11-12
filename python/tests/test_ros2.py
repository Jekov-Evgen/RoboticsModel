import importlib
import math
import sys

import numpy as np
import pytest

import robotics_model as rm
from robotics_model.ros2 import SimulationBridge, to_ros_quaternion, to_ros_vector


# ---------------------------------------------------------------- ROS-independent core (real tests) ----------
def test_frame_conversion_is_a_proper_rotation():
    assert np.allclose(to_ros_vector([1, 2, 3]), [1, -3, 2])  # (x, y, z) -> (x, -z, y)
    # An engine rotation of +90 deg about Y (up) is a +90 deg yaw in ROS (about z): w = x = y = ..., z = sin(45 deg).
    q = to_ros_quaternion([math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0])
    assert np.allclose(q, [math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)], atol=1e-12)
    assert abs(np.linalg.norm(to_ros_quaternion([0.5, 0.5, 0.5, 0.5])) - 1) < 1e-12


def test_joint_state_uses_child_body_names(examples):
    bridge = SimulationBridge(rm.load_scene(str(examples / "manipulator_3dof.robot")))
    state = bridge.joint_state()
    assert state["name"] == ["turret", "upper_arm", "forearm"]
    assert len(state["position"]) == len(state["velocity"]) == len(state["effort"]) == 3


def test_joint_commands_move_the_arm_and_report_unknown_names(examples):
    bridge = SimulationBridge(rm.load_scene(str(examples / "manipulator_3dof.robot")))
    assert bridge.command_joints(["turret", "forearm", "ghost"], [0.5, 0.8, 1.0]) == ["ghost"]
    bridge.advance(6.0)
    pos = dict(zip(*(bridge.joint_state()[k] for k in ("name", "position"))))
    assert abs(pos["turret"] - 0.5) < 0.01 and abs(pos["forearm"] - 0.8) < 0.01
    with pytest.raises(ValueError):
        bridge.command_joints(["turret"], [0.1, 0.2])


def test_advance_carries_the_remainder_and_honours_real_time_factor():
    w = rm.World()
    bridge = SimulationBridge(w, real_time_factor=2.0)
    taken = sum(bridge.advance(0.01) for _ in range(100))  # 1 s of wall time -> 2 s of simulation
    assert abs(w.time - 2.0) < 1e-9 and taken == 480
    bridge.paused = True
    assert bridge.advance(1.0) == 0
    with pytest.raises(ValueError):
        SimulationBridge(w, real_time_factor=0)


def test_reset_restores_the_initial_scene(examples):
    w = rm.load_scene(str(examples / "manipulator_3dof.robot"))
    bridge = SimulationBridge(w)
    start = w.positions().copy()
    bridge.advance(3.0)
    assert not np.allclose(w.positions(), start)
    bridge.reset()
    assert np.allclose(w.positions(), start) and w.time == 0


def test_tf_poses_are_in_ros_axes():
    w = rm.World()
    w.add_body("sphere", size=(0.2, 0.2, 0.2), mass=0, position=(1, 2, 3), name="marker")
    pose = SimulationBridge(w).body_poses()[0]
    assert pose["frame"] == "marker" and pose["parent"] == "world"
    assert np.allclose(pose["translation"], [1, -3, 2])
    assert np.allclose(SimulationBridge(w, ros_frame=False).body_poses()[0]["translation"], [1, 2, 3])


def test_cmd_vel_drives_the_rover_and_turns_counter_clockwise():
    w = rm.demos.rover()
    bridge = SimulationBridge(w)
    assert abs(bridge.wheel_radius - 0.38) < 1e-6 and abs(bridge.track_width - 1.3) < 1e-6
    assert bridge.command_velocity(1.0, 0.0) == 4
    bridge.advance(4.0)
    assert w.body(0).velocity[0] > 0.8  # moving forward (engine +X is ROS +x)
    w2 = rm.demos.rover()
    b2 = SimulationBridge(w2)
    b2.command_velocity(0.0, 1.0)  # ROS: positive yaw rate = counter-clockwise from above
    b2.advance(3.0)
    assert w2.body(0).angular_velocity[1] > 0.1  # about the engine's up axis (+Y)
    assert SimulationBridge(rm.World()).command_velocity(1.0, 0.0) == 0  # no wheels


def test_sensor_messages(examples):
    bridge = SimulationBridge(rm.demos.sensor_rover())
    bridge.advance(0.5)
    imu = bridge.imu(bridge.sensors_of("imu")[0])
    assert abs(imu["linear_acceleration"][2] - 9.81) < 0.4  # at rest the IMU reads +g along ROS z (up)
    assert abs(np.linalg.norm(imu["orientation"]) - 1) < 1e-6
    scan = bridge.laser_scan(bridge.sensors_of("lidar")[0])
    assert len(scan["ranges"]) == 241 and scan["angle_min"] < 0 < scan["angle_max"]
    assert abs(scan["angle_increment"] * (len(scan["ranges"]) - 1) - (scan["angle_max"] - scan["angle_min"])) < 1e-9
    assert scan["range_min"] < scan["range_max"]
    with pytest.raises(ValueError):
        bridge.imu(bridge.sensors_of("lidar")[0])


# ----------------------------------------------------------------- node logic against stand-in rclpy ---------
@pytest.fixture
def node_module():
    import fake_ros  # tests/ is on sys.path under pytest's rootdir/conftest import mode

    installed = fake_ros.install()
    sys.modules.pop("robotics_model.ros2.node", None)
    module = importlib.import_module("robotics_model.ros2.node")
    yield module
    for name in installed + ["robotics_model.ros2.node"]:
        sys.modules.pop(name, None)


def test_node_publishes_and_obeys(node_module, examples):
    node = node_module.SimulationNode(rm.load_scene(str(examples / "manipulator_3dof.robot")))
    step_timer = node.timers[0][1]
    publish_timer = node.timers[1][1]
    assert abs(node.timers[0][0] - 0.005) < 1e-12 and abs(node.timers[1][0] - 0.02) < 1e-12

    node.subscriptions["joint_commands"](type("Cmd", (), {"name": ["turret"], "position": [0.4]})())
    node.bridge.advance(4.0)  # simulate 4 s without waiting
    publish_timer()

    clock = node.publishers["/clock"].sent[-1]
    assert abs(clock.clock.sec + clock.clock.nanosec * 1e-9 - 4.0) < 0.01
    joints = node.publishers["joint_states"].sent[-1]
    assert joints.name == ["turret", "upper_arm", "forearm"] and abs(joints.position[0] - 0.4) < 0.01
    tf = node.publishers["/tf"].sent[-1]
    assert [t.child_frame_id for t in tf.transforms][:2] == ["base", "turret"] and tf.transforms[0].header.frame_id == "world"
    assert abs(tf.transforms[0].transform.rotation.w - 1.0) < 1e-6  # unrotated base

    # reset and pause services
    node.services["reset"](None, node_module.Empty.Response())
    assert node.bridge.time == 0
    reply = node.services["pause"](type("R", (), {"data": True})(), node_module.SetBool.Response())
    assert reply.success and reply.message == "paused" and node.bridge.paused
    step_timer()  # paused: must not advance
    assert node.bridge.time == 0


def test_node_publishes_sensors_once_per_sample(node_module):
    node = node_module.SimulationNode(rm.demos.sensor_rover())
    node.bridge.advance(0.5)
    node.timers[1][1]()
    node.timers[1][1]()  # no new sample in between: nothing is published twice
    topics = {t: len(p.sent) for t, p in node.publishers.items()}
    assert topics["lidar/scan"] == 1 and topics["imu/data"] == 1
    scan = node.publishers["lidar/scan"].sent[0]
    assert scan.header.frame_id == "rover_sensor" and len(scan.ranges) == 241
    node.subscriptions["cmd_vel"](type("T", (), {"linear": type("L", (), {"x": 1.0})(), "angular": type("A", (), {"z": 0.0})()})())
    node.bridge.advance(3.0)
    assert node.bridge.world.body(0).velocity[0] > 0.5

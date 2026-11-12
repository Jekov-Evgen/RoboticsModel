"""ROS 2 node that runs a Robotics Model scene and publishes it.

Needs a ROS 2 installation (``rclpy`` and the standard message packages). The simulation and the conversions live in
``bridge.py`` and do not need ROS.

    python -m robotics_model.ros2 --ros-args -p scene:=examples/manipulator_3dof.robot

Topics
    publishes   /clock                      rosgraph_msgs/Clock            simulation time
                /joint_states               sensor_msgs/JointState         actuated joints, named after the child body
                /tf                         tf2_msgs/TFMessage             pose of every body relative to "world"
                /<sensor>/data              sensor_msgs/Imu                every IMU sensor
                /<sensor>/scan              sensor_msgs/LaserScan          every 2D lidar sensor
    subscribes  /joint_commands             sensor_msgs/JointState         position setpoints for named joints
                /cmd_vel                    geometry_msgs/Twist            differential-drive rovers
    services    /reset                      std_srvs/Empty                 restore the initial scene
                /pause                      std_srvs/SetBool               pause / resume the simulation
"""

from __future__ import annotations

import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_srvs.srv import Empty, SetBool
from tf2_msgs.msg import TFMessage

import robotics_model as rm

from .bridge import SimulationBridge


def _stamp(seconds: float) -> Time:
    t = Time()
    t.sec = int(seconds)
    t.nanosec = int(round((seconds - int(seconds)) * 1e9))
    if t.nanosec >= 1_000_000_000:
        t.sec += 1
        t.nanosec -= 1_000_000_000
    return t


class SimulationNode(Node):
    """Steps the simulation from a timer and publishes its state."""

    def __init__(self, world: rm.World | None = None, node_name: str = "robotics_model_sim"):
        super().__init__(node_name)
        self.declare_parameter("scene", "")
        self.declare_parameter("urdf", "")
        self.declare_parameter("fix_base", False)
        self.declare_parameter("ros_frame", True)
        self.declare_parameter("real_time_factor", 1.0)
        self.declare_parameter("step_rate", 200.0)  # Hz of the stepping timer (wall time)
        self.declare_parameter("publish_rate", 50.0)  # Hz of joint states / tf / sensor topics (wall time)
        self.declare_parameter("wheel_radius", 0.0)  # 0 = estimate from the scene
        self.declare_parameter("track_width", 0.0)
        self.declare_parameter("turn_scale", 1.0)

        param = lambda name: self.get_parameter(name).value  # noqa: E731
        if world is None:
            world = self._load_world(param("scene"), param("urdf"), param("fix_base"))
        self.bridge = SimulationBridge(
            world,
            ros_frame=param("ros_frame"),
            real_time_factor=param("real_time_factor"),
            wheel_radius=param("wheel_radius") or None,
            track_width=param("track_width") or None,
            turn_scale=param("turn_scale"),
        )
        self._last_wall = time.monotonic()
        self._published_samples: dict[int, int] = {}

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.sensor_pubs: dict[int, object] = {}
        for kind, message in (("imu", Imu), ("lidar", LaserScan)):
            for sensor in self.bridge.sensors_of(kind):
                suffix = "data" if kind == "imu" else "scan"
                self.sensor_pubs[sensor.index] = (kind, self.create_publisher(message, f"{sensor.name}/{suffix}", 10))

        self.create_subscription(JointState, "joint_commands", self._on_joint_command, 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_service(Empty, "reset", self._on_reset)
        self.create_service(SetBool, "pause", self._on_pause)
        self.create_timer(1.0 / param("step_rate"), self._on_step)
        self.create_timer(1.0 / param("publish_rate"), self._on_publish)
        self.get_logger().info(
            f"simulating {self.bridge.world.num_bodies} bodies, {len(self.bridge.world.actuated_joints)} actuated joints, "
            f"real-time factor {self.bridge.real_time_factor}"
        )

    @staticmethod
    def _load_world(scene: str, urdf: str, fix_base: bool) -> rm.World:
        if scene:
            return rm.load_scene(scene)
        if urdf:
            return rm.load_urdf(urdf, fix_base=fix_base).world
        return rm.demos.manipulator()

    # ---- timers ------------------------------------------------------------------------------------------
    def _on_step(self) -> None:
        now = time.monotonic()
        self.bridge.advance(now - self._last_wall)
        self._last_wall = now

    def _on_publish(self) -> None:
        stamp = _stamp(self.bridge.time)
        clock = Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)

        state = self.bridge.joint_state()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name, msg.position, msg.velocity, msg.effort = state["name"], state["position"], state["velocity"], state["effort"]
        self.joint_pub.publish(msg)

        tf = TFMessage()
        for pose in self.bridge.body_poses():
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = pose["parent"]
            t.child_frame_id = pose["frame"]
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pose["translation"]
            w, x, y, z = pose["rotation"]
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = x, y, z, w
            tf.transforms.append(t)
        self.tf_pub.publish(tf)

        for sensor in self.bridge.world.sensors:
            entry = self.sensor_pubs.get(sensor.index)
            if entry is None or not sensor.valid or self._published_samples.get(sensor.index) == sensor.samples:
                continue
            self._published_samples[sensor.index] = sensor.samples
            kind, publisher = entry
            if kind == "imu":
                d = self.bridge.imu(sensor)
                out = Imu()
                out.header.stamp, out.header.frame_id = stamp, d["frame"]
                out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = d["linear_acceleration"]
                out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = d["angular_velocity"]
                w, x, y, z = d["orientation"]
                out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = x, y, z, w
            else:
                d = self.bridge.laser_scan(sensor)
                out = LaserScan()
                out.header.stamp, out.header.frame_id = stamp, d["frame"]
                out.angle_min, out.angle_max, out.angle_increment = d["angle_min"], d["angle_max"], d["angle_increment"]
                out.range_min, out.range_max = d["range_min"], d["range_max"]
                out.ranges = d["ranges"]
            publisher.publish(out)

    # ---- inputs ------------------------------------------------------------------------------------------
    def _on_joint_command(self, msg: JointState) -> None:
        unknown = self.bridge.command_joints(list(msg.name), list(msg.position))
        if unknown:
            self.get_logger().warn(f"ignored unknown or invalid joints: {unknown}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self.bridge.command_velocity(msg.linear.x, msg.angular.z) == 0:
            self.get_logger().warn("cmd_vel ignored: the scene has no differential-drive wheels")

    def _on_reset(self, request, response):
        self.bridge.reset()
        return response

    def _on_pause(self, request, response):
        self.bridge.paused = bool(request.data)
        response.success = True
        response.message = "paused" if self.bridge.paused else "running"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

"""Minimal stand-ins for rclpy and the message packages, with the same field layout as the real ROS 2 messages.

They let the node logic run in CI without a ROS installation. They do NOT verify that the real rclpy API is used
correctly: that needs a ROS 2 environment (see docs/ROS2.md).
"""

import sys
import types


class _Ns:
    """Attribute bag that creates nested namespaces on demand (like a message with sub-messages)."""

    def __init__(self, **fields):
        self.__dict__.update(fields)


def _msg(_class_name, /, **fields):
    def init(self):
        for key, default in fields.items():
            setattr(self, key, default() if callable(default) else default)

    return type(_class_name, (), {"__init__": init})


Time = _msg("Time", sec=0, nanosec=0)
Header = _msg("Header", stamp=Time, frame_id="")
Vector3 = _msg("Vector3", x=0.0, y=0.0, z=0.0)
Quaternion = _msg("Quaternion", x=0.0, y=0.0, z=0.0, w=1.0)
Transform = _msg("Transform", translation=Vector3, rotation=Quaternion)
TransformStamped = _msg("TransformStamped", header=Header, child_frame_id="", transform=Transform)
TFMessage = _msg("TFMessage", transforms=list)
Clock = _msg("Clock", clock=Time)
JointState = _msg("JointState", header=Header, name=list, position=list, velocity=list, effort=list)
Imu = _msg("Imu", header=Header, orientation=Quaternion, angular_velocity=Vector3, linear_acceleration=Vector3)
LaserScan = _msg("LaserScan", header=Header, angle_min=0.0, angle_max=0.0, angle_increment=0.0, range_min=0.0, range_max=0.0, ranges=list)
Twist = _msg("Twist", linear=Vector3, angular=Vector3)


class Empty:
    class Request:  # noqa: D106
        pass

    class Response:  # noqa: D106
        pass


class SetBool:
    class Request:
        def __init__(self):
            self.data = False

    class Response:
        def __init__(self):
            self.success = False
            self.message = ""


class _Publisher:
    def __init__(self, topic):
        self.topic, self.sent = topic, []

    def publish(self, msg):
        self.sent.append(msg)


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, text):
        self.messages.append(("info", text))

    def warn(self, text):
        self.messages.append(("warn", text))


class Node:
    def __init__(self, name):
        self.name = name
        self._params, self.publishers, self.subscriptions, self.services, self.timers = {}, {}, {}, {}, []
        self._logger = _Logger()

    def declare_parameter(self, name, default):
        self._params[name] = default

    def get_parameter(self, name):
        return _Ns(value=self._params[name])

    def create_publisher(self, msg_type, topic, qos):
        pub = _Publisher(topic)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_service(self, srv_type, name, callback):
        self.services[name] = callback

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_logger(self):
        return self._logger


def install():
    """Registers the fake modules in sys.modules; returns the list of names to remove again."""
    modules = {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "builtin_interfaces": types.ModuleType("builtin_interfaces"),
        "builtin_interfaces.msg": types.ModuleType("builtin_interfaces.msg"),
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
        "rosgraph_msgs": types.ModuleType("rosgraph_msgs"),
        "rosgraph_msgs.msg": types.ModuleType("rosgraph_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "std_srvs": types.ModuleType("std_srvs"),
        "std_srvs.srv": types.ModuleType("std_srvs.srv"),
        "tf2_msgs": types.ModuleType("tf2_msgs"),
        "tf2_msgs.msg": types.ModuleType("tf2_msgs.msg"),
    }
    modules["rclpy.node"].Node = Node
    modules["builtin_interfaces.msg"].Time = Time
    modules["geometry_msgs.msg"].TransformStamped = TransformStamped
    modules["geometry_msgs.msg"].Twist = Twist
    modules["rosgraph_msgs.msg"].Clock = Clock
    modules["sensor_msgs.msg"].JointState = JointState
    modules["sensor_msgs.msg"].Imu = Imu
    modules["sensor_msgs.msg"].LaserScan = LaserScan
    modules["std_srvs.srv"].Empty = Empty
    modules["std_srvs.srv"].SetBool = SetBool
    modules["tf2_msgs.msg"].TFMessage = TFMessage
    sys.modules.update(modules)
    return list(modules)

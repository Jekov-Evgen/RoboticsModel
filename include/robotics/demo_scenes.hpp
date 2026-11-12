#pragma once
// Built-in example robots used by the editor, CLI and tests.

#include "robotics/world.hpp"

namespace robotics {

// Two-link motorized arm with a tool and a free ball.
[[nodiscard]] World makeDemo();

// Single hinge driven by a PID step response.
[[nodiscard]] World makeControlDemo();

// Slider carriage on a fixed rail driven by a PID position step (limits 0..0.5 m), carrying a free
// hinged flap whose swing is limited to +-35 degrees.
[[nodiscard]] World makeLinearActuatorDemo();

// Four-wheel differential-drive rover (paused, wheels at zero speed).
[[nodiscard]] World makeRoverDemo();

// Starter rover with sensors: 2D lidar on the sensor head, IMU on the chassis, a front camera, wheel
// encoders, and a few static obstacles to look at.
[[nodiscard]] World makeSensorRoverDemo();

// PID test bench whose controller reads a noisy (sigma 0.02 rad), quantized (256 CPR), delayed (30 ms)
// encoder instead of the ideal joint angle; also carries a force/torque sensor on the joint.
[[nodiscard]] World makeNoisyPidDemo();

// Demo controls: forward and turn are wheel speeds in rad/s; feedback-controlled wheels are untouched.
// Only cylinders on hinge joints count as wheels (slider and other joints are ignored).
// Returns the number of wheels that received the command.
int driveRover(World& world, double forward, double turn);

// True for a drivable wheel: an unlimited hinge whose child is a cylinder spinning about its own axis
// (cylinder Z parallel to the joint axis) with a roughly horizontal axis. Excludes arm joints that happen
// to carry cylindrical links. Used by the editor's WASD driving and camera follow.
[[nodiscard]] bool isWheelJoint(const World& world, const Joint& joint);

} // namespace robotics

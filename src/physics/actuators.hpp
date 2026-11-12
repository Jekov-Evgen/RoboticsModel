#pragma once

// Joint sensing and actuation (internal): encoders, feedback controllers, motor telemetry.

namespace robotics {
class World;
} // namespace robotics

namespace robotics::detail {

// Updates the coordinate (Joint::angle) of every actuated joint from the body poses: the continuous
// multi-turn hinge angle, or the prismatic displacement along the axis.
void updateEncoders(World& world);

// Samples every actuated joint, evaluates feedback controllers and applies their effort impulses
// (torque about a hinge axis, force along a prismatic axis).
// Appends one telemetry sample per actuated joint (bounded by Joint::historyCapacity).
void applyControllers(World& world, double dt);

// After the solver: stores the speed-motor effort/saturation in the step's telemetry sample.
void recordMotorTelemetry(World& world, double dt);

} // namespace robotics::detail

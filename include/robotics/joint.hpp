#pragma once
// Joints connect two bodies. Data only; solving lives in src/physics/joint_solver.cpp.
//
// Adding a joint type: see docs/ARCHITECTURE.md, "How to add a joint type".
// Every switch over JointType in the core is exhaustive (-Wswitch), so the compiler lists the
// places that need a decision for the new type.

#include "robotics/controller.hpp"
#include "robotics/math.hpp"

#include <cstddef>
#include <deque>
#include <string_view>

namespace robotics {

// Stored in .robot files and in the editor's segmented control by numeric value:
// append new types at the end only.
// Prismatic is called "Slider" in the editor.
enum class JointType { Fixed, Ball, Hinge, Prismatic };

[[nodiscard]] constexpr bool isKnownJointType(JointType type) {
    switch (type) {
    case JointType::Fixed:
    case JointType::Ball:
    case JointType::Hinge:
    case JointType::Prismatic:
        return true;
    }
    return false;
}

// True for joint types with one actuated degree of freedom: they get an encoder, a speed motor,
// feedback controllers, range limits and telemetry history.
[[nodiscard]] constexpr bool isActuatedJointType(JointType type) {
    switch (type) {
    case JointType::Hinge:
    case JointType::Prismatic:
        return true;
    case JointType::Fixed:
    case JointType::Ball:
        return false;
    }
    return false;
}

[[nodiscard]] constexpr std::string_view jointTypeName(JointType type) {
    switch (type) {
    case JointType::Fixed:
        return "Fixed";
    case JointType::Ball:
        return "Ball";
    case JointType::Hinge:
        return "Hinge";
    case JointType::Prismatic:
        return "Prismatic";
    }
    return "Unknown";
}

struct Joint {
    // Number of telemetry samples kept in `history`: 10 s at the default 240 Hz step.
    static constexpr std::size_t historyCapacity = 2400;

    // --- Topology -------------------------------------------------------------------------
    JointType type{JointType::Hinge};
    int a{}; // Parent body index.
    int b{}; // Child body index.

    // --- Frames (body-local, set by World::connect) -------------------------------------
    Vec3 anchorA{};
    Vec3 anchorB{};
    Vec3 axisA{0, 0, 1}; // Hinge rotation axis / prismatic slide axis (unit) in A's frame.
    Vec3 axisB{0, 0, 1}; // Same axis in B's frame.
    Quat reference{};    // Relative orientation A^-1 * B at connect(); zero angle and locked orientation
                         // of fixed and prismatic joints.

    // --- Actuator ---------------------------------------------------------------------------
    // Units follow the joint coordinate: hinge rad/s and N·m, prismatic m/s and N.
    // Speed motor: target joint speed and maximum effort; maxTorque 0 disables the motor.
    // maxTorque ("max effort", force for a prismatic joint) also bounds feedback controllers.
    double motorSpeed{};
    double maxTorque{};
    ControllerConfig controller;
    ControllerState controllerState;

    // --- Limits (hinge and prismatic) ------------------------------------------------------
    // Allowed range of the joint coordinate `angle` (rad or m); lowerLimit <= upperLimit.
    // Solved as unbounded one-sided constraints, so they also stop motors and controllers.
    bool limitsEnabled{};
    double lowerLimit{};
    double upperLimit{};

    // --- Encoder (zero at connect()) --------------------------------------------------------
    // `angle` is the joint coordinate: hinge = continuous multi-turn angle (rad); prismatic =
    // displacement of B's anchor from A's anchor along the axis (m).
    double angle{};
    double lastWrappedAngle{}; // Hinge only: previous angle wrapped to (-pi, pi].
    bool encoderInitialized{};

    // --- Telemetry --------------------------------------------------------------------------
    std::deque<ControlSample> history; // Bounded to historyCapacity samples, one per step.

    // --- Solver results of the last World::step() (reset at the start of every step) --------
    // All impulses are accumulated over solver iterations and act on body A (body B receives the
    // opposite). Divide by the step dt for an average force/torque, e.g. for visualization.
    double motorImpulse{};         // Speed-motor impulse about/along the joint axis (N·m·s or N·s).
    Vec3 reactionImpulse{};        // Linear constraint impulse (N·s), world space, motor and limits excluded.
    Vec3 reactionAngularImpulse{}; // Angular constraint impulse (N·m·s), world space, motor excluded.
    double lowerLimitImpulse{};    // >= 0: limit impulse pushing B towards larger `angle`.
    double upperLimitImpulse{};    // >= 0: limit impulse pushing B towards smaller `angle`.

    [[nodiscard]] bool isActuated() const { return isActuatedJointType(type); }

    // Joint coordinate (rad for hinges, m for prismatic joints); alias of `angle`.
    [[nodiscard]] double position() const { return angle; }

    // Enables the range limits (e.g. URDF <limit lower upper>). Throws for a non-finite or inverted range;
    // World::validate() additionally rejects limits on joint types without a free coordinate.
    void setLimits(double lower, double upper);

    [[nodiscard]] bool usesSpeedMotor() const { return controller.type == ControllerType::SpeedMotor; }

    [[nodiscard]] bool connects(int first, int second) const {
        return (a == first && b == second) || (a == second && b == first);
    }
};

} // namespace robotics

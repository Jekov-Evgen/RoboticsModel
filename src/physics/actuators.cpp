#include "actuators.hpp"

#include "robotics/world.hpp"

#include <cmath>

namespace robotics::detail {
namespace {

// Tolerance (N·m·s or N·s) when deciding whether the motor impulse hit its bound.
constexpr double saturationTolerance = 1e-12;

// Joint axis in world space (attached to body A).
Vec3 worldAxis(const World& world, const Joint& joint) {
    return world.bodies[static_cast<std::size_t>(joint.a)].orientation.rotate(joint.axisA);
}

// Prismatic coordinate: displacement of B's anchor from A's anchor along the axis (zero at connect()).
void updatePrismaticEncoder(Joint& joint, const Body& a, const Body& b) {
    Vec3 axis = a.orientation.rotate(joint.axisA);
    joint.angle = (b.point(joint.anchorB) - a.point(joint.anchorA)).dot(axis);
    joint.lastWrappedAngle = joint.angle;
    joint.encoderInitialized = true;
}

// Joint speed (rate of Joint::angle): rad/s for hinges, m/s for prismatic joints.
double jointSpeed(const Joint& joint, const Body& a, const Body& b) {
    Vec3 axis = a.orientation.rotate(joint.axisA);
    if (joint.type == JointType::Prismatic) {
        Vec3 point = b.point(joint.anchorB);
        return (b.pointVelocity(point) - a.pointVelocity(point)).dot(axis);
    }
    return (b.angularVelocity - a.angularVelocity).dot(axis);
}

} // namespace

void updateEncoders(World& world) {
    for (auto& joint : world.joints) {
        if (!joint.isActuated()) {
            continue;
        }
        const Body& a = world.bodies[static_cast<std::size_t>(joint.a)];
        const Body& b = world.bodies[static_cast<std::size_t>(joint.b)];
        if (joint.type == JointType::Prismatic) {
            updatePrismaticEncoder(joint, a, b);
            continue;
        }
        Quat relative = a.orientation.conjugate() * b.orientation;
        Quat delta = (relative * joint.reference.conjugate()).normalized();
        // Swing-twist decomposition: twist about the joint axis, wrapped to (-pi, pi].
        double twist = Vec3{delta.x, delta.y, delta.z}.dot(joint.axisA);
        double wrapped = std::remainder(2 * std::atan2(twist, delta.w), 2 * pi);
        if (!joint.encoderInitialized) {
            joint.angle = wrapped;
            joint.encoderInitialized = true;
        } else {
            // Unwrap: accumulate the shortest change since the previous sample.
            joint.angle += std::remainder(wrapped - joint.lastWrappedAngle, 2 * pi);
        }
        joint.lastWrappedAngle = wrapped;
    }
}

void applyControllers(World& world, double dt) {
    // Measure every joint before applying any torque: independent of joint order.
    for (auto& joint : world.joints) {
        if (!joint.isActuated()) {
            continue;
        }
        const Body& a = world.bodies[static_cast<std::size_t>(joint.a)];
        const Body& b = world.bodies[static_cast<std::size_t>(joint.b)];
        double speed = jointSpeed(joint, a, b);
        ControlSample sample;
        if (joint.usesSpeedMotor()) {
            // The motor itself runs inside the solver; effort is filled in by recordMotorTelemetry().
            sample.time = world.time;
            sample.target = joint.motorSpeed;
            sample.measured = speed;
            sample.error = joint.motorSpeed - speed;
        } else {
            // Sensor feedback (robotics::updateSensors) replaces the ideal measurement when present.
            const ControllerState& state = joint.controllerState;
            double measuredAngle = state.sensorFeedback ? state.feedbackPosition : joint.angle;
            double measuredSpeed = state.sensorFeedback ? state.feedbackSpeed : speed;
            double measured =
                joint.controller.variable == ControlVariable::Position ? measuredAngle : measuredSpeed;
            sample = evaluateController(joint.controller, joint.controllerState, world.time, measured,
                                        measuredSpeed, joint.maxTorque, dt);
        }
        sample.angle = joint.angle;
        sample.speed = speed;
        joint.history.push_back(sample);
        if (joint.history.size() > Joint::historyCapacity) {
            joint.history.pop_front();
        }
    }
    for (auto& joint : world.joints) {
        if (!joint.isActuated() || joint.usesSpeedMotor()) {
            continue;
        }
        Vec3 impulse = worldAxis(world, joint) * (joint.history.back().effort * dt);
        Body& a = world.bodies[static_cast<std::size_t>(joint.a)];
        Body& b = world.bodies[static_cast<std::size_t>(joint.b)];
        if (joint.type == JointType::Prismatic) {
            // Linear actuator force along the axis, acting at B's anchor on both bodies.
            Vec3 point = b.point(joint.anchorB);
            a.impulse(-impulse, point);
            b.impulse(impulse, point);
        } else {
            a.angularImpulse(-impulse);
            b.angularImpulse(impulse);
        }
    }
}

void recordMotorTelemetry(World& world, double dt) {
    for (auto& joint : world.joints) {
        if (!joint.isActuated() || !joint.usesSpeedMotor() || joint.history.empty()) {
            continue;
        }
        ControlSample& sample = joint.history.back();
        sample.effort = -joint.motorImpulse / dt;
        sample.saturated =
            joint.maxTorque > 0 && std::abs(joint.motorImpulse) >= joint.maxTorque * dt - saturationTolerance;
    }
}

} // namespace robotics::detail

namespace robotics {

void World::resetControllers() {
    for (auto& joint : joints) {
        joint.controllerState = {};
        joint.history.clear();
    }
}

} // namespace robotics

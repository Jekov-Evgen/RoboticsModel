#include "robotics/world.hpp"

#include "actuators.hpp"
#include "collision.hpp"
#include "contact_solver.hpp"
#include "integrator.hpp"
#include "joint_direct.hpp"
#include "joint_solver.hpp"
#include "solver_body.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robotics {
namespace {

// Tolerances used by validate().
constexpr double unitLengthTolerance = 1e-5;
constexpr double equalDiameterTolerance = 1e-8;
constexpr double minBodySize = 0.001;
constexpr double minAxisLength = 1e-8;

bool isBodyIndex(int index, std::size_t bodyCount) {
    return index >= 0 && index < static_cast<int>(bodyCount);
}

void validateBody(const Body& body) {
    if (!body.position.finite() || !body.velocity.finite() || !body.angularVelocity.finite()
        || !body.force.finite() || !body.torque.finite() || !body.size.finite() || body.size.x < minBodySize
        || body.size.y < minBodySize || body.size.z < minBodySize || !std::isfinite(body.mass)
        || body.mass < 0 || !std::isfinite(body.friction) || body.friction < 0
        || !std::isfinite(body.restitution) || body.restitution < 0 || body.restitution > 1) {
        throw std::runtime_error("Invalid body: " + body.name);
    }
    if (!isKnownShape(body.shape)) {
        throw std::runtime_error("Unknown shape");
    }
    const Quat& q = body.orientation;
    double norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (!std::isfinite(norm) || std::abs(norm - 1) > unitLengthTolerance) {
        throw std::runtime_error("Orientation must be a unit quaternion");
    }
    if (body.shape == Shape::ConvexHull && (!body.hull || body.hull->faces.empty())) {
        throw std::runtime_error("Convex hull body without hull data: " + body.name);
    }
    if (body.shape == Shape::Sphere
        && (std::abs(body.size.x - body.size.y) > equalDiameterTolerance
            || std::abs(body.size.x - body.size.z) > equalDiameterTolerance)) {
        throw std::runtime_error("Sphere needs three equal diameters");
    }
    if (body.shape == Shape::Cylinder && std::abs(body.size.x - body.size.y) > equalDiameterTolerance) {
        throw std::runtime_error("Cylinder X and Y diameters must match");
    }
}

void validateLimits(double lower, double upper) {
    if (!std::isfinite(lower) || !std::isfinite(upper)) {
        throw std::runtime_error("Joint limits must be finite");
    }
    if (lower > upper) {
        throw std::runtime_error("Joint lower limit must not exceed the upper limit");
    }
}

void validateJoint(const Joint& joint, std::size_t bodyCount) {
    if (!isBodyIndex(joint.a, bodyCount) || !isBodyIndex(joint.b, bodyCount) || joint.a == joint.b
        || !joint.anchorA.finite() || !joint.anchorB.finite() || !joint.axisA.finite()
        || !joint.axisB.finite() || std::abs(joint.axisA.length() - 1) > unitLengthTolerance
        || std::abs(joint.axisB.length() - 1) > unitLengthTolerance || !std::isfinite(joint.motorSpeed)
        || !std::isfinite(joint.maxTorque) || joint.maxTorque < 0) {
        throw std::runtime_error("Invalid joint");
    }
    if (!isKnownJointType(joint.type)) {
        throw std::runtime_error("Unknown joint type");
    }
    (void)joint.reference.normalized(); // Throws for a degenerate reference orientation.
    joint.controller.validate();
    if (!joint.isActuated() && !joint.usesSpeedMotor()) {
        throw std::runtime_error("Feedback controllers require a hinge or slider joint");
    }
    if (!std::isfinite(joint.angle) || !std::isfinite(joint.lastWrappedAngle)) {
        throw std::runtime_error("Invalid joint encoder");
    }
    validateLimits(joint.lowerLimit, joint.upperLimit);
    if (joint.limitsEnabled && !joint.isActuated()) {
        throw std::runtime_error("Joint limits require a hinge or slider joint");
    }
}

} // namespace

void Joint::setLimits(double lower, double upper) {
    validateLimits(lower, upper);
    lowerLimit = lower;
    upperLimit = upper;
    limitsEnabled = true;
}

int World::addBody(Body body) {
    body.orientation = body.orientation.normalized();
    bodies.push_back(body);
    try {
        validate();
    } catch (...) {
        bodies.pop_back();
        throw;
    }
    return static_cast<int>(bodies.size()) - 1;
}

int World::connect(int a, int b, JointType type, Vec3 worldAnchor, Vec3 worldAxis) {
    if (!isBodyIndex(a, bodies.size()) || !isBodyIndex(b, bodies.size()) || a == b) {
        throw std::runtime_error("A joint needs two different existing bodies");
    }
    if (!worldAnchor.finite() || !worldAxis.finite() || worldAxis.length() < minAxisLength) {
        throw std::runtime_error("Invalid joint anchor or axis");
    }
    const Body& bodyA = bodies[static_cast<std::size_t>(a)];
    const Body& bodyB = bodies[static_cast<std::size_t>(b)];
    Joint joint;
    joint.a = a;
    joint.b = b;
    joint.type = type;
    joint.anchorA = bodyA.localPoint(worldAnchor);
    joint.anchorB = bodyB.localPoint(worldAnchor);
    joint.axisA = bodyA.orientation.conjugate().rotate(worldAxis.normalized());
    joint.axisB = bodyB.orientation.conjugate().rotate(worldAxis.normalized());
    joint.reference = bodyA.orientation.conjugate() * bodyB.orientation;
    joints.push_back(joint);
    return static_cast<int>(joints.size()) - 1;
}

void World::removeBody(int index) {
    if (!isBodyIndex(index, bodies.size())) {
        throw std::runtime_error("Invalid body index");
    }
    std::erase_if(joints, [=](const Joint& joint) { return joint.a == index || joint.b == index; });
    for (auto& joint : joints) {
        if (joint.a > index) {
            --joint.a;
        }
        if (joint.b > index) {
            --joint.b;
        }
    }
    std::erase_if(sensors, [=](const Sensor& sensor) { return sensor.body == index; });
    for (auto& sensor : sensors) {
        if (sensor.body > index) {
            --sensor.body;
        }
    }
    bodies.erase(bodies.begin() + index);
    contacts.clear();
}

void World::validate() const {
    if (!gravity.finite() || iterations < minIterations || iterations > maxIterations
        || substeps < minSubsteps || substeps > maxSubsteps) {
        throw std::runtime_error("Invalid world settings");
    }
    for (const auto& body : bodies) {
        validateBody(body);
    }
    for (const auto& joint : joints) {
        validateJoint(joint, bodies.size());
    }
}

void World::step(double dt) {
    if (!std::isfinite(dt) || dt <= 0 || dt > maxTimeStep) {
        throw std::runtime_error("Physics dt must be in (0, 1/30] seconds");
    }
    if (substeps < minSubsteps || substeps > maxSubsteps || iterations < minIterations
        || iterations > maxIterations) {
        throw std::runtime_error("Invalid world settings");
    }
    // 1. Sense and actuate (feedback controllers apply torque impulses here).
    detail::updateEncoders(*this);
    detail::applyControllers(*this, dt);

    // 2. Collision detection once per step; previous contacts provide warm-start impulses.
    std::vector<Contact> previous;
    previous.swap(contacts);
    detail::ContinuousCollision continuous;
    if (continuousCollision) {
        continuous = {dt, gravity};
    }
    detail::detectContacts(bodies, joints, contacts, continuous);
    if (warmStarting) {
        detail::matchContacts(contacts, previous);
    } else {
        for (auto& contact : contacts) {
            contact.normalImpulse = 0;
            contact.tangentImpulse = {};
        }
    }

    // 3. Substeps. Impulses are stored as totals over a step; one substep starts from total * h / dt.
    const double h = dt / substeps;
    const double warmScale = warmStarting ? h / dt : 0.0;
    const int passes = (iterations + substeps - 1) / substeps;
    // Position errors are small after the exact joint projection; half the passes suffice for contacts.
    const int positionPasses = (passes + 1) / 2;
    std::vector<detail::SolverBody> state;
    state.reserve(bodies.size() + 1);
    for (const auto& body : bodies) {
        state.push_back(detail::makeSolverBody(body));
    }
    state.push_back(detail::SolverBody{}); // Ground: static, at the origin.

    std::vector<detail::JointConstraint> jointRows;
    std::vector<detail::ContactConstraint> contactRows;
    detail::initJoints(joints, jointRows, warmScale);
    detail::initContacts(contacts, bodies, state, contactRows, warmScale);

    detail::DirectJointSolver direct;
    direct.analyze(jointRows, state);
    detail::ImpactSchedule impacts;
    impacts.init(contactRows, joints, state.size());
    std::vector<detail::SolverBody> pseudo;
    for (int substep = 0; substep < substeps; ++substep) {
        impacts.advance(contactRows, state, substep * h, h);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            detail::integrateVelocity(state[i], gravity, bodies[i].force, bodies[i].torque, h);
        }
        detail::prepareJoints(jointRows, state);
        detail::prepareContacts(contactRows, state);
        direct.factor(jointRows, state);
        direct.computeResponses(jointRows, state);
        // Velocity passes: warm-started impulses without position bias (no energy from correction).
        detail::warmStartJoints(jointRows, state);
        detail::warmStartContacts(contactRows, state);
        for (int pass = 0; pass < passes; ++pass) {
            detail::solveJoints(jointRows, state, h, detail::SolvePass::Velocity);
            detail::solveContacts(contactRows, state, h, detail::SolvePass::Velocity);
            direct.solve(jointRows, state, h, detail::SolvePass::Velocity);
        }
        // Position passes (split impulses): pseudo-velocities that remove joint drift, limit violations
        // and penetration. They move the bodies in this substep's integration, then are discarded.
        pseudo = state;
        for (auto& body : pseudo) {
            body.velocity = {};
            body.angularVelocity = {};
        }
        detail::resetPseudoImpulses(jointRows);
        detail::resetPseudoImpulses(contactRows);
        for (int pass = 0; pass < positionPasses; ++pass) {
            detail::solveJoints(jointRows, pseudo, h, detail::SolvePass::Position);
            detail::solveContacts(contactRows, pseudo, h, detail::SolvePass::Position);
            direct.solve(jointRows, pseudo, h, detail::SolvePass::Position);
        }
        for (std::size_t i = 0; i < state.size(); ++i) {
            if (impacts.stopped(i)) {
                continue;
            }
            detail::integratePosition(state[i], state[i].velocity + pseudo[i].velocity,
                                      state[i].angularVelocity + pseudo[i].angularVelocity, h);
        }
        detail::accumulateJointTotals(jointRows);
        detail::accumulateContactTotals(contactRows);
    }
    detail::applyRestitution(contactRows, state);
    detail::storeJointImpulses(jointRows);
    detail::storeContactImpulses(contactRows, contacts);
    detail::recordMotorTelemetry(*this, dt);
    // Speculative contacts that never closed were no contact at all.
    std::erase_if(contacts, [](const Contact& c) { return c.speculative && c.normalImpulse <= 0; });

    // 4. Write back poses and velocities; clear force accumulators.
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        Body& body = bodies[i];
        if (body.mass > 0) {
            body.position = state[i].position;
            body.orientation = state[i].orientation;
            body.velocity = state[i].velocity;
            body.angularVelocity = state[i].angularVelocity;
        }
        body.force = {};
        body.torque = {};
    }
    time += dt;
    detail::updateEncoders(*this);
}

double World::maxAnchorError() const {
    double result = 0;
    for (const auto& joint : joints) {
        const Body& a = bodies[static_cast<std::size_t>(joint.a)];
        const Body& b = bodies[static_cast<std::size_t>(joint.b)];
        Vec3 error = a.point(joint.anchorA) - b.point(joint.anchorB);
        if (joint.type == JointType::Prismatic) {
            // Sliding along the axis is the joint's motion, not drift.
            Vec3 axis = a.orientation.rotate(joint.axisA);
            error -= axis * error.dot(axis);
        }
        result = std::max(result, error.length());
    }
    return result;
}

} // namespace robotics

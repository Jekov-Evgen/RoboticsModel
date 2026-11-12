#include "joint_solver.hpp"

#include <algorithm>
#include <cmath>

namespace robotics::detail {
namespace {

// Fraction of a limit violation removed per substep by the position (pseudo-velocity) pass.
constexpr double jointBaumgarte = 0.5;
// Clamp (m/s or rad/s) of the limit correction pseudo-velocity.
constexpr double maxJointCorrectionVelocity = 5.0;

// Two unit directions perpendicular to unit `axis` (shared by hinge and prismatic joints).
void perpendicularBasis(Vec3 axis, Vec3& t, Vec3& u) {
    t = axis.cross(std::abs(axis.x) < 0.8 ? Vec3{1, 0, 0} : Vec3{0, 1, 0}).normalized();
    u = axis.cross(t);
}

double inverseOrZero(double k) {
    return k > 1e-12 ? 1 / k : 0;
}

bool hasSpeedMotor(const Joint& joint) {
    return joint.isActuated() && joint.maxTorque > 0 && joint.usesSpeedMotor();
}

bool hasLimits(const Joint& joint) {
    return joint.isActuated() && joint.limitsEnabled;
}

// Continuous hinge angle for the current poses, unwrapped from the encoder value at the step start.
double hingeCoordinate(const JointConstraint& c, const SolverBody& a, const SolverBody& b) {
    const Joint& joint = *c.joint;
    Quat relative = a.orientation.conjugate() * b.orientation;
    Quat delta = (relative * joint.reference.conjugate()).normalized();
    double twist = Vec3{delta.x, delta.y, delta.z}.dot(joint.axisA);
    double wrapped = std::remainder(2 * std::atan2(twist, delta.w), 2 * pi);
    return c.angle0 + std::remainder(wrapped - c.wrapped0, 2 * pi);
}

Vec3 pointVelocityError(const JointConstraint& c, const SolverBody& a, const SolverBody& b) {
    return a.pointVelocity(c.rA) - b.pointVelocity(c.rB);
}

// Generalized impulse that increases the joint coordinate (B along +axis relative to A).
void applyFreeImpulse(const JointConstraint& c, SolverBody& a, SolverBody& b, double impulse,
                      std::vector<SolverBody>& bodies) {
    if (!c.response.empty()) {
        for (const BodyDelta& d : c.response) {
            SolverBody& body = bodies[static_cast<std::size_t>(d.body)];
            body.velocity += d.linear * impulse;
            body.angularVelocity += d.angular * impulse;
        }
        return;
    }
    if (c.joint->type == JointType::Prismatic) {
        a.applyImpulse(c.axis * -impulse, c.rA);
        b.applyImpulse(c.axis * impulse, c.rB);
    } else {
        a.applyAngularImpulse(c.axis * -impulse);
        b.applyAngularImpulse(c.axis * impulse);
    }
}

// Rate of the joint coordinate: rad/s (hinge) or m/s (prismatic).
double freeSpeed(const JointConstraint& c, const SolverBody& a, const SolverBody& b) {
    if (c.joint->type == JointType::Prismatic) {
        return -pointVelocityError(c, a, b).dot(c.axis);
    }
    return (b.angularVelocity - a.angularVelocity).dot(c.axis);
}

// Velocity motor: accumulated impulse bounded by maxTorque * h per substep.
void solveMotor(JointConstraint& c, SolverBody& a, SolverBody& b, double h, std::vector<SolverBody>& bodies) {
    const Joint& joint = *c.joint;
    double speed = -freeSpeed(c, a, b); // Same sign convention as Joint::motorImpulse (acts on A).
    double impulse = c.axialMass * (-joint.motorSpeed - speed);
    double previous = c.motor;
    c.motor = std::clamp(previous + impulse, -joint.maxTorque * h, joint.maxTorque * h);
    applyFreeImpulse(c, a, b, -(c.motor - previous), bodies);
}

// One-sided limit row: keeps side * (q - limit) >= 0 with an accumulated impulse >= 0 (no bounce).
// Velocity pass: inside the range the row is speculative (it only prevents crossing the limit within
// this substep); a violated limit is held (no approach). Position pass: a violation is pushed back by a
// pseudo-velocity, which moves the joint but never becomes real velocity.
void solveLimitRow(JointConstraint& c, SolverBody& a, SolverBody& b, double side, double limit,
                   double& accumulated, double h, SolvePass pass, std::vector<SolverBody>& bodies) {
    double gap = side * (c.coordinate - limit);
    double minSpeed = 0;
    if (gap >= 0) {
        minSpeed = -gap / h;
    } else if (pass == SolvePass::Position) {
        minSpeed = std::min(-jointBaumgarte * gap / h, maxJointCorrectionVelocity);
    }
    double impulse = c.axialMass * (minSpeed - side * freeSpeed(c, a, b));
    double previous = accumulated;
    accumulated = std::max(previous + impulse, 0.0);
    applyFreeImpulse(c, a, b, side * (accumulated - previous), bodies);
}

} // namespace

void initJoints(std::vector<Joint>& joints, std::vector<JointConstraint>& constraints, double warmScale) {
    constraints.clear();
    constraints.reserve(joints.size());
    for (auto& joint : joints) {
        JointConstraint c;
        c.joint = &joint;
        c.a = joint.a;
        c.b = joint.b;
        c.angle0 = joint.angle;
        c.wrapped0 = joint.lastWrappedAngle;
        // Bilateral rows are not warm started: DirectJointSolver solves them exactly in every pass, and
        // carrying their impulses over would preserve self-equilibrated internal loads (e.g. a rigid
        // four-wheel chassis twisting its load onto two diagonal wheels) that the exact projection
        // cannot see.
        c.motor = hasSpeedMotor(joint) ? joint.motorImpulse * warmScale : 0;
        c.lower = hasLimits(joint) ? joint.lowerLimitImpulse * warmScale : 0;
        c.upper = hasLimits(joint) ? joint.upperLimitImpulse * warmScale : 0;
        constraints.push_back(c);
    }
}

void prepareJoints(std::vector<JointConstraint>& constraints, const std::vector<SolverBody>& bodies) {
    for (auto& c : constraints) {
        const Joint& joint = *c.joint;
        const SolverBody& a = bodies[static_cast<std::size_t>(c.a)];
        const SolverBody& b = bodies[static_cast<std::size_t>(c.b)];
        Mat3 angularK = a.invInertia + b.invInertia;
        Vec3 pointA = a.point(joint.anchorA);
        switch (joint.type) {
        case JointType::Fixed:
        case JointType::Ball:
        case JointType::Hinge: {
            Vec3 pointB = b.point(joint.anchorB);
            c.rA = pointA - a.position;
            c.rB = pointB - b.position;
            c.pointError = pointA - pointB;
            if (joint.type == JointType::Fixed) {
                c.angularError =
                    rotationVector((a.orientation * joint.reference) * b.orientation.conjugate());
            } else if (joint.type == JointType::Hinge) {
                c.axis = a.orientation.rotate(joint.axisA);
                perpendicularBasis(c.axis, c.t, c.u);
                c.angularError = b.orientation.rotate(joint.axisB).cross(c.axis);
                c.axialMass = inverseOrZero(c.axis.dot(angularK * c.axis));
                c.coordinate = hingeCoordinate(c, a, b);
            }
            break;
        }
        case JointType::Prismatic: {
            // Rows, motor and limits act at B's anchor on both bodies: the Jacobian is then the exact
            // derivative of C = (pB - pA)·t, including the axis turning with A.
            Vec3 point = b.point(joint.anchorB);
            c.axis = a.orientation.rotate(joint.axisA);
            perpendicularBasis(c.axis, c.t, c.u);
            c.rA = point - a.position;
            c.rB = point - b.position;
            Mat3 linearK = pointMassMatrix(a, b, c.rA, c.rB);
            c.pointError = pointA - point;
            c.pointError -= c.axis * c.pointError.dot(c.axis);
            c.angularError = rotationVector((a.orientation * joint.reference) * b.orientation.conjugate());
            c.axialMass = inverseOrZero(c.axis.dot(linearK * c.axis));
            c.coordinate = (point - pointA).dot(c.axis);
            break;
        }
        }
    }
}

void warmStartJoints(std::vector<JointConstraint>& constraints, std::vector<SolverBody>& bodies) {
    for (auto& c : constraints) {
        // Bilateral impulses restart from zero every substep (see initJoints); motors and limits are
        // warm started from the previous substep / step.
        c.linear = {};
        c.angular = {};
        if (c.joint->isActuated()) {
            applyFreeImpulse(c, bodies[static_cast<std::size_t>(c.a)], bodies[static_cast<std::size_t>(c.b)],
                             c.lower - c.upper - c.motor, bodies);
        }
    }
}

void solveJoints(std::vector<JointConstraint>& constraints, std::vector<SolverBody>& bodies, double h,
                 SolvePass pass) {
    const bool position = pass == SolvePass::Position;
    for (auto& c : constraints) {
        SolverBody& a = bodies[static_cast<std::size_t>(c.a)];
        SolverBody& b = bodies[static_cast<std::size_t>(c.b)];
        const Joint& joint = *c.joint;
        if (!position && hasSpeedMotor(joint)) {
            solveMotor(c, a, b, h, bodies);
        }
        if (hasLimits(joint)) {
            solveLimitRow(c, a, b, 1, joint.lowerLimit, position ? c.pseudoLower : c.lower, h, pass, bodies);
            solveLimitRow(c, a, b, -1, joint.upperLimit, position ? c.pseudoUpper : c.upper, h, pass, bodies);
        }
    }
}

void resetPseudoImpulses(std::vector<JointConstraint>& constraints) {
    for (auto& c : constraints) {
        c.pseudoLinear = {};
        c.pseudoAngular = {};
        c.pseudoLower = 0;
        c.pseudoUpper = 0;
    }
}

void accumulateJointTotals(std::vector<JointConstraint>& constraints) {
    for (auto& c : constraints) {
        c.totalLinear += c.linear;
        c.totalAngular += c.angular;
        c.totalMotor += c.motor;
        c.totalLower += c.lower;
        c.totalUpper += c.upper;
    }
}

void storeJointImpulses(std::vector<JointConstraint>& constraints) {
    for (auto& c : constraints) {
        Joint& joint = *c.joint;
        joint.reactionImpulse = c.totalLinear;
        joint.reactionAngularImpulse = c.totalAngular;
        joint.motorImpulse = c.totalMotor;
        joint.lowerLimitImpulse = c.totalLower;
        joint.upperLimitImpulse = c.totalUpper;
    }
}

} // namespace robotics::detail

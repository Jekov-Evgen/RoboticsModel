#pragma once
// Solver-side copy of a body for one World::step() (internal).
//
// The step copies poses and velocities into SolverBody, integrates them in substeps and writes them
// back. Mass properties are cached per substep (world inverse inertia tensor I⁻¹ = R·diag(1/I)·Rᵀ) so
// constraint rows never re-rotate vectors. The ground plane and static bodies have zero inverse mass;
// one extra SolverBody at the end of the array stands for the ground.

#include "robotics/body.hpp"

#include <algorithm>
#include <cmath>

namespace robotics::detail {

// Velocity: real velocities, no position bias, warm-started accumulators, motors and friction.
// Position: split-impulse pseudo-velocities (zero at the start of each substep) that only correct
// position errors; they move bodies in the integration of that substep and are then discarded.
enum class SolvePass { Velocity, Position };

struct SolverBody {
    Vec3 position{};
    Quat orientation{};
    Vec3 velocity{};
    Vec3 angularVelocity{};
    double invMass{};
    Vec3 inertia{};                // Principal moments (local frame).
    Mat3 invInertia{Mat3::zero()}; // World space.

    [[nodiscard]] bool dynamic() const { return invMass > 0; }

    [[nodiscard]] Vec3 point(Vec3 local) const { return position + orientation.rotate(local); }

    [[nodiscard]] Vec3 pointVelocity(Vec3 offset) const { return velocity + angularVelocity.cross(offset); }

    // Impulse p at lever arm r (from the center of mass).
    void applyImpulse(Vec3 p, Vec3 r) {
        velocity += p * invMass;
        angularVelocity += invInertia * r.cross(p);
    }

    void applyAngularImpulse(Vec3 l) { angularVelocity += invInertia * l; }

    void updateInertia() {
        if (!dynamic()) {
            return;
        }
        Mat3 r = Mat3::rotation(orientation);
        invInertia = r * Mat3::diagonal({1 / inertia.x, 1 / inertia.y, 1 / inertia.z}) * r.transposed();
    }
};

[[nodiscard]] inline SolverBody makeSolverBody(const Body& body) {
    SolverBody s;
    s.position = body.position;
    s.orientation = body.orientation;
    s.velocity = body.velocity;
    s.angularVelocity = body.angularVelocity;
    if (body.mass > 0) {
        s.invMass = 1 / body.mass;
        s.inertia = body.principalInertia();
        s.updateInertia();
    }
    return s;
}

// Inverse effective mass of a point constraint along unit n with lever arms rA, rB.
[[nodiscard]] inline double pointMass(const SolverBody& a, const SolverBody& b, Vec3 ra, Vec3 rb, Vec3 n) {
    Vec3 ca = ra.cross(n);
    Vec3 cb = rb.cross(n);
    return a.invMass + b.invMass + ca.dot(a.invInertia * ca) + cb.dot(b.invInertia * cb);
}

// 3x3 inverse effective mass matrix of a point-to-point constraint:
// K = (mA⁻¹ + mB⁻¹)·1 − [rA]× IA⁻¹ [rA]× − [rB]× IB⁻¹ [rB]×.
[[nodiscard]] inline Mat3 pointMassMatrix(const SolverBody& a, const SolverBody& b, Vec3 ra, Vec3 rb) {
    Mat3 sa = Mat3::skew(ra);
    Mat3 sb = Mat3::skew(rb);
    return Mat3::diagonal(Vec3{1, 1, 1} * (a.invMass + b.invMass)) - sa * a.invInertia * sa
           - sb * b.invInertia * sb;
}

} // namespace robotics::detail

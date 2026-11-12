#include "integrator.hpp"

#include <cmath>

namespace robotics::detail {
namespace {

// Newton iterations of the implicit midpoint gyroscopic step (converges quadratically; h·|ω| is small).
constexpr int gyroscopicIterations = 3;

} // namespace

void integrateVelocity(SolverBody& body, Vec3 gravity, Vec3 force, Vec3 torque, double h) {
    if (!body.dynamic()) {
        return;
    }
    body.velocity += (gravity + force * body.invMass) * h;

    const Vec3 inertia = body.inertia;
    if (inertia.x == inertia.y && inertia.y == inertia.z) {
        // Isotropic body (sphere): ω × (Iω) = 0.
        body.angularVelocity += body.invInertia * torque * h;
        return;
    }
    Quat toBody = body.orientation.conjugate();
    Vec3 w0 = toBody.rotate(body.angularVelocity);
    Vec3 impulse = toBody.rotate(torque) * h;
    Mat3 inertiaMatrix = Mat3::diagonal(inertia);
    // Solve F(w) = I (w - w0) - h τ - h (I w̄) × w̄ = 0 with w̄ = (w0 + w) / 2.
    Vec3 w = w0;
    for (int iteration = 0; iteration < gyroscopicIterations; ++iteration) {
        Vec3 mid = (w0 + w) * 0.5;
        Vec3 momentum = componentProduct(inertia, mid);
        Vec3 residual = componentProduct(inertia, w - w0) - impulse - momentum.cross(mid) * h;
        // dF/dw = I - (h/2) (skew(I w̄) - skew(w̄) I).
        Mat3 jacobian = inertiaMatrix - (Mat3::skew(momentum) - Mat3::skew(mid) * inertiaMatrix) * (0.5 * h);
        w -= jacobian.inverse() * residual;
    }
    body.angularVelocity = body.orientation.rotate(w);
}

void integratePosition(SolverBody& body, Vec3 velocity, Vec3 angularVelocity, double h) {
    if (!body.dynamic()) {
        return;
    }
    body.position += velocity * h;
    double speed = angularVelocity.length();
    if (speed > 1e-12) {
        body.orientation =
            (Quat::axisAngle(angularVelocity / speed, speed * h) * body.orientation).normalized();
    }
    body.updateInertia();
}

} // namespace robotics::detail

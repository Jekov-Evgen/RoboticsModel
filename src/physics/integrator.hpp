#pragma once
// Rigid-body time integration for one substep (internal).

#include "solver_body.hpp"

namespace robotics::detail {

// Velocity update by external force/torque (world space) and the gyroscopic term: Euler's equation
// I·dω/dt = τ − ω × (I·ω) in the body frame, integrated with the implicit midpoint rule (Newton
// iterations). The midpoint rule conserves the quadratic invariants |L| and kinetic energy of a
// torque-free body exactly, so free tumbling neither gains energy (explicit Euler) nor damps it
// (implicit Euler, as Bullet's implicit gyroscopic solve does); the intermediate-axis instability
// (Dzhanibekov effect) is reproduced.
void integrateVelocity(SolverBody& body, Vec3 gravity, Vec3 force, Vec3 torque, double h);

// Position and orientation update with the given velocities (real + split-impulse pseudo-velocity);
// refreshes the world inverse inertia.
void integratePosition(SolverBody& body, Vec3 velocity, Vec3 angularVelocity, double h);

} // namespace robotics::detail

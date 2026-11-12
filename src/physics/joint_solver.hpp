#pragma once
// Joint constraint solver (internal). Rows are prepared once per substep from the current poses.
// Motors and limits are solved by sequential impulses here; the bilateral rows (anchor, hinge/slider
// locks, orientation) by DirectJointSolver (joint_direct.hpp). Split impulses: the velocity pass has
// no position bias and its accumulated impulses are warm started (from the previous substep and, across
// steps, from Joint's reported impulses); the position pass corrects drift with pseudo-velocities that
// move the bodies during integration and are then discarded, so correction never adds kinetic energy.

#include "robotics/joint.hpp"
#include "solver_body.hpp"

#include <vector>

namespace robotics::detail {

// Velocity change of one body per unit generalized impulse (see JointConstraint::response).
struct BodyDelta {
    int body{};
    Vec3 linear;
    Vec3 angular;
};

struct JointConstraint {
    Joint* joint{};
    int a{};
    int b{};
    // Encoder state at the start of the step (hinge unwrap reference).
    double angle0{};
    double wrapped0{};

    // Per-substep geometry.
    Vec3 rA;
    Vec3 rB;
    Vec3 pointError; // C = pA - pB.
    Vec3 axis;       // Free axis (world), hinge/prismatic.
    Vec3 t;
    Vec3 u;
    Vec3 angularError;
    double axialMass{}; // Effective mass (not inverse) along the free axis, for motor and limits.
    // Constrained response of a unit impulse on the free axis: velocity changes of every body of the
    // articulation after the bilateral joint rows react (computed by DirectJointSolver). With it a
    // motor or limit row reaches its target in one update, as if solved with articulated-body
    // inertia, and never disturbs the other joints. Empty: plain two-body response.
    std::vector<BodyDelta> response;
    double coordinate{};

    // Accumulated impulses of the current substep (world space; warm started).
    Vec3 linear;
    Vec3 angular;
    double motor{};
    double lower{};
    double upper{};

    // Pseudo impulses of the position pass (reset every substep, never reported or warm started).
    Vec3 pseudoLinear;
    Vec3 pseudoAngular;
    double pseudoLower{};
    double pseudoUpper{};

    // Totals over the step (reported in Joint after the step).
    Vec3 totalLinear;
    Vec3 totalAngular;
    double totalMotor{};
    double totalLower{};
    double totalUpper{};
};

// Builds constraints for all joints; `warmScale` (0 disables) converts the previous step's reported
// impulses into per-substep impulses (h / dt_previous).
void initJoints(std::vector<Joint>& joints, std::vector<JointConstraint>& constraints, double warmScale);

// Recomputes lever arms, errors, effective masses and the joint coordinate from the current poses.
void prepareJoints(std::vector<JointConstraint>& constraints, const std::vector<SolverBody>& bodies);

void warmStartJoints(std::vector<JointConstraint>& constraints, std::vector<SolverBody>& bodies);

// One sequential-impulse pass over the bounded joint rows: speed motors (velocity pass only) and
// limits. The bilateral rows (anchors, locks) are solved exactly by DirectJointSolver.
void solveJoints(std::vector<JointConstraint>& constraints, std::vector<SolverBody>& bodies, double h,
                 SolvePass pass);

void resetPseudoImpulses(std::vector<JointConstraint>& constraints);

// Adds the substep's accumulated impulses to the step totals.
void accumulateJointTotals(std::vector<JointConstraint>& constraints);

// Writes the step totals into Joint::motorImpulse, reactionImpulse, ... .
void storeJointImpulses(std::vector<JointConstraint>& constraints);

} // namespace robotics::detail

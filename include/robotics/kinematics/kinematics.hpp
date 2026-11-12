#pragma once
// Serial-chain kinematics on top of a World: forward kinematics, Jacobian and damped-least-squares inverse
// kinematics. Headless, no UI dependencies.
//
//   Chain chain = makeChain(world, tipBody);                     // base -> ... -> tip, found from the joint graph
//   std::vector<double> q = chainCoordinates(world, chain);      // current joint coordinates
//   Pose tip = forwardKinematics(world, chain, q);
//   IkResult ik = solveIk(world, chain, target, std::nullopt, q);
//
// Joint conventions are the engine's: a hinge coordinate is the twist of B about axisA relative to the pose at
// connect(), a prismatic coordinate is the displacement of B's anchor along axisA. Fixed joints are part of the
// chain with no coordinate. Ball joints are not supported. The geometry comes from the World at the moment of the
// call, so kinematics follow edits of the robot.

#include "robotics/math.hpp"
#include "robotics/world.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace robotics {

struct Pose {
    Vec3 position{};
    Quat orientation{};
};

struct Chain {
    int root{};                // Body at the base of the chain (static, or simply without a parent joint).
    int tip{};                 // Body that carries the controlled point.
    Vec3 tipPoint{};           // The controlled point in the tip body's frame.
    std::vector<int> joints;   // World::joints indices from the root to the tip, fixed joints included.
    std::vector<int> actuated; // The subset of `joints` that has a coordinate (hinge, prismatic), same order.

    [[nodiscard]] std::size_t dofs() const { return actuated.size(); }
};

// Follows parent joints (the joint whose child is the body) from `tipBody` up to the root. Throws
// std::runtime_error for a ball joint, a closed loop or an invalid index.
[[nodiscard]] Chain makeChain(const World& world, int tipBody, Vec3 tipPoint = {});

// Current coordinates (Joint::angle) of the chain's actuated joints.
[[nodiscard]] std::vector<double> chainCoordinates(const World& world, const Chain& chain);

// Pose of the controlled point for coordinates `q` (one per actuated joint), starting from the root's current pose.
[[nodiscard]] Pose forwardKinematics(const World& world, const Chain& chain, const std::vector<double>& q);

// Geometric Jacobian at `q`: one column per actuated joint, each {vx, vy, vz, wx, wy, wz} in world axes
// (linear velocity of the controlled point, angular velocity of the tip) per unit joint velocity.
[[nodiscard]] std::vector<std::array<double, 6>> jacobian(const World& world, const Chain& chain,
                                                          const std::vector<double>& q);

struct IkOptions {
    int maxIterations{200};
    double positionTolerance{1e-4};    // metres
    double orientationTolerance{1e-3}; // radians (only when a target orientation is given)
    double damping{0.05};              // Levenberg-Marquardt lambda: trades accuracy near singularities for stability
    double maxStep{0.3};               // largest change of any coordinate per iteration (rad or m)
    bool respectLimits{true};          // clamp to Joint limits
};

struct IkResult {
    std::vector<double> q;
    bool converged{};
    double positionError{};    // metres
    double orientationError{}; // radians; 0 when no orientation target
    int iterations{};
};

// Iterates from `seed` towards the target position (and orientation, if given). Always returns the best
// coordinates found; check `converged`.
[[nodiscard]] IkResult solveIk(const World& world, const Chain& chain, Vec3 targetPosition,
                               const std::optional<Quat>& targetOrientation, const std::vector<double>& seed,
                               const IkOptions& options = {});

} // namespace robotics

#pragma once
// Continuous collision detection (internal): conservative advancement + speculative contacts.
//
// A body is `active` in a step when it has Body::ccd or moves more than a fraction of its smallest
// extent; everything else keeps the discrete pipeline bit for bit. Active bodies get bounds swept over
// the step. For a swept pair (or the ground) that does not touch yet, conservative advancement finds the
// time of impact along the predicted motion: GJK distance, then a step of distance / (largest closing
// speed of any two surface points), until the gap closes or the step ends. No impact, no contact: a body
// flying past the edge of a plate is not stopped by that plate's infinite plane (the "ghost collision" of
// plain speculative contacts). Impacts become speculative contacts built by the discrete narrow phase at
// the impact pose, so a flat landing gets a full manifold; the solver's ImpactSchedule stops the bodies
// there.

#include "broadphase.hpp"
#include "robotics/body.hpp"

namespace robotics::detail {

// Gap (m) at which advancement stops and the impact counts as reached.
inline constexpr double ccdTargetGap = 0.0002;

struct BodyMotion {
    Vec3 velocity;        // Linear velocity over the step, including gravity and applied force.
    Vec3 angularVelocity; // Without spin about a symmetry axis (invisible for spheres and cylinders).
    bool active{};
};

[[nodiscard]] BodyMotion predictMotion(const Body& body, double dt, Vec3 gravity);

// worldBounds() of the start and end of the motion, and everything a rotation can reach in between.
[[nodiscard]] Aabb sweptBounds(const Body& body, const BodyMotion& motion, double dt);

void advancePose(const Body& body, const BodyMotion& motion, double time, Vec3& position, Quat& orientation);

struct Impact {
    double time{};         // Seconds from the start of the step.
    double closingSpeed{}; // Bound used for the last advancement (m/s).
    Vec3 normal;           // From B towards A.
    Vec3 pointA;           // Closest surface points at the impact (world).
    Vec3 pointB;
};

// True if A and B come within ccdTargetGap during [0, dt].
[[nodiscard]] bool pairTimeOfImpact(const Body& a, const BodyMotion& motionA, const Body& b,
                                    const BodyMotion& motionB, double dt, Impact& impact);

// True if the body reaches the ground plane during [0, dt].
[[nodiscard]] bool groundTimeOfImpact(const Body& body, const BodyMotion& motion, double dt, Impact& impact);

} // namespace robotics::detail

#pragma once
// Exact ray casts against the collision shapes (box, sphere, cylinder, convex hull) and the ground plane y = 0.
// Used by range sensors; also handy for picking and tools. Headless, no UI dependencies.

#include "robotics/body.hpp"
#include "robotics/contact.hpp"
#include "robotics/math.hpp"

namespace robotics {

class World;

// Body index meaning "ignore nothing" for raycast(..., ignoreBody).
inline constexpr int noBodyIndex = -2;

struct RayHit {
    bool hit{};
    int body{groundBodyIndex}; // Body index, or groundBodyIndex for the ground plane.
    double distance{};         // Along the (unit) direction, in metres.
    Vec3 point{};              // World space.
    Vec3 normal{};             // Unit surface normal facing the ray origin (world space).
};

// Rays use a unit `direction` and report hits with 0 <= distance <= maxDistance. A ray that starts inside a
// shape (or below the ground) hits it at distance 0 with the normal opposite to the ray: the sensor is
// occluded, as in most physics engines' "initial overlap" convention.
[[nodiscard]] RayHit raycastBody(const Body& body, Vec3 origin, Vec3 direction, double maxDistance);
[[nodiscard]] RayHit raycastGround(Vec3 origin, Vec3 direction, double maxDistance);

// Nearest hit among all bodies except `ignoreBody` and, when `includeGround`, the ground plane.
[[nodiscard]] RayHit raycast(const World& world, Vec3 origin, Vec3 direction, double maxDistance,
                             int ignoreBody = noBodyIndex, bool includeGround = true);

} // namespace robotics

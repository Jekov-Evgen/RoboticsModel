#pragma once
// Broad phase (internal): candidate body pairs whose world AABBs overlap.
//
// Sweep and prune along the axis of largest AABB-center spread. Pairs of two static bodies and pairs
// connected by a joint are skipped. The result is sorted by (first, second) with first < second, so
// the narrow phase and the solver see the same order as the former O(n²) double loop: results stay
// bit-identical run to run regardless of the sweep axis.

#include "robotics/body.hpp"
#include "robotics/joint.hpp"

#include <utility>
#include <vector>

namespace robotics::detail {

struct Aabb {
    Vec3 min;
    Vec3 max;

    [[nodiscard]] bool overlaps(const Aabb& o) const {
        return min.x <= o.max.x && o.min.x <= max.x && min.y <= o.max.y && o.min.y <= max.y
               && min.z <= o.max.z && o.min.z <= max.z;
    }
};

using BodyPair = std::pair<int, int>;

// Conservative world-space bounds of the body's collision shape (plus a tiny margin).
[[nodiscard]] Aabb worldBounds(const Body& body);

// Sorted pairs (i < k) of bodies with overlapping bounds that may collide. `bounds` replaces the
// worldBounds() of every body, e.g. with swept bounds for continuous collision.
[[nodiscard]] std::vector<BodyPair> broadphasePairs(const std::vector<Body>& bodies,
                                                    const std::vector<Joint>& joints,
                                                    const std::vector<Aabb>* bounds = nullptr);

// Reference O(n²) implementation with the same filtering rules (used by tests).
[[nodiscard]] std::vector<BodyPair> bruteForcePairs(const std::vector<Body>& bodies,
                                                    const std::vector<Joint>& joints);

} // namespace robotics::detail

#include "broadphase.hpp"

#include <algorithm>
#include <cmath>

namespace robotics::detail {
namespace {

// Bounds margin (m): absorbs rounding so touching shapes always reach the narrow phase.
constexpr double boundsMargin = 1e-6;

// Sorted list of jointed pairs (i < k) for O(log n) lookup.
std::vector<BodyPair> jointedPairs(const std::vector<Joint>& joints) {
    std::vector<BodyPair> result;
    result.reserve(joints.size());
    for (const auto& joint : joints) {
        result.emplace_back(std::min(joint.a, joint.b), std::max(joint.a, joint.b));
    }
    std::sort(result.begin(), result.end());
    return result;
}

bool mayCollide(const std::vector<Body>& bodies, const std::vector<BodyPair>& jointed, int i, int k) {
    if (bodies[static_cast<std::size_t>(i)].isStatic() && bodies[static_cast<std::size_t>(k)].isStatic()) {
        return false;
    }
    // Adjacent robot links commonly overlap at their joint.
    return !std::binary_search(jointed.begin(), jointed.end(), BodyPair{std::min(i, k), std::max(i, k)});
}

} // namespace

Aabb worldBounds(const Body& body) {
    if (body.shape == Shape::ConvexHull && body.hull) {
        // Exact: a hull is rarely symmetric about its centre of mass.
        Mat3 r = Mat3::rotation(body.orientation);
        Vec3 low{1e300, 1e300, 1e300};
        Vec3 high = -low;
        for (Vec3 v : body.hull->vertices) {
            Vec3 p = r * v;
            for (int i = 0; i < 3; ++i) {
                low[i] = std::min(low[i], p[i]);
                high[i] = std::max(high[i], p[i]);
            }
        }
        Vec3 margin = Vec3{1, 1, 1} * boundsMargin;
        return {body.position + low - margin, body.position + high + margin};
    }
    Vec3 half;
    switch (body.shape) {
    case Shape::Sphere:
        half = Vec3{1, 1, 1} * (body.size.x * 0.5);
        break;
    case Shape::Cylinder: {
        // Exact bounds of a disc sweep: r·sqrt(1 - a_i²) + (w/2)·|a_i| per world axis i.
        Vec3 axis = body.orientation.rotate({0, 0, 1});
        double r = body.size.x * 0.5;
        double w = body.size.z * 0.5;
        for (int i = 0; i < 3; ++i) {
            half[i] = r * std::sqrt(std::max(0.0, 1 - axis[i] * axis[i])) + w * std::abs(axis[i]);
        }
        break;
    }
    case Shape::Box:
    default: {
        Mat3 r = Mat3::rotation(body.orientation);
        Vec3 e = body.size * 0.5;
        for (int i = 0; i < 3; ++i) {
            Vec3 row = i == 0 ? r.r0 : (i == 1 ? r.r1 : r.r2);
            half[i] = std::abs(row.x) * e.x + std::abs(row.y) * e.y + std::abs(row.z) * e.z;
        }
        break;
    }
    }
    half += Vec3{1, 1, 1} * boundsMargin;
    return {body.position - half, body.position + half};
}

std::vector<BodyPair> broadphasePairs(const std::vector<Body>& bodies, const std::vector<Joint>& joints,
                                      const std::vector<Aabb>* sweptBounds) {
    std::vector<BodyPair> pairs;
    const int count = static_cast<int>(bodies.size());
    if (count < 2) {
        return pairs;
    }
    std::vector<Aabb> bounds(bodies.size());
    Vec3 mean;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        bounds[i] = sweptBounds ? (*sweptBounds)[i] : worldBounds(bodies[i]);
        mean += bodies[i].position;
    }
    mean = mean / count;
    // Sweep axis: largest variance of the centers (fewest overlaps along it).
    Vec3 variance;
    for (const auto& body : bodies) {
        Vec3 d = body.position - mean;
        variance += componentProduct(d, d);
    }
    int axis =
        variance.x >= variance.y ? (variance.x >= variance.z ? 0 : 2) : (variance.y >= variance.z ? 1 : 2);

    std::vector<int> order(bodies.size());
    for (int i = 0; i < count; ++i) {
        order[static_cast<std::size_t>(i)] = i;
    }
    std::sort(order.begin(), order.end(), [&](int l, int r) {
        double lv = bounds[static_cast<std::size_t>(l)].min[axis];
        double rv = bounds[static_cast<std::size_t>(r)].min[axis];
        return lv < rv || (lv == rv && l < r);
    });

    auto jointed = jointedPairs(joints);
    std::vector<int> active;
    for (int index : order) {
        const Aabb& box = bounds[static_cast<std::size_t>(index)];
        // Drop intervals that end before this one starts (they cannot overlap anything later).
        std::erase_if(active, [&](int other) {
            return bounds[static_cast<std::size_t>(other)].max[axis] < box.min[axis];
        });
        for (int other : active) {
            if (box.overlaps(bounds[static_cast<std::size_t>(other)])
                && mayCollide(bodies, jointed, index, other)) {
                pairs.emplace_back(std::min(index, other), std::max(index, other));
            }
        }
        active.push_back(index);
    }
    std::sort(pairs.begin(), pairs.end());
    return pairs;
}

std::vector<BodyPair> bruteForcePairs(const std::vector<Body>& bodies, const std::vector<Joint>& joints) {
    std::vector<BodyPair> pairs;
    auto jointed = jointedPairs(joints);
    const int count = static_cast<int>(bodies.size());
    for (int i = 0; i < count; ++i) {
        for (int k = i + 1; k < count; ++k) {
            if (worldBounds(bodies[static_cast<std::size_t>(i)])
                    .overlaps(worldBounds(bodies[static_cast<std::size_t>(k)]))
                && mayCollide(bodies, jointed, i, k)) {
                pairs.emplace_back(i, k);
            }
        }
    }
    return pairs;
}

} // namespace robotics::detail

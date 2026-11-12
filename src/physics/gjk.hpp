#pragma once
// GJK distance and EPA penetration depth between convex point sets (internal).
//
// A shape is the convex hull of a small point set with a rigid transform, which covers boxes (8 points),
// cylinder prisms and hulls; a sphere is one point, and callers add its radius to the result. The support
// mapping is a linear scan: at most 64 points, and no adjacency to keep consistent.
//
// GJK walks the Minkowski difference A - B towards the origin, reducing the simplex with Voronoi-region
// tests (equivalent to Johnson's sub-algorithm, but it needs no determinant table and handles flat
// simplices by falling back to their faces and edges). EPA starts from GJK's simplex, blown up to a
// tetrahedron if needed, and expands the polytope face by face towards the surface closest to the origin.

#include "robotics/math.hpp"

#include <array>

namespace robotics::detail {

struct ConvexPoints {
    const Vec3* points{};
    int count{};
    Vec3 position{};
    Mat3 rotation{};

    // World-space point of the set farthest along the world-space direction.
    [[nodiscard]] Vec3 support(Vec3 direction) const;
};

struct SimplexVertex {
    Vec3 w; // a - b.
    Vec3 a;
    Vec3 b;
};

struct GjkResult {
    bool overlapping{}; // The origin is inside (or within rounding of) the Minkowski difference.
    double distance{};  // 0 when overlapping.
    Vec3 pointA;        // Closest points; meaningful when not overlapping.
    Vec3 pointB;
    int iterations{};
    int simplexCount{};
    std::array<SimplexVertex, 4> simplex{};
};

[[nodiscard]] GjkResult gjkDistance(const ConvexPoints& a, const ConvexPoints& b);

struct EpaResult {
    bool valid{};   // False only if the shapes do not overlap or the polytope degenerates.
    double depth{}; // Smallest translation that separates the shapes, >= 0.
    Vec3 normal;    // Unit: moving B by normal * depth (or A by the opposite) separates them.
    Vec3 pointA;    // Deepest point on A's surface and its partner on B's: pointA - pointB = normal * depth.
    Vec3 pointB;
};

// `gjk` must be the result of gjkDistance(a, b) with overlapping == true.
[[nodiscard]] EpaResult epaPenetration(const ConvexPoints& a, const ConvexPoints& b, const GjkResult& gjk);

} // namespace robotics::detail

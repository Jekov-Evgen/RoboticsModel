#include "robotics/sensors/raycast.hpp"

#include "robotics/world.hpp"

#include <cmath>
#include <limits>

namespace robotics {
namespace {

constexpr double parallelEpsilon = 1e-15;

// Ray in the body's local frame: distance along the local direction (unit, rotation preserves length)
// equals the world distance.
struct LocalHit {
    bool hit{};
    double distance{};
    Vec3 normal{}; // Local frame.
};

LocalHit inside(Vec3 direction) {
    return {true, 0, -direction};
}

LocalHit rayBox(Vec3 origin, Vec3 direction, Vec3 half) {
    double nearest = -std::numeric_limits<double>::infinity();
    double farthest = std::numeric_limits<double>::infinity();
    Vec3 normal{};
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(direction[axis]) < parallelEpsilon) {
            if (std::abs(origin[axis]) > half[axis]) {
                return {};
            }
            continue;
        }
        double t1 = (-half[axis] - origin[axis]) / direction[axis];
        double t2 = (half[axis] - origin[axis]) / direction[axis];
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        if (t1 > nearest) {
            nearest = t1;
            normal = {};
            normal[axis] = direction[axis] > 0 ? -1 : 1;
        }
        farthest = std::min(farthest, t2);
        if (nearest > farthest) {
            return {};
        }
    }
    if (farthest < 0) {
        return {};
    }
    if (nearest < 0) {
        return inside(direction);
    }
    return {true, nearest, normal};
}

LocalHit raySphere(Vec3 origin, Vec3 direction, double radius) {
    double b = origin.dot(direction);
    double c = origin.length2() - radius * radius;
    if (c <= 0) {
        return inside(direction);
    }
    if (b > 0) {
        return {};
    }
    double discriminant = b * b - c;
    if (discriminant < 0) {
        return {};
    }
    double t = -b - std::sqrt(discriminant);
    return {true, t, (origin + direction * t) / radius};
}

// Cylinder around the local Z axis: radius, half length along Z.
LocalHit rayCylinder(Vec3 origin, Vec3 direction, double radius, double halfLength) {
    double radial = origin.x * origin.x + origin.y * origin.y - radius * radius;
    if (radial <= 0 && std::abs(origin.z) <= halfLength) {
        return inside(direction);
    }
    LocalHit best;
    best.distance = std::numeric_limits<double>::infinity();
    // Curved side: entry root of |(o + t d)_xy|^2 = r^2.
    double a = direction.x * direction.x + direction.y * direction.y;
    if (a > parallelEpsilon) {
        double b = origin.x * direction.x + origin.y * direction.y;
        double discriminant = b * b - a * radial;
        if (discriminant >= 0) {
            double t = (-b - std::sqrt(discriminant)) / a;
            Vec3 p = origin + direction * t;
            if (t >= 0 && std::abs(p.z) <= halfLength) {
                best = {true, t, Vec3{p.x, p.y, 0} / radius};
            }
        }
    }
    // Flat caps.
    if (std::abs(direction.z) > parallelEpsilon) {
        for (double side : {-1.0, 1.0}) {
            double t = (side * halfLength - origin.z) / direction.z;
            Vec3 p = origin + direction * t;
            if (t >= 0 && t < best.distance && p.x * p.x + p.y * p.y <= radius * radius
                && direction.z * side < 0) {
                best = {true, t, Vec3{0, 0, side}};
            }
        }
    }
    return best;
}

} // namespace

// Per-shape dispatch: the switch is exhaustive (no default) so -Wswitch flags new shapes.
RayHit raycastBody(const Body& body, Vec3 origin, Vec3 direction, double maxDistance) {
    Quat inverse = body.orientation.conjugate();
    Vec3 localOrigin = inverse.rotate(origin - body.position);
    Vec3 localDirection = inverse.rotate(direction);
    LocalHit local;
    switch (body.shape) {
    case Shape::Box:
        local = rayBox(localOrigin, localDirection, body.size * 0.5);
        break;
    case Shape::Sphere:
        local = raySphere(localOrigin, localDirection, body.size.x * 0.5);
        break;
    case Shape::Cylinder:
        local = rayCylinder(localOrigin, localDirection, body.size.x * 0.5, body.size.z * 0.5);
        break;
    case Shape::ConvexHull:
        if (body.hull) {
            local.hit = raycastConvexHull(*body.hull, localOrigin, localDirection, maxDistance, local.distance,
                                          local.normal);
        }
        break;
    }
    RayHit hit;
    if (!local.hit || local.distance > maxDistance) {
        return hit;
    }
    hit.hit = true;
    hit.distance = local.distance;
    hit.point = origin + direction * local.distance;
    hit.normal = body.orientation.rotate(local.normal);
    return hit;
}

RayHit raycastGround(Vec3 origin, Vec3 direction, double maxDistance) {
    RayHit hit;
    hit.body = groundBodyIndex;
    if (origin.y < 0) {
        hit.hit = maxDistance >= 0;
        hit.point = origin;
        hit.normal = -direction;
        return hit;
    }
    if (direction.y > -parallelEpsilon) {
        return hit;
    }
    double t = -origin.y / direction.y;
    if (t > maxDistance) {
        return hit;
    }
    hit.hit = true;
    hit.distance = t;
    hit.point = origin + direction * t;
    hit.point.y = 0;
    hit.normal = {0, 1, 0};
    return hit;
}

RayHit raycast(const World& world, Vec3 origin, Vec3 direction, double maxDistance, int ignoreBody,
               bool includeGround) {
    RayHit best;
    double limit = maxDistance;
    if (includeGround) {
        RayHit ground = raycastGround(origin, direction, limit);
        if (ground.hit) {
            best = ground;
            limit = ground.distance;
        }
    }
    for (int i = 0; i < static_cast<int>(world.bodies.size()); ++i) {
        if (i == ignoreBody) {
            continue;
        }
        const Body& body = world.bodies[static_cast<std::size_t>(i)];
        // Cheap bounding-sphere rejection before the exact test.
        double radius = body.size.length() * 0.5;
        Vec3 toCenter = body.position - origin;
        double along = toCenter.dot(direction);
        double distance2 = toCenter.length2() - along * along;
        if (distance2 > radius * radius || along < -radius || along - radius > limit) {
            continue;
        }
        RayHit hit = raycastBody(body, origin, direction, limit);
        if (hit.hit && (!best.hit || hit.distance < best.distance)) {
            hit.body = i;
            best = hit;
            limit = hit.distance;
        }
    }
    return best;
}

} // namespace robotics

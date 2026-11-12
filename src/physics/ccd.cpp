#include "ccd.hpp"

#include "collision.hpp"
#include "gjk.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace robotics::detail {
namespace {

// A body moving farther than this fraction of its smallest extent per step gets continuous collision.
// At a quarter, an unswept body still overlaps anything it reaches long before its centre gets past it.
constexpr double autoMotionFraction = 0.25;

constexpr int maxAdvancements = 48;
// Closing speed (m/s) below which two bodies will not meet within any step.
constexpr double minClosingSpeed = 1e-9;

// The shape as GJK sees it: a point set (for a sphere, its centre) plus a radius around it.
class LocalShape {
public:
    explicit LocalShape(const Body& body) {
        switch (body.shape) {
        case Shape::Sphere:
            storage_[0] = {};
            points_ = storage_.data();
            count_ = 1;
            margin_ = body.size.x * 0.5;
            break;
        case Shape::Cylinder: {
            constexpr int n = cylinderPrismSides;
            double vertexRadius = body.size.x * 0.5 / std::cos(pi / n);
            double half = body.size.z * 0.5;
            for (int k = 0; k < n; ++k) {
                double angle = 2 * pi * k / n;
                Vec3 radial{vertexRadius * std::cos(angle), vertexRadius * std::sin(angle), 0};
                storage_[static_cast<std::size_t>(k)] = radial + Vec3{0, 0, -half};
                storage_[static_cast<std::size_t>(n + k)] = radial + Vec3{0, 0, half};
            }
            points_ = storage_.data();
            count_ = 2 * n;
            break;
        }
        case Shape::ConvexHull:
            points_ = body.hull->vertices.data();
            count_ = static_cast<int>(body.hull->vertices.size());
            break;
        case Shape::Box:
            for (int i = 0; i < 8; ++i) {
                storage_[static_cast<std::size_t>(i)] = {(i & 1 ? 0.5 : -0.5) * body.size.x,
                                                         (i & 2 ? 0.5 : -0.5) * body.size.y,
                                                         (i & 4 ? 0.5 : -0.5) * body.size.z};
            }
            points_ = storage_.data();
            count_ = 8;
            break;
        }
        for (int i = 0; i < count_; ++i) {
            reach_ = std::max(reach_, points_[i].length());
        }
    }

    LocalShape(const LocalShape&) = delete;
    LocalShape& operator=(const LocalShape&) = delete;

    [[nodiscard]] ConvexPoints at(Vec3 position, Quat orientation) const {
        return {points_, count_, position, Mat3::rotation(orientation)};
    }

    [[nodiscard]] double margin() const { return margin_; }

    // Largest distance of a surface point from the centre, as far as rotation can move it.
    [[nodiscard]] double reach() const { return reach_; }

private:
    std::array<Vec3, 2 * cylinderPrismSides> storage_;
    const Vec3* points_{};
    int count_{};
    double margin_{};
    double reach_{};
};

double smallestExtent(const Body& body) {
    switch (body.shape) {
    case Shape::Sphere:
        return body.size.x;
    case Shape::Cylinder:
        return std::min(body.size.x, body.size.z);
    case Shape::ConvexHull:
        return body.hull->minWidth;
    case Shape::Box:
        break;
    }
    return std::min({body.size.x, body.size.y, body.size.z});
}

double boundingRadius(const Body& body) {
    switch (body.shape) {
    case Shape::Sphere:
        return body.size.x * 0.5;
    case Shape::Cylinder:
        return std::hypot(body.size.x, body.size.z) * 0.5;
    case Shape::ConvexHull:
        return body.hull->radius;
    case Shape::Box:
        break;
    }
    return body.size.length() * 0.5;
}

} // namespace

BodyMotion predictMotion(const Body& body, double dt, Vec3 gravity) {
    BodyMotion motion;
    if (body.mass <= 0) {
        return motion; // Static bodies do not move, whatever their velocity fields say.
    }
    motion.velocity = body.velocity + (gravity + body.force * body.inverseMass()) * dt;
    if (body.shape == Shape::Cylinder) {
        Vec3 axis = body.orientation.rotate({0, 0, 1});
        motion.angularVelocity = body.angularVelocity - axis * body.angularVelocity.dot(axis);
    } else if (body.shape != Shape::Sphere) {
        motion.angularVelocity = body.angularVelocity;
    }
    double travel = (motion.velocity.length() + motion.angularVelocity.length() * boundingRadius(body)) * dt;
    motion.active = body.ccd || travel > autoMotionFraction * smallestExtent(body);
    return motion;
}

Aabb sweptBounds(const Body& body, const BodyMotion& motion, double dt) {
    Aabb box = worldBounds(body);
    if (motion.angularVelocity.length2() > 0) {
        Vec3 reach = Vec3{1, 1, 1} * boundingRadius(body);
        for (int i = 0; i < 3; ++i) {
            box.min[i] = std::min(box.min[i], body.position[i] - reach[i]);
            box.max[i] = std::max(box.max[i], body.position[i] + reach[i]);
        }
    }
    Vec3 move = motion.velocity * dt;
    for (int i = 0; i < 3; ++i) {
        box.min[i] += std::min(move[i], 0.0);
        box.max[i] += std::max(move[i], 0.0);
    }
    return box;
}

void advancePose(const Body& body, const BodyMotion& motion, double time, Vec3& position, Quat& orientation) {
    position = body.position + motion.velocity * time;
    orientation = body.orientation;
    double speed = motion.angularVelocity.length();
    if (speed > 1e-12) {
        orientation =
            (Quat::axisAngle(motion.angularVelocity / speed, speed * time) * orientation).normalized();
    }
}

bool pairTimeOfImpact(const Body& a, const BodyMotion& motionA, const Body& b, const BodyMotion& motionB,
                      double dt, Impact& impact) {
    LocalShape shapeA(a);
    LocalShape shapeB(b);
    const double spin =
        motionA.angularVelocity.length() * shapeA.reach() + motionB.angularVelocity.length() * shapeB.reach();
    const Vec3 relative = motionA.velocity - motionB.velocity;
    double time = 0;
    bool known = false;
    for (int i = 0; i < maxAdvancements; ++i) {
        Vec3 positionA;
        Vec3 positionB;
        Quat orientationA;
        Quat orientationB;
        advancePose(a, motionA, time, positionA, orientationA);
        advancePose(b, motionB, time, positionB, orientationB);
        GjkResult gjk = gjkDistance(shapeA.at(positionA, orientationA), shapeB.at(positionB, orientationB));
        if (gjk.overlapping || gjk.distance <= 0) {
            break; // Touching cores: the last closest points are the impact.
        }
        double gap = gjk.distance - shapeA.margin() - shapeB.margin();
        Vec3 normal = (gjk.pointA - gjk.pointB) / gjk.distance;
        double closing = spin - relative.dot(normal);
        impact = {time, closing, normal, gjk.pointA - normal * shapeA.margin(),
                  gjk.pointB + normal * shapeB.margin()};
        known = true;
        if (gap <= ccdTargetGap) {
            return closing > minClosingSpeed;
        }
        if (closing <= minClosingSpeed) {
            return false;
        }
        time += gap / closing;
        if (time > dt) {
            return false;
        }
    }
    return known; // Out of iterations while still closing in: rather a contact than a tunnel.
}

bool groundTimeOfImpact(const Body& body, const BodyMotion& motion, double dt, Impact& impact) {
    LocalShape shape(body);
    const double spin = motion.angularVelocity.length() * shape.reach();
    double closing = spin - motion.velocity.y;
    if (closing <= minClosingSpeed) {
        return false;
    }
    double time = 0;
    for (int i = 0; i < maxAdvancements; ++i) {
        Vec3 position;
        Quat orientation;
        advancePose(body, motion, time, position, orientation);
        Vec3 lowest = shape.at(position, orientation).support({0, -1, 0});
        lowest.y -= shape.margin();
        impact = {time, closing, {0, 1, 0}, lowest, {lowest.x, 0, lowest.z}};
        if (lowest.y <= ccdTargetGap) {
            return true;
        }
        time += lowest.y / closing;
        if (time > dt) {
            return false;
        }
    }
    return true;
}

} // namespace robotics::detail

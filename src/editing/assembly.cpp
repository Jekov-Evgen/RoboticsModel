#include "robotics/editing/assembly.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robotics {
namespace {

void rezeroEncoder(Joint& joint) {
    joint.angle = 0;
    joint.lastWrappedAngle = 0;
    joint.encoderInitialized = false;
}

} // namespace

std::vector<char> assemblyMembers(const World& world, int index) {
    if (index < 0 || index >= static_cast<int>(world.bodies.size())) {
        throw std::runtime_error("Invalid part index");
    }
    std::vector<char> members(world.bodies.size(), 0);
    members[static_cast<std::size_t>(index)] = 1;
    for (bool grew = true; grew;) {
        grew = false;
        for (const auto& joint : world.joints) {
            auto a = static_cast<std::size_t>(joint.a);
            auto b = static_cast<std::size_t>(joint.b);
            if (members[a] && !members[b]) {
                members[b] = 1;
                grew = true;
            }
        }
    }
    return members;
}

void moveAssembly(World& world, int index, Vec3 delta) {
    std::vector<char> moved = assemblyMembers(world, index);
    for (std::size_t i = 0; i < world.bodies.size(); ++i) {
        if (moved[i]) {
            world.bodies[i].position += delta;
        }
    }
    for (auto& joint : world.joints) {
        bool movedA = moved[static_cast<std::size_t>(joint.a)];
        bool movedB = moved[static_cast<std::size_t>(joint.b)];
        if (movedA == movedB) {
            continue;
        }
        const Body& a = world.bodies[static_cast<std::size_t>(joint.a)];
        const Body& b = world.bodies[static_cast<std::size_t>(joint.b)];
        if (movedB) {
            joint.anchorA = a.localPoint(b.point(joint.anchorB));
        } else {
            joint.anchorB = b.localPoint(a.point(joint.anchorA));
        }
        rezeroEncoder(joint);
    }
}

void rotateAssembly(World& world, int index, Vec3 pivot, Vec3 axis, double angle) {
    if (!pivot.finite() || !axis.finite() || axis.length() < 1e-9 || !std::isfinite(angle)) {
        throw std::runtime_error("Invalid rotation");
    }
    std::vector<char> moved = assemblyMembers(world, index);
    Quat rotation = Quat::axisAngle(axis, angle);
    for (std::size_t i = 0; i < world.bodies.size(); ++i) {
        if (!moved[i]) {
            continue;
        }
        Body& body = world.bodies[i];
        body.position = pivot + rotation.rotate(body.position - pivot);
        body.orientation = (rotation * body.orientation).normalized();
        body.velocity = rotation.rotate(body.velocity);
        body.angularVelocity = rotation.rotate(body.angularVelocity);
    }
    for (auto& joint : world.joints) {
        bool movedA = moved[static_cast<std::size_t>(joint.a)];
        bool movedB = moved[static_cast<std::size_t>(joint.b)];
        if (movedA == movedB) {
            continue;
        }
        const Body& a = world.bodies[static_cast<std::size_t>(joint.a)];
        const Body& b = world.bodies[static_cast<std::size_t>(joint.b)];
        // Local frames of the rotated side are still right: derive the other side's from them.
        if (movedB) {
            joint.anchorA = a.localPoint(b.point(joint.anchorB));
            joint.axisA = a.orientation.conjugate().rotate(b.orientation.rotate(joint.axisB)).normalized();
        } else {
            joint.anchorB = b.localPoint(a.point(joint.anchorA));
            joint.axisB = b.orientation.conjugate().rotate(a.orientation.rotate(joint.axisA)).normalized();
        }
        joint.reference = (a.orientation.conjugate() * b.orientation).normalized();
        rezeroEncoder(joint);
    }
}

double lowestPoint(const Body& body) {
    switch (body.shape) {
    case Shape::Sphere:
        return body.position.y - body.size.x / 2;
    case Shape::Cylinder: {
        // Support point along -Y: the rim of the lower cap.
        double spin = body.orientation.rotate({0, 0, 1}).y;
        double radial = std::sqrt(std::max(0.0, 1 - spin * spin));
        return body.position.y - std::abs(spin) * body.size.z / 2 - radial * body.size.x / 2;
    }
    case Shape::ConvexHull:
        if (body.hull) {
            double lowest = body.position.y;
            for (Vec3 vertex : body.hull->vertices) {
                lowest = std::min(lowest, body.point(vertex).y);
            }
            return lowest;
        }
        break;
    case Shape::Box:
        break;
    }
    double lowest = body.position.y;
    for (Vec3 corner : body.vertices()) {
        lowest = std::min(lowest, corner.y);
    }
    return lowest;
}

double assemblyLowestPoint(const World& world, int index) {
    std::vector<char> members = assemblyMembers(world, index);
    double lowest = 1e300;
    for (std::size_t i = 0; i < world.bodies.size(); ++i) {
        if (members[i]) {
            lowest = std::min(lowest, lowestPoint(world.bodies[i]));
        }
    }
    return lowest;
}

double dropAssemblyToFloor(World& world, int index) {
    double shift = -assemblyLowestPoint(world, index);
    moveAssembly(world, index, {0, shift, 0});
    return shift;
}

double snapToStep(double value, double step) {
    return step > 0 ? std::round(value / step) * step : value;
}

} // namespace robotics

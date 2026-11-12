#include "robotics/body.hpp"

namespace robotics {

Vec3 Body::principalInertia() const {
    if (mass <= 0) {
        return {};
    }
    switch (shape) {
    case Shape::Sphere: {
        double inertia = 0.1 * mass * size.x * size.x; // (2/5) m r^2 with r = size.x / 2.
        return {inertia, inertia, inertia};
    }
    case Shape::Cylinder: {
        double radius = size.x * 0.5;
        double axial = 0.5 * mass * radius * radius;
        double transverse = mass * (3 * radius * radius + size.z * size.z) / 12;
        return {transverse, transverse, axial};
    }
    case Shape::ConvexHull:
        if (hull) {
            return hull->unitInertia * mass;
        }
        [[fallthrough]];
    case Shape::Box:
    default:
        return {mass * (size.y * size.y + size.z * size.z) / 12,
                mass * (size.x * size.x + size.z * size.z) / 12,
                mass * (size.x * size.x + size.y * size.y) / 12};
    }
}

Vec3 Body::inverseInertia(Vec3 worldVector) const {
    if (mass <= 0) {
        return {};
    }
    Vec3 inertia = principalInertia();
    Vec3 diagonal{1 / inertia.x, 1 / inertia.y, 1 / inertia.z};
    return orientation.rotate(componentProduct(diagonal, orientation.conjugate().rotate(worldVector)));
}

void Body::impulse(Vec3 linearImpulse, Vec3 worldPoint) {
    if (mass <= 0) {
        return;
    }
    velocity += linearImpulse * inverseMass();
    angularVelocity += inverseInertia((worldPoint - position).cross(linearImpulse));
}

void Body::angularImpulse(Vec3 impulse) {
    angularVelocity += inverseInertia(impulse);
}

std::array<Vec3, 8> Body::vertices() const {
    std::array<Vec3, 8> result;
    for (int i = 0; i < 8; ++i) {
        Vec3 corner{(i & 1 ? 0.5 : -0.5) * size.x, (i & 2 ? 0.5 : -0.5) * size.y,
                    (i & 4 ? 0.5 : -0.5) * size.z};
        result[static_cast<std::size_t>(i)] = point(corner);
    }
    return result;
}

} // namespace robotics

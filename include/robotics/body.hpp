#pragma once
// Rigid body: geometry, mass properties and dynamic state.

#include "robotics/convex_hull.hpp"
#include "robotics/math.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace robotics {

// Stored in .robot files by numeric value: append new shapes at the end only.
// ConvexHull: the polytope in Body::hull (robotics/convex_hull.hpp), e.g. built from a collision mesh.
enum class Shape { Box, Sphere, Cylinder, ConvexHull };

[[nodiscard]] constexpr bool isKnownShape(Shape shape) {
    return shape == Shape::Box || shape == Shape::Sphere || shape == Shape::Cylinder
           || shape == Shape::ConvexHull;
}

// Display-only 3D model (STL/OBJ) attached to a body; physics ignores it. Vertices are scaled per axis,
// then placed with `position`/`orientation` in the body frame. See include/robotics/io/mesh.hpp.
struct VisualMesh {
    std::string path;
    Vec3 scale{1, 1, 1};
    Vec3 position{};
    Quat orientation{};
};

struct Body {
    std::string name{"link"};
    Shape shape{Shape::Box};
    // Box: edge lengths. Sphere: diameters. Cylinder: X=Y=diameter, Z=width. ConvexHull: symmetric
    // bounds of the hull about the centre of mass (hull->boundingSize(), set by setConvexHull).
    Vec3 size{0.5, 0.5, 0.5};
    // Shape::ConvexHull only: immutable hull data in the body frame (centre of mass, principal axes),
    // shared between copies of the body. Required for that shape, ignored otherwise.
    std::shared_ptr<const ConvexHull> hull;
    double mass{1};           // Kilograms; 0 means an immovable (static) body.

    // Dynamic state in world space.
    Vec3 position{};
    Vec3 velocity{};
    Vec3 angularVelocity{};
    Quat orientation{};

    // External force/torque accumulators; applied and cleared by World::step().
    Vec3 force{};
    Vec3 torque{};

    // Contact material.
    double friction{0.65};
    double restitution{0.05};

    // Continuous collision detection: always generate speculative contacts for this body, so it cannot
    // tunnel through thin bodies even when moving slowly relative to its own size. Fast bodies (motion
    // per step above a fraction of their smallest extent) get them automatically; see
    // World::continuousCollision.
    bool ccd{};

    // Rendering only. Non-empty `visuals` are drawn instead of the collision primitive; `primitiveHidden`
    // hides the primitive of a body without visuals (e.g. an importer's placeholder). Both stay colliding.
    std::vector<VisualMesh> visuals;
    bool primitiveHidden{};

    [[nodiscard]] bool isStatic() const { return mass <= 0; }

    [[nodiscard]] double inverseMass() const { return mass > 0 ? 1 / mass : 0; }

    // Principal moments of inertia (kg·m²) about the body's local X, Y, Z axes through the center.
    // Zero for static bodies. The single source of mass properties for the solver.
    [[nodiscard]] Vec3 principalInertia() const;

    // World-space inverse inertia tensor applied to a world-space vector.
    [[nodiscard]] Vec3 inverseInertia(Vec3 worldVector) const;

    [[nodiscard]] Vec3 point(Vec3 local) const { return position + orientation.rotate(local); }

    [[nodiscard]] Vec3 localPoint(Vec3 world) const {
        return orientation.conjugate().rotate(world - position);
    }

    [[nodiscard]] Vec3 pointVelocity(Vec3 worldPoint) const {
        return velocity + angularVelocity.cross(worldPoint - position);
    }

    // Instantaneous velocity changes; no effect on static bodies.
    void impulse(Vec3 linearImpulse, Vec3 worldPoint);
    void angularImpulse(Vec3 impulse);

    void applyForce(Vec3 worldForce, Vec3 worldPoint) {
        force += worldForce;
        torque += (worldPoint - position).cross(worldForce);
    }

    // Box corners in world space (also used as conservative bounds for cylinders and hulls).
    [[nodiscard]] std::array<Vec3, 8> vertices() const;
};

} // namespace robotics

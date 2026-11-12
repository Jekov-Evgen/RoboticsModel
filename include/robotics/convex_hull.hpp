#pragma once
// Convex hulls as collision shapes (Shape::ConvexHull): quickhull construction, exact polyhedral mass
// properties and ray casts. Headless and dependency-free.
//
//   std::vector<Vec3> points = ...;                  // e.g. a mesh's vertices, already scaled
//   auto hull = buildConvexHull(points);              // throws for flat/degenerate input
//   Body body;
//   setConvexHull(body, hull);                        // shape, size (bounds) and hull data
//   body.mass = 1000 * hull->volume;                  // density 1000 kg/m^3, or any given mass
//   hull->placeBody(meshPosition, meshOrientation, body.position, body.orientation);
//
// Frames. The hull is stored in the BODY frame: origin at the centre of mass, axes along the principal
// axes of inertia, so the diagonal-inertia Body model applies unchanged. The input points live in the
// SOURCE frame (typically the scaled mesh frame). They are related by
//     sourcePoint = sourceRotation.rotate(bodyPoint) + sourceOffset.
// A body whose source frame sits at world pose (P, R) therefore has position P + R·sourceOffset and
// orientation R·sourceRotation (placeBody). A visual mesh drawn in the body frame uses the inverse
// transform (visualPosition / visualOrientation). Building is deterministic: the same points and
// options always give bit-identical hulls. Vertices are sorted lexicographically, so a hull persisted as
// its body-frame vertex list and rebuilt with HullBuildOptions::preserveFrame keeps the same vertices,
// bit for bit and in the same order (.robot does this; the list needs no mesh file on load).

#include "robotics/math.hpp"

#include <memory>
#include <string>
#include <vector>

namespace robotics {

struct Body;

struct ConvexHull {
    struct Face {
        Vec3 normal;     // Outward unit normal (body frame).
        double offset{}; // Plane: normal·p = offset; every hull vertex satisfies normal·v <= offset.
        int first{};     // First index in faceVertices.
        int count{};     // Number of vertices (>= 3), counter-clockwise about the normal.
    };

    struct Edge {
        int v0{};
        int v1{};
        int face0{}; // Face that runs v0 -> v1 in its counter-clockwise order.
        int face1{}; // Face that runs v1 -> v0.
    };

    std::vector<Vec3> vertices; // Body frame, sorted (x, y, z), at most maxConvexHullVertices.
    std::vector<Face> faces;    // Coplanar triangles are merged into polygons.
    std::vector<int> faceVertices;
    std::vector<Edge> edges;

    double volume{};             // m^3. Mass = density * volume.
    Vec3 unitInertia;            // Principal moments per kilogram (m^2): inertia = mass * unitInertia.
    Vec3 halfExtents;            // Largest |coordinate| of the vertices per body axis.
    double radius{};             // Largest vertex distance from the centre of mass.
    double minWidth{};           // Smallest extent between a face plane and the opposite side.
    double approximationError{}; // Farthest input point outside the hull (m), > 0 only when capped.

    // Body frame in the source frame (see the header comment).
    Vec3 sourceOffset;   // Centre of mass in source coordinates.
    Quat sourceRotation; // Principal axes in source coordinates.

    // Optional provenance for persistence/UI (not used by the physics): mesh file and per-axis scale.
    std::string sourcePath;
    Vec3 sourceScale{1, 1, 1};

    // Symmetric bounds around the centre of mass: the Body::size of a hull body.
    [[nodiscard]] Vec3 boundingSize() const { return halfExtents * 2; }

    [[nodiscard]] Vec3 toBody(Vec3 sourcePoint) const {
        return sourceRotation.conjugate().rotate(sourcePoint - sourceOffset);
    }

    [[nodiscard]] Vec3 toSource(Vec3 bodyPoint) const {
        return sourceRotation.rotate(bodyPoint) + sourceOffset;
    }

    // World pose of the body when the source frame is at (sourcePosition, sourceOrientation).
    void placeBody(Vec3 sourcePosition, Quat sourceOrientation, Vec3& position, Quat& orientation) const {
        position = sourcePosition + sourceOrientation.rotate(sourceOffset);
        orientation = (sourceOrientation * sourceRotation).normalized();
    }

    // Pose of the source frame in the body frame: VisualMesh::position / orientation for the mesh the
    // hull was built from (with VisualMesh::scale = sourceScale).
    [[nodiscard]] Vec3 visualPosition() const { return sourceRotation.conjugate().rotate(-sourceOffset); }

    [[nodiscard]] Quat visualOrientation() const { return sourceRotation.conjugate(); }
};

// Hard cap of hull vertices (the narrow phase uses fixed-size buffers).
inline constexpr int maxConvexHullVertices = 64;

struct HullBuildOptions {
    // Simplification: the hull keeps the farthest points first and stops at this many vertices
    // (4..maxConvexHullVertices). The result lies inside the exact hull; see approximationError.
    int maxVertices{maxConvexHullVertices};
    // Points within this fraction of the input extent from a face count as on it (duplicates and
    // nearly coplanar points are dropped). Also the planarity tolerance for merging coplanar triangles.
    double relativeTolerance{1e-6};
    // Adjacent triangles whose normals differ by less than this angle (radians) and whose vertices are
    // within the tolerance of a common plane become one polygonal face.
    double coplanarAngle{1e-3};
    // The points are a hull's body-frame vertex list: keep them verbatim (identity source frame) instead
    // of recentring. Throws unless they really are centred on their principal axes.
    bool preserveFrame{false};
};

// Quickhull of a point cloud with exact mass properties, centred at the centre of mass and rotated
// to the principal axes. Throws std::runtime_error for non-finite input, fewer than 4 points or a
// flat (coplanar/collinear/coincident) cloud.
[[nodiscard]] std::shared_ptr<const ConvexHull> buildConvexHull(const std::vector<Vec3>& points,
                                                                const HullBuildOptions& options = {});

// Mass properties of a closed, outward-oriented triangle mesh (Mirtich / Eberly polyhedral integrals)
// for unit density: volume, centroid and inertia tensor about the centroid.
struct PolyhedronMass {
    double volume{};
    Vec3 centroid;
    Mat3 inertia{Mat3::zero()};
};

[[nodiscard]] PolyhedronMass polyhedronMassProperties(const std::vector<Vec3>& vertices,
                                                      const std::vector<int>& triangleIndices);

// Eigen decomposition of a symmetric 3x3 matrix by cyclic Jacobi rotations: m = V·diag(values)·Vᵀ
// with the columns of the proper rotation V as eigenvectors. Off-diagonal terms below 1e-12 of the
// norm are treated as zero, so an (almost) diagonal input returns V = identity.
void diagonalizeSymmetric(const Mat3& m, Vec3& values, Mat3& vectors);

// Makes `body` a hull body: shape, hull and size = hull->boundingSize(). Mass is left unchanged.
void setConvexHull(Body& body, std::shared_ptr<const ConvexHull> hull);

// Ray against a hull in its body frame (unit direction). Returns true for a hit at 0 <= distance <=
// maxDistance with the outward face normal. A ray starting inside hits at distance 0 with
// normal = -direction (the "initial overlap" convention of robotics/sensors/raycast.hpp, whose
// raycastBody() and raycast() are the world-space entry points).
[[nodiscard]] bool raycastConvexHull(const ConvexHull& hull, Vec3 origin, Vec3 direction, double maxDistance,
                                     double& distance, Vec3& normal);

} // namespace robotics

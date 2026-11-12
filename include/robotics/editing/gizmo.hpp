#pragma once
// Geometry of the 3D transform gizmos: handle layout, mouse hit testing and drag math.
// Pure functions of a camera description and a mouse position, shared by the viewport (drawing and input)
// and the tests. Handles have a constant size on screen; axes are the world axes.

#include "robotics/math.hpp"

namespace robotics {

enum class GizmoTool { Select, Move, Rotate };
enum class GizmoHandle { None, AxisX, AxisY, AxisZ, PlaneYZ, PlaneXZ, PlaneXY, RingX, RingY, RingZ };

struct GizmoPoint { // View coordinates in points, origin bottom-left.
    double x{};
    double y{};
};

// Pinhole camera as used by the viewport.
struct GizmoView {
    Vec3 eye;
    Vec3 right;
    Vec3 up;
    Vec3 forward;
    double focal{1}; // Cotangent of half the vertical field of view.
    double width{1}; // View size in points.
    double height{1};

    [[nodiscard]] Vec3 ray(GizmoPoint p) const; // Unit direction through a view point.
    // False for points at or behind the camera.
    [[nodiscard]] bool project(Vec3 world, GizmoPoint& out) const;
    // World length of one point at the depth of `at`.
    [[nodiscard]] double worldPerPoint(Vec3 at) const;
};

// Handle sizes on screen, in points.
inline constexpr double gizmoArrowLength = 112;
inline constexpr double gizmoPlaneStart = 30;
inline constexpr double gizmoPlaneEnd = 56;
inline constexpr double gizmoRingRadius = 96;
inline constexpr double gizmoPickTolerance = 9;

// 0, 1, 2 for the handle's axis (arrow direction, plane normal, ring axis); -1 for None.
[[nodiscard]] int gizmoAxisIndex(GizmoHandle handle);
[[nodiscard]] bool gizmoIsPlane(GizmoHandle handle);
[[nodiscard]] bool gizmoIsRing(GizmoHandle handle);
// World axis as a unit vector.
[[nodiscard]] Vec3 gizmoAxis(int index);

// False when the handle is too close to edge-on to be drawn or dragged reliably.
[[nodiscard]] bool gizmoHandleUsable(const GizmoView& view, Vec3 center, GizmoHandle handle);

// Handle under the mouse for the tool (None for GizmoTool::Select).
[[nodiscard]] GizmoHandle gizmoHitTest(const GizmoView& view, Vec3 center, GizmoTool tool, GizmoPoint mouse);

// One drag of a handle. Filled by beginGizmoDrag, updated by gizmoDragPosition / gizmoDragAngle.
struct GizmoDrag {
    GizmoHandle handle{GizmoHandle::None};
    Vec3 center;      // Part position when the drag began.
    Vec3 grab;        // Point on the axis or plane under the mouse when the drag began.
    Vec3 target;      // Last valid unsnapped target position.
    double lastRaw{}; // Ring: angle of the mouse in the ring plane at the last update.
    double total{};   // Ring: accumulated unsnapped angle.
};

[[nodiscard]] GizmoDrag beginGizmoDrag(const GizmoView& view, Vec3 center, GizmoHandle handle,
                                       GizmoPoint mouse);

// Arrow or plane handle: position the part should have. With step > 0 the dragged world coordinates land on
// multiples of step; the other coordinate keeps its value. Mouse rays nearly parallel to the axis or plane
// keep the last valid target instead of jumping.
[[nodiscard]] Vec3 gizmoDragPosition(GizmoDrag& drag, const GizmoView& view, GizmoPoint mouse, double step);

// Ring handle: signed angle (radians, right-handed about the world axis) since the drag began; several
// turns accumulate. With step > 0 the angle lands on multiples of step.
[[nodiscard]] double gizmoDragAngle(GizmoDrag& drag, const GizmoView& view, GizmoPoint mouse, double step);

} // namespace robotics

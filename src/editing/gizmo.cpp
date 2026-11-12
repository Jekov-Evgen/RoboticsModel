#include "robotics/editing/gizmo.hpp"

#include "robotics/editing/assembly.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace robotics {
namespace {

constexpr double maxDragDistance = 500; // m; beyond this a ray is treated as missing the axis or plane.

double segmentDistance(GizmoPoint p, GizmoPoint a, GizmoPoint b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double length2 = dx * dx + dy * dy;
    double t = length2 > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0.0, 1.0) : 0;
    return std::hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t));
}

bool insideQuad(GizmoPoint p, const std::array<GizmoPoint, 4>& quad) {
    int sign = 0;
    for (std::size_t k = 0; k < 4; ++k) {
        GizmoPoint a = quad[k];
        GizmoPoint b = quad[(k + 1) % 4];
        double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if (cross != 0) {
            int s = cross > 0 ? 1 : -1;
            if (sign != 0 && s != sign) {
                return false;
            }
            sign = s;
        }
    }
    return true;
}

// Ring-plane basis with e1 x e2 = axis, so increasing angles turn right-handed about the axis.
void ringBasis(int axis, Vec3& e1, Vec3& e2) {
    e1 = gizmoAxis((axis + 1) % 3);
    e2 = gizmoAxis((axis + 2) % 3);
}

bool rayPlane(Vec3 origin, Vec3 direction, Vec3 point, Vec3 normal, Vec3& hit) {
    double denominator = direction.dot(normal);
    if (std::abs(denominator) < 1e-4) {
        return false;
    }
    double t = (point - origin).dot(normal) / denominator;
    if (t <= 0 || t > maxDragDistance) {
        return false;
    }
    hit = origin + direction * t;
    return true;
}

// Parameter along the line center + axis * s closest to the mouse ray; false when nearly parallel.
bool closestOnAxis(const GizmoView& view, GizmoPoint mouse, Vec3 center, Vec3 axis, double& s) {
    Vec3 d = view.ray(mouse);
    Vec3 w = center - view.eye;
    double b = axis.dot(d);
    double denominator = 1 - b * b;
    if (denominator < 1e-4) {
        return false;
    }
    double result = (b * d.dot(w) - axis.dot(w)) / denominator;
    double along = (w + axis * result).dot(d); // Distance along the ray of the closest point.
    if (along <= 0 || std::abs(result) > maxDragDistance) {
        return false;
    }
    s = result;
    return true;
}

bool ringAngle(const GizmoView& view, GizmoPoint mouse, Vec3 center, int axis, double& angle) {
    Vec3 hit;
    if (!rayPlane(view.eye, view.ray(mouse), center, gizmoAxis(axis), hit)) {
        return false;
    }
    Vec3 e1;
    Vec3 e2;
    ringBasis(axis, e1, e2);
    Vec3 v = hit - center;
    angle = std::atan2(v.dot(e2), v.dot(e1));
    return true;
}

} // namespace

Vec3 GizmoView::ray(GizmoPoint p) const {
    double nx = 2 * p.x / width - 1;
    double ny = 2 * p.y / height - 1;
    return (forward + right * (nx * (width / height) / focal) + up * (ny / focal)).normalized();
}

bool GizmoView::project(Vec3 world, GizmoPoint& out) const {
    Vec3 v = world - eye;
    double depth = v.dot(forward);
    if (depth < 1e-4) {
        return false;
    }
    double nx = v.dot(right) / depth * focal / (width / height);
    double ny = v.dot(up) / depth * focal;
    out = {(nx + 1) * width / 2, (ny + 1) * height / 2};
    return true;
}

double GizmoView::worldPerPoint(Vec3 at) const {
    double depth = std::max((at - eye).dot(forward), 1e-3);
    return 2 * depth / (focal * height);
}

int gizmoAxisIndex(GizmoHandle handle) {
    switch (handle) {
    case GizmoHandle::AxisX:
    case GizmoHandle::PlaneYZ:
    case GizmoHandle::RingX:
        return 0;
    case GizmoHandle::AxisY:
    case GizmoHandle::PlaneXZ:
    case GizmoHandle::RingY:
        return 1;
    case GizmoHandle::AxisZ:
    case GizmoHandle::PlaneXY:
    case GizmoHandle::RingZ:
        return 2;
    case GizmoHandle::None:
        break;
    }
    return -1;
}

bool gizmoIsPlane(GizmoHandle handle) {
    return handle == GizmoHandle::PlaneYZ || handle == GizmoHandle::PlaneXZ || handle == GizmoHandle::PlaneXY;
}

bool gizmoIsRing(GizmoHandle handle) {
    return handle == GizmoHandle::RingX || handle == GizmoHandle::RingY || handle == GizmoHandle::RingZ;
}

Vec3 gizmoAxis(int index) {
    return index == 0 ? Vec3{1, 0, 0} : (index == 1 ? Vec3{0, 1, 0} : Vec3{0, 0, 1});
}

bool gizmoHandleUsable(const GizmoView& view, Vec3 center, GizmoHandle handle) {
    if (handle == GizmoHandle::None) {
        return false;
    }
    double facing = std::abs((center - view.eye).normalized().dot(gizmoAxis(gizmoAxisIndex(handle))));
    if (gizmoIsRing(handle)) {
        return facing > 0.05; // Seen exactly edge-on a ring is a line.
    }
    return gizmoIsPlane(handle) ? facing > 0.12 : facing < 0.985;
}

GizmoHandle gizmoHitTest(const GizmoView& view, Vec3 center, GizmoTool tool, GizmoPoint mouse) {
    GizmoPoint origin;
    if (tool == GizmoTool::Select || !view.project(center, origin)) {
        return GizmoHandle::None;
    }
    double unit = view.worldPerPoint(center);
    GizmoHandle best = GizmoHandle::None;
    double bestDistance = gizmoPickTolerance;
    if (tool == GizmoTool::Move) {
        constexpr std::array planes{GizmoHandle::PlaneYZ, GizmoHandle::PlaneXZ, GizmoHandle::PlaneXY};
        for (GizmoHandle handle : planes) {
            if (!gizmoHandleUsable(view, center, handle)) {
                continue;
            }
            int axis = gizmoAxisIndex(handle);
            Vec3 u = gizmoAxis((axis + 1) % 3) * unit;
            Vec3 v = gizmoAxis((axis + 2) % 3) * unit;
            std::array<GizmoPoint, 4> quad;
            if (view.project(center + u * gizmoPlaneStart + v * gizmoPlaneStart, quad[0])
                && view.project(center + u * gizmoPlaneEnd + v * gizmoPlaneStart, quad[1])
                && view.project(center + u * gizmoPlaneEnd + v * gizmoPlaneEnd, quad[2])
                && view.project(center + u * gizmoPlaneStart + v * gizmoPlaneEnd, quad[3])
                && insideQuad(mouse, quad)) {
                return handle;
            }
        }
        constexpr std::array axes{GizmoHandle::AxisX, GizmoHandle::AxisY, GizmoHandle::AxisZ};
        for (GizmoHandle handle : axes) {
            GizmoPoint tip;
            if (!gizmoHandleUsable(view, center, handle)
                || !view.project(center + gizmoAxis(gizmoAxisIndex(handle)) * (gizmoArrowLength * unit),
                                 tip)) {
                continue;
            }
            double distance = segmentDistance(mouse, origin, tip);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = handle;
            }
        }
        return best;
    }
    constexpr std::array rings{GizmoHandle::RingX, GizmoHandle::RingY, GizmoHandle::RingZ};
    constexpr int segments = 72;
    for (GizmoHandle handle : rings) {
        if (!gizmoHandleUsable(view, center, handle)) {
            continue;
        }
        Vec3 e1;
        Vec3 e2;
        ringBasis(gizmoAxisIndex(handle), e1, e2);
        GizmoPoint previous;
        bool hasPrevious = false;
        for (int k = 0; k <= segments; ++k) {
            double angle = 2 * pi * k / segments;
            GizmoPoint point;
            Vec3 world = center + (e1 * std::cos(angle) + e2 * std::sin(angle)) * (gizmoRingRadius * unit);
            bool visible = view.project(world, point);
            if (visible && hasPrevious) {
                double distance = segmentDistance(mouse, previous, point);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    best = handle;
                }
            }
            previous = point;
            hasPrevious = visible;
        }
    }
    return best;
}

GizmoDrag beginGizmoDrag(const GizmoView& view, Vec3 center, GizmoHandle handle, GizmoPoint mouse) {
    GizmoDrag drag;
    drag.handle = handle;
    drag.center = center;
    drag.grab = center;
    drag.target = center;
    int axis = gizmoAxisIndex(handle);
    if (gizmoIsRing(handle)) {
        ringAngle(view, mouse, center, axis, drag.lastRaw);
    } else if (gizmoIsPlane(handle)) {
        rayPlane(view.eye, view.ray(mouse), center, gizmoAxis(axis), drag.grab);
    } else if (axis >= 0) {
        double s = 0;
        closestOnAxis(view, mouse, center, gizmoAxis(axis), s);
        drag.grab = center + gizmoAxis(axis) * s;
    }
    return drag;
}

Vec3 gizmoDragPosition(GizmoDrag& drag, const GizmoView& view, GizmoPoint mouse, double step) {
    int axis = gizmoAxisIndex(drag.handle);
    if (axis < 0 || gizmoIsRing(drag.handle)) {
        return drag.center;
    }
    if (gizmoIsPlane(drag.handle)) {
        Vec3 hit;
        if (rayPlane(view.eye, view.ray(mouse), drag.center, gizmoAxis(axis), hit)) {
            drag.target = drag.center + (hit - drag.grab);
        }
    } else {
        double s = 0;
        if (closestOnAxis(view, mouse, drag.center, gizmoAxis(axis), s)) {
            drag.target = drag.center + gizmoAxis(axis) * s - (drag.grab - drag.center);
        }
    }
    Vec3 result = drag.center;
    for (int k = 0; k < 3; ++k) {
        bool dragged = gizmoIsPlane(drag.handle) ? k != axis : k == axis;
        if (dragged) {
            result[k] = snapToStep(drag.target[k], step);
        }
    }
    return result;
}

double gizmoDragAngle(GizmoDrag& drag, const GizmoView& view, GizmoPoint mouse, double step) {
    int axis = gizmoAxisIndex(drag.handle);
    double raw = 0;
    if (gizmoIsRing(drag.handle) && ringAngle(view, mouse, drag.center, axis, raw)) {
        drag.total += std::remainder(raw - drag.lastRaw, 2 * pi);
        drag.lastRaw = raw;
    }
    return snapToStep(drag.total, step);
}

} // namespace robotics

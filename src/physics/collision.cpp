#include "collision.hpp"

#include "broadphase.hpp"
#include "ccd.hpp"
#include "gjk.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace robotics::detail {
namespace {

// Approach speed (m/s) above which restitution is applied; slower contacts come to rest.
constexpr double restitutionVelocityThreshold = 1.0;

constexpr Vec3 groundNormal{0, 1, 0};

// ---- Convex polyhedra for box, cylinder and hull pairs -------------------------------------------
//
// Boxes are exact; a cylinder is a prism with `cylinderSides` lateral faces whose faces touch the
// true cylinder (circumscribed), so a cylinder resting on a lateral face or a cap sits at its true
// radius / half width. Ground, sphere-cylinder and the AABB use the analytic cylinder. Shape::ConvexHull
// bodies bring their own topology; a closed polyhedron with V vertices has at most 2V - 4 faces and
// 3V - 6 edges.

constexpr int cylinderSides = cylinderPrismSides;
constexpr int maxHullVertices = maxConvexHullVertices;
static_assert(2 * cylinderSides <= maxHullVertices);
constexpr int maxHullFaces = 2 * maxHullVertices - 4;
constexpr int maxFaceVertices = maxHullVertices;
constexpr int maxEdgeDirections = cylinderSides / 2 + 1;
constexpr int maxHullEdges = 3 * maxHullVertices - 6;
// Vertex lists of the generated faces: cylinder caps and four per lateral face.
constexpr int maxFaceIndices = 2 * cylinderSides + 4 * cylinderSides;
constexpr int maxManifoldPoints = 4;
// Clipping can add one vertex per reference side.
constexpr int maxClipVertices = maxFaceVertices + maxFaceVertices;

// SAT tie-breaking: an edge axis (or the second body's face) must be clearly better to be chosen.
// Keeps the reference feature stable from step to step (coherent feature ids for warm starting).
constexpr double featureRelativeTolerance = 0.95;
constexpr double featureAbsoluteTolerance = 0.0005;

// Near-ties in the manifold reduction go to the earlier point (fraction of the compared value, and
// metres of depth), so a hull resting on a polygon keeps the same four vertices from step to step.
constexpr double hullTieFraction = 1e-3;
constexpr double hullTieDepth = 0.0005;

// A separating distance below this is treated as touching by the GJK early-out.
constexpr double gjkSeparationMargin = 1e-9;

struct HullFace {
    Vec3 normal;
    int count;
    const int* vertex; // Counter-clockwise about the outward normal.
};

struct HullEdge {
    int v0;
    int v1;
    int direction; // Index into Hull::directions; -1 for convex hull bodies.
    int face0;     // The two faces that meet at this edge.
    int face1;
};

struct Hull {
    Vec3 center;
    int vertexCount{};
    std::array<Vec3, maxHullVertices> vertices;
    int faceCount{};
    std::array<HullFace, maxHullFaces> faces;
    int directionCount{};
    std::array<Vec3, maxEdgeDirections> directions; // Unique edge directions (up to sign).
    int edgeCount{};
    std::array<HullEdge, maxHullEdges> edges;
    int indexCount{};
    std::array<int, maxFaceIndices> indices; // Storage of HullFace::vertex for boxes and cylinders.
};

struct ClipVertex {
    Vec3 p;
    std::uint32_t id{};    // Feature id of the point (see clipAgainstSide).
    std::uint32_t label{}; // Edge leaving this vertex: incident edge k (< 64) or 64 + reference side.
};

struct ManifoldPoint {
    Vec3 p; // Point on the incident hull.
    double depth{};
    std::uint32_t id{};
};

// Working memory of the narrow phase, allocated once per detectContacts().
struct Scratch {
    Hull hullA;
    Hull hullB;
    std::array<ClipVertex, maxClipVertices> clipA;
    std::array<ClipVertex, maxClipVertices> clipB;
    std::array<ManifoldPoint, maxClipVertices> points;
    std::vector<Contact> speculative;
};

void addFace(Hull& hull, Vec3 normal, std::initializer_list<int> indices) {
    HullFace& face = hull.faces[static_cast<std::size_t>(hull.faceCount++)];
    int* vertex = hull.indices.data() + hull.indexCount;
    face.normal = normal;
    face.count = 0;
    face.vertex = vertex;
    for (int index : indices) {
        vertex[face.count++] = index;
    }
    hull.indexCount += face.count;
    const auto& v = hull.vertices;
    Vec3 winding =
        (v[static_cast<std::size_t>(vertex[1])] - v[static_cast<std::size_t>(vertex[0])])
            .cross(v[static_cast<std::size_t>(vertex[2])] - v[static_cast<std::size_t>(vertex[0])]);
    if (winding.dot(normal) < 0) {
        std::reverse(vertex, vertex + face.count);
    }
}

void buildBoxHull(const Body& body, Hull& hull) {
    Mat3 r = Mat3::rotation(body.orientation);
    std::array<Vec3, 3> axes{r * Vec3{1, 0, 0}, r * Vec3{0, 1, 0}, r * Vec3{0, 0, 1}};
    hull.center = body.position;
    hull.vertexCount = 8;
    for (int i = 0; i < 8; ++i) {
        Vec3 local{(i & 1 ? 0.5 : -0.5) * body.size.x, (i & 2 ? 0.5 : -0.5) * body.size.y,
                   (i & 4 ? 0.5 : -0.5) * body.size.z};
        hull.vertices[static_cast<std::size_t>(i)] = body.position + r * local;
    }
    hull.faceCount = 0;
    hull.indexCount = 0;
    addFace(hull, -axes[0], {0, 2, 6, 4});
    addFace(hull, axes[0], {1, 3, 7, 5});
    addFace(hull, -axes[1], {0, 1, 5, 4});
    addFace(hull, axes[1], {2, 3, 7, 6});
    addFace(hull, -axes[2], {0, 1, 3, 2});
    addFace(hull, axes[2], {4, 5, 7, 6});
    hull.directionCount = 3;
    for (std::size_t k = 0; k < 3; ++k) {
        hull.directions[k] = axes[k];
    }
    hull.edgeCount = 0;
    for (int d = 0; d < 3; ++d) {
        for (int i = 0; i < 8; ++i) {
            if (!(i & (1 << d))) {
                // Faces come in pairs per axis (2·axis: negative side); the corner's bits pick the side.
                int e = (d + 1) % 3;
                int f = (d + 2) % 3;
                hull.edges[static_cast<std::size_t>(hull.edgeCount++)] = {
                    i, i | (1 << d), d, 2 * e + ((i >> e) & 1), 2 * f + ((i >> f) & 1)};
            }
        }
    }
}

// Unit circle table for the cylinder prism (vertex k at angle 2πk/N).
const std::array<std::array<double, 2>, cylinderSides>& circleTable() {
    static const auto table = [] {
        std::array<std::array<double, 2>, cylinderSides> t{};
        for (int k = 0; k < cylinderSides; ++k) {
            double angle = 2 * pi * k / cylinderSides;
            t[static_cast<std::size_t>(k)] = {std::cos(angle), std::sin(angle)};
        }
        return t;
    }();
    return table;
}

void buildCylinderHull(const Body& body, Hull& hull) {
    const auto& circle = circleTable();
    Mat3 r = Mat3::rotation(body.orientation);
    constexpr int n = cylinderSides;
    double halfAngle = pi / n;
    double vertexRadius = body.size.x * 0.5 / std::cos(halfAngle); // Faces touch the true cylinder.
    double halfWidth = body.size.z * 0.5;
    hull.center = body.position;
    hull.vertexCount = 2 * n;
    for (int k = 0; k < n; ++k) {
        auto c = circle[static_cast<std::size_t>(k)];
        Vec3 radial{vertexRadius * c[0], vertexRadius * c[1], 0};
        hull.vertices[static_cast<std::size_t>(k)] = body.position + r * (radial + Vec3{0, 0, -halfWidth});
        hull.vertices[static_cast<std::size_t>(n + k)] = body.position + r * (radial + Vec3{0, 0, halfWidth});
    }
    Vec3 axis = r * Vec3{0, 0, 1};
    hull.faceCount = 2;
    HullFace& bottom = hull.faces[0];
    HullFace& top = hull.faces[1];
    bottom.normal = -axis;
    top.normal = axis;
    bottom.count = top.count = n;
    bottom.vertex = hull.indices.data();
    top.vertex = hull.indices.data() + n;
    hull.indexCount = 2 * n;
    for (int k = 0; k < n; ++k) {
        hull.indices[static_cast<std::size_t>(k)] = n - 1 - k; // Clockwise seen from +Z = CCW about -Z.
        hull.indices[static_cast<std::size_t>(n + k)] = n + k;
    }
    hull.directionCount = 1 + n / 2;
    hull.directions[0] = axis;
    hull.edgeCount = 0;
    for (int k = 0; k < n; ++k) {
        int next = (k + 1) % n;
        double mid = 2 * pi * k / n + halfAngle;
        Vec3 outward = r * Vec3{std::cos(mid), std::sin(mid), 0};
        addFace(hull, outward, {k, next, n + next, n + k});
        if (k < n / 2) {
            hull.directions[static_cast<std::size_t>(1 + k)] = r * Vec3{-std::sin(mid), std::cos(mid), 0};
        }
        int ring = 1 + k % (n / 2);
        int side = 2 + k; // Lateral face between vertices k and k + 1.
        hull.edges[static_cast<std::size_t>(hull.edgeCount++)] = {k, next, ring, 0, side};
        hull.edges[static_cast<std::size_t>(hull.edgeCount++)] = {n + k, n + next, ring, 1, side};
        hull.edges[static_cast<std::size_t>(hull.edgeCount++)] = {k, n + k, 0, side, 2 + (k + n - 1) % n};
    }
}

// World-space copy of a Shape::ConvexHull; the face vertex lists are shared with the body's hull.
void buildConvexHullShape(const Body& body, Hull& hull) {
    const ConvexHull& source = *body.hull;
    Mat3 r = Mat3::rotation(body.orientation);
    hull.center = body.position;
    hull.vertexCount = static_cast<int>(source.vertices.size());
    for (std::size_t i = 0; i < source.vertices.size(); ++i) {
        hull.vertices[i] = body.position + r * source.vertices[i];
    }
    hull.faceCount = static_cast<int>(source.faces.size());
    for (std::size_t f = 0; f < source.faces.size(); ++f) {
        const ConvexHull::Face& face = source.faces[f];
        hull.faces[f] = {r * face.normal, face.count,
                         source.faceVertices.data() + static_cast<std::ptrdiff_t>(face.first)};
    }
    hull.directionCount = 0;
    hull.edgeCount = static_cast<int>(source.edges.size());
    for (std::size_t e = 0; e < source.edges.size(); ++e) {
        const ConvexHull::Edge& edge = source.edges[e];
        hull.edges[e] = {edge.v0, edge.v1, -1, edge.face0, edge.face1};
    }
}

void buildHull(const Body& body, Hull& hull) {
    if (body.shape == Shape::Cylinder) {
        buildCylinderHull(body, hull);
    } else if (body.shape == Shape::ConvexHull) {
        buildConvexHullShape(body, hull);
    } else {
        buildBoxHull(body, hull);
    }
}

void project(const Hull& hull, Vec3 axis, double& low, double& high) {
    low = std::numeric_limits<double>::max();
    high = -std::numeric_limits<double>::max();
    for (int i = 0; i < hull.vertexCount; ++i) {
        double d = axis.dot(hull.vertices[static_cast<std::size_t>(i)]);
        low = std::min(low, d);
        high = std::max(high, d);
    }
}

struct FaceQuery {
    double separation{-std::numeric_limits<double>::max()};
    int face{-1};
};

// Largest signed distance of `other` from a face plane of `hull` (> 0: separated).
FaceQuery queryFaces(const Hull& hull, const Hull& other) {
    FaceQuery best;
    for (int f = 0; f < hull.faceCount; ++f) {
        const HullFace& face = hull.faces[static_cast<std::size_t>(f)];
        double plane = face.normal.dot(hull.vertices[static_cast<std::size_t>(face.vertex[0])]);
        double low = 0;
        double high = 0;
        project(other, face.normal, low, high);
        double separation = low - plane;
        if (separation > best.separation) {
            best = {separation, f};
        }
    }
    return best;
}

struct EdgeQuery {
    double separation{-std::numeric_limits<double>::max()};
    Vec3 axis; // From A towards B.
    int directionA{-1};
    int directionB{-1};
};

EdgeQuery queryEdges(const Hull& a, const Hull& b) {
    EdgeQuery best;
    for (int i = 0; i < a.directionCount; ++i) {
        for (int k = 0; k < b.directionCount; ++k) {
            Vec3 axis =
                a.directions[static_cast<std::size_t>(i)].cross(b.directions[static_cast<std::size_t>(k)]);
            double length = axis.length();
            if (length < 1e-6) {
                continue; // Parallel edges: covered by the face axes.
            }
            axis = axis / length;
            double lowA = 0;
            double highA = 0;
            double lowB = 0;
            double highB = 0;
            project(a, axis, lowA, highA);
            project(b, axis, lowB, highB);
            double forward = lowB - highA;  // B beyond A along +axis.
            double backward = lowA - highB; // B beyond A along -axis.
            double separation = std::max(forward, backward);
            if (separation > best.separation) {
                best = {separation, forward >= backward ? axis : -axis, i, k};
            }
        }
    }
    return best;
}

// Edge pairs of two general polyhedra. Only pairs that form a face of the Minkowski difference can
// define a separating axis: their arcs on the Gauss map must cross (D. Gregorius, "The Separating Axis
// Test between Convex Polyhedra", GDC 2013). That test needs no vertex projection, so the cost is
// O(edges A x edges B) tiny determinants instead of O(directions² x vertices).
struct GaussEdgeQuery {
    double separation{-std::numeric_limits<double>::max()};
    Vec3 axis; // From A towards B.
    int edgeA{-1};
    int edgeB{-1};
};

GaussEdgeQuery queryEdgesGaussMap(const Hull& a, const Hull& b) {
    GaussEdgeQuery best;
    for (int i = 0; i < a.edgeCount; ++i) {
        const HullEdge& ea = a.edges[static_cast<std::size_t>(i)];
        Vec3 pa = a.vertices[static_cast<std::size_t>(ea.v0)];
        Vec3 da = a.vertices[static_cast<std::size_t>(ea.v1)] - pa;
        Vec3 n1 = a.faces[static_cast<std::size_t>(ea.face0)].normal;
        Vec3 n2 = a.faces[static_cast<std::size_t>(ea.face1)].normal;
        Vec3 arcA = n1.cross(n2);
        for (int k = 0; k < b.edgeCount; ++k) {
            const HullEdge& eb = b.edges[static_cast<std::size_t>(k)];
            // B's arc is negated: the Minkowski difference holds -B.
            Vec3 m1 = -b.faces[static_cast<std::size_t>(eb.face0)].normal;
            Vec3 m2 = -b.faces[static_cast<std::size_t>(eb.face1)].normal;
            double s1 = m1.dot(arcA);
            double s2 = m2.dot(arcA);
            if (s1 * s2 >= 0) {
                continue;
            }
            Vec3 arcB = m1.cross(m2);
            double s3 = n1.dot(arcB);
            double s4 = n2.dot(arcB);
            if (s3 * s4 >= 0 || s1 * s4 <= 0) {
                continue;
            }
            Vec3 pb = b.vertices[static_cast<std::size_t>(eb.v0)];
            Vec3 axis = da.cross(b.vertices[static_cast<std::size_t>(eb.v1)] - pb);
            double length = axis.length();
            if (length < 1e-9 * std::sqrt(da.length2())) {
                continue; // Parallel edges: covered by the face axes.
            }
            axis = axis / length;
            if (axis.dot(pa - a.center) < 0) {
                axis = -axis; // Outward from A.
            }
            double separation = axis.dot(pb - pa);
            if (separation > best.separation) {
                best = {separation, axis, i, k};
            }
        }
    }
    return best;
}

// ---- Clipping -----------------------------------------------------------------------------------

constexpr std::uint32_t sideLabelBase = maxFaceVertices;

// Point ids take 14 bits, faces 7 bits each (see faceContacts).
constexpr std::uint32_t edgeSideIdBase = maxFaceVertices;
constexpr std::uint32_t cornerIdBase = edgeSideIdBase + maxFaceVertices * maxFaceVertices;
constexpr std::uint32_t deepVertexIdBase = cornerIdBase + maxFaceVertices * maxFaceVertices;
static_assert(deepVertexIdBase + maxHullVertices < (1u << 14) && maxHullFaces < (1 << 7));
constexpr std::uint32_t referenceOnBFeature = 1u << 28;
constexpr std::uint32_t edgeFeature = 1u << 29;
constexpr std::uint32_t speculativeFeature = 1u << 30;
static_assert(maxHullEdges < (1 << 8));

// Sutherland–Hodgman against one reference side plane (inside: normal·p <= offset).
// Ids: incident vertex k -> k; incident edge e x side s -> edgeSideIdBase + 64e + s; reference corner of
// sides (s1, s2) -> cornerIdBase + 64 s1 + s2. Stable while the same features touch, which is what warm
// starting needs.
int clipAgainstSide(const ClipVertex* in, int count, ClipVertex* out, Vec3 normal, double offset,
                    std::uint32_t side) {
    int produced = 0;
    // Merged hull faces are convex only within the tolerance, so the buffer bound needs an explicit check.
    for (int i = 0; i < count && produced + 2 <= maxClipVertices; ++i) {
        const ClipVertex& p = in[i];
        const ClipVertex& q = in[(i + 1) % count];
        double dp = normal.dot(p.p) - offset;
        double dq = normal.dot(q.p) - offset;
        auto intersection = [&] {
            ClipVertex x;
            x.p = p.p + (q.p - p.p) * (dp / (dp - dq));
            x.id = p.label < sideLabelBase
                       ? edgeSideIdBase + maxFaceVertices * p.label + side
                       : cornerIdBase + maxFaceVertices * (p.label - sideLabelBase) + side;
            return x;
        };
        if (dp <= 0 && dq <= 0) {
            out[produced++] = q;
        } else if (dp <= 0 && dq > 0) {
            ClipVertex x = intersection();
            x.label = sideLabelBase + side;
            out[produced++] = x;
        } else if (dp > 0 && dq <= 0) {
            ClipVertex x = intersection();
            x.label = p.label;
            out[produced++] = x;
            out[produced++] = q;
        }
    }
    return produced;
}

// Keeps at most four points that span the largest area, starting with the deepest point. Ties within
// `depthTie` (m) and `tieFraction` go to the earlier point; both are zero for boxes and cylinders.
int reduceManifold(ManifoldPoint* points, int count, Vec3 normal, double depthTie = 0,
                   double tieFraction = 0) {
    if (count <= maxManifoldPoints) {
        return count;
    }
    std::array<int, maxManifoldPoints> chosen{};
    const double margin = 1 + tieFraction;
    int first = 0;
    for (int i = 1; i < count; ++i) {
        if (points[i].depth > points[first].depth + depthTie) {
            first = i;
        }
    }
    int second = first;
    double best = -1;
    for (int i = 0; i < count; ++i) {
        double d = (points[i].p - points[first].p).length2();
        if (d > best * margin) {
            best = d;
            second = i;
        }
    }
    int third = first;
    int fourth = first;
    double most = 0;
    double least = 0;
    for (int i = 0; i < count; ++i) {
        double area = (points[first].p - points[i].p).cross(points[second].p - points[i].p).dot(normal);
        if (area > most * margin) {
            most = area;
            third = i;
        }
        if (area < least * margin) {
            least = area;
            fourth = i;
        }
    }
    chosen = {first, second, third, fourth};
    std::array<ManifoldPoint, maxManifoldPoints> kept{};
    int keptCount = 0;
    for (int index : chosen) {
        bool duplicate = false;
        for (int k = 0; k < keptCount; ++k) {
            duplicate = duplicate || kept[static_cast<std::size_t>(k)].id == points[index].id;
        }
        if (!duplicate) {
            kept[static_cast<std::size_t>(keptCount++)] = points[index];
        }
    }
    std::copy(kept.begin(), kept.begin() + keptCount, points);
    return keptCount;
}

// Closest points of segments p1-q1 and p2-q2.
void closestSegmentPoints(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2, Vec3& c1, Vec3& c2) {
    Vec3 d1 = q1 - p1;
    Vec3 d2 = q2 - p2;
    Vec3 r = p1 - p2;
    double a = d1.length2();
    double e = d2.length2();
    double f = d2.dot(r);
    double c = d1.dot(r);
    double b = d1.dot(d2);
    double denom = a * e - b * b;
    double s = denom > 1e-14 * a * e ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
    double t = e > 1e-14 ? (b * s + f) / e : 0.0;
    if (t < 0) {
        t = 0;
        s = a > 1e-14 ? std::clamp(-c / a, 0.0, 1.0) : 0.0;
    } else if (t > 1) {
        t = 1;
        s = a > 1e-14 ? std::clamp((b - c) / a, 0.0, 1.0) : 0.0;
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

// Appends a contact from the point on A, the point on B and the normal from B towards A.
void pushContact(std::vector<Contact>& contacts, int a, int b, Vec3 pointA, Vec3 pointB, Vec3 normal,
                 std::uint32_t feature) {
    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = normal;
    contact.depth = std::max(0.0, (pointB - pointA).dot(normal));
    contact.point = (pointA + pointB) * 0.5;
    contact.feature = feature;
    contacts.push_back(contact);
}

// Face contact: clip the incident face against the reference face's side planes.
void faceContacts(const Hull& reference, int referenceFace, const Hull& incident, bool referenceIsA, int a,
                  int b, bool stableTies, Scratch& scratch, std::vector<Contact>& contacts) {
    const HullFace& ref = reference.faces[static_cast<std::size_t>(referenceFace)];
    Vec3 n = ref.normal;
    int incidentFace = 0;
    double mostOpposed = std::numeric_limits<double>::max();
    for (int f = 0; f < incident.faceCount; ++f) {
        double d = incident.faces[static_cast<std::size_t>(f)].normal.dot(n);
        if (d < mostOpposed) {
            mostOpposed = d;
            incidentFace = f;
        }
    }
    const HullFace& inc = incident.faces[static_cast<std::size_t>(incidentFace)];
    int count = inc.count;
    for (int k = 0; k < count; ++k) {
        scratch.clipA[static_cast<std::size_t>(k)] = {
            incident.vertices[static_cast<std::size_t>(inc.vertex[k])], static_cast<std::uint32_t>(k),
            static_cast<std::uint32_t>(k)};
    }
    ClipVertex* in = scratch.clipA.data();
    ClipVertex* out = scratch.clipB.data();
    for (int s = 0; s < ref.count && count > 0; ++s) {
        Vec3 v0 = reference.vertices[static_cast<std::size_t>(ref.vertex[s])];
        Vec3 v1 = reference.vertices[static_cast<std::size_t>(ref.vertex[(s + 1) % ref.count])];
        Vec3 sideNormal = (v1 - v0).cross(n);
        count =
            clipAgainstSide(in, count, out, sideNormal, sideNormal.dot(v0), static_cast<std::uint32_t>(s));
        std::swap(in, out);
    }
    double plane = n.dot(reference.vertices[static_cast<std::size_t>(ref.vertex[0])]);
    auto& points = scratch.points;
    int pointCount = 0;
    for (int k = 0; k < count; ++k) {
        double depth = plane - n.dot(in[k].p);
        if (depth >= 0) {
            points[static_cast<std::size_t>(pointCount++)] = {in[k].p, depth, in[k].id};
        }
    }
    if (pointCount == 0) {
        // Degenerate clip (grazing corner): deepest incident vertex.
        double deepest = 0;
        for (int i = 0; i < incident.vertexCount; ++i) {
            double depth = plane - n.dot(incident.vertices[static_cast<std::size_t>(i)]);
            if (depth > deepest) {
                deepest = depth;
                points[0] = {incident.vertices[static_cast<std::size_t>(i)], depth,
                             deepVertexIdBase + static_cast<std::uint32_t>(i)};
                pointCount = 1;
            }
        }
    }
    pointCount = stableTies ? reduceManifold(points.data(), pointCount, n, hullTieDepth, hullTieFraction)
                            : reduceManifold(points.data(), pointCount, n);
    std::uint32_t base = (referenceIsA ? 0u : referenceOnBFeature)
                         | (static_cast<std::uint32_t>(referenceFace) << 21)
                         | (static_cast<std::uint32_t>(incidentFace) << 14);
    for (int k = 0; k < pointCount; ++k) {
        const ManifoldPoint& p = points[static_cast<std::size_t>(k)];
        Vec3 onReference = p.p + n * p.depth;
        if (referenceIsA) {
            pushContact(contacts, a, b, onReference, p.p, -n, base | p.id);
        } else {
            pushContact(contacts, a, b, p.p, onReference, n, base | p.id);
        }
    }
}

// Support edge of `hull` with the given direction index, farthest along `axis`.
const HullEdge& supportEdge(const Hull& hull, int direction, Vec3 axis, int& index) {
    double best = -std::numeric_limits<double>::max();
    index = 0;
    for (int e = 0; e < hull.edgeCount; ++e) {
        const HullEdge& edge = hull.edges[static_cast<std::size_t>(e)];
        if (edge.direction != direction) {
            continue;
        }
        double d = axis.dot(hull.vertices[static_cast<std::size_t>(edge.v0)]
                            + hull.vertices[static_cast<std::size_t>(edge.v1)]);
        if (d > best) {
            best = d;
            index = e;
        }
    }
    return hull.edges[static_cast<std::size_t>(index)];
}

// Separating axis test with face clipping (up to 4 points) or edge-edge closest points (1 point).
void hullContacts(const Body& bodyA, const Body& bodyB, int a, int b, Scratch& scratch,
                  std::vector<Contact>& contacts) {
    Hull& hullA = scratch.hullA;
    Hull& hullB = scratch.hullB;
    buildHull(bodyA, hullA);
    buildHull(bodyB, hullB);
    const bool general = bodyA.shape == Shape::ConvexHull || bodyB.shape == Shape::ConvexHull;
    if (general) {
        // Most broad-phase pairs of large hulls are separated: GJK rejects them after a few supports.
        GjkResult gap = gjkDistance({hullA.vertices.data(), hullA.vertexCount, {}, {}},
                                    {hullB.vertices.data(), hullB.vertexCount, {}, {}});
        if (!gap.overlapping && gap.distance > gjkSeparationMargin) {
            return;
        }
    }
    FaceQuery faceA = queryFaces(hullA, hullB);
    if (faceA.separation > 0) {
        return;
    }
    FaceQuery faceB = queryFaces(hullB, hullA);
    if (faceB.separation > 0) {
        return;
    }
    double bestFace = std::max(faceA.separation, faceB.separation);
    if (general) {
        GaussEdgeQuery edge = queryEdgesGaussMap(hullA, hullB);
        if (edge.separation > 0) {
            return;
        }
        if (edge.edgeA >= 0
            && edge.separation > featureRelativeTolerance * bestFace + featureAbsoluteTolerance) {
            const HullEdge& ea = hullA.edges[static_cast<std::size_t>(edge.edgeA)];
            const HullEdge& eb = hullB.edges[static_cast<std::size_t>(edge.edgeB)];
            Vec3 pointA;
            Vec3 pointB;
            closestSegmentPoints(hullA.vertices[static_cast<std::size_t>(ea.v0)],
                                 hullA.vertices[static_cast<std::size_t>(ea.v1)],
                                 hullB.vertices[static_cast<std::size_t>(eb.v0)],
                                 hullB.vertices[static_cast<std::size_t>(eb.v1)], pointA, pointB);
            pushContact(contacts, a, b, pointA, pointB, -edge.axis,
                        edgeFeature | (static_cast<std::uint32_t>(edge.edgeA) << 8)
                            | static_cast<std::uint32_t>(edge.edgeB));
            return;
        }
        if (faceB.separation > featureRelativeTolerance * faceA.separation + featureAbsoluteTolerance) {
            faceContacts(hullB, faceB.face, hullA, false, a, b, true, scratch, contacts);
        } else {
            faceContacts(hullA, faceA.face, hullB, true, a, b, true, scratch, contacts);
        }
        return;
    }
    EdgeQuery edge = queryEdges(hullA, hullB);
    if (edge.separation > 0) {
        return;
    }
    if (edge.directionA >= 0
        && edge.separation > featureRelativeTolerance * bestFace + featureAbsoluteTolerance) {
        int indexA = 0;
        int indexB = 0;
        const HullEdge& ea = supportEdge(hullA, edge.directionA, edge.axis, indexA);
        const HullEdge& eb = supportEdge(hullB, edge.directionB, -edge.axis, indexB);
        Vec3 pointA;
        Vec3 pointB;
        closestSegmentPoints(hullA.vertices[static_cast<std::size_t>(ea.v0)],
                             hullA.vertices[static_cast<std::size_t>(ea.v1)],
                             hullB.vertices[static_cast<std::size_t>(eb.v0)],
                             hullB.vertices[static_cast<std::size_t>(eb.v1)], pointA, pointB);
        std::uint32_t feature =
            edgeFeature | (static_cast<std::uint32_t>(indexA) << 8) | static_cast<std::uint32_t>(indexB);
        pushContact(contacts, a, b, pointA, pointB, -edge.axis, feature);
        return;
    }
    if (faceB.separation > featureRelativeTolerance * faceA.separation + featureAbsoluteTolerance) {
        faceContacts(hullB, faceB.face, hullA, false, a, b, false, scratch, contacts);
    } else {
        faceContacts(hullA, faceA.face, hullB, true, a, b, false, scratch, contacts);
    }
}

// ---- Ground plane --------------------------------------------------------------------------

void addGroundContact(std::vector<Contact>& contacts, int index, Vec3 lowest, std::uint32_t feature) {
    Contact contact;
    contact.a = index;
    contact.b = groundBodyIndex;
    contact.normal = groundNormal;
    contact.depth = -lowest.y;
    contact.point = {lowest.x, lowest.y * 0.5, lowest.z};
    contact.feature = feature;
    contacts.push_back(contact);
}

void collideWithGround(const Body& body, int index, Scratch& scratch, std::vector<Contact>& contacts) {
    switch (body.shape) {
    case Shape::Sphere: {
        double radius = body.size.x * 0.5;
        if (body.position.y <= radius) {
            addGroundContact(contacts, index, {body.position.x, body.position.y - radius, body.position.z},
                             0);
        }
        break;
    }
    case Shape::Cylinder: {
        // Analytic rim points, no faceting: per cap the lowest rim point and the three points at 90°
        // steps from it. Lying on the side -> one line of 2 points; standing on a cap -> 4 points;
        // tilted -> the touching subset. Continuous in the orientation (no switch at "vertical").
        Vec3 up = body.orientation.conjugate().rotate({0, 1, 0});
        double radial = std::hypot(up.x, up.y);
        double dx = radial > 1e-12 ? -up.x / radial : 1.0;
        double dy = radial > 1e-12 ? -up.y / radial : 0.0;
        double radius = body.size.x / 2;
        for (int cap = 0; cap < 2; ++cap) {
            double z = (cap == 0 ? -0.5 : 0.5) * body.size.z;
            for (int k = 0; k < 4; ++k) {
                double c = k == 0 ? 1 : (k == 2 ? -1 : 0);
                double s = k == 1 ? 1 : (k == 3 ? -1 : 0);
                Vec3 local{radius * (dx * c - dy * s), radius * (dx * s + dy * c), z};
                Vec3 point = body.point(local);
                if (point.y <= 0) {
                    addGroundContact(contacts, index, point, static_cast<std::uint32_t>(cap * 4 + k));
                }
            }
        }
        break;
    }
    case Shape::ConvexHull: {
        // Exact: every vertex below the plane, reduced to the four deepest/extreme ones.
        Mat3 r = Mat3::rotation(body.orientation);
        auto& points = scratch.points;
        int count = 0;
        const auto& vertices = body.hull->vertices;
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            Vec3 p = body.position + r * vertices[i];
            if (p.y <= 0) {
                points[static_cast<std::size_t>(count++)] = {p, -p.y, static_cast<std::uint32_t>(i)};
            }
        }
        count = reduceManifold(points.data(), count, groundNormal, hullTieDepth, hullTieFraction);
        for (int k = 0; k < count; ++k) {
            addGroundContact(contacts, index, points[static_cast<std::size_t>(k)].p,
                             points[static_cast<std::size_t>(k)].id);
        }
        break;
    }
    case Shape::Box:
    default: {
        auto vertices = body.vertices();
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            if (vertices[i].y <= 0) {
                addGroundContact(contacts, index, vertices[i], static_cast<std::uint32_t>(i));
            }
        }
        break;
    }
    }
}

// ---- Sphere pairs (normal points from the second argument towards the first) ---------------

bool sphereSphere(const Body& a, const Body& b, Contact& contact) {
    Vec3 delta = a.position - b.position;
    double distance = delta.length();
    contact.depth = (a.size.x + b.size.x) * 0.5 - distance;
    if (contact.depth < 0) {
        return false;
    }
    contact.normal = distance > 1e-10 ? delta / distance : Vec3{1, 0, 0};
    contact.point = a.position - contact.normal * (a.size.x * 0.5 - contact.depth * 0.5);
    return true;
}

bool sphereBox(const Body& sphere, const Body& box, Contact& contact) {
    Vec3 local = box.localPoint(sphere.position);
    Vec3 half = box.size * 0.5;
    Vec3 closest{std::clamp(local.x, -half.x, half.x), std::clamp(local.y, -half.y, half.y),
                 std::clamp(local.z, -half.z, half.z)};
    Vec3 delta = local - closest;
    double distance = delta.length();
    double radius = sphere.size.x * 0.5;
    if (distance > radius) {
        return false;
    }
    if (distance > 1e-10) {
        contact.normal = box.orientation.rotate(delta / distance);
        contact.depth = radius - distance;
    } else {
        // Sphere centre inside the box: push out through the nearest face.
        int axis = 0;
        double gap = half.x - std::abs(local.x);
        for (int k = 1; k < 3; ++k) {
            if (half[k] - std::abs(local[k]) < gap) {
                axis = k;
                gap = half[k] - std::abs(local[k]);
            }
        }
        Vec3 faceNormal;
        faceNormal[axis] = local[axis] >= 0 ? 1 : -1;
        closest = local;
        closest[axis] = faceNormal[axis] * half[axis];
        contact.normal = box.orientation.rotate(faceNormal);
        contact.depth = radius + gap;
    }
    contact.point = box.point(closest) - contact.normal * (contact.depth * 0.5);
    return true;
}

bool sphereCylinder(const Body& sphere, const Body& cylinder, Contact& contact) {
    Vec3 local = cylinder.localPoint(sphere.position);
    double radius = cylinder.size.x / 2;
    double halfWidth = cylinder.size.z / 2;
    double radial = std::hypot(local.x, local.y);
    Vec3 outward = radial > 1e-12 ? Vec3{local.x / radial, local.y / radial, 0} : Vec3{1, 0, 0};
    Vec3 closest = outward * std::min(radial, radius);
    closest.z = std::clamp(local.z, -halfWidth, halfWidth);
    Vec3 delta = local - closest;
    double distance = delta.length();
    double ballRadius = sphere.size.x / 2;
    if (distance > ballRadius) {
        return false;
    }
    if (distance > 1e-10) {
        contact.normal = cylinder.orientation.rotate(delta / distance);
        contact.depth = ballRadius - distance;
    } else if (radius - radial < halfWidth - std::abs(local.z)) {
        // Centre inside, closest to the curved surface.
        closest = outward * radius;
        closest.z = local.z;
        contact.normal = cylinder.orientation.rotate(outward);
        contact.depth = ballRadius + radius - radial;
    } else {
        // Centre inside, closest to a cap.
        Vec3 capNormal{0, 0, local.z >= 0 ? 1.0 : -1.0};
        closest = local;
        closest.z = capNormal.z * halfWidth;
        contact.normal = cylinder.orientation.rotate(capNormal);
        contact.depth = ballRadius + halfWidth - std::abs(local.z);
    }
    contact.point = cylinder.point(closest) - contact.normal * (contact.depth * 0.5);
    return true;
}

// Sphere against a hull, in the hull's frame: GJK gives the closest point of the hull to the sphere's
// centre; a centre inside the hull needs EPA (exact here: the Minkowski difference is the hull itself).
bool sphereHull(const Body& sphere, const Body& body, Contact& contact) {
    const ConvexHull& hull = *body.hull;
    Vec3 local = body.localPoint(sphere.position);
    ConvexPoints shape{hull.vertices.data(), static_cast<int>(hull.vertices.size()), {}, {}};
    ConvexPoints centre{&local, 1, {}, {}};
    GjkResult closest = gjkDistance(shape, centre);
    double radius = sphere.size.x * 0.5;
    if (!closest.overlapping) {
        if (closest.distance > radius) {
            return false;
        }
        contact.normal = body.orientation.rotate((local - closest.pointA) / closest.distance);
        contact.depth = radius - closest.distance;
        contact.point = body.point(closest.pointA) - contact.normal * (contact.depth * 0.5);
        return true;
    }
    Vec3 normal;
    double inside = 0;
    EpaResult epa = epaPenetration(shape, centre, closest);
    if (epa.valid) {
        normal = epa.normal;
        inside = epa.depth;
    } else {
        // The centre sits on the surface of a sliver: the nearest face plane is as good.
        inside = std::numeric_limits<double>::max();
        for (const auto& face : hull.faces) {
            double gap = face.offset - face.normal.dot(local);
            if (gap < inside) {
                inside = gap;
                normal = face.normal;
            }
        }
    }
    contact.normal = body.orientation.rotate(normal);
    contact.depth = radius + inside;
    contact.point = body.point(local + normal * inside) - contact.normal * (contact.depth * 0.5);
    return true;
}

bool sphereShape(const Body& sphere, const Body& other, Contact& contact) {
    switch (other.shape) {
    case Shape::Cylinder:
        return sphereCylinder(sphere, other, contact);
    case Shape::ConvexHull:
        return sphereHull(sphere, other, contact);
    case Shape::Box:
    case Shape::Sphere:
        break;
    }
    return sphereBox(sphere, other, contact);
}

// Narrow phase for one body pair; appends 0..4 contacts.
void collidePair(const Body& a, const Body& b, int ia, int ib, Scratch& scratch,
                 std::vector<Contact>& contacts) {
    Contact contact;
    contact.a = ia;
    contact.b = ib;
    bool hit = false;
    if (a.shape == Shape::Sphere && b.shape == Shape::Sphere) {
        hit = sphereSphere(a, b, contact);
    } else if (a.shape == Shape::Sphere) {
        hit = sphereShape(a, b, contact);
    } else if (b.shape == Shape::Sphere) {
        hit = sphereShape(b, a, contact);
        contact.normal = -contact.normal;
    } else {
        hullContacts(a, b, ia, ib, scratch, contacts);
        return;
    }
    if (hit) {
        contacts.push_back(contact);
    }
}

// ---- Continuous collision -----------------------------------------------------------------------

// The shape at a time in the step: only what the narrow phase reads.
Body posedAt(const Body& body, const BodyMotion& motion, double time) {
    Body posed;
    posed.shape = body.shape;
    posed.size = body.size;
    posed.hull = body.hull;
    advancePose(body, motion, time, posed.position, posed.orientation);
    return posed;
}

Vec3 pointVelocityAt(const Body& posed, const BodyMotion& motion, Vec3 point) {
    return motion.velocity + motion.angularVelocity.cross(point - posed.position);
}

// Turns the impact into contacts for the solver, which wakes them at the impact (see ImpactSchedule).
//
// `b` is null for the ground plane; ib is then groundBodyIndex.
void addImpactContacts(const Body& a, const BodyMotion& motionA, const Body* b, const BodyMotion& motionB,
                       int ia, int ib, const Impact& impact, double dt, Scratch& scratch,
                       std::vector<Contact>& contacts) {
    // Slightly past the impact, so the discrete narrow phase sees an overlap and builds a full manifold.
    double overlapTime = std::min(dt, impact.time + 2 * ccdTargetGap / std::max(impact.closingSpeed, 1e-9));
    Body posedA = posedAt(a, motionA, overlapTime);
    Body posedB = b ? posedAt(*b, motionB, overlapTime) : Body{};
    auto& found = scratch.speculative;
    found.clear();
    if (b) {
        collidePair(posedA, posedB, ia, ib, scratch, found);
    } else {
        collideWithGround(posedA, ia, scratch, found);
    }
    if (found.empty()) {
        // Still a hair apart (prism against true cylinder, rounding): use the closest points themselves.
        Contact contact;
        contact.a = ia;
        contact.b = ib;
        contact.normal = impact.normal;
        contact.point = (impact.pointA + impact.pointB) * 0.5;
        contact.depth = (impact.pointB - impact.pointA).dot(impact.normal);
        found.push_back(contact);
    }
    for (Contact& contact : found) {
        Vec3 onA = contact.point - contact.normal * (contact.depth * 0.5);
        Vec3 onB = contact.point + contact.normal * (contact.depth * 0.5);
        Vec3 approach =
            pointVelocityAt(posedA, motionA, onA) - (b ? pointVelocityAt(posedB, motionB, onB) : Vec3{});
        double normalVelocity = approach.dot(contact.normal);
        double restitution = b ? std::min(a.restitution, b->restitution) : a.restitution;
        contact.bounce = normalVelocity < -restitutionVelocityThreshold ? -restitution * normalVelocity : 0;
        contact.speculative = true;
        contact.impactTime = impact.time;
        contact.leverA = onA - posedA.position;
        contact.leverB = b ? onB - posedB.position : onB;
        contact.depth = 0;
        contact.feature |= speculativeFeature;
        contacts.push_back(contact);
    }
}

} // namespace

void detectContacts(const std::vector<Body>& bodies, const std::vector<Joint>& joints,
                    std::vector<Contact>& contacts, const ContinuousCollision& continuous) {
    contacts.clear();
    const int count = static_cast<int>(bodies.size());
    Scratch scratch;

    // Predicted motion of the step; `swept` stays empty unless a body is fast or flagged.
    std::vector<BodyMotion> motions;
    std::vector<Aabb> swept;
    if (continuous.dt > 0) {
        motions.reserve(bodies.size());
        bool active = false;
        for (const auto& body : bodies) {
            motions.push_back(predictMotion(body, continuous.dt, continuous.gravity));
            active = active || motions.back().active;
        }
        if (active) {
            swept.reserve(bodies.size());
            for (std::size_t i = 0; i < bodies.size(); ++i) {
                swept.push_back(motions[i].active ? sweptBounds(bodies[i], motions[i], continuous.dt)
                                                  : worldBounds(bodies[i]));
            }
        }
    }
    auto isActive = [&](int index) {
        return !swept.empty() && motions[static_cast<std::size_t>(index)].active;
    };

    for (int i = 0; i < count; ++i) {
        const Body& body = bodies[static_cast<std::size_t>(i)];
        if (body.mass == 0) {
            continue;
        }
        std::size_t before = contacts.size();
        if (worldBounds(body).min.y <= 0) {
            collideWithGround(body, i, scratch, contacts);
        }
        Impact impact;
        if (contacts.size() == before && isActive(i)
            && groundTimeOfImpact(body, motions[static_cast<std::size_t>(i)], continuous.dt, impact)) {
            addImpactContacts(body, motions[static_cast<std::size_t>(i)], nullptr, {}, i, groundBodyIndex,
                              impact, continuous.dt, scratch, contacts);
        }
    }
    for (auto [i, k] : broadphasePairs(bodies, joints, swept.empty() ? nullptr : &swept)) {
        const Body& a = bodies[static_cast<std::size_t>(i)];
        const Body& b = bodies[static_cast<std::size_t>(k)];
        std::size_t before = contacts.size();
        collidePair(a, b, i, k, scratch, contacts);
        Impact impact;
        if (contacts.size() == before && (isActive(i) || isActive(k))) {
            const BodyMotion& motionA = motions[static_cast<std::size_t>(i)];
            const BodyMotion& motionB = motions[static_cast<std::size_t>(k)];
            if (pairTimeOfImpact(a, motionA, b, motionB, continuous.dt, impact)) {
                addImpactContacts(a, motionA, &b, motionB, i, k, impact, continuous.dt, scratch, contacts);
            }
        }
    }
    for (auto& contact : contacts) {
        if (contact.speculative) {
            continue; // addImpactContacts() used the velocities at the impact.
        }
        const Body& a = bodies[static_cast<std::size_t>(contact.a)];
        const Body* b = contact.touchesGround() ? nullptr : &bodies[static_cast<std::size_t>(contact.b)];
        Vec3 velocity = a.pointVelocity(contact.point) - (b ? b->pointVelocity(contact.point) : Vec3{});
        double normalVelocity = velocity.dot(contact.normal);
        double restitution = b ? std::min(a.restitution, b->restitution) : a.restitution;
        contact.bounce = normalVelocity < -restitutionVelocityThreshold ? -restitution * normalVelocity : 0;
    }
}

} // namespace robotics::detail

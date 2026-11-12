// Convex hull construction (quickhull), polyhedral mass properties, principal axes and ray casts.
#include "robotics/convex_hull.hpp"

#include "robotics/body.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace robotics {
namespace {

// ---- Quickhull ------------------------------------------------------------------------------------
//
// Classic incremental quickhull (Barber, Dobkin & Huhdanpaa 1996) over a triangle mesh with explicit
// adjacency. Every iteration adds the globally farthest outside point, so stopping at the vertex cap
// leaves the best greedy approximation. Points within `tolerance` of a face are treated as on it,
// which drops duplicates and (nearly) coplanar points. The horizon is traced in order by a depth-first
// walk over the visible faces (as in J. Lloyd's QuickHull3D).

struct QuickFace {
    std::array<int, 3> v{};            // Point indices, counter-clockwise seen from outside.
    std::array<int, 3> neighbor{};     // Face across edge v[i] -> v[(i + 1) % 3].
    std::array<int, 3> neighborEdge{}; // Index of that edge in the neighbour.
    Vec3 normal;
    double offset{};
    std::vector<int> outside;
    int farthest{-1};
    double farthestDistance{};
    bool alive{true};
    bool visible{};
};

struct InconsistentHull : std::runtime_error {
    InconsistentHull() : std::runtime_error("Convex hull construction failed (inconsistent horizon)") {}
};

class QuickHull {
public:
    QuickHull(const std::vector<Vec3>& points, double tolerance) : points_(points), tolerance_(tolerance) {}

    std::vector<QuickFace> faces;
    double approximationError{};

    void run(int maxVertices) {
        initialSimplex();
        for (;;) {
            int best = -1;
            for (std::size_t f = 0; f < faces.size(); ++f) {
                const QuickFace& face = faces[f];
                if (face.alive && face.farthest >= 0
                    && (best < 0
                        || face.farthestDistance > faces[static_cast<std::size_t>(best)].farthestDistance)) {
                    best = static_cast<int>(f);
                }
            }
            if (best < 0) {
                return;
            }
            if (vertexCount() >= maxVertices) {
                approximationError = faces[static_cast<std::size_t>(best)].farthestDistance;
                return;
            }
            addPoint(best);
        }
    }

private:
    const std::vector<Vec3>& points_;
    double tolerance_;
    std::vector<std::pair<int, int>> horizon_; // (visible face, edge index)
    std::vector<int> visible_;

    [[nodiscard]] const Vec3& point(int index) const { return points_[static_cast<std::size_t>(index)]; }

    // Vertices of the current hull (adding a point can swallow earlier ones, so this is not a counter).
    [[nodiscard]] int vertexCount() const {
        std::vector<int> used;
        for (const auto& face : faces) {
            if (face.alive) {
                used.insert(used.end(), face.v.begin(), face.v.end());
            }
        }
        std::sort(used.begin(), used.end());
        return static_cast<int>(std::unique(used.begin(), used.end()) - used.begin());
    }

    [[nodiscard]] double distance(const QuickFace& face, int index) const {
        return face.normal.dot(point(index)) - face.offset;
    }

    int makeFace(int a, int b, int c) {
        QuickFace face;
        face.v = {a, b, c};
        Vec3 pa = point(a);
        Vec3 pb = point(b);
        Vec3 pc = point(c);
        Vec3 normal = (pb - pa).cross(pc - pa);
        double area = normal.length();
        if (!(area > 0)) {
            throw InconsistentHull(); // The eye lies on the line of a horizon edge.
        }
        face.normal = normal / area;
        face.offset = face.normal.dot((pa + pb + pc) / 3);
        faces.push_back(std::move(face));
        return static_cast<int>(faces.size()) - 1;
    }

    void assign(int index, const std::vector<int>& candidates) {
        int best = -1;
        double bestDistance = tolerance_;
        for (int f : candidates) {
            double d = distance(faces[static_cast<std::size_t>(f)], index);
            if (d > bestDistance) {
                bestDistance = d;
                best = f;
            }
        }
        if (best < 0) {
            return; // Inside (or on) the current hull.
        }
        QuickFace& face = faces[static_cast<std::size_t>(best)];
        face.outside.push_back(index);
        if (face.farthest < 0 || bestDistance > face.farthestDistance) {
            face.farthest = index;
            face.farthestDistance = bestDistance;
        }
    }

    void initialSimplex() {
        const int count = static_cast<int>(points_.size());
        // Extreme points along the axes; the widest pair starts the simplex.
        std::array<int, 6> extreme{};
        for (int i = 0; i < count; ++i) {
            for (int axis = 0; axis < 3; ++axis) {
                if (point(i)[axis] < point(extreme[static_cast<std::size_t>(2 * axis)])[axis]) {
                    extreme[static_cast<std::size_t>(2 * axis)] = i;
                }
                if (point(i)[axis] > point(extreme[static_cast<std::size_t>(2 * axis + 1)])[axis]) {
                    extreme[static_cast<std::size_t>(2 * axis + 1)] = i;
                }
            }
        }
        int i0 = 0;
        int i1 = 0;
        double widest = -1;
        for (int axis = 0; axis < 3; ++axis) {
            int a = extreme[static_cast<std::size_t>(2 * axis)];
            int b = extreme[static_cast<std::size_t>(2 * axis + 1)];
            double d = (point(b) - point(a)).length2();
            if (d > widest) {
                widest = d;
                i0 = a;
                i1 = b;
            }
        }
        if (std::sqrt(widest) <= tolerance_) {
            throw std::runtime_error("Convex hull: all points coincide");
        }
        Vec3 line = (point(i1) - point(i0)).normalized();
        int i2 = -1;
        double farthest = tolerance_;
        for (int i = 0; i < count; ++i) {
            Vec3 d = point(i) - point(i0);
            double off = (d - line * d.dot(line)).length();
            if (off > farthest) {
                farthest = off;
                i2 = i;
            }
        }
        if (i2 < 0) {
            throw std::runtime_error("Convex hull: the points are collinear");
        }
        Vec3 normal = (point(i1) - point(i0)).cross(point(i2) - point(i0)).normalized();
        int i3 = -1;
        farthest = tolerance_;
        for (int i = 0; i < count; ++i) {
            double off = std::abs(normal.dot(point(i) - point(i0)));
            if (off > farthest) {
                farthest = off;
                i3 = i;
            }
        }
        if (i3 < 0) {
            throw std::runtime_error("Convex hull: the points are coplanar (a flat shape has no volume)");
        }

        std::array<int, 4> t{i0, i1, i2, i3};
        Vec3 centroid = (point(i0) + point(i1) + point(i2) + point(i3)) / 4;
        constexpr std::array<std::array<int, 3>, 4> layout{{{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}}};
        for (const auto& l : layout) {
            int a = t[static_cast<std::size_t>(l[0])];
            int b = t[static_cast<std::size_t>(l[1])];
            int c = t[static_cast<std::size_t>(l[2])];
            Vec3 n = (point(b) - point(a)).cross(point(c) - point(a));
            if (n.dot(centroid - point(a)) > 0) {
                std::swap(b, c);
            }
            makeFace(a, b, c);
        }
        for (int f = 0; f < 4; ++f) {
            for (int i = 0; i < 3; ++i) {
                QuickFace& face = faces[static_cast<std::size_t>(f)];
                int a = face.v[static_cast<std::size_t>(i)];
                int b = face.v[static_cast<std::size_t>((i + 1) % 3)];
                for (int g = 0; g < 4; ++g) {
                    for (int j = 0; j < 3; ++j) {
                        const QuickFace& other = faces[static_cast<std::size_t>(g)];
                        if (g != f && other.v[static_cast<std::size_t>(j)] == b
                            && other.v[static_cast<std::size_t>((j + 1) % 3)] == a) {
                            face.neighbor[static_cast<std::size_t>(i)] = g;
                            face.neighborEdge[static_cast<std::size_t>(i)] = j;
                        }
                    }
                }
            }
        }
        std::vector<int> all{0, 1, 2, 3};
        for (int i = 0; i < count; ++i) {
            if (i != i0 && i != i1 && i != i2 && i != i3) {
                assign(i, all);
            }
        }
    }

    void computeHorizon(int eye, int face, int entryEdge) {
        faces[static_cast<std::size_t>(face)].visible = true;
        visible_.push_back(face);
        int edges = entryEdge < 0 ? 3 : 2;
        for (int k = 0; k < edges; ++k) {
            int edge = entryEdge < 0 ? k : (entryEdge + 1 + k) % 3;
            const QuickFace& f = faces[static_cast<std::size_t>(face)];
            int neighbor = f.neighbor[static_cast<std::size_t>(edge)];
            const QuickFace& n = faces[static_cast<std::size_t>(neighbor)];
            if (n.visible) {
                continue;
            }
            if (distance(n, eye) > tolerance_) {
                computeHorizon(eye, neighbor, f.neighborEdge[static_cast<std::size_t>(edge)]);
            } else {
                horizon_.emplace_back(face, edge);
            }
        }
    }

    void addPoint(int faceIndex) {
        int eye = faces[static_cast<std::size_t>(faceIndex)].farthest;
        horizon_.clear();
        visible_.clear();
        computeHorizon(eye, faceIndex, -1);

        std::vector<int> created;
        created.reserve(horizon_.size());
        for (auto [face, edge] : horizon_) {
            const QuickFace& old = faces[static_cast<std::size_t>(face)];
            int a = old.v[static_cast<std::size_t>(edge)];
            int b = old.v[static_cast<std::size_t>((edge + 1) % 3)];
            int outer = old.neighbor[static_cast<std::size_t>(edge)];
            int outerEdge = old.neighborEdge[static_cast<std::size_t>(edge)];
            int index = makeFace(a, b, eye);
            QuickFace& made = faces[static_cast<std::size_t>(index)];
            made.neighbor[0] = outer;
            made.neighborEdge[0] = outerEdge;
            QuickFace& o = faces[static_cast<std::size_t>(outer)];
            o.neighbor[static_cast<std::size_t>(outerEdge)] = index;
            o.neighborEdge[static_cast<std::size_t>(outerEdge)] = 0;
            created.push_back(index);
        }
        // Ring of new faces: edge 1 (b -> eye) of one face meets edge 2 (eye -> a) of the face with a == b.
        for (int f : created) {
            int b = faces[static_cast<std::size_t>(f)].v[1];
            int next = -1;
            for (int g : created) {
                if (faces[static_cast<std::size_t>(g)].v[0] == b) {
                    if (next >= 0) {
                        throw InconsistentHull();
                    }
                    next = g;
                }
            }
            if (next < 0) {
                throw InconsistentHull();
            }
            faces[static_cast<std::size_t>(f)].neighbor[1] = next;
            faces[static_cast<std::size_t>(f)].neighborEdge[1] = 2;
            faces[static_cast<std::size_t>(next)].neighbor[2] = f;
            faces[static_cast<std::size_t>(next)].neighborEdge[2] = 1;
        }
        for (int f : visible_) {
            QuickFace& face = faces[static_cast<std::size_t>(f)];
            face.alive = false;
            std::vector<int> orphans;
            orphans.swap(face.outside);
            face.farthest = -1;
            for (int p : orphans) {
                if (p != eye) {
                    assign(p, created);
                }
            }
        }
    }
};

// ---- Polygonal faces from the triangle hull ---------------------------------------------------------

struct Polygon {
    std::vector<int> loop; // Point indices, counter-clockwise.
};

// Groups adjacent coplanar triangles (flood fill against the seed plane) and returns one boundary loop
// per group. Throws InconsistentHull when a group's boundary is not a single loop.
std::vector<Polygon> mergeCoplanar(const QuickHull& quick, const std::vector<Vec3>& points, double tolerance,
                                   double cosAngle, bool merge) {
    const auto& faces = quick.faces;
    std::vector<int> group(faces.size(), -1);
    std::vector<Polygon> polygons;
    int groups = 0;
    for (std::size_t s = 0; s < faces.size(); ++s) {
        if (!faces[s].alive || group[s] >= 0) {
            continue;
        }
        int g = groups++;
        group[s] = g;
        std::vector<int> members{static_cast<int>(s)};
        if (merge) {
            const Vec3 n = faces[s].normal;
            const double d = faces[s].offset;
            for (std::size_t m = 0; m < members.size(); ++m) {
                const QuickFace& t = faces[static_cast<std::size_t>(members[m])];
                for (int nb : t.neighbor) {
                    const QuickFace& other = faces[static_cast<std::size_t>(nb)];
                    if (!other.alive || group[static_cast<std::size_t>(nb)] >= 0
                        || other.normal.dot(n) < cosAngle) {
                        continue;
                    }
                    bool planar = true;
                    for (int v : other.v) {
                        planar =
                            planar && std::abs(n.dot(points[static_cast<std::size_t>(v)]) - d) <= tolerance;
                    }
                    if (planar) {
                        group[static_cast<std::size_t>(nb)] = g;
                        members.push_back(nb);
                    }
                }
            }
        }
        // Boundary edges of the group, chained into one loop.
        std::vector<std::pair<int, int>> boundary;
        for (int m : members) {
            const QuickFace& t = faces[static_cast<std::size_t>(m)];
            for (int i = 0; i < 3; ++i) {
                if (group[static_cast<std::size_t>(t.neighbor[static_cast<std::size_t>(i)])] != g) {
                    boundary.emplace_back(t.v[static_cast<std::size_t>(i)],
                                          t.v[static_cast<std::size_t>((i + 1) % 3)]);
                }
            }
        }
        Polygon polygon;
        int start = boundary.front().first;
        int current = start;
        do {
            auto next = std::find_if(boundary.begin(), boundary.end(),
                                     [&](const auto& e) { return e.first == current; });
            if (next == boundary.end() || polygon.loop.size() >= boundary.size()) {
                throw InconsistentHull();
            }
            polygon.loop.push_back(current);
            current = next->second;
        } while (current != start);
        if (polygon.loop.size() != boundary.size()) {
            throw InconsistentHull(); // Several loops: the group is not a disc.
        }
        polygons.push_back(std::move(polygon));
    }
    return polygons;
}

Vec3 newellNormal(const std::vector<Vec3>& vertices, const std::vector<int>& loop) {
    Vec3 n;
    for (std::size_t i = 0; i < loop.size(); ++i) {
        Vec3 a = vertices[static_cast<std::size_t>(loop[i])];
        Vec3 b = vertices[static_cast<std::size_t>(loop[(i + 1) % loop.size()])];
        n += a.cross(b);
    }
    return n.normalized();
}

// Assembles the final hull from polygons over the input points; vertices on fewer than three faces
// (on an edge or inside a merged face) are dropped.
std::shared_ptr<ConvexHull> assemble(const std::vector<Vec3>& points, std::vector<Polygon> polygons,
                                     double extent, bool preserveFrame) {
    std::vector<int> faceCount(points.size(), 0);
    for (const auto& polygon : polygons) {
        for (int v : polygon.loop) {
            ++faceCount[static_cast<std::size_t>(v)];
        }
    }
    for (auto& polygon : polygons) {
        std::vector<int> kept;
        for (int v : polygon.loop) {
            if (faceCount[static_cast<std::size_t>(v)] >= 3) {
                kept.push_back(v);
            }
        }
        if (kept.size() < 3) {
            throw InconsistentHull();
        }
        polygon.loop = std::move(kept);
    }
    // Compact vertex numbering in order of first use (deterministic).
    std::vector<int> remap(points.size(), -1);
    std::vector<Vec3> vertices;
    for (auto& polygon : polygons) {
        for (int& v : polygon.loop) {
            if (remap[static_cast<std::size_t>(v)] < 0) {
                remap[static_cast<std::size_t>(v)] = static_cast<int>(vertices.size());
                vertices.push_back(points[static_cast<std::size_t>(v)]);
            }
            v = remap[static_cast<std::size_t>(v)];
        }
    }
    if (static_cast<int>(vertices.size()) > maxConvexHullVertices) {
        throw InconsistentHull();
    }

    // Mass properties (fan triangulation; coordinates relative to the vertex mean for precision).
    Vec3 mean;
    for (Vec3 v : vertices) {
        mean += v;
    }
    mean = mean / static_cast<double>(vertices.size());
    std::vector<Vec3> shifted;
    for (Vec3 v : vertices) {
        shifted.push_back(v - mean);
    }
    std::vector<int> triangles;
    for (const auto& polygon : polygons) {
        for (std::size_t k = 1; k + 1 < polygon.loop.size(); ++k) {
            triangles.insert(triangles.end(), {polygon.loop[0], polygon.loop[k], polygon.loop[k + 1]});
        }
    }
    PolyhedronMass mass = polyhedronMassProperties(shifted, triangles);
    if (!(mass.volume > 1e-12 * extent * extent * extent)) {
        throw std::runtime_error(
            "Convex hull: the points are (nearly) coplanar (a flat shape has no volume)");
    }

    auto hull = std::make_shared<ConvexHull>();
    hull->volume = mass.volume;
    Mat3 unitTensor = mass.inertia * (1 / mass.volume);
    if (preserveFrame) {
        // The points already are a body frame: keep them bit for bit and only verify that claim.
        double norm = std::abs(unitTensor.r0.x) + std::abs(unitTensor.r1.y) + std::abs(unitTensor.r2.z);
        double offDiagonal =
            std::abs(unitTensor.r0.y) + std::abs(unitTensor.r0.z) + std::abs(unitTensor.r1.z);
        if ((mean + mass.centroid).length() > 1e-9 * extent || offDiagonal > 1e-9 * norm) {
            throw std::runtime_error("Convex hull: the vertices are not centred on the principal axes");
        }
        hull->unitInertia = {unitTensor.r0.x, unitTensor.r1.y, unitTensor.r2.z};
        hull->vertices = vertices;
    } else {
        // Principal axes, matched to the source axes as closely as possible (identity for aligned input).
        Vec3 values;
        Mat3 v;
        diagonalizeSymmetric(unitTensor, values, v);
        Mat3 vt = v.transposed(); // Rows = eigenvectors.
        static constexpr std::array<std::array<int, 3>, 6> permutations{
            {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}};
        auto row = [&](int i) {
            return i == 0 ? vt.r0 : (i == 1 ? vt.r1 : vt.r2);
        };
        std::array<int, 3> best = permutations[0];
        double bestScore = -1;
        for (const auto& p : permutations) {
            double score = std::abs(row(p[0]).x) + std::abs(row(p[1]).y) + std::abs(row(p[2]).z);
            if (score > bestScore + 1e-12) {
                bestScore = score;
                best = p;
            }
        }
        std::array<Vec3, 3> axes{};
        Vec3 moments;
        for (int i = 0; i < 3; ++i) {
            Vec3 axis = row(best[static_cast<std::size_t>(i)]);
            axes[static_cast<std::size_t>(i)] = axis[i] < 0 ? -axis : axis;
            moments[i] = values[best[static_cast<std::size_t>(i)]];
        }
        if (axes[0].cross(axes[1]).dot(axes[2]) < 0) {
            int weakest = 0;
            for (int i = 1; i < 3; ++i) {
                if (std::abs(axes[static_cast<std::size_t>(i)][i])
                    < std::abs(axes[static_cast<std::size_t>(weakest)][weakest])) {
                    weakest = i;
                }
            }
            axes[static_cast<std::size_t>(weakest)] = -axes[static_cast<std::size_t>(weakest)];
        }
        // Columns of the rotation = principal axes in source coordinates.
        Mat3 rotation = Mat3{axes[0], axes[1], axes[2]}.transposed();
        hull->unitInertia = moments;
        hull->sourceOffset = mean + mass.centroid;
        hull->sourceRotation = quatFromMatrix(rotation);
        Mat3 toBody = rotation.transposed();
        for (Vec3 p : shifted) {
            hull->vertices.push_back(toBody * (p - mass.centroid));
        }
    }

    // Canonical vertex order: the hull of a persisted (sorted) vertex list numbers them the same way.
    std::vector<int> order(hull->vertices.size());
    std::iota(order.begin(), order.end(), 0);
    auto lexicographic = [](Vec3 a, Vec3 b) {
        return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
    };
    std::sort(order.begin(), order.end(), [&](int l, int r) {
        return lexicographic(hull->vertices[static_cast<std::size_t>(l)],
                             hull->vertices[static_cast<std::size_t>(r)]);
    });
    std::vector<int> rank(order.size());
    std::vector<Vec3> sorted(order.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
        rank[static_cast<std::size_t>(order[i])] = static_cast<int>(i);
        sorted[i] = hull->vertices[static_cast<std::size_t>(order[i])];
    }
    hull->vertices = std::move(sorted);

    for (const auto& polygon : polygons) {
        ConvexHull::Face face;
        face.first = static_cast<int>(hull->faceVertices.size());
        face.count = static_cast<int>(polygon.loop.size());
        for (int index : polygon.loop) {
            hull->faceVertices.push_back(rank[static_cast<std::size_t>(index)]);
        }
        std::vector<int> loop(hull->faceVertices.begin() + face.first, hull->faceVertices.end());
        face.normal = newellNormal(hull->vertices, loop);
        face.offset = -std::numeric_limits<double>::max();
        double lowest = std::numeric_limits<double>::max();
        for (Vec3 p : hull->vertices) { // All vertices: merged faces are planar only within the tolerance.
            face.offset = std::max(face.offset, face.normal.dot(p));
            lowest = std::min(lowest, face.normal.dot(p));
        }
        hull->minWidth =
            hull->faces.empty() ? face.offset - lowest : std::min(hull->minWidth, face.offset - lowest);
        hull->faces.push_back(face);
    }
    // Edges with their two faces; each undirected edge must appear once in each direction.
    std::vector<std::array<int, 4>> directed; // (low, high, face, forward)
    for (std::size_t f = 0; f < hull->faces.size(); ++f) {
        const auto& face = hull->faces[f];
        for (int k = 0; k < face.count; ++k) {
            int a = hull->faceVertices[static_cast<std::size_t>(face.first + k)];
            int b = hull->faceVertices[static_cast<std::size_t>(face.first + (k + 1) % face.count)];
            directed.push_back({std::min(a, b), std::max(a, b), static_cast<int>(f), a < b ? 1 : 0});
        }
    }
    std::sort(directed.begin(), directed.end());
    if (directed.size() % 2 != 0) {
        throw InconsistentHull();
    }
    for (std::size_t i = 0; i < directed.size(); i += 2) {
        const auto& x = directed[i];
        const auto& y = directed[i + 1];
        if (x[0] != y[0] || x[1] != y[1] || x[3] == y[3]) {
            throw InconsistentHull();
        }
        const auto& forward = x[3] == 1 ? x : y; // The face that runs low -> high.
        const auto& backward = x[3] == 1 ? y : x;
        hull->edges.push_back({x[0], x[1], forward[2], backward[2]});
    }
    for (Vec3 p : hull->vertices) {
        for (int axis = 0; axis < 3; ++axis) {
            hull->halfExtents[axis] = std::max(hull->halfExtents[axis], std::abs(p[axis]));
        }
        hull->radius = std::max(hull->radius, p.length());
    }
    return hull;
}

void subexpressions(double w0, double w1, double w2, double& f1, double& f2, double& f3, double& g0,
                    double& g1, double& g2) {
    double temp0 = w0 + w1;
    f1 = temp0 + w2;
    double temp1 = w0 * w0;
    double temp2 = temp1 + w1 * temp0;
    f2 = temp2 + w2 * f1;
    f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
    g0 = f2 + w0 * (f1 + w0);
    g1 = f2 + w1 * (f1 + w1);
    g2 = f2 + w2 * (f1 + w2);
}

} // namespace

PolyhedronMass polyhedronMassProperties(const std::vector<Vec3>& vertices,
                                        const std::vector<int>& triangleIndices) {
    // D. Eberly, "Polyhedral Mass Properties (Revisited)", after B. Mirtich (1996): surface integrals of
    // 1, x, y, z, x², y², z², xy, yz, zx over the closed mesh via the divergence theorem.
    std::array<double, 10> integral{};
    for (std::size_t t = 0; t + 2 < triangleIndices.size(); t += 3) {
        Vec3 p0 = vertices[static_cast<std::size_t>(triangleIndices[t])];
        Vec3 p1 = vertices[static_cast<std::size_t>(triangleIndices[t + 1])];
        Vec3 p2 = vertices[static_cast<std::size_t>(triangleIndices[t + 2])];
        Vec3 d = (p1 - p0).cross(p2 - p0);
        double f1x, f2x, f3x, g0x, g1x, g2x;
        double f1y, f2y, f3y, g0y, g1y, g2y;
        double f1z, f2z, f3z, g0z, g1z, g2z;
        subexpressions(p0.x, p1.x, p2.x, f1x, f2x, f3x, g0x, g1x, g2x);
        subexpressions(p0.y, p1.y, p2.y, f1y, f2y, f3y, g0y, g1y, g2y);
        subexpressions(p0.z, p1.z, p2.z, f1z, f2z, f3z, g0z, g1z, g2z);
        integral[0] += d.x * f1x;
        integral[1] += d.x * f2x;
        integral[2] += d.y * f2y;
        integral[3] += d.z * f2z;
        integral[4] += d.x * f3x;
        integral[5] += d.y * f3y;
        integral[6] += d.z * f3z;
        integral[7] += d.x * (p0.y * g0x + p1.y * g1x + p2.y * g2x);
        integral[8] += d.y * (p0.z * g0y + p1.z * g1y + p2.z * g2y);
        integral[9] += d.z * (p0.x * g0z + p1.x * g1z + p2.x * g2z);
    }
    constexpr std::array<double, 10> factor{1.0 / 6,  1.0 / 24, 1.0 / 24,  1.0 / 24,  1.0 / 60,
                                            1.0 / 60, 1.0 / 60, 1.0 / 120, 1.0 / 120, 1.0 / 120};
    for (std::size_t i = 0; i < integral.size(); ++i) {
        integral[i] *= factor[i];
    }
    PolyhedronMass result;
    result.volume = integral[0];
    if (!(std::abs(result.volume) > 0)) {
        return result;
    }
    Vec3 c = Vec3{integral[1], integral[2], integral[3]} / result.volume;
    double m = result.volume;
    result.centroid = c;
    double xx = integral[5] + integral[6] - m * (c.y * c.y + c.z * c.z);
    double yy = integral[4] + integral[6] - m * (c.z * c.z + c.x * c.x);
    double zz = integral[4] + integral[5] - m * (c.x * c.x + c.y * c.y);
    double xy = -(integral[7] - m * c.x * c.y);
    double yz = -(integral[8] - m * c.y * c.z);
    double zx = -(integral[9] - m * c.z * c.x);
    result.inertia = {{xx, xy, zx}, {xy, yy, yz}, {zx, yz, zz}};
    return result;
}

void diagonalizeSymmetric(const Mat3& m, Vec3& values, Mat3& vectors) {
    double a[3][3] = {{m.r0.x, m.r0.y, m.r0.z}, {m.r1.x, m.r1.y, m.r1.z}, {m.r2.x, m.r2.y, m.r2.z}};
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double norm = 0;
    for (auto& r : a) {
        for (double x : r) {
            norm += x * x;
        }
    }
    norm = std::sqrt(norm);
    const double threshold = 1e-12 * norm;
    constexpr std::array<std::array<int, 2>, 3> pairs{{{0, 1}, {0, 2}, {1, 2}}};
    for (int sweep = 0; sweep < 50; ++sweep) {
        bool rotated = false;
        for (auto [p, q] : pairs) {
            double apq = a[p][q];
            if (!(std::abs(apq) > threshold)) {
                a[p][q] = a[q][p] = 0;
                continue;
            }
            rotated = true;
            double theta = (a[q][q] - a[p][p]) / (2 * apq);
            double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1));
            double c = 1 / std::sqrt(t * t + 1);
            double s = t * c;
            a[p][p] -= t * apq;
            a[q][q] += t * apq;
            a[p][q] = a[q][p] = 0;
            for (int r = 0; r < 3; ++r) {
                if (r != p && r != q) {
                    double arp = a[r][p];
                    double arq = a[r][q];
                    a[r][p] = a[p][r] = c * arp - s * arq;
                    a[r][q] = a[q][r] = s * arp + c * arq;
                }
                double vrp = v[r][p];
                double vrq = v[r][q];
                v[r][p] = c * vrp - s * vrq;
                v[r][q] = s * vrp + c * vrq;
            }
        }
        if (!rotated) {
            break;
        }
    }
    values = {a[0][0], a[1][1], a[2][2]};
    vectors = {{v[0][0], v[0][1], v[0][2]}, {v[1][0], v[1][1], v[1][2]}, {v[2][0], v[2][1], v[2][2]}};
    if (vectors.determinant() < 0) {
        vectors.r0.z = -vectors.r0.z;
        vectors.r1.z = -vectors.r1.z;
        vectors.r2.z = -vectors.r2.z;
    }
}

std::shared_ptr<const ConvexHull> buildConvexHull(const std::vector<Vec3>& points,
                                                  const HullBuildOptions& options) {
    if (points.size() < 4) {
        throw std::runtime_error("Convex hull needs at least 4 points");
    }
    if (!(options.relativeTolerance > 0) || !std::isfinite(options.relativeTolerance)
        || !(options.coplanarAngle >= 0) || options.maxVertices < 4) {
        throw std::runtime_error("Invalid convex hull options");
    }
    Vec3 low = points.front();
    Vec3 high = points.front();
    for (const Vec3& p : points) {
        if (!p.finite()) {
            throw std::runtime_error("Convex hull: non-finite point");
        }
        for (int axis = 0; axis < 3; ++axis) {
            low[axis] = std::min(low[axis], p[axis]);
            high[axis] = std::max(high[axis], p[axis]);
        }
    }
    Vec3 size = high - low;
    double extent = std::max({size.x, size.y, size.z});
    if (!(extent > 0)) {
        throw std::runtime_error("Convex hull: all points coincide");
    }
    // Tolerance relative to the extent, but never below what the coordinates' magnitude can resolve.
    double magnitude = std::max({std::abs(low.x), std::abs(low.y), std::abs(low.z), std::abs(high.x),
                                 std::abs(high.y), std::abs(high.z)});
    double tolerance =
        std::max(options.relativeTolerance * extent, 16 * std::numeric_limits<double>::epsilon() * magnitude);
    int maxVertices = std::min(options.maxVertices, maxConvexHullVertices);
    // A persisted vertex list is already a hull: keep every vertex, merge faces as usual.
    double pointTolerance = options.preserveFrame ? tolerance * 1e-6 : tolerance;
    double cosAngle = std::cos(options.coplanarAngle);

    // Rare tolerance-induced inconsistencies: retry unmerged, then with a coarser tolerance.
    for (int attempt = 0; attempt < 4; ++attempt) {
        try {
            QuickHull quick(points, pointTolerance);
            quick.run(maxVertices);
            std::shared_ptr<ConvexHull> hull;
            try {
                hull = assemble(points, mergeCoplanar(quick, points, tolerance, cosAngle, true), extent,
                                options.preserveFrame);
            } catch (const InconsistentHull&) {
                hull = assemble(points, mergeCoplanar(quick, points, tolerance, cosAngle, false), extent,
                                options.preserveFrame);
            }
            hull->approximationError = quick.approximationError;
            return hull;
        } catch (const InconsistentHull&) {
            pointTolerance *= 10;
        }
    }
    throw std::runtime_error("Convex hull construction failed for this point cloud");
}

void setConvexHull(Body& body, std::shared_ptr<const ConvexHull> hull) {
    if (!hull) {
        throw std::runtime_error("setConvexHull: no hull");
    }
    body.shape = Shape::ConvexHull;
    body.size = hull->boundingSize();
    body.hull = std::move(hull);
}

bool raycastConvexHull(const ConvexHull& hull, Vec3 origin, Vec3 direction, double maxDistance,
                       double& distance, Vec3& normal) {
    // Cyrus–Beck clipping of the ray against every face plane.
    double enter = -std::numeric_limits<double>::max();
    double exit = maxDistance;
    Vec3 enterNormal;
    bool inside = true;
    for (const auto& face : hull.faces) {
        double gap = face.normal.dot(origin) - face.offset; // > 0: outside this plane.
        double speed = face.normal.dot(direction);
        inside = inside && gap <= 0;
        if (std::abs(speed) < 1e-15) {
            if (gap > 0) {
                return false;
            }
            continue;
        }
        double t = -gap / speed;
        if (speed < 0) {
            if (t > enter) {
                enter = t;
                enterNormal = face.normal;
            }
        } else {
            exit = std::min(exit, t);
        }
        if (enter > exit) {
            return false;
        }
    }
    if (inside) {
        distance = 0;
        normal = -direction;
        return true;
    }
    if (enter < 0 || enter > maxDistance) {
        return false;
    }
    distance = enter;
    normal = enterNormal;
    return true;
}

} // namespace robotics

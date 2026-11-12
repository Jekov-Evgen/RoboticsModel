#include "gjk.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace robotics::detail {
namespace {

constexpr int maxGjkIterations = 64;
// Stop when a new support point improves the distance by less than this fraction.
constexpr double gjkRelativeTolerance = 1e-10;
// Squared distance (m²) below which the shapes count as touching.
constexpr double touchingDistance2 = 1e-20;
// Sine of the angle under which a tetrahedron counts as flat.
constexpr double flatTetrahedron = 1e-10;

constexpr int maxEpaIterations = 96;
constexpr double epaRelativeTolerance = 1e-9;

SimplexVertex supportVertex(const ConvexPoints& a, const ConvexPoints& b, Vec3 direction) {
    Vec3 pa = a.support(direction);
    Vec3 pb = b.support(-direction);
    return {pa - pb, pa, pb};
}

// A sub-simplex closest to the origin: its vertex indices and barycentric weights.
struct Closest {
    int count{};
    std::array<int, 3> vertex{};
    std::array<double, 3> weight{};
    Vec3 point;
};

Closest closestOnSegment(const std::array<SimplexVertex, 4>& s, int ia, int ib) {
    Vec3 a = s[static_cast<std::size_t>(ia)].w;
    Vec3 b = s[static_cast<std::size_t>(ib)].w;
    Vec3 ab = b - a;
    double length2 = ab.length2();
    double t = length2 > 0 ? -a.dot(ab) / length2 : 0;
    if (t <= 0) {
        return {1, {ia, 0, 0}, {1, 0, 0}, a};
    }
    if (t >= 1) {
        return {1, {ib, 0, 0}, {1, 0, 0}, b};
    }
    return {2, {ia, ib, 0}, {1 - t, t, 0}, a + ab * t};
}

Closest closestOnTriangle(const std::array<SimplexVertex, 4>& s, int ia, int ib, int ic) {
    Vec3 a = s[static_cast<std::size_t>(ia)].w;
    Vec3 b = s[static_cast<std::size_t>(ib)].w;
    Vec3 c = s[static_cast<std::size_t>(ic)].w;
    Vec3 n = (b - a).cross(c - a);
    double n2 = n.length2();
    if (n2 > 0) {
        // Barycentric coordinates of the origin's projection; all positive: it lies inside the triangle.
        double u = n.dot(b.cross(c)) / n2;
        double v = n.dot(c.cross(a)) / n2;
        double w = 1 - u - v;
        if (u > 0 && v > 0 && w > 0) {
            return {3, {ia, ib, ic}, {u, v, w}, a * u + b * v + c * w};
        }
    }
    Closest best = closestOnSegment(s, ia, ib);
    for (auto [p, q] : {std::pair{ib, ic}, std::pair{ic, ia}}) {
        Closest other = closestOnSegment(s, p, q);
        if (other.point.length2() < best.point.length2()) {
            best = other;
        }
    }
    return best;
}

// Reduces the simplex to the feature closest to the origin. Returns false if the origin is inside the
// tetrahedron; otherwise `closest` holds the reduced feature.
bool closestOnSimplex(const std::array<SimplexVertex, 4>& s, int count, Closest& closest) {
    if (count == 1) {
        closest = {1, {0, 0, 0}, {1, 0, 0}, s[0].w};
        return true;
    }
    if (count == 2) {
        closest = closestOnSegment(s, 0, 1);
        return true;
    }
    if (count == 3) {
        closest = closestOnTriangle(s, 0, 1, 2);
        return true;
    }
    static constexpr std::array<std::array<int, 4>, 4> faces{
        {{0, 1, 2, 3}, {0, 3, 1, 2}, {0, 2, 3, 1}, {1, 3, 2, 0}}}; // Three corners and the opposite one.
    bool outside = false;
    double best = std::numeric_limits<double>::max();
    for (const auto& f : faces) {
        Vec3 a = s[static_cast<std::size_t>(f[0])].w;
        Vec3 n = (s[static_cast<std::size_t>(f[1])].w - a).cross(s[static_cast<std::size_t>(f[2])].w - a);
        Vec3 toOpposite = s[static_cast<std::size_t>(f[3])].w - a;
        double originSide = n.dot(-a);
        double oppositeSide = n.dot(toOpposite);
        // Polytopes have coplanar vertices, so flat tetrahedra are common. Their signs are pure rounding
        // and must not read as "contains the origin": flat counts as outside of every face.
        double flat = flatTetrahedron * std::sqrt(n.length2() * toOpposite.length2());
        if (std::abs(oppositeSide) > flat && originSide * oppositeSide > 0) {
            continue; // The origin and the opposite corner lie on the same side of this face.
        }
        outside = true;
        Closest candidate = closestOnTriangle(s, f[0], f[1], f[2]);
        if (candidate.point.length2() < best) {
            best = candidate.point.length2();
            closest = candidate;
        }
    }
    return outside;
}

void reduceSimplex(GjkResult& result, const Closest& closest) {
    std::array<SimplexVertex, 4> kept{};
    for (int i = 0; i < closest.count; ++i) {
        kept[static_cast<std::size_t>(i)] =
            result.simplex[static_cast<std::size_t>(closest.vertex[static_cast<std::size_t>(i)])];
    }
    result.simplex = kept;
    result.simplexCount = closest.count;
}

// ---- EPA polytope ---------------------------------------------------------------------------------

struct EpaFace {
    std::array<int, 3> v{}; // Counter-clockwise seen from outside.
    Vec3 normal;
    double distance{}; // Signed distance of the plane from the origin; < 0: the origin is outside it.
};

class Polytope {
public:
    std::vector<SimplexVertex> vertices;
    std::vector<EpaFace> faces;

    explicit Polytope(const std::array<SimplexVertex, 4>& tetrahedron)
        : vertices(tetrahedron.begin(), tetrahedron.end()) {
        interior_ = (vertices[0].w + vertices[1].w + vertices[2].w + vertices[3].w) * 0.25;
        addFace(0, 1, 2);
        addFace(0, 3, 1);
        addFace(0, 2, 3);
        addFace(1, 3, 2);
    }

    [[nodiscard]] int closestFace() const {
        int best = -1;
        for (int f = 0; f < static_cast<int>(faces.size()); ++f) {
            if (best < 0
                || faces[static_cast<std::size_t>(f)].distance
                       < faces[static_cast<std::size_t>(best)].distance) {
                best = f;
            }
        }
        return best;
    }

    // Removes the faces that see the new vertex and connects it to the horizon.
    void expand(const SimplexVertex& vertex) {
        const int index = static_cast<int>(vertices.size());
        vertices.push_back(vertex);
        std::vector<std::pair<int, int>> horizon;
        auto addEdge = [&](int a, int b) {
            auto twin = std::find(horizon.begin(), horizon.end(), std::pair{b, a});
            if (twin != horizon.end()) {
                horizon.erase(twin); // Shared by two removed faces: inside the hole.
            } else {
                horizon.emplace_back(a, b);
            }
        };
        std::erase_if(faces, [&](const EpaFace& face) {
            Vec3 on = vertices[static_cast<std::size_t>(face.v[0])].w;
            if (face.normal.dot(vertex.w - on) <= 0) {
                return false;
            }
            addEdge(face.v[0], face.v[1]);
            addEdge(face.v[1], face.v[2]);
            addEdge(face.v[2], face.v[0]);
            return true;
        });
        for (auto [a, b] : horizon) {
            addFace(a, b, index);
        }
    }

private:
    Vec3 interior_;

    void addFace(int a, int b, int c) {
        Vec3 pa = vertices[static_cast<std::size_t>(a)].w;
        Vec3 normal = (vertices[static_cast<std::size_t>(b)].w - pa)
                          .cross(vertices[static_cast<std::size_t>(c)].w - pa);
        double length = normal.length();
        if (!(length > 1e-14)) {
            return; // Sliver: the neighbours cover it.
        }
        normal = normal / length;
        if (normal.dot(interior_ - pa) > 0) {
            normal = -normal;
            std::swap(b, c);
        }
        faces.push_back({{a, b, c}, normal, normal.dot(pa)});
    }
};

// Grows GJK's terminal simplex (possibly a point, segment or triangle on the boundary) to a tetrahedron
// with volume, using supports along the axes and the current plane normal.
bool buildTetrahedron(const ConvexPoints& a, const ConvexPoints& b, const GjkResult& gjk,
                      std::array<SimplexVertex, 4>& out) {
    int count = std::min(gjk.simplexCount, 4);
    std::copy(gjk.simplex.begin(), gjk.simplex.begin() + count, out.begin());
    if (count == 0) {
        out[0] = supportVertex(a, b, {1, 0, 0});
        count = 1;
    }
    if (count == 4) {
        Vec3 e1 = out[1].w - out[0].w;
        Vec3 e2 = out[2].w - out[0].w;
        Vec3 e3 = out[3].w - out[0].w;
        if (std::abs(e1.cross(e2).dot(e3)) > 1e-18) {
            return true;
        }
        count = 3;
    }
    // Distance of a candidate from the affine hull of the first `count` vertices.
    auto gain = [&](Vec3 w) {
        Vec3 d = w - out[0].w;
        if (count == 1) {
            return d.length();
        }
        if (count == 2) {
            Vec3 axis = (out[1].w - out[0].w).normalized();
            return (d - axis * d.dot(axis)).length();
        }
        return std::abs(d.dot((out[1].w - out[0].w).cross(out[2].w - out[0].w).normalized()));
    };
    while (count < 4) {
        std::array<Vec3, 8> directions{Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, -1, 0},
                                       Vec3{0, 0, 1}, Vec3{0, 0, -1}, Vec3{},        Vec3{}};
        if (count >= 3) {
            Vec3 n = (out[1].w - out[0].w).cross(out[2].w - out[0].w).normalized();
            directions[6] = n;
            directions[7] = -n;
        }
        SimplexVertex best;
        double bestGain = 1e-10;
        bool found = false;
        for (Vec3 direction : directions) {
            if (direction.length2() == 0) {
                continue;
            }
            SimplexVertex candidate = supportVertex(a, b, direction);
            double g = gain(candidate.w);
            if (g > bestGain) {
                bestGain = g;
                best = candidate;
                found = true;
            }
        }
        if (!found) {
            return false; // Flat Minkowski difference: no volume to expand.
        }
        out[static_cast<std::size_t>(count++)] = best;
    }
    return true;
}

} // namespace

Vec3 ConvexPoints::support(Vec3 direction) const {
    Vec3 local = rotation.transposed() * direction;
    int best = 0;
    double most = -std::numeric_limits<double>::max();
    for (int i = 0; i < count; ++i) {
        double d = points[i].dot(local);
        if (d > most) {
            most = d;
            best = i;
        }
    }
    return position + rotation * points[best];
}

GjkResult gjkDistance(const ConvexPoints& a, const ConvexPoints& b) {
    GjkResult result;
    Vec3 start = a.position - b.position;
    if (start.length2() < 1e-20) {
        start = {1, 0, 0};
    }
    result.simplex[0] = supportVertex(a, b, -start);
    result.simplexCount = 1;
    Closest closest{1, {0, 0, 0}, {1, 0, 0}, result.simplex[0].w};
    Vec3 v = closest.point;

    auto finish = [&](const Closest& c) {
        result.distance = c.point.length();
        result.pointA = {};
        result.pointB = {};
        for (int i = 0; i < c.count; ++i) {
            const SimplexVertex& s =
                result.simplex[static_cast<std::size_t>(c.vertex[static_cast<std::size_t>(i)])];
            result.pointA += s.a * c.weight[static_cast<std::size_t>(i)];
            result.pointB += s.b * c.weight[static_cast<std::size_t>(i)];
        }
        reduceSimplex(result, c);
        return result;
    };

    for (result.iterations = 1; result.iterations <= maxGjkIterations; ++result.iterations) {
        double v2 = v.length2();
        if (v2 < touchingDistance2) {
            result.overlapping = true;
            result.distance = 0;
            return result;
        }
        SimplexVertex next = supportVertex(a, b, -v);
        // Lower bound of the distance from the supporting plane: no further progress possible.
        if (v2 - v.dot(next.w) <= gjkRelativeTolerance * v2) {
            return finish(closest);
        }
        bool duplicate = false;
        for (int i = 0; i < result.simplexCount; ++i) {
            duplicate = duplicate || (result.simplex[static_cast<std::size_t>(i)].w - next.w).length2() == 0;
        }
        if (duplicate) {
            return finish(closest);
        }
        // Keep the old simplex until the new one is known to be closer (rounding on flat simplices).
        std::array<SimplexVertex, 4> previous = result.simplex;
        int previousCount = result.simplexCount;
        result.simplex[static_cast<std::size_t>(result.simplexCount++)] = next;
        Closest reduced;
        if (!closestOnSimplex(result.simplex, result.simplexCount, reduced)) {
            result.overlapping = true;
            result.distance = 0;
            return result; // simplexCount == 4: the tetrahedron contains the origin.
        }
        if (reduced.point.length2() >= v2) {
            result.simplex = previous;
            result.simplexCount = previousCount;
            return finish(closest);
        }
        reduceSimplex(result, reduced);
        // Indices now run 0..count-1 in the kept order.
        closest = reduced;
        for (int i = 0; i < closest.count; ++i) {
            closest.vertex[static_cast<std::size_t>(i)] = i;
        }
        v = closest.point;
    }
    return finish(closest);
}

EpaResult epaPenetration(const ConvexPoints& a, const ConvexPoints& b, const GjkResult& gjk) {
    EpaResult result;
    std::array<SimplexVertex, 4> tetrahedron{};
    if (!gjk.overlapping || !buildTetrahedron(a, b, gjk, tetrahedron)) {
        return result;
    }
    Polytope polytope(tetrahedron);
    int face = -1;
    for (int iteration = 0; iteration < maxEpaIterations; ++iteration) {
        face = polytope.closestFace();
        if (face < 0) {
            return result;
        }
        const EpaFace& closest = polytope.faces[static_cast<std::size_t>(face)];
        SimplexVertex next = supportVertex(a, b, closest.normal);
        double reach = closest.normal.dot(next.w);
        if (reach - closest.distance <= epaRelativeTolerance * std::max(1e-3, std::abs(reach))) {
            break;
        }
        polytope.expand(next);
    }
    face = polytope.closestFace();
    if (face < 0) {
        return result;
    }
    const EpaFace& f = polytope.faces[static_cast<std::size_t>(face)];
    const SimplexVertex& s0 = polytope.vertices[static_cast<std::size_t>(f.v[0])];
    const SimplexVertex& s1 = polytope.vertices[static_cast<std::size_t>(f.v[1])];
    const SimplexVertex& s2 = polytope.vertices[static_cast<std::size_t>(f.v[2])];
    // Barycentric coordinates of the origin's projection onto the closest face.
    Vec3 p = f.normal * f.distance;
    Vec3 n = (s1.w - s0.w).cross(s2.w - s0.w);
    double n2 = n.length2();
    double u = n.dot((s1.w - p).cross(s2.w - p)) / n2;
    double v = n.dot((s2.w - p).cross(s0.w - p)) / n2;
    double w = 1 - u - v;
    result.valid = true;
    result.depth = std::max(0.0, f.distance);
    result.normal = f.normal;
    result.pointA = s0.a * u + s1.a * v + s2.a * w;
    result.pointB = s0.b * u + s1.b * v + s2.b * w;
    return result;
}

} // namespace robotics::detail

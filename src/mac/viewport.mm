#import "viewport.h"
#include "robotics/demo_scenes.hpp"
#include "robotics/editing/assembly.hpp"
#include "robotics/io/mesh.hpp"
#include "robotics/sensors/sensor_system.hpp"
#include <simd/simd.h>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace robotics;

namespace {
// Every attribute is a float4 so CPU and Metal layouts match without packing rules.
// p.w = emissive amount, n.w = specular strength, c.a = selection highlight.
struct GPUVertex {
    simd_float4 p;
    simd_float4 n;
    simd_float4 c;
};

struct Uniforms {
    simd_float4x4 vp;
    simd_float4 eye;
    simd_float4 light;
    simd_float4 mode;
    simd_float4 tint;
    simd_float4 sky0;
    simd_float4 sky1;
};

struct Material {
    simd_float3 color;
    float emissive{0}, shine{0.35f};
};

struct Mesh {
    std::vector<GPUVertex> v;
    std::vector<std::pair<int, int>> bodies; // first vertex, vertex count per body
    int bodyVertices{0};
    int overlayVertices{0}; // Translucent collision shapes follow the bodies ("Show Collision Shapes").
    int jointVertices{0};   // Joint markers start here; world axes come before them.
    int gizmoVertices{-1};  // Transform gizmo start (appended after everything else); -1 without a gizmo.
    std::vector<std::string> meshErrors; // Visual meshes that could not be loaded (path: reason).
};

const char* kShaders = R"METAL(
#include <metal_stdlib>
using namespace metal;
struct V { float4 p; float4 n; float4 c; };
struct U { float4x4 vp; float4 eye; float4 light; float4 mode; float4 tint; float4 sky0; float4 sky1; };
struct O { float4 pos [[position]]; float3 w; float3 n; float4 c; float e; float s; };

vertex O meshVS(uint id [[vertex_id]], const device V* v [[buffer(0)]], constant U& u [[buffer(1)]]) {
    V x = v[id]; float3 p = x.p.xyz;
    if (u.mode.x == 1.0) { // planar shadow on the floor along the light direction
        float3 L = normalize(u.light.xyz);
        p = p - L * (max(p.y, 0.0) / max(L.y, 0.08)); p.y = 0.004;
    }
    O o; o.pos = u.vp * float4(p, 1.0); o.w = p; o.n = x.n.xyz; o.c = x.c; o.e = x.p.w; o.s = x.n.w;
    // Transform gizmo: squeezed into the nearest slice of the reversed-Z range, always in front of the scene
    // while its own arrows and rings still sort correctly against each other.
    if (u.mode.x == 4.0) o.pos.z = mix(o.pos.z, o.pos.w, 0.985);
    return o;
}
fragment float4 meshFS(O i [[stage_in]], constant U& u [[buffer(1)]]) {
    if (u.mode.x == 1.0) return float4(0.0, 0.0, 0.0, 0.30);
    if (u.mode.x == 2.0) return float4(u.tint.rgb, u.mode.y);
    float3 V = normalize(u.eye.xyz - i.w);
    float3 n = normalize(i.n); if (dot(n, V) < 0.0) n = -n;
    float3 L = normalize(u.light.xyz);
    float3 base = i.c.rgb;
    float diff = max(dot(n, L), 0.0);
    float hemi = 0.5 + 0.5 * n.y;
    float3 ambient = mix(float3(0.13, 0.12, 0.12), float3(0.38, 0.43, 0.52), hemi);
    float3 H = normalize(L + V);
    float spec = pow(max(dot(n, H), 0.0), 48.0) * i.s;
    float facing = max(dot(n, V), 0.0);
    float3 col = base * (ambient + diff * float3(1.0, 0.96, 0.9) * 0.9) + spec * 0.6
               + pow(1.0 - facing, 3.0) * 0.06 * float3(0.6, 0.75, 1.0);
    col = mix(col, base, i.e);
    col += i.c.a * u.tint.rgb * (0.07 + 0.85 * pow(1.0 - facing, 2.2));
    float fog = 1.0 - exp(-length(u.eye.xyz - i.w) * 0.008);
    return float4(mix(col, u.sky0.rgb, fog * 0.85), u.mode.x == 3.0 ? u.mode.y : 1.0);
}

struct GO { float4 pos [[position]]; float3 w; };
vertex GO groundVS(uint id [[vertex_id]], constant U& u [[buffer(1)]]) {
    const float2 c[6] = { float2(-1,-1), float2(1,-1), float2(1,1), float2(-1,-1), float2(1,1), float2(-1,1) };
    float3 p = float3(c[id].x * 800.0, 0.0, c[id].y * 800.0);
    GO o; o.pos = u.vp * float4(p, 1.0); o.w = p; return o;
}
float gridLine(float2 p, float spacing, float width) {
    float2 q = p / spacing; float2 w = max(fwidth(q), float2(1e-5));
    float2 g = abs(fract(q - 0.5) - 0.5) / (w * width);
    return 1.0 - min(min(g.x, g.y), 1.0);
}
fragment float4 groundFS(GO i [[stage_in]], constant U& u [[buffer(1)]]) {
    float2 p = i.w.xz; float2 fw = max(fwidth(p), float2(1e-5));
    bool inside = abs(p.x) <= 40.0 && abs(p.y) <= 40.0;
    float3 col = inside ? float3(0.125, 0.152, 0.19) : float3(0.08, 0.094, 0.118);
    float density = max(fw.x, fw.y);
    float minor = gridLine(p, 1.0, 1.0) * (1.0 - smoothstep(0.06, 0.3, density));
    float major = gridLine(p, 5.0, 1.3) * (1.0 - smoothstep(0.4, 1.6, density));
    col = mix(col, float3(0.25, 0.30, 0.36), minor * (inside ? 0.6 : 0.25));
    col = mix(col, float3(0.36, 0.43, 0.51), major * (inside ? 0.8 : 0.3));
    float bx = abs(p.y) <= 40.0 ? 1.0 - clamp(abs(abs(p.x) - 40.0) / (fw.x * 1.6), 0.0, 1.0) : 0.0;
    float bz = abs(p.x) <= 40.0 ? 1.0 - clamp(abs(abs(p.y) - 40.0) / (fw.y * 1.6), 0.0, 1.0) : 0.0;
    col = mix(col, float3(0.16, 0.74, 0.84), max(bx, bz));
    float fog = 1.0 - exp(-length(u.eye.xyz - i.w) * 0.008);
    return float4(mix(col, u.sky0.rgb, fog * 0.92), 1.0);
}

struct SO { float4 pos [[position]]; float t; };
vertex SO skyVS(uint id [[vertex_id]]) {
    const float2 c[3] = { float2(-1,-1), float2(3,-1), float2(-1,3) };
    SO o; o.pos = float4(c[id], 0.0, 1.0); o.t = c[id].y * 0.5 + 0.5; return o;
}
fragment float4 skyFS(SO i [[stage_in]], constant U& u [[buffer(1)]]) {
    return float4(mix(u.sky0.rgb, u.sky1.rgb, saturate(i.t)), 1.0);
}
)METAL";

simd_float4 f4(Vec3 a, float w) {
    return simd_make_float4(static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z), w);
}

simd_float3 rgb(double r, double g, double b) {
    return simd_make_float3(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b));
}

simd_float3 hsv(double h, double s, double v) {
    h = std::fmod(h, 1.0);
    if (h < 0) {
        h += 1;
    }
    double i = std::floor(h * 6);
    double f = h * 6 - i;
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);
    switch (static_cast<int>(i) % 6) {
    case 0:
        return rgb(v, t, p);
    case 1:
        return rgb(q, v, p);
    case 2:
        return rgb(p, v, t);
    case 3:
        return rgb(p, q, v);
    case 4:
        return rgb(t, p, v);
    default:
        return rgb(v, p, q);
    }
}

void addTri(Mesh& m, Vec3 a, Vec3 b, Vec3 c, Vec3 na, Vec3 nb, Vec3 nc, const Material& mat, float hl) {
    // Consistent counter-clockwise winding seen from outside, so back faces can be culled.
    if ((b - a).cross(c - a).dot(na + nb + nc) < 0) {
        std::swap(b, c);
        std::swap(nb, nc);
    }
    auto push = [&](Vec3 p, Vec3 n) {
        m.v.push_back({f4(p, mat.emissive), f4(n, mat.shine),
                       simd_make_float4(mat.color.x, mat.color.y, mat.color.z, hl)});
    };
    push(a, na);
    push(b, nb);
    push(c, nc);
}

void addPolygon(Mesh& m, const std::vector<Vec3>& pts, Vec3 inside, const Material& mat, float hl) {
    if (pts.size() < 3) {
        return;
    }
    Vec3 center{};
    for (Vec3 p : pts) {
        center += p;
    }
    center = center / static_cast<double>(pts.size());
    Vec3 n = (pts[1] - pts[0]).cross(pts[2] - pts[0]).normalized();
    if (n.dot(center - inside) < 0) {
        n = -n;
    }
    for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
        addTri(m, pts[0], pts[i], pts[i + 1], n, n, n, mat, hl);
    }
}

void basis(Vec3 d, Vec3& u, Vec3& v) {
    Vec3 t = std::abs(d.y) < 0.9 ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
    u = d.cross(t).normalized();
    v = d.cross(u);
}

void addRod(Mesh& m, Vec3 a, Vec3 b, double r, const Material& mat) {
    Vec3 d = b - a;
    if (d.length() < 1e-9) {
        return;
    }
    d = d.normalized();
    Vec3 u;
    Vec3 v;
    basis(d, u, v);
    constexpr int S = 14;
    std::vector<Vec3> capA, capB;
    for (int k = 0; k < S; ++k) {
        double a0 = 2 * robotics::pi * k / S;
        double a1 = 2 * robotics::pi * (k + 1) / S;
        Vec3 n0 = u * std::cos(a0) + v * std::sin(a0);
        Vec3 n1 = u * std::cos(a1) + v * std::sin(a1);
        addTri(m, a + n0 * r, a + n1 * r, b + n1 * r, n0, n1, n1, mat, 0);
        addTri(m, a + n0 * r, b + n1 * r, b + n0 * r, n0, n1, n0, mat, 0);
        capA.push_back(a + n0 * r);
        capB.push_back(b + n0 * r);
    }
    addPolygon(m, capA, (a + b) * 0.5, mat, 0);
    addPolygon(m, capB, (a + b) * 0.5, mat, 0);
}

void addBall(Mesh& m, Vec3 c, double r, const Material& mat) {
    constexpr int rows = 8;
    constexpr int cols = 14;
    auto dir = [&](int row, int col) {
        double lat = -robotics::pi / 2 + robotics::pi * row / rows;
        double lon = 2 * robotics::pi * col / cols;
        return Vec3{std::cos(lat) * std::cos(lon), std::sin(lat), std::cos(lat) * std::sin(lon)};
    };
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            Vec3 a = dir(y, x);
            Vec3 b = dir(y + 1, x);
            Vec3 d = dir(y + 1, x + 1);
            Vec3 e = dir(y, x + 1);
            addTri(m, c + a * r, c + b * r, c + d * r, a, b, d, mat, 0);
            addTri(m, c + a * r, c + d * r, c + e * r, a, d, e, mat, 0);
        }
    }
}

// Box centred at c with half extents along the orthonormal directions d, u, v.
void addOrientedBox(Mesh& m, Vec3 c, Vec3 d, Vec3 u, Vec3 v, Vec3 half, const Material& mat) {
    auto corner = [&](int k) {
        return c + d * ((k & 1) ? half.x : -half.x) + u * ((k & 2) ? half.y : -half.y)
               + v * ((k & 4) ? half.z : -half.z);
    };
    constexpr int faces[6][4] = {{0, 4, 6, 2}, {1, 3, 7, 5}, {0, 1, 5, 4},
                                 {2, 6, 7, 3}, {0, 2, 3, 1}, {4, 5, 7, 6}};
    for (auto& f : faces) {
        addPolygon(m, {corner(f[0]), corner(f[1]), corner(f[2]), corner(f[3])}, c, mat, 0);
    }
}

// Slider gizmo: a rail along the axis through the parent's anchor and a box on the child's anchor.
// With limits the rail spans exactly lower..upper and ends in stop plates.
void addSliderGizmo(Mesh& m, const World& world, const Joint& j, const Material& mat, const Material& stop,
                    double g) {
    const Body& a = world.bodies[j.a];
    Vec3 origin = a.point(j.anchorA);
    Vec3 axis = a.orientation.rotate(j.axisA).normalized();
    double lower = j.limitsEnabled ? j.lowerLimit : std::min(0.0, j.angle) - 0.3 * g;
    double upper = j.limitsEnabled ? j.upperLimit : std::max(0.0, j.angle) + 0.3 * g;
    addRod(m, origin + axis * lower, origin + axis * upper, 0.01 * g, mat);
    Vec3 u;
    Vec3 v;
    basis(axis, u, v);
    Vec3 carriage = world.bodies[j.b].point(j.anchorB);
    addOrientedBox(m, carriage, axis, u, v, Vec3{0.035, 0.05, 0.05} * g, mat);
    if (j.limitsEnabled) {
        for (double limit : {lower, upper}) {
            addOrientedBox(m, origin + axis * limit, axis, u, v, Vec3{0.008, 0.065, 0.065} * g, stop);
        }
    }
}

// Hinge limit gizmo: a thin arc over the allowed range and a tick at each limit angle, drawn in the
// parent's frame around the anchor. Zero angle points from the anchor towards the child.
void addHingeLimitGizmo(Mesh& m, const World& world, const Joint& j, Vec3 anchor, const Material& arc,
                        const Material& stop, double g) {
    const Body& a = world.bodies[j.a];
    Vec3 axisA = j.axisA.normalized();
    // Zero direction: the child's anchor-to-centre direction at the connected pose, in A's frame.
    Vec3 zero = j.reference.rotate(-j.anchorB);
    zero -= axisA * zero.dot(axisA);
    if (zero.length() < 1e-6) {
        Vec3 unused;
        basis(axisA, zero, unused);
    }
    zero = zero.normalized();
    const double radius = 0.22 * g;
    auto direction = [&](double angle) {
        return a.orientation.rotate(Quat::axisAngle(axisA, angle).rotate(zero));
    };
    double span = std::min(j.upperLimit - j.lowerLimit, 2 * robotics::pi);
    int segments = std::max(1, static_cast<int>(std::ceil(span / 0.2)));
    for (int k = 0; k < segments; ++k) {
        double t0 = j.lowerLimit + span * k / segments;
        double t1 = j.lowerLimit + span * (k + 1) / segments;
        addRod(m, anchor + direction(t0) * radius, anchor + direction(t1) * radius, 0.006 * g, arc);
    }
    for (double limit : {j.lowerLimit, j.upperLimit}) {
        addRod(m, anchor + direction(limit) * (0.06 * g), anchor + direction(limit) * (radius + 0.06 * g),
               0.009 * g, stop);
    }
}

Material bodyMaterial(const Body& b, int index) {
    Material m;
    double hue = b.mass == 0 ? 0.60 : std::fmod(0.48 + index * 0.13, 1.0);
    m.color = b.mass == 0 ? hsv(hue, 0.12, 0.58) : hsv(hue, 0.55, 0.86);
    if (b.name == "rover_chassis") {
        m.color = hsv(0.51, 0.78, 0.82);
    }
    if (b.name == "rover_deck") {
        m.color = hsv(0.61, 0.40, 0.34);
    }
    if (b.name == "rover_sensor") {
        m.color = hsv(0.09, 0.75, 0.92);
    }
    m.shine = 0.45f;
    return m;
}

// Body::visuals: STL/OBJ triangles placed in the body frame. Returns false when nothing could be drawn.
bool addVisualMeshes(Mesh& mesh, const Body& b, const Material& base, float hl, MeshCache& cache) {
    bool drawn = false;
    for (const VisualMesh& visual : b.visuals) {
        std::string error;
        auto model = cache.find(visual.path, &error);
        if (!model) {
            mesh.meshErrors.push_back(visual.path + "\n" + error);
            continue;
        }
        drawn = true;
        // Normals of a non-uniformly scaled mesh transform with the inverse scale.
        Vec3 inverseScale{1 / visual.scale.x, 1 / visual.scale.y, 1 / visual.scale.z};
        auto place = [&](Vec3 p) {
            return b.point(visual.position + visual.orientation.rotate(componentProduct(p, visual.scale)));
        };
        auto direction = [&](Vec3 n) {
            return b.orientation.rotate(visual.orientation.rotate(componentProduct(n, inverseScale)))
                .normalized();
        };
        const auto& vertices = model->vertices;
        const auto& normals = model->normals;
        for (std::size_t k = 0; k + 2 < vertices.size(); k += 3) {
            addTri(mesh, place(vertices[k]), place(vertices[k + 1]), place(vertices[k + 2]),
                   direction(normals[k]), direction(normals[k + 1]), direction(normals[k + 2]), base, hl);
        }
    }
    return drawn;
}

void addPrimitive(Mesh& mesh, const Body& b, const Material& base, float hl, bool plain) {
    const Material rubber = plain ? base : Material{rgb(0.075, 0.08, 0.09), 0, 0.12f};
    const Material tread = plain ? base : Material{rgb(0.15, 0.16, 0.17), 0, 0.1f};
    const Material hub = plain ? base : Material{hsv(0.58, 0.18, 0.62), 0, 0.9f};
    const Material glow = plain ? base : Material{rgb(0.15, 0.9, 0.95), 0.85f, 0.2f};
    const Material screen{rgb(0.05, 0.06, 0.07), 0, 0.9f};
    if (b.shape == Shape::Box) {
        auto v = b.vertices();
        constexpr int faces[6][4] = {{0, 4, 6, 2}, {1, 3, 7, 5}, {0, 1, 5, 4},
                                     {2, 6, 7, 3}, {0, 2, 3, 1}, {4, 5, 7, 6}};
        for (auto& f : faces) {
            addPolygon(mesh, {v[f[0]], v[f[1]], v[f[2]], v[f[3]]}, b.position, base, hl);
        }
        if (b.name == "rover_sensor" && !plain) {
            double x = b.size.x / 2 + 0.003;
            addPolygon(mesh,
                       {b.point({x, -0.075, -0.25}), b.point({x, 0.075, -0.25}), b.point({x, 0.075, 0.25}),
                        b.point({x, -0.075, 0.25})},
                       b.position, screen, hl);
            for (double z : {-0.14, 0.14}) {
                addPolygon(mesh,
                           {b.point({x + 0.003, -0.032, z - 0.045}), b.point({x + 0.003, 0.032, z - 0.045}),
                            b.point({x + 0.003, 0.032, z + 0.045}), b.point({x + 0.003, -0.032, z + 0.045})},
                           b.position, glow, hl);
            }
        }
    } else if (b.shape == Shape::Cylinder) {
        constexpr int S = 48;
        double r = b.size.x / 2;
        double w = b.size.z / 2;
        auto at = [&](double angle, double radius, double z) {
            return b.point({radius * std::cos(angle), radius * std::sin(angle), z});
        };
        auto radial = [&](double angle) {
            return b.orientation.rotate({std::cos(angle), std::sin(angle), 0});
        };
        for (int k = 0; k < S; ++k) {
            double a0 = 2 * robotics::pi * k / S;
            double a1 = 2 * robotics::pi * (k + 1) / S;
            const Material& mat = k % 4 == 0 ? tread : rubber;
            addTri(mesh, at(a0, r, -w), at(a1, r, -w), at(a1, r, w), radial(a0), radial(a1), radial(a1), mat,
                   hl);
            addTri(mesh, at(a0, r, -w), at(a1, r, w), at(a0, r, w), radial(a0), radial(a1), radial(a0), mat,
                   hl);
        }
        for (double side : {-1.0, 1.0}) {
            std::vector<Vec3> cap, disc;
            if (plain) {
                for (int k = 0; k < S; ++k) {
                    cap.push_back(at(2 * robotics::pi * k / S, r, side * w));
                }
                addPolygon(mesh, cap, b.position, rubber, hl);
                continue;
            }
            for (int k = 0; k < S; ++k) {
                double a = 2 * robotics::pi * k / S;
                cap.push_back(at(a, r, side * w));
                disc.push_back(at(a, r * 0.56, side * (w + 0.004)));
            }
            addPolygon(mesh, cap, b.position, rubber, hl);
            addPolygon(mesh, disc, b.position, hub, hl);
            for (int k = 0; k < 3; ++k) {
                double a = 2 * robotics::pi * k / 3;
                double z = side * (w + 0.008);
                addPolygon(mesh,
                           {at(a - 0.14, r * 0.15, z), at(a - 0.14, r * 0.48, z), at(a + 0.14, r * 0.48, z),
                            at(a + 0.14, r * 0.15, z)},
                           b.position, glow, hl);
            }
        }
    } else {
        constexpr int rows = 18;
        constexpr int cols = 32;
        double r = b.size.x * 0.5;
        auto dir = [&](int row, int col) {
            double lat = -robotics::pi / 2 + robotics::pi * row / rows;
            double lon = 2 * robotics::pi * col / cols;
            return Vec3{std::cos(lat) * std::cos(lon), std::sin(lat), std::cos(lat) * std::sin(lon)};
        };
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                Material mat = base;
                if (!plain && ((x / 4) + (y / 3)) % 2) {
                    mat.color *= 0.86f; // Stripes make rolling visible.
                }
                Vec3 d[4] = {dir(y, x), dir(y + 1, x), dir(y + 1, x + 1), dir(y, x + 1)}, p[4], n[4];
                for (int k = 0; k < 4; ++k) {
                    p[k] = b.point(d[k] * r);
                    n[k] = b.orientation.rotate(d[k]);
                }
                addTri(mesh, p[0], p[1], p[2], n[0], n[1], n[2], mat, hl);
                addTri(mesh, p[0], p[2], p[3], n[0], n[2], n[3], mat, hl);
            }
        }
    }
}

// Thin square bar from a to b (cheaper than addRod for hundreds of lidar beams).
void addSegment(Mesh& m, Vec3 a, Vec3 b, double thickness, const Material& mat) {
    Vec3 d = b - a;
    double length = d.length();
    if (length < 1e-9) {
        return;
    }
    d = d / length;
    Vec3 u;
    Vec3 v;
    basis(d, u, v);
    addOrientedBox(m, (a + b) * 0.5, d, u, v, Vec3{length / 2, thickness, thickness}, mat);
}

// Sensor gizmos: lidar/rangefinder rays (green) with hit points (red), camera frustum, IMU axes triad and
// a small mount marker for every sensor. Sensors of the selected part are drawn brighter.
void addSensorGizmos(Mesh& m, const World& world, int selected, double g) {
    const Material ray{rgb(0.3, 0.95, 0.45), 0.8f, 0.1f};
    const Material hit{rgb(1.0, 0.25, 0.2), 0.9f, 0.2f};
    const Material frame{rgb(0.75, 0.85, 1.0), 0.7f, 0.3f};
    const Material marker{rgb(0.95, 0.9, 0.35), 0.6f, 0.4f};
    for (const auto& sensor : world.sensors) {
        if (sensor.body < 0 || sensor.body >= static_cast<int>(world.bodies.size())) {
            continue;
        }
        bool active = sensor.body == selected;
        SensorPose pose = sensorPose(world, sensor);
        Vec3 x = pose.orientation.rotate({1, 0, 0});
        Vec3 y = pose.orientation.rotate({0, 1, 0});
        Vec3 z = pose.orientation.rotate({0, 0, 1});
        double s = (active ? 1.0 : 0.75) * g;
        if (sensor.type != SensorType::Imu) {
            addOrientedBox(m, pose.position, x, y, z, Vec3{0.025, 0.025, 0.025} * s, marker);
        }
        switch (sensor.type) {
        case SensorType::Imu:
            addRod(m, pose.position, pose.position + x * (0.25 * s), 0.008 * s,
                   {rgb(0.95, 0.3, 0.3), 0.7f, 0.2f});
            addRod(m, pose.position, pose.position + y * (0.25 * s), 0.008 * s,
                   {rgb(0.35, 0.9, 0.4), 0.7f, 0.2f});
            addRod(m, pose.position, pose.position + z * (0.25 * s), 0.008 * s,
                   {rgb(0.35, 0.55, 1.0), 0.7f, 0.2f});
            break;
        case SensorType::Rangefinder:
        case SensorType::Lidar2D: {
            const auto& state = sensor.state;
            if (state.valid && !state.rayEnds.empty()) {
                for (std::size_t k = 0; k < state.rayEnds.size(); ++k) {
                    bool hitPoint = k < state.rayHits.size() && state.rayHits[k];
                    addSegment(m, state.rayOrigin, state.rayEnds[k], 0.0025 * g, ray);
                    if (hitPoint) {
                        Vec3 p = state.rayEnds[k];
                        addOrientedBox(m, p, x, y, z, Vec3{0.018, 0.018, 0.018} * g, hit);
                    }
                }
            } else if (sensor.type == SensorType::Rangefinder) {
                addSegment(m, pose.position, pose.position + x * std::min(sensor.maxRange, 1.0), 0.003 * g,
                           ray);
            } else { // Not sampled yet: show the field of view edges.
                double reach = std::min(sensor.maxRange, 1.0);
                for (int beam : {0, sensor.beams / 2, sensor.beams - 1}) {
                    Vec3 d = pose.orientation.rotate(lidarBeamDirection(sensor, beam));
                    addSegment(m, pose.position, pose.position + d * reach, 0.003 * g, ray);
                }
            }
            break;
        }
        case SensorType::Camera: {
            double depth = 0.35 * s;
            double halfHeight = std::tan(sensor.fov / 2) * depth;
            double halfWidth = halfHeight * sensor.width / std::max(1, sensor.height);
            Vec3 c = pose.position + x * depth;
            Vec3 corners[4] = {c + y * halfHeight + z * halfWidth, c + y * halfHeight - z * halfWidth,
                               c - y * halfHeight - z * halfWidth, c - y * halfHeight + z * halfWidth};
            for (int k = 0; k < 4; ++k) {
                addSegment(m, pose.position, corners[k], 0.004 * s, frame);
                addSegment(m, corners[k], corners[(k + 1) % 4], 0.004 * s, frame);
            }
            addSegment(m, c + y * halfHeight, c + y * (halfHeight * 1.4), 0.004 * s, frame); // "Up" tick.
            break;
        }
        case SensorType::JointEncoder:
        case SensorType::JointVelocity:
        case SensorType::ForceTorque:
        case SensorType::Contact:
            break;
        }
    }
}

// Bodies with visual meshes draw the meshes; others their collision primitive. With showCollision the
// primitives hidden that way are appended as a translucent overlay range.
Mesh buildMesh(const World& world, int selected, bool gizmos, bool showCollision, MeshCache& cache) {
    Mesh mesh;
    std::vector<int> overlay;
    for (int i = 0; i < static_cast<int>(world.bodies.size()); ++i) {
        const Body& b = world.bodies[i];
        int start = static_cast<int>(mesh.v.size());
        float hl = i == selected ? 1.0f : 0.0f;
        Material base = bodyMaterial(b, i);
        bool visual = !b.visuals.empty() && addVisualMeshes(mesh, b, base, hl, cache);
        // A body whose meshes failed to load falls back to its primitive, even if normally hidden.
        if (!visual && (!b.primitiveHidden || !b.visuals.empty())) {
            addPrimitive(mesh, b, base, hl, false);
        } else if (showCollision) {
            overlay.push_back(i);
        }
        mesh.bodies.push_back({start, static_cast<int>(mesh.v.size()) - start});
    }
    mesh.bodyVertices = static_cast<int>(mesh.v.size());
    const Material collision{rgb(0.3, 0.85, 1.0), 0.35f, 0.3f};
    for (int i : overlay) {
        addPrimitive(mesh, world.bodies[i], collision, i == selected ? 1.0f : 0.0f, true);
    }
    mesh.overlayVertices = static_cast<int>(mesh.v.size()) - mesh.bodyVertices;
    mesh.jointVertices = static_cast<int>(mesh.v.size());
    if (gizmos) {
        // Gizmo scale: 1 for scenes of ordinary size, smaller for small (e.g. imported) robots.
        double g = 1;
        if (!world.bodies.empty()) {
            Vec3 lo{1e30, 1e30, 1e30};
            Vec3 hi{-1e30, -1e30, -1e30};
            for (const Body& body : world.bodies) {
                for (Vec3 c : body.vertices()) {
                    lo = {std::min(lo.x, c.x), std::min(lo.y, c.y), std::min(lo.z, c.z)};
                    hi = {std::max(hi.x, c.x), std::max(hi.y, c.y), std::max(hi.z, c.z)};
                }
            }
            g = std::clamp((hi - lo).length() / 2 / 0.6, 0.15, 1.0);
        }
        addRod(mesh, {0, 0, 0}, {1.2 * g, 0, 0}, 0.012 * g, {rgb(0.95, 0.3, 0.3), 0.6f, 0.2f});
        addRod(mesh, {0, 0, 0}, {0, 1.2 * g, 0}, 0.012 * g, {rgb(0.35, 0.9, 0.4), 0.6f, 0.2f});
        addRod(mesh, {0, 0, 0}, {0, 0, 1.2 * g}, 0.012 * g, {rgb(0.35, 0.55, 1.0), 0.6f, 0.2f});
        mesh.jointVertices = static_cast<int>(mesh.v.size());
        for (const auto& j : world.joints) {
            bool active = j.a == selected || j.b == selected;
            Material orange{active ? rgb(1.0, 0.72, 0.3) : rgb(0.95, 0.55, 0.18), active ? 0.7f : 0.45f,
                            0.5f};
            Material stop{active ? rgb(1.0, 0.35, 0.3) : rgb(0.9, 0.25, 0.22), 0.6f, 0.4f};
            Vec3 anchor = world.bodies[j.a].point(j.anchorA);
            if (j.type == JointType::Prismatic) {
                addSliderGizmo(mesh, world, j, orange, stop, g);
                continue;
            }
            addBall(mesh, anchor, (j.type == JointType::Ball ? 0.055 : 0.04) * g, orange);
            if (j.type == JointType::Hinge) {
                Vec3 axis = world.bodies[j.a].orientation.rotate(j.axisA) * (0.24 * g);
                addRod(mesh, anchor - axis, anchor + axis, 0.012 * g, orange);
                if (j.limitsEnabled) {
                    addHingeLimitGizmo(mesh, world, j, anchor, orange, stop, g);
                }
            }
        }
        addSensorGizmos(mesh, world, selected, g);
    }
    return mesh;
}

// Cone from the base center along `direction`.
void addCone(Mesh& m, Vec3 base, Vec3 direction, double length, double radius, const Material& mat) {
    Vec3 d = direction.normalized();
    Vec3 u;
    Vec3 v;
    basis(d, u, v);
    constexpr int S = 16;
    Vec3 tip = base + d * length;
    std::vector<Vec3> cap;
    for (int k = 0; k < S; ++k) {
        double a0 = 2 * robotics::pi * k / S;
        double a1 = 2 * robotics::pi * (k + 1) / S;
        Vec3 n0 = u * std::cos(a0) + v * std::sin(a0);
        Vec3 n1 = u * std::cos(a1) + v * std::sin(a1);
        addTri(m, base + n0 * radius, base + n1 * radius, tip, n0, n1, (n0 + n1).normalized(), mat, 0);
        cap.push_back(base + n0 * radius);
    }
    addPolygon(m, cap, tip, mat, 0);
}

// Tube along the circle arc around `center` in the plane of e1, e2, from angle a0 to a1.
void addArc(Mesh& m, Vec3 center, Vec3 e1, Vec3 e2, double radius, double tube, double a0, double a1,
            const Material& mat) {
    constexpr int sides = 6;
    int segments = std::max(2, static_cast<int>(std::ceil(std::abs(a1 - a0) / (2 * robotics::pi) * 96)));
    Vec3 axis = e1.cross(e2);
    auto ringPoint = [&](int step, int side, Vec3* normal) {
        double a = a0 + (a1 - a0) * step / segments;
        double b = 2 * robotics::pi * side / sides;
        Vec3 radial = e1 * std::cos(a) + e2 * std::sin(a);
        Vec3 n = radial * std::cos(b) + axis * std::sin(b);
        *normal = n;
        return center + radial * radius + n * tube;
    };
    for (int k = 0; k < segments; ++k) {
        for (int s = 0; s < sides; ++s) {
            Vec3 n0, n1, n2, n3;
            Vec3 p0 = ringPoint(k, s, &n0);
            Vec3 p1 = ringPoint(k + 1, s, &n1);
            Vec3 p2 = ringPoint(k + 1, s + 1, &n2);
            Vec3 p3 = ringPoint(k, s + 1, &n3);
            addTri(m, p0, p1, p2, n0, n1, n2, mat, 0);
            addTri(m, p0, p2, p3, n0, n2, n3, mat, 0);
        }
    }
}

// Move arrows and plane squares, or rotate rings, in screen-constant size. `active` is drawn highlighted;
// while a ring is dragged `sweepStart`/`sweep` mark the angle turned so far.
void addTransformGizmo(Mesh& m, const GizmoView& view, Vec3 center, GizmoTool tool, GizmoHandle active,
                       double sweepStart, double sweep) {
    const double unit = view.worldPerPoint(center);
    const simd_float3 axisColors[3] = {rgb(0.95, 0.27, 0.27), rgb(0.33, 0.85, 0.36), rgb(0.3, 0.52, 1.0)};
    const simd_float3 highlight = rgb(1.0, 0.86, 0.25);
    auto material = [&](GizmoHandle handle, int axis, double dim = 1) {
        simd_float3 color = handle == active ? highlight : axisColors[axis] * static_cast<float>(dim);
        return Material{color, 0.92f, 0.1f};
    };
    m.gizmoVertices = static_cast<int>(m.v.size());
    if (tool == GizmoTool::Move) {
        constexpr GizmoHandle arrows[3] = {GizmoHandle::AxisX, GizmoHandle::AxisY, GizmoHandle::AxisZ};
        constexpr GizmoHandle planes[3] = {GizmoHandle::PlaneYZ, GizmoHandle::PlaneXZ, GizmoHandle::PlaneXY};
        for (int k = 0; k < 3; ++k) {
            if (gizmoHandleUsable(view, center, arrows[k])) {
                Vec3 axis = gizmoAxis(k);
                Material mat = material(arrows[k], k);
                double neck = gizmoArrowLength - 20;
                addRod(m, center + axis * (10 * unit), center + axis * (neck * unit), 1.7 * unit, mat);
                addCone(m, center + axis * (neck * unit), axis, 20 * unit, 7 * unit, mat);
            }
            if (gizmoHandleUsable(view, center, planes[k])) {
                Vec3 u = gizmoAxis((k + 1) % 3) * unit;
                Vec3 v = gizmoAxis((k + 2) % 3) * unit;
                addPolygon(m,
                           {center + u * gizmoPlaneStart + v * gizmoPlaneStart,
                            center + u * gizmoPlaneEnd + v * gizmoPlaneStart,
                            center + u * gizmoPlaneEnd + v * gizmoPlaneEnd,
                            center + u * gizmoPlaneStart + v * gizmoPlaneEnd},
                           center, material(planes[k], k, 0.72), 0);
            }
        }
        addBall(m, center, 5 * unit, {rgb(0.93, 0.93, 0.95), 0.9f, 0.1f});
    } else if (tool == GizmoTool::Rotate) {
        constexpr GizmoHandle rings[3] = {GizmoHandle::RingX, GizmoHandle::RingY, GizmoHandle::RingZ};
        for (int k = 0; k < 3; ++k) {
            if (!gizmoHandleUsable(view, center, rings[k])) {
                continue;
            }
            Vec3 e1 = gizmoAxis((k + 1) % 3);
            Vec3 e2 = gizmoAxis((k + 2) % 3);
            double radius = gizmoRingRadius * unit;
            addArc(m, center, e1, e2, radius, (rings[k] == active ? 2.4 : 1.6) * unit, 0, 2 * robotics::pi,
                   material(rings[k], k));
            if (rings[k] == active) {
                const Material swept{highlight, 0.95f, 0.1f};
                addArc(m, center, e1, e2, radius, 3.4 * unit, sweepStart, sweepStart + sweep, swept);
                for (double a : {sweepStart, sweepStart + sweep}) {
                    addRod(m, center, center + (e1 * std::cos(a) + e2 * std::sin(a)) * radius, 1.1 * unit,
                           swept);
                }
            }
        }
        addBall(m, center, 4 * unit, {rgb(0.93, 0.93, 0.95), 0.9f, 0.1f});
    }
}

struct Camera {
    Vec3 eye;
    Vec3 right;
    Vec3 up;
    Vec3 forward;
    double f{}, aspect{1};

    static Camera make(double yaw, double pitch, double distance, Vec3 target, double aspect) {
        Camera c;
        c.eye = target
                + Vec3{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)}
                      * distance;
        c.forward = (target - c.eye).normalized();
        c.right = c.forward.cross({0, 1, 0}).normalized();
        c.up = c.right.cross(c.forward);
        c.f = 1 / std::tan(48 * robotics::pi / 180 / 2);
        c.aspect = aspect;
        return c;
    }

    // Sensor camera: looks along the sensor's +X with +Y up; `fov` is the vertical field of view.
    static Camera fromPose(Vec3 eye, Quat orientation, double fov, double aspect) {
        Camera c;
        c.eye = eye;
        c.forward = orientation.rotate({1, 0, 0});
        c.up = orientation.rotate({0, 1, 0});
        c.right = c.forward.cross(c.up);
        c.f = 1 / std::tan(fov / 2);
        c.aspect = aspect;
        return c;
    }

    simd_float4x4 viewProjection() const {
        auto row = [&](Vec3 axis, double s) {
            return simd_make_float4(static_cast<float>(axis.x * s), static_cast<float>(axis.y * s),
                                    static_cast<float>(axis.z * s), static_cast<float>(-axis.dot(eye) * s));
        };
        // Reversed Z with an infinite far plane: depth = near / view distance.
        return simd_matrix_from_rows(row(right, f / aspect), row(up, f), simd_make_float4(0, 0, 0, 0.03f),
                                     row(forward, 1));
    }

    Vec3 ray(double nx, double ny) const {
        return (forward + right * (nx * aspect / f) + up * (ny / f)).normalized();
    }
};

bool rayTriangle(Vec3 o, Vec3 d, Vec3 a, Vec3 b, Vec3 c, double& t) {
    Vec3 e1 = b - a;
    Vec3 e2 = c - a;
    Vec3 p = d.cross(e2);
    double det = e1.dot(p);
    if (std::abs(det) < 1e-12) {
        return false;
    }
    double inv = 1 / det;
    Vec3 s = o - a;
    double u = s.dot(p) * inv;
    if (u < 0 || u > 1) {
        return false;
    }
    Vec3 q = s.cross(e1);
    double v = d.dot(q) * inv;
    if (v < 0 || u + v > 1) {
        return false;
    }
    t = e2.dot(q) * inv;
    return t > 1e-6;
}

Vec3 vec(simd_float4 v) {
    return {v.x, v.y, v.z};
}

// Overlays stay above the timeline bar at the bottom of the viewport.
constexpr CGFloat kBottomInset = 60;

NSVisualEffectView* pill(NSView* parent, NSRect frame, NSAutoresizingMaskOptions mask) {
    NSVisualEffectView* v = [[NSVisualEffectView alloc] initWithFrame:frame];
    v.material = NSVisualEffectMaterialHUDWindow;
    v.blendingMode = NSVisualEffectBlendingModeWithinWindow;
    v.state = NSVisualEffectStateActive;
    v.wantsLayer = YES;
    v.layer.cornerRadius = 9;
    v.layer.masksToBounds = YES;
    v.autoresizingMask = mask;
    [parent addSubview:v];
    return v;
}

NSTextField* hudLabel(NSView* parent, NSRect frame, CGFloat size, NSFontWeight weight, NSColor* color) {
    NSTextField* t = [NSTextField labelWithString:@""];
    t.frame = frame;
    t.font = [NSFont systemFontOfSize:size weight:weight];
    t.textColor = color;
    t.lineBreakMode = NSLineBreakByTruncatingTail;
    [parent addSubview:t];
    return t;
}
} // namespace

@implementation SceneView {
    id<MTLCommandQueue> _queue;
    id<MTLRenderPipelineState> _meshPipe, _groundPipe, _skyPipe;
    id<MTLRenderPipelineState> _meshPipe1, _groundPipe1, _skyPipe1; // Single-sample (sensor cameras).
    BOOL _singleSample;
    NSVisualEffectView* _pip;
    NSImageView* _pipColor;
    NSImageView* _pipDepth;
    NSTextField* _pipTitle;
    id<MTLDepthStencilState> _dsAlways, _dsWrite, _dsShadow, _dsBehind, _dsTest;
    double _yaw;
    double _pitch;
    double _distance;
    Vec3 _target;
    NSPoint _last;
    NSPoint _press;
    BOOL _dragged;
    BOOL _moving;
    double _moveHeight;
    Vec3 _moveLast;
    NSVisualEffectView* _hud;
    NSTextField* _hudState;
    NSTextField* _hudDetail;
    NSView* _hudDot;
    // Transform gizmo.
    NSVisualEffectView* _toolPill;
    NSSegmentedControl* _toolControl;
    NSPopUpButton* _moveSnapControl;
    NSPopUpButton* _rotateSnapControl;
    NSVisualEffectView* _readoutPill;
    NSTextField* _readoutText;
    NSTrackingArea* _tracking;
    GizmoHandle _activeHandle; // Handle being dragged; None otherwise.
    GizmoDrag _gizmoDrag;
    GizmoView _dragView; // Camera frozen for the drag, so a following camera cannot feed back into it.
    BOOL _gizmoStarted;  // The first real motion happened (one undo step per drag, none for a plain click).
    double _sweepStart;  // Ring drag: mouse angle when the drag began.
    double _sweep;       // Ring drag: applied angle.
    BOOL _rightDragged;
    BOOL _contextClick;          // Control-click, treated as a right click.
    const World* _worldOverride; // Rendered instead of the host's world (library thumbnails).
    NSVisualEffectView* _toast;
    NSTextField* _toastText;
    NSUInteger _toastToken;
    NSTextField* _empty;
    Vec3 _followOffset; // Camera target relative to the followed chassis.
    MeshCache _meshes;
    std::set<std::string> _reportedMeshErrors;
}

- (instancetype)initWithFrame:(NSRect)frame {
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (!device) {
        throw std::runtime_error("Metal is not available on this Mac");
    }
    self = [super initWithFrame:frame device:device];
    if (!self) {
        return nil;
    }
    self.colorPixelFormat = MTLPixelFormatBGRA8Unorm;
    self.depthStencilPixelFormat = MTLPixelFormatDepth32Float_Stencil8;
    self.sampleCount = 4;
    self.clearDepth = 0;
    self.clearStencil = 0;
    self.clearColor = MTLClearColorMake(0.05, 0.07, 0.1, 1);
    self.enableSetNeedsDisplay = YES;
    self.paused = YES;
    self.delegate = self;
    _queue = [device newCommandQueue];
    NSError* error = nil;
    id<MTLLibrary> library = [device newLibraryWithSource:@(kShaders) options:nil error:&error];
    if (!library) {
        throw std::runtime_error(std::string("Shader compilation failed: ")
                                 + error.localizedDescription.UTF8String);
    }
    auto pipeline = [&](NSString* vs, NSString* fs, BOOL blend, NSUInteger samples = 4) {
        MTLRenderPipelineDescriptor* d = [[MTLRenderPipelineDescriptor alloc] init];
        d.vertexFunction = [library newFunctionWithName:vs];
        d.fragmentFunction = [library newFunctionWithName:fs];
        d.colorAttachments[0].pixelFormat = self.colorPixelFormat;
        d.depthAttachmentPixelFormat = self.depthStencilPixelFormat;
        d.stencilAttachmentPixelFormat = self.depthStencilPixelFormat;
        d.rasterSampleCount = samples;
        if (blend) {
            auto* c = d.colorAttachments[0];
            c.blendingEnabled = YES;
            c.sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
            c.destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
            c.sourceAlphaBlendFactor = MTLBlendFactorOne;
            c.destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        }
        NSError* e = nil;
        id<MTLRenderPipelineState> state = [device newRenderPipelineStateWithDescriptor:d error:&e];
        if (!state) {
            throw std::runtime_error(std::string("Pipeline creation failed: ")
                                     + e.localizedDescription.UTF8String);
        }
        return state;
    };
    _meshPipe = pipeline(@"meshVS", @"meshFS", YES);
    _groundPipe = pipeline(@"groundVS", @"groundFS", NO);
    _skyPipe = pipeline(@"skyVS", @"skyFS", NO);
    _meshPipe1 = pipeline(@"meshVS", @"meshFS", YES, 1);
    _groundPipe1 = pipeline(@"groundVS", @"groundFS", NO, 1);
    _skyPipe1 = pipeline(@"skyVS", @"skyFS", NO, 1);
    auto depth = [&](MTLCompareFunction compare, BOOL write, BOOL stencil) {
        MTLDepthStencilDescriptor* d = [[MTLDepthStencilDescriptor alloc] init];
        d.depthCompareFunction = compare;
        d.depthWriteEnabled = write;
        if (stencil) { // Each shadow pixel darkens once even where shadows overlap.
            MTLStencilDescriptor* s = [[MTLStencilDescriptor alloc] init];
            s.stencilCompareFunction = MTLCompareFunctionEqual;
            s.depthStencilPassOperation = MTLStencilOperationIncrementClamp;
            d.frontFaceStencil = s;
            d.backFaceStencil = s;
        }
        return [device newDepthStencilStateWithDescriptor:d];
    };
    _dsAlways = depth(MTLCompareFunctionAlways, NO, NO);
    _dsWrite = depth(MTLCompareFunctionGreater, YES, NO);
    _dsShadow = depth(MTLCompareFunctionGreaterEqual, NO, YES);
    _dsBehind = depth(MTLCompareFunctionLess, NO, NO);
    _dsTest = depth(MTLCompareFunctionGreater, NO, NO);
    [self buildOverlays];
    [self homeCamera];
    return self;
}

- (BOOL)acceptsFirstResponder {
    return YES;
}

- (BOOL)acceptsFirstMouse:(NSEvent*)event {
    (void)event;
    return YES;
}

#pragma mark Overlays

- (void)buildOverlays {
    NSRect b = self.bounds;
    _hud = pill(self, NSMakeRect(14, b.size.height - 66, 330, 52), NSViewMaxXMargin | NSViewMinYMargin);
    _hudDot = [[NSView alloc] initWithFrame:NSMakeRect(14, 32, 9, 9)];
    _hudDot.wantsLayer = YES;
    _hudDot.layer.cornerRadius = 4.5;
    [_hud addSubview:_hudDot];
    _hudState = hudLabel(_hud, NSMakeRect(30, 27, 290, 18), 13, NSFontWeightSemibold, NSColor.labelColor);
    _hudDetail =
        hudLabel(_hud, NSMakeRect(14, 7, 306, 17), 11.5, NSFontWeightRegular, NSColor.secondaryLabelColor);
    _hudDetail.font = [NSFont monospacedDigitSystemFontOfSize:11.5 weight:NSFontWeightRegular];

    NSSegmentedControl* views =
        [NSSegmentedControl segmentedControlWithLabels:@[ @"Perspective", @"Top", @"Front", @"Side" ]
                                          trackingMode:NSSegmentSwitchTrackingMomentary
                                                target:self
                                                action:@selector(presetClicked:)];
    views.controlSize = NSControlSizeSmall;
    views.font = [NSFont systemFontOfSize:11];
    [views sizeToFit];
    NSVisualEffectView* viewPill = pill(self,
                                        NSMakeRect(b.size.width - views.frame.size.width - 30,
                                                   b.size.height - 46, views.frame.size.width + 16, 32),
                                        NSViewMinXMargin | NSViewMinYMargin);
    views.frame =
        NSMakeRect(8, (32 - views.frame.size.height) / 2, views.frame.size.width, views.frame.size.height);
    [viewPill addSubview:views];
    views.toolTip = @"Camera presets. Press F to re-center on the robot.";

    [self buildGizmoOverlay];

    NSDictionary* key = @{
        NSFontAttributeName : [NSFont systemFontOfSize:11 weight:NSFontWeightSemibold],
        NSForegroundColorAttributeName : NSColor.labelColor
    };
    NSDictionary* text = @{
        NSFontAttributeName : [NSFont systemFontOfSize:11],
        NSForegroundColorAttributeName : NSColor.secondaryLabelColor
    };
    NSMutableAttributedString* hints = [[NSMutableAttributedString alloc] init];
    NSArray* items = @[
        @[ @"Drag", @"orbit" ], @[ @"⇧-drag", @"pan" ], @[ @"Scroll", @"zoom" ],
        @[ @"1 2 3", @"select/move/rotate" ], @[ @"F", @"focus" ], @[ @"Space", @"run/pause" ],
        @[ @"WASD", @"drive" ], @[ @"X", @"brake" ]
    ];
    for (NSArray* item in items) {
        if (hints.length) {
            [hints appendAttributedString:[[NSAttributedString alloc] initWithString:@"   ·   "
                                                                          attributes:text]];
        }
        [hints appendAttributedString:[[NSAttributedString alloc] initWithString:item[0] attributes:key]];
        [hints appendAttributedString:[[NSAttributedString alloc]
                                          initWithString:[@" " stringByAppendingString:item[1]]
                                              attributes:text]];
    }
    NSTextField* hintLabel = [NSTextField labelWithAttributedString:hints];
    [hintLabel sizeToFit];
    NSVisualEffectView* hintPill =
        pill(self, NSMakeRect(14, kBottomInset, hintLabel.frame.size.width + 24, 28),
             NSViewMaxXMargin | NSViewMaxYMargin);
    hintLabel.frame = NSMakeRect(12, (28 - hintLabel.frame.size.height) / 2, hintLabel.frame.size.width,
                                 hintLabel.frame.size.height);
    [hintPill addSubview:hintLabel];

    _toast = pill(self, NSMakeRect((b.size.width - 420) / 2, kBottomInset + 40, 420, 34),
                  NSViewMinXMargin | NSViewMaxXMargin | NSViewMaxYMargin);
    _toast.hidden = YES;
    _toastText = hudLabel(_toast, NSMakeRect(14, 8, 392, 18), 12.5, NSFontWeightMedium, NSColor.labelColor);
    _toastText.alignment = NSTextAlignmentCenter;

    // Sensor camera picture-in-picture: RGB and depth side by side (top right).
    _pip = pill(self, NSMakeRect(b.size.width - 362, b.size.height - 214, 350, 160),
                NSViewMinXMargin | NSViewMinYMargin);
    _pip.hidden = YES;
    _pipTitle = hudLabel(_pip, NSMakeRect(10, 136, 330, 18), 11, NSFontWeightSemibold, NSColor.labelColor);
    _pipColor = [[NSImageView alloc] initWithFrame:NSMakeRect(8, 8, 164, 124)];
    _pipDepth = [[NSImageView alloc] initWithFrame:NSMakeRect(178, 8, 164, 124)];
    for (NSImageView* view in @[ _pipColor, _pipDepth ]) {
        view.imageScaling = NSImageScaleProportionallyUpOrDown;
        view.wantsLayer = YES;
        view.layer.magnificationFilter = kCAFilterNearest;
        [_pip addSubview:view];
    }
    _pipColor.toolTip = @"Camera sensor: color image";
    _pipDepth.toolTip = @"Camera sensor: depth (near = bright)";

    _empty =
        [NSTextField wrappingLabelWithString:@"The scene is empty\nAdd your first part with the + Add button "
                                             @"in the toolbar,\nor open a ready-made robot from Examples."];
    _empty.alignment = NSTextAlignmentCenter;
    _empty.font = [NSFont systemFontOfSize:14];
    _empty.textColor = NSColor.secondaryLabelColor;
    _empty.frame = NSMakeRect((b.size.width - 460) / 2, b.size.height / 2 - 40, 460, 80);
    _empty.autoresizingMask = NSViewMinXMargin | NSViewMaxXMargin | NSViewMinYMargin | NSViewMaxYMargin;
    _empty.hidden = YES;
    [self addSubview:_empty];
}

// Tool switch, snap settings and Drop to Floor, below the status pill.
- (void)buildGizmoOverlay {
    NSRect b = self.bounds;
    _toolPill = pill(self, NSMakeRect(14, b.size.height - 110, 486, 36), NSViewMaxXMargin | NSViewMinYMargin);
    _toolControl = [NSSegmentedControl segmentedControlWithLabels:@[ @"Select", @"Move", @"Rotate" ]
                                                     trackingMode:NSSegmentSwitchTrackingSelectOne
                                                           target:self
                                                           action:@selector(toolClicked:)];
    _toolControl.controlSize = NSControlSizeSmall;
    _toolControl.font = [NSFont systemFontOfSize:11];
    _toolControl.selectedSegment = 0;
    [_toolControl setImage:[NSImage imageWithSystemSymbolName:@"cursorarrow"
                                     accessibilityDescription:@"Select"]
                forSegment:0];
    [_toolControl setImage:[NSImage imageWithSystemSymbolName:@"move.3d" accessibilityDescription:@"Move"]
                forSegment:1];
    [_toolControl setImage:[NSImage imageWithSystemSymbolName:@"rotate.3d" accessibilityDescription:@"Rotate"]
                forSegment:2];
    [_toolControl sizeToFit];
    _toolControl.frame = NSMakeRect(8, (36 - _toolControl.frame.size.height) / 2,
                                    _toolControl.frame.size.width, _toolControl.frame.size.height);
    _toolControl.toolTip =
        @"Select (1): click parts. Move (2) and Rotate (3): drag the handles on the selected "
        @"part while the simulation is paused.";
    [_toolPill addSubview:_toolControl];
    CGFloat x = NSMaxX(_toolControl.frame) + 10;
    NSTextField* snap =
        hudLabel(_toolPill, NSMakeRect(x, 10, 34, 15), 11, NSFontWeightRegular, NSColor.secondaryLabelColor);
    snap.stringValue = @"Snap";
    x += 34;
    auto popup = [&](NSArray<NSString*>* titles, CGFloat width, SEL action, NSString* tip) {
        NSPopUpButton* button = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(x, 7, width, 22)
                                                           pullsDown:NO];
        button.controlSize = NSControlSizeSmall;
        button.font = [NSFont systemFontOfSize:11];
        [button addItemsWithTitles:titles];
        button.target = self;
        button.action = action;
        button.toolTip = tip;
        [_toolPill addSubview:button];
        x += width + 4;
        return button;
    };
    _moveSnapControl =
        popup(@[ @"Grid off", @"1 cm", @"5 cm", @"10 cm", @"25 cm" ], 78, @selector(snapChanged:),
              @"Grid step for the Move tool. Hold ⌘ while dragging to invert snapping.");
    _rotateSnapControl = popup(@[ @"Angle off", @"5°", @"15°", @"45°" ], 84, @selector(snapChanged:),
                               @"Angle step for the Rotate tool. Hold ⌘ while dragging to invert snapping.");
    NSButton* drop = [NSButton buttonWithTitle:@"Drop" target:self action:@selector(dropClicked:)];
    drop.controlSize = NSControlSizeSmall;
    drop.font = [NSFont systemFontOfSize:11];
    drop.bezelStyle = NSBezelStyleRounded;
    drop.image = [NSImage imageWithSystemSymbolName:@"arrow.down.to.line" accessibilityDescription:nil];
    drop.imagePosition = NSImageLeading;
    drop.toolTip = @"Drop to Floor: place the selected part and everything attached to it on the ground.";
    [drop sizeToFit];
    drop.frame =
        NSMakeRect(x + 2, (36 - drop.frame.size.height) / 2, drop.frame.size.width, drop.frame.size.height);
    [_toolPill addSubview:drop];
    NSRect frame = _toolPill.frame;
    frame.size.width = NSMaxX(drop.frame) + 8;
    _toolPill.frame = frame;

    _readoutPill = pill(self, NSMakeRect(0, 0, 220, 26), NSViewNotSizable);
    _readoutPill.hidden = YES;
    _readoutText =
        hudLabel(_readoutPill, NSMakeRect(10, 5, 200, 16), 12, NSFontWeightSemibold, NSColor.labelColor);
    _readoutText.font = [NSFont monospacedDigitSystemFontOfSize:12 weight:NSFontWeightSemibold];
}

- (void)toolClicked:(NSSegmentedControl*)sender {
    self.gizmoTool = static_cast<GizmoTool>(sender.selectedSegment);
    [self.window makeFirstResponder:self];
}

- (void)setGizmoTool:(GizmoTool)tool {
    _gizmoTool = tool;
    _toolControl.selectedSegment = static_cast<NSInteger>(tool);
    _hoveredHandle = GizmoHandle::None;
    self.needsDisplay = YES;
    if (tool != GizmoTool::Select && ![self.host sceneGizmoAvailable]) {
        [self showToast:@"Select a part and pause the simulation to use the handles." error:NO];
    }
}

- (void)snapChanged:(id)sender {
    constexpr double moves[] = {0, 0.01, 0.05, 0.1, 0.25};
    constexpr double turns[] = {0, 5, 15, 45};
    _moveSnap = moves[_moveSnapControl.indexOfSelectedItem];
    _rotateSnap = turns[_rotateSnapControl.indexOfSelectedItem] * robotics::pi / 180;
    (void)sender;
    [self.window makeFirstResponder:self];
}

- (void)setMoveSnap:(double)step {
    _moveSnap = step;
    constexpr double moves[] = {0, 0.01, 0.05, 0.1, 0.25};
    for (NSInteger k = 0; k < 5; ++k) {
        if (std::abs(moves[k] - step) < 1e-9) {
            [_moveSnapControl selectItemAtIndex:k];
        }
    }
}

- (void)setRotateSnap:(double)step {
    _rotateSnap = step;
    constexpr double turns[] = {0, 5, 15, 45};
    for (NSInteger k = 0; k < 4; ++k) {
        if (std::abs(turns[k] * robotics::pi / 180 - step) < 1e-9) {
            [_rotateSnapControl selectItemAtIndex:k];
        }
    }
}

- (void)dropClicked:(id)sender {
    (void)sender;
    [self.host sceneDropToFloor];
    [self.window makeFirstResponder:self];
}

- (NSString*)gizmoReadout {
    return _readoutPill.hidden ? nil : _readoutText.stringValue;
}

- (void)presetClicked:(NSSegmentedControl*)sender {
    [self viewPreset:sender.selectedSegment];
    [self.window makeFirstResponder:self];
}

- (void)setStatus:(NSString*)state detail:(NSString*)detail running:(BOOL)running {
    _hudState.stringValue = state;
    _hudDetail.stringValue = detail;
    _hudDot.layer.backgroundColor = (running ? NSColor.systemGreenColor : NSColor.systemOrangeColor).CGColor;
}

- (void)showToast:(NSString*)text error:(BOOL)error {
    _toastText.stringValue = text;
    _toastText.textColor =
        error ? [NSColor colorWithCalibratedRed:1 green:0.55 blue:0.5 alpha:1] : NSColor.labelColor;
    CGFloat width = std::min<CGFloat>(
        self.bounds.size.width - 40,
        std::max<CGFloat>(260,
                          [text sizeWithAttributes:@{NSFontAttributeName : _toastText.font}].width + 40));
    _toast.frame = NSMakeRect((self.bounds.size.width - width) / 2, kBottomInset + 40, width, 34);
    _toastText.frame = NSMakeRect(12, 8, width - 24, 18);
    _toast.hidden = NO;
    _toast.alphaValue = 1;
    NSUInteger token = ++_toastToken;
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, static_cast<int64_t>((error ? 4.5 : 2.5) * NSEC_PER_SEC)),
                   dispatch_get_main_queue(), ^{
                       if (token != self->_toastToken) {
                           return;
                       }
                       [NSAnimationContext
                           runAnimationGroup:^(NSAnimationContext* c) {
                               c.duration = 0.3;
                               self->_toast.animator.alphaValue = 0;
                           }
                           completionHandler:^{
                               if (token == self->_toastToken) {
                                   self->_toast.hidden = YES;
                               }
                           }];
                   });
}

#pragma mark Camera

// World-space bounds of all collision shapes (box corners; conservative for spheres and cylinders).
- (BOOL)worldBoundsMin:(Vec3*)lo max:(Vec3*)hi {
    const World* world = [self.host sceneWorld];
    if (!world || world->bodies.empty()) {
        return NO;
    }
    *lo = {1e30, 1e30, 1e30};
    *hi = {-1e30, -1e30, -1e30};
    auto grow = [&](Vec3 c) {
        *lo = {std::min(lo->x, c.x), std::min(lo->y, c.y), std::min(lo->z, c.z)};
        *hi = {std::max(hi->x, c.x), std::max(hi->y, c.y), std::max(hi->z, c.z)};
    };
    for (const Body& body : world->bodies) {
        for (Vec3 c : body.vertices()) {
            grow(c);
        }
        for (const VisualMesh& visual : body.visuals) { // Corners of each visual mesh's bounding box.
            auto model = _meshes.find(visual.path);
            if (!model || model->empty()) {
                continue;
            }
            MeshBounds bounds = model->bounds();
            for (int k = 0; k < 8; ++k) {
                Vec3 corner{(k & 1) ? bounds.max.x : bounds.min.x, (k & 2) ? bounds.max.y : bounds.min.y,
                            (k & 4) ? bounds.max.z : bounds.min.z};
                grow(body.point(visual.position
                                + visual.orientation.rotate(componentProduct(corner, visual.scale))));
            }
        }
    }
    return YES;
}

- (void)homeCamera {
    _yaw = 0.8;
    _pitch = 0.42;
    _distance = 6.5;
    _target = {0, 0.8, 0};
    _followOffset = {0, 0.15, 0};
    _follow = YES;
    Vec3 lo;
    Vec3 hi;
    // Small robots (e.g. imported URDF models) would be a few pixels wide at the default distance.
    if ([self worldBoundsMin:&lo max:&hi] && (hi - lo).length() / 2 < 0.6) {
        [self frameWorld];
    }
    self.needsDisplay = YES;
}

- (void)frameWorld {
    Vec3 lo;
    Vec3 hi;
    if (![self worldBoundsMin:&lo max:&hi]) {
        [self homeCamera];
        return;
    }
    double radius = std::max((hi - lo).length() / 2, 0.15);
    _target = (lo + hi) * 0.5;
    _distance = std::clamp(radius * 3.6, 0.8, 150.0);
    _follow = YES;
    _followOffset = {0, 0.15, 0};
    if (const World* world = [self.host sceneWorld]) {
        for (const auto& joint : world->joints) {
            if (isWheelJoint(*world, joint)) {
                _followOffset = _target - world->bodies[joint.a].position;
                break;
            }
        }
    }
    self.needsDisplay = YES;
}

- (void)setShowCollision:(BOOL)showCollision {
    _showCollision = showCollision;
    self.needsDisplay = YES;
}

- (void)reloadMeshes {
    _meshes.clear();
    _reportedMeshErrors.clear();
    self.needsDisplay = YES;
}

- (void)reportMeshErrors:(const std::vector<std::string>&)errors {
    for (const auto& error : errors) {
        if (_reportedMeshErrors.insert(error).second) {
            NSString* text = @(error.c_str());
            NSString* file = [[text componentsSeparatedByString:@"\n"].firstObject lastPathComponent];
            [self
                showToast:[NSString stringWithFormat:
                                        @"Can't show 3D model %@ — drawing the collision shape instead", file]
                    error:YES];
        }
    }
}

- (void)viewPreset:(NSInteger)preset {
    switch (preset) {
    case 1:
        _yaw = 0;
        _pitch = 1.53;
        break;
    case 2:
        _yaw = robotics::pi / 2;
        _pitch = 0.12;
        break;
    case 3:
        _yaw = 0;
        _pitch = 0.12;
        break;
    default:
        _yaw = 0.8;
        _pitch = 0.42;
        break;
    }
    self.needsDisplay = YES;
}

- (Camera)camera {
    const World* world = [self.host sceneWorld];
    if (_follow && world && _activeHandle == GizmoHandle::None) {
        for (const auto& joint : world->joints) {
            if (isWheelJoint(*world, joint)) {
                _target = world->bodies[joint.a].position + _followOffset;
                break;
            }
        }
    }
    double aspect = self.bounds.size.height > 0 ? self.bounds.size.width / self.bounds.size.height : 1;
    return Camera::make(_yaw, _pitch, _distance, _target, aspect);
}

- (Vec3)cameraTarget {
    return _target;
}

- (GizmoView)gizmoView {
    Camera camera = [self camera];
    GizmoView view;
    view.eye = camera.eye;
    view.right = camera.right;
    view.up = camera.up;
    view.forward = camera.forward;
    view.focal = camera.f;
    view.width = self.bounds.size.width;
    view.height = self.bounds.size.height;
    return view;
}

- (Vec3)rayAt:(NSPoint)p camera:(const Camera&)camera {
    return camera.ray(2 * p.x / self.bounds.size.width - 1, 2 * p.y / self.bounds.size.height - 1);
}

#pragma mark Rendering

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {
    (void)view;
    (void)size;
}

- (void)encode:(id<MTLRenderCommandEncoder>)encoder {
    [self encode:encoder camera:[self camera] gizmos:YES];
}

- (void)encode:(id<MTLRenderCommandEncoder>)encoder camera:(const Camera&)camera gizmos:(BOOL)gizmos {
    const World* world = _worldOverride ? _worldOverride : [self.host sceneWorld];
    int selected = gizmos ? [self.host sceneSelected] : -1;
    id<MTLRenderPipelineState> skyPipe = _singleSample ? _skyPipe1 : _skyPipe;
    id<MTLRenderPipelineState> groundPipe = _singleSample ? _groundPipe1 : _groundPipe;
    id<MTLRenderPipelineState> meshPipe = _singleSample ? _meshPipe1 : _meshPipe;
    Uniforms u{};
    u.vp = camera.viewProjection();
    u.eye = f4(camera.eye, 1);
    u.light = f4(Vec3{-0.45, 1, 0.55}.normalized(), 0);
    u.tint = simd_make_float4(0.35f, 0.85f, 1.0f, 1);
    u.sky0 = simd_make_float4(0.115f, 0.14f, 0.18f, 1);
    u.sky1 = simd_make_float4(0.035f, 0.05f, 0.075f, 1);
    [encoder setFrontFacingWinding:MTLWindingCounterClockwise];
    [encoder setRenderPipelineState:skyPipe];
    [encoder setDepthStencilState:_dsAlways];
    [encoder setFragmentBytes:&u length:sizeof u atIndex:1];
    [encoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [encoder setRenderPipelineState:groundPipe];
    [encoder setDepthStencilState:_dsWrite];
    [encoder setVertexBytes:&u length:sizeof u atIndex:1];
    [encoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:6];
    if (gizmos) {
        _empty.hidden = !world || !world->bodies.empty();
    }
    if (!world) {
        return;
    }
    Mesh mesh = buildMesh(*world, selected, gizmos, gizmos && _showCollision, _meshes);
    int jointEnd = static_cast<int>(mesh.v.size());
    if (gizmos && [self gizmoVisible]) {
        Vec3 center = world->bodies[selected].position;
        GizmoHandle shown = _activeHandle != GizmoHandle::None ? _activeHandle : _hoveredHandle;
        addTransformGizmo(mesh, [self gizmoView], center, _gizmoTool, shown, _sweepStart, _sweep);
    }
    if (!mesh.meshErrors.empty()) {
        std::vector<std::string> errors = mesh.meshErrors;
        dispatch_async(dispatch_get_main_queue(), ^{
            [self reportMeshErrors:errors];
        });
    }
    if (mesh.v.empty()) {
        return;
    }
    id<MTLBuffer> buffer = [self.device newBufferWithBytes:mesh.v.data()
                                                    length:mesh.v.size() * sizeof(GPUVertex)
                                                   options:MTLResourceStorageModeShared];
    [encoder setRenderPipelineState:meshPipe];
    [encoder setVertexBuffer:buffer offset:0 atIndex:0];
    auto draw = [&](float mode, float alpha, int start, int count) {
        if (count <= 0) {
            return;
        }
        u.mode = simd_make_float4(mode, alpha, 0, 0);
        [encoder setVertexBytes:&u length:sizeof u atIndex:1];
        [encoder setFragmentBytes:&u length:sizeof u atIndex:1];
        [encoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:start vertexCount:count];
    };
    [encoder setDepthStencilState:_dsShadow];
    [encoder setStencilReferenceValue:0];
    draw(1, 0, 0, mesh.bodyVertices);
    [encoder setDepthStencilState:_dsWrite];
    draw(0, 0, 0, mesh.bodyVertices);
    draw(0, 0, mesh.jointVertices, jointEnd - mesh.jointVertices);
    if (mesh.overlayVertices > 0) { // Collision shapes: shaded, translucent, depth-tested, front faces only.
        [encoder setDepthStencilState:_dsTest];
        [encoder setCullMode:MTLCullModeBack];
        draw(3, 0.4f, mesh.bodyVertices, mesh.overlayVertices);
        [encoder setCullMode:MTLCullModeNone];
    }
    // X-ray pass: joint markers stay faintly visible through parts; solid geometry never is.
    [encoder setDepthStencilState:_dsBehind];
    [encoder setCullMode:MTLCullModeBack];
    u.tint = simd_make_float4(1.0f, 0.62f, 0.2f, 1);
    draw(2, 0.3f, mesh.jointVertices, jointEnd - mesh.jointVertices);
    [encoder setCullMode:MTLCullModeNone];
    if (mesh.gizmoVertices >= 0) {
        [encoder setDepthStencilState:_dsWrite];
        draw(4, 0, mesh.gizmoVertices, static_cast<int>(mesh.v.size()) - mesh.gizmoVertices);
    }
}

- (void)drawInMTKView:(MTKView*)view {
    MTLRenderPassDescriptor* pass = view.currentRenderPassDescriptor;
    id<CAMetalDrawable> drawable = view.currentDrawable;
    if (!pass || !drawable) {
        return;
    }
    id<MTLCommandBuffer> commands = [_queue commandBuffer];
    id<MTLRenderCommandEncoder> encoder = [commands renderCommandEncoderWithDescriptor:pass];
    [self encode:encoder];
    [encoder endEncoding];
    [commands presentDrawable:drawable];
    [commands commit];
}

- (NSBitmapImageRep*)snapshot {
    CGFloat scale = self.window ? self.window.backingScaleFactor : 2;
    NSUInteger w = static_cast<NSUInteger>(self.bounds.size.width * scale),
               h = static_cast<NSUInteger>(self.bounds.size.height * scale);
    if (w == 0 || h == 0) {
        return nil;
    }
    auto texture = [&](MTLPixelFormat format, NSUInteger samples) {
        MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:format
                                                                                     width:w
                                                                                    height:h
                                                                                 mipmapped:NO];
        d.textureType = samples > 1 ? MTLTextureType2DMultisample : MTLTextureType2D;
        d.sampleCount = samples;
        d.usage = MTLTextureUsageRenderTarget;
        d.storageMode = MTLStorageModePrivate;
        return [self.device newTextureWithDescriptor:d];
    };
    id<MTLTexture> color = texture(self.colorPixelFormat, 4);
    id<MTLTexture> resolved = texture(self.colorPixelFormat, 1);
    id<MTLTexture> depth = texture(self.depthStencilPixelFormat, 4);
    MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = color;
    pass.colorAttachments[0].resolveTexture = resolved;
    pass.colorAttachments[0].loadAction = MTLLoadActionClear;
    pass.colorAttachments[0].storeAction = MTLStoreActionMultisampleResolve;
    pass.colorAttachments[0].clearColor = self.clearColor;
    pass.depthAttachment.texture = depth;
    pass.depthAttachment.clearDepth = 0;
    pass.depthAttachment.loadAction = MTLLoadActionClear;
    pass.depthAttachment.storeAction = MTLStoreActionDontCare;
    pass.stencilAttachment.texture = depth;
    pass.stencilAttachment.clearStencil = 0;
    pass.stencilAttachment.loadAction = MTLLoadActionClear;
    pass.stencilAttachment.storeAction = MTLStoreActionDontCare;
    id<MTLBuffer> pixels = [self.device newBufferWithLength:w * h * 4 options:MTLResourceStorageModeShared];
    id<MTLCommandBuffer> commands = [_queue commandBuffer];
    id<MTLRenderCommandEncoder> encoder = [commands renderCommandEncoderWithDescriptor:pass];
    [self encode:encoder];
    [encoder endEncoding];
    id<MTLBlitCommandEncoder> blit = [commands blitCommandEncoder];
    [blit copyFromTexture:resolved
                     sourceSlice:0
                     sourceLevel:0
                    sourceOrigin:MTLOriginMake(0, 0, 0)
                      sourceSize:MTLSizeMake(w, h, 1)
                        toBuffer:pixels
               destinationOffset:0
          destinationBytesPerRow:w * 4
        destinationBytesPerImage:w * h * 4];
    [blit endEncoding];
    [commands commit];
    [commands waitUntilCompleted];
    NSBitmapImageRep* bitmap =
        [[NSBitmapImageRep alloc] initWithBitmapDataPlanes:nullptr
                                                pixelsWide:static_cast<NSInteger>(w)
                                                pixelsHigh:static_cast<NSInteger>(h)
                                             bitsPerSample:8
                                           samplesPerPixel:4
                                                  hasAlpha:YES
                                                  isPlanar:NO
                                            colorSpaceName:NSDeviceRGBColorSpace
                                               bytesPerRow:static_cast<NSInteger>(w * 4)
                                              bitsPerPixel:32];
    const unsigned char* src = static_cast<const unsigned char*>(pixels.contents);
    unsigned char* dst = bitmap.bitmapData;
    for (NSUInteger i = 0; i < w * h; ++i) {
        dst[i * 4] = src[i * 4 + 2];
        dst[i * 4 + 1] = src[i * 4 + 1];
        dst[i * 4 + 2] = src[i * 4];
        dst[i * 4 + 3] = 255;
    }
    bitmap.size = self.bounds.size;
    return bitmap;
}

#pragma mark Sensor cameras

- (BOOL)renderSensorCamera:(const robotics::Sensor&)sensor
                     color:(std::vector<unsigned char>&)rgba
                     depth:(std::vector<float>&)depth {
    const World* world = [self.host sceneWorld];
    if (!world || sensor.type != SensorType::Camera || sensor.width < 1 || sensor.height < 1) {
        return NO;
    }
    SensorPose pose = sensorPose(*world, sensor);
    Camera camera = Camera::fromPose(pose.position, pose.orientation, sensor.fov,
                                     static_cast<double>(sensor.width) / static_cast<double>(sensor.height));
    return [self renderCamera:camera
                        width:sensor.width
                       height:sensor.height
                     maxRange:sensor.maxRange
                        color:rgba
                        depth:depth];
}

// Single-sample render of the viewport's world (or _worldOverride) without gizmos.
- (BOOL)renderCamera:(const Camera&)camera
               width:(int)width
              height:(int)height
            maxRange:(double)maxRange
               color:(std::vector<unsigned char>&)rgba
               depth:(std::vector<float>&)depth {
    const auto w = static_cast<NSUInteger>(width);
    const auto h = static_cast<NSUInteger>(height);
    auto texture = [&](MTLPixelFormat format) {
        MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:format
                                                                                     width:w
                                                                                    height:h
                                                                                 mipmapped:NO];
        d.usage = MTLTextureUsageRenderTarget;
        d.storageMode = MTLStorageModePrivate;
        return [self.device newTextureWithDescriptor:d];
    };
    id<MTLTexture> color = texture(self.colorPixelFormat);
    id<MTLTexture> depthTexture = texture(self.depthStencilPixelFormat);
    MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = color;
    pass.colorAttachments[0].loadAction = MTLLoadActionClear;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    pass.colorAttachments[0].clearColor = self.clearColor;
    pass.depthAttachment.texture = depthTexture;
    pass.depthAttachment.clearDepth = 0;
    pass.depthAttachment.loadAction = MTLLoadActionClear;
    pass.depthAttachment.storeAction = MTLStoreActionStore;
    pass.stencilAttachment.texture = depthTexture;
    pass.stencilAttachment.clearStencil = 0;
    pass.stencilAttachment.loadAction = MTLLoadActionClear;
    pass.stencilAttachment.storeAction = MTLStoreActionDontCare;
    id<MTLBuffer> pixels = [self.device newBufferWithLength:w * h * 4 options:MTLResourceStorageModeShared];
    id<MTLBuffer> depths = [self.device newBufferWithLength:w * h * 4 options:MTLResourceStorageModeShared];
    id<MTLCommandBuffer> commands = [_queue commandBuffer];
    id<MTLRenderCommandEncoder> encoder = [commands renderCommandEncoderWithDescriptor:pass];
    _singleSample = YES;
    [self encode:encoder camera:camera gizmos:NO];
    _singleSample = NO;
    [encoder endEncoding];
    id<MTLBlitCommandEncoder> blit = [commands blitCommandEncoder];
    [blit copyFromTexture:color
                     sourceSlice:0
                     sourceLevel:0
                    sourceOrigin:MTLOriginMake(0, 0, 0)
                      sourceSize:MTLSizeMake(w, h, 1)
                        toBuffer:pixels
               destinationOffset:0
          destinationBytesPerRow:w * 4
        destinationBytesPerImage:w * h * 4];
    [blit copyFromTexture:depthTexture
                     sourceSlice:0
                     sourceLevel:0
                    sourceOrigin:MTLOriginMake(0, 0, 0)
                      sourceSize:MTLSizeMake(w, h, 1)
                        toBuffer:depths
               destinationOffset:0
          destinationBytesPerRow:w * 4
        destinationBytesPerImage:w * h * 4
                         options:MTLBlitOptionDepthFromDepthStencil];
    [blit endEncoding];
    [commands commit];
    [commands waitUntilCompleted];
    if (commands.status != MTLCommandBufferStatusCompleted) {
        return NO;
    }
    const auto* bgra = static_cast<const unsigned char*>(pixels.contents);
    const auto* stored = static_cast<const float*>(depths.contents);
    rgba.resize(w * h * 4);
    depth.resize(w * h);
    // Reversed Z, infinite far plane: stored = near / z with near = 0.03 m (see Camera::viewProjection).
    // Metric depth is the distance along the optical axis; empty pixels (sky) read maxRange.
    for (NSUInteger i = 0; i < w * h; ++i) {
        rgba[i * 4] = bgra[i * 4 + 2];
        rgba[i * 4 + 1] = bgra[i * 4 + 1];
        rgba[i * 4 + 2] = bgra[i * 4];
        rgba[i * 4 + 3] = 255;
        double z = stored[i] > 0 ? 0.03 / stored[i] : maxRange;
        depth[i] = static_cast<float>(std::min(z, maxRange));
    }
    return YES;
}

- (NSImage*)thumbnailOfWorld:(const World&)world width:(int)width height:(int)height {
    if (world.bodies.empty()) {
        return nil;
    }
    Vec3 lo{1e30, 1e30, 1e30};
    Vec3 hi{-1e30, -1e30, -1e30};
    for (const Body& body : world.bodies) {
        for (Vec3 c : body.vertices()) {
            lo = {std::min(lo.x, c.x), std::min(lo.y, c.y), std::min(lo.z, c.z)};
            hi = {std::max(hi.x, c.x), std::max(hi.y, c.y), std::max(hi.z, c.z)};
        }
    }
    double radius = std::max((hi - lo).length() / 2, 0.05);
    Camera camera =
        Camera::make(0.75, 0.42, radius * 3.1, (lo + hi) * 0.5, static_cast<double>(width) / height);
    std::vector<unsigned char> rgba;
    std::vector<float> depth;
    _worldOverride = &world;
    BOOL rendered = [self renderCamera:camera width:width height:height maxRange:100 color:rgba depth:depth];
    _worldOverride = nullptr;
    if (!rendered) {
        return nil;
    }
    NSBitmapImageRep* bitmap = [[NSBitmapImageRep alloc] initWithBitmapDataPlanes:nullptr
                                                                       pixelsWide:width
                                                                       pixelsHigh:height
                                                                    bitsPerSample:8
                                                                  samplesPerPixel:4
                                                                         hasAlpha:YES
                                                                         isPlanar:NO
                                                                   colorSpaceName:NSDeviceRGBColorSpace
                                                                      bytesPerRow:width * 4
                                                                     bitsPerPixel:32];
    std::copy(rgba.begin(), rgba.end(), bitmap.bitmapData);
    NSImage* image = [[NSImage alloc] initWithSize:NSMakeSize(width, height)];
    [image addRepresentation:bitmap];
    return image;
}

- (void)showCameraColor:(NSImage*)color depth:(NSImage*)depthImage title:(NSString*)title {
    if (!color) {
        _pip.hidden = YES;
        return;
    }
    _pipColor.image = color;
    _pipDepth.image = depthImage;
    _pipTitle.stringValue = title;
    _pip.hidden = NO;
}

#pragma mark Picking and input

- (int)pick:(NSPoint)p hit:(Vec3*)hit {
    const World* world = [self.host sceneWorld];
    if (!world) {
        return -1;
    }
    Camera camera = [self camera];
    Vec3 dir = [self rayAt:p camera:camera];
    Mesh mesh = buildMesh(*world, -1, false, false, _meshes);
    double best = 1e30;
    int result = -1;
    for (int body = 0; body < static_cast<int>(mesh.bodies.size()); ++body) {
        auto [start, count] = mesh.bodies[body];
        for (int k = start; k + 2 < start + count; k += 3) {
            double t;
            if (rayTriangle(camera.eye, dir, vec(mesh.v[k].p), vec(mesh.v[k + 1].p), vec(mesh.v[k + 2].p), t)
                && t < best) {
                best = t;
                result = body;
            }
        }
    }
    if (hit && result >= 0) {
        *hit = camera.eye + dir * best;
    }
    return result;
}

- (BOOL)planePoint:(NSPoint)p height:(double)h result:(Vec3*)out {
    Camera camera = [self camera];
    Vec3 dir = [self rayAt:p camera:camera];
    if (std::abs(dir.y) < 1e-6) {
        return NO;
    }
    double t = (h - camera.eye.y) / dir.y;
    if (t <= 0 || t > 500) {
        return NO;
    }
    *out = camera.eye + dir * t;
    return YES;
}

#pragma mark Transform gizmo input

- (BOOL)gizmoVisible {
    const World* world = [self.host sceneWorld];
    int selected = [self.host sceneSelected];
    return _gizmoTool != GizmoTool::Select && world && selected >= 0
           && selected < static_cast<int>(world->bodies.size()) && [self.host sceneGizmoAvailable];
}

- (void)updateTrackingAreas {
    [super updateTrackingAreas];
    if (_tracking) {
        [self removeTrackingArea:_tracking];
    }
    _tracking = [[NSTrackingArea alloc] initWithRect:NSZeroRect
                                             options:NSTrackingMouseMoved | NSTrackingMouseEnteredAndExited
                                                     | NSTrackingActiveInKeyWindow | NSTrackingInVisibleRect
                                               owner:self
                                            userInfo:nil];
    [self addTrackingArea:_tracking];
}

- (void)updateHover:(NSPoint)p {
    GizmoHandle handle = GizmoHandle::None;
    if ([self gizmoVisible]) {
        Vec3 center = [self.host sceneWorld]->bodies[[self.host sceneSelected]].position;
        handle = gizmoHitTest([self gizmoView], center, _gizmoTool, { p.x, p.y });
    }
    if (handle != _hoveredHandle) {
        _hoveredHandle = handle;
        [(handle == GizmoHandle::None ? NSCursor.arrowCursor : NSCursor.openHandCursor) set];
        self.needsDisplay = YES;
    }
}

- (void)mouseMoved:(NSEvent*)event {
    [self updateHover:[self convertPoint:event.locationInWindow fromView:nil]];
}

- (void)mouseExited:(NSEvent*)event {
    (void)event;
    if (_hoveredHandle != GizmoHandle::None) {
        _hoveredHandle = GizmoHandle::None;
        [NSCursor.arrowCursor set];
        self.needsDisplay = YES;
    }
}

- (BOOL)beginGizmoDragAt:(NSPoint)p {
    if (![self gizmoVisible]) {
        return NO;
    }
    Vec3 center = [self.host sceneWorld]->bodies[[self.host sceneSelected]].position;
    _dragView = [self gizmoView];
    GizmoHandle handle = gizmoHitTest(_dragView, center, _gizmoTool, {p.x, p.y});
    if (handle == GizmoHandle::None) {
        return NO;
    }
    _gizmoDrag = beginGizmoDrag(_dragView, center, handle, {p.x, p.y});
    _activeHandle = handle;
    _gizmoStarted = NO;
    _sweep = 0;
    _sweepStart = _gizmoDrag.lastRaw;
    [NSCursor.closedHandCursor set];
    self.needsDisplay = YES;
    return YES;
}

// Effective snapping for this event: ⌘ inverts the setting (off -> a default step).
- (double)snapStep:(double)setting fallback:(double)fallback flags:(NSEventModifierFlags)flags {
    if (flags & NSEventModifierFlagCommand) {
        return setting > 0 ? 0 : fallback;
    }
    return setting;
}

- (void)updateGizmoDragAt:(NSPoint)p flags:(NSEventModifierFlags)flags {
    const char* axisNames = "XYZ";
    int axis = gizmoAxisIndex(_activeHandle);
    int selected = [self.host sceneSelected];
    NSString* text;
    if (gizmoIsRing(_activeHandle)) {
        double step = [self snapStep:_rotateSnap fallback:15 * robotics::pi / 180 flags:flags];
        double angle = gizmoDragAngle(_gizmoDrag, _dragView, {p.x, p.y}, step);
        if (!_gizmoStarted && angle == 0) {
            return;
        }
        if (!_gizmoStarted && ![self.host sceneBeginRotate]) {
            [self cancelGizmoDrag];
            return;
        }
        _gizmoStarted = YES;
        _sweep = angle;
        [self.host sceneRotateAxis:gizmoAxis(axis) pivot:_gizmoDrag.center angle:angle];
        text = [NSString stringWithFormat:@"%c  %+.1f°%@", axisNames[axis], angle * 180 / robotics::pi,
                                          step > 0 ? [NSString stringWithFormat:@"  ·  snap %.0f°",
                                                                                step * 180 / robotics::pi]
                                                   : @""];
    } else {
        double step = [self snapStep:_moveSnap fallback:0.05 flags:flags];
        Vec3 target = gizmoDragPosition(_gizmoDrag, _dragView, {p.x, p.y}, step);
        Vec3 current = [self.host sceneWorld]->bodies[selected].position;
        if (!_gizmoStarted && (target - current).length() < 1e-9) {
            return;
        }
        if (!_gizmoStarted && ![self.host sceneBeginMove:selected]) {
            [self cancelGizmoDrag];
            return;
        }
        _gizmoStarted = YES;
        [self.host sceneMoveBy:target - current];
        Vec3 now = [self.host sceneWorld]->bodies[selected].position;
        Vec3 delta = now - _gizmoDrag.center;
        NSString* snap = step > 0 ? [NSString stringWithFormat:@"  ·  snap %.0f cm", step * 100] : @"";
        text = gizmoIsPlane(_activeHandle)
                   ? [NSString stringWithFormat:@"Δ %.3f m%@", delta.length(), snap]
                   : [NSString stringWithFormat:@"%c  %.3f m  (Δ %+.3f)%@", axisNames[axis], now[axis],
                                                delta[axis], snap];
    }
    _readoutText.stringValue = text;
    CGFloat width = [text sizeWithAttributes:@{NSFontAttributeName : _readoutText.font}].width + 24;
    NSRect bounds = self.bounds;
    _readoutPill.frame = NSMakeRect(std::min(p.x + 18, bounds.size.width - width - 6),
                                    std::min(p.y + 14, bounds.size.height - 32), width, 26);
    _readoutText.frame = NSMakeRect(10, 5, width - 20, 16);
    _readoutPill.hidden = NO;
}

- (void)cancelGizmoDrag {
    _activeHandle = GizmoHandle::None;
    _readoutPill.hidden = YES;
    _sweep = 0;
    [NSCursor.arrowCursor set];
    self.needsDisplay = YES;
}

- (void)endGizmoDrag {
    BOOL started = _gizmoStarted;
    [self cancelGizmoDrag];
    if (started) {
        [self.host sceneEndMove];
        [self holdCamera];
    }
}

- (void)holdCamera {
    const World* world = [self.host sceneWorld];
    for (const auto& joint : world->joints) {
        if (isWheelJoint(*world, joint)) {
            _followOffset = _target - world->bodies[joint.a].position;
            break;
        }
    }
}

- (void)flagsChanged:(NSEvent*)event {
    if (_activeHandle != GizmoHandle::None) { // ⌘ pressed or released while holding still.
        [self updateGizmoDragAt:_last flags:event.modifierFlags];
        self.needsDisplay = YES;
    }
    [super flagsChanged:event];
}

#pragma mark Mouse

- (void)mouseDown:(NSEvent*)event {
    [self.window makeFirstResponder:self];
    _last = [self convertPoint:event.locationInWindow fromView:nil];
    _press = _last;
    _dragged = NO;
    _moving = NO;
    _contextClick = (event.modifierFlags & NSEventModifierFlagControl) != 0;
    if (_contextClick) {
        return;
    }
    if (!(event.modifierFlags & NSEventModifierFlagOption) && [self beginGizmoDragAt:_last]) {
        return;
    }
    if (event.modifierFlags & NSEventModifierFlagOption) {
        Vec3 hit;
        int body = [self pick:_last hit:&hit];
        if (body >= 0 && [self.host sceneBeginMove:body]) {
            _moving = YES;
            _moveHeight = hit.y;
            _moveLast = hit;
            [[NSCursor closedHandCursor] push];
        }
    }
}

- (void)panBy:(NSPoint)delta {
    Camera camera = [self camera];
    double s = _distance * 0.0018;
    _target -= camera.right * (delta.x * s);
    _target -= camera.up * (delta.y * s);
    _follow = NO;
}

- (void)mouseDragged:(NSEvent*)event {
    NSPoint p = [self convertPoint:event.locationInWindow fromView:nil];
    if (std::hypot(p.x - _press.x, p.y - _press.y) > 3) {
        _dragged = YES;
    }
    if (_activeHandle != GizmoHandle::None) {
        [self updateGizmoDragAt:p flags:event.modifierFlags];
        _last = p;
        self.needsDisplay = YES;
        return;
    }
    if (_moving) {
        if (event.modifierFlags & NSEventModifierFlagShift) {
            Vec3 delta{0, (p.y - _last.y) * _distance * 0.0018, 0};
            [self.host sceneMoveBy:delta];
            _moveHeight += delta.y;
            _moveLast.y += delta.y;
        } else {
            Vec3 point;
            if ([self planePoint:p height:_moveHeight result:&point]) {
                [self.host sceneMoveBy:point - _moveLast];
                _moveLast = point;
            }
        }
    } else if (event.modifierFlags & NSEventModifierFlagShift) {
        [self panBy:NSMakePoint(p.x - _last.x, p.y - _last.y)];
    } else {
        _yaw -= (p.x - _last.x) * 0.008;
        _pitch = std::clamp(_pitch - (p.y - _last.y) * 0.008, 0.03, 1.53);
    }
    _last = p;
    self.needsDisplay = YES;
}

- (void)mouseUp:(NSEvent*)event {
    if (_activeHandle != GizmoHandle::None) {
        [self endGizmoDrag];
        return;
    }
    if (_contextClick && !_dragged) {
        _contextClick = NO;
        [self showContextMenu:event];
        return;
    }
    if (_moving) {
        _moving = NO;
        [NSCursor pop];
        [self.host sceneEndMove];
        return;
    }
    if (_dragged) {
        return;
    }
    [self.host scenePicked:[self pick:[self convertPoint:event.locationInWindow fromView:nil] hit:nullptr]];
}

- (void)showContextMenu:(NSEvent*)event {
    NSPoint p = [self convertPoint:event.locationInWindow fromView:nil];
    NSMenu* menu = [self.host sceneContextMenuForBody:[self pick:p hit:nullptr]];
    if (menu) {
        [NSMenu popUpContextMenu:menu withEvent:event forView:self];
    }
}

- (void)rightMouseDown:(NSEvent*)event {
    _last = [self convertPoint:event.locationInWindow fromView:nil];
    _press = _last;
    _rightDragged = NO;
}

- (void)rightMouseUp:(NSEvent*)event {
    if (!_rightDragged) { // A click, not a pan.
        [self showContextMenu:event];
    }
}

- (void)rightMouseDragged:(NSEvent*)event {
    NSPoint p = [self convertPoint:event.locationInWindow fromView:nil];
    if (std::hypot(p.x - _press.x, p.y - _press.y) > 3) {
        _rightDragged = YES;
    }
    [self panBy:NSMakePoint(p.x - _last.x, p.y - _last.y)];
    _last = p;
    self.needsDisplay = YES;
}

- (void)otherMouseDown:(NSEvent*)event {
    [self rightMouseDown:event];
}

- (void)otherMouseDragged:(NSEvent*)event {
    [self rightMouseDragged:event];
}

- (void)scrollWheel:(NSEvent*)event {
    double dy = event.hasPreciseScrollingDeltas ? event.scrollingDeltaY : event.scrollingDeltaY * 4;
    _distance = std::clamp(_distance * std::exp(dy * 0.006), 0.8, 150.0);
    self.needsDisplay = YES;
}

- (void)magnifyWithEvent:(NSEvent*)event {
    _distance = std::clamp(_distance / (1 + event.magnification), 0.8, 150.0);
    self.needsDisplay = YES;
}

- (void)keyDown:(NSEvent*)event {
    if (event.modifierFlags & (NSEventModifierFlagCommand | NSEventModifierFlagControl)) {
        [super keyDown:event];
        return;
    }
    // Physical key codes (ANSI layout) so controls work with any input language, e.g. Russian.
    switch (event.keyCode) {
    case 49:
        [self.host sceneKey:@"run"];
        break;
    case 3:
        [self homeCamera];
        break;
    case 13:
        [self.host sceneKey:@"w"];
        break;
    case 0:
        [self.host sceneKey:@"a"];
        break;
    case 1:
        [self.host sceneKey:@"s"];
        break;
    case 2:
        [self.host sceneKey:@"d"];
        break;
    case 7:
        [self.host sceneKey:@"x"];
        break;
    case 51:
    case 117:
        [self.host sceneKey:@"delete"];
        break;
    case 18: // 1, 2, 3: gizmo tools.
    case 19:
    case 20:
        self.gizmoTool = static_cast<GizmoTool>(event.keyCode - 18);
        break;
    case 123:
        [self.host sceneKey:@"frame-back"];
        break;
    case 124:
        [self.host sceneKey:@"frame-forward"];
        break;
    case 53:
        [self.host sceneKey:@"live"];
        break;
    default:
        [super keyDown:event];
    }
}

@end

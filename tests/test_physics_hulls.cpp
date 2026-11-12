// Convex hulls: quickhull, polyhedral mass properties, ray casts, GJK/EPA, contacts with every shape.
#include "test_harness.hpp"

#include "../src/physics/gjk.hpp"
#include "robotics/io/robot_format.hpp"
#include "robotics/sensors/raycast.hpp"
#include "robotics/sensors/sensor_system.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <random>
#include <set>
#include <sstream>

using namespace robotics;
using namespace robotics::detail;
using namespace test;

namespace {

void nearVec(Vec3 a, Vec3 b, double tolerance, const char* message) {
    near((a - b).length(), 0, tolerance, message);
}

std::vector<Vec3> boxPoints(Vec3 size, Vec3 center = {}) {
    std::vector<Vec3> points;
    for (int i = 0; i < 8; ++i) {
        points.push_back(center
                         + Vec3{(i & 1 ? 0.5 : -0.5) * size.x, (i & 2 ? 0.5 : -0.5) * size.y,
                                (i & 4 ? 0.5 : -0.5) * size.z});
    }
    return points;
}

std::vector<Vec3> randomCloud(std::mt19937& random, int count, double radius = 1) {
    std::uniform_real_distribution<double> unit(-1, 1);
    std::vector<Vec3> points;
    for (int i = 0; i < count; ++i) {
        points.push_back(Vec3{unit(random), unit(random), unit(random)} * radius);
    }
    return points;
}

// Structural invariants every hull must satisfy.
void checkHull(const ConvexHull& hull, const std::vector<Vec3>* source = nullptr, double tolerance = 1e-9) {
    int v = static_cast<int>(hull.vertices.size());
    int f = static_cast<int>(hull.faces.size());
    int e = static_cast<int>(hull.edges.size());
    check(v >= 4 && v <= maxConvexHullVertices, "Hull vertex count");
    check(v - e + f == 2, "Euler characteristic of a closed convex surface");
    check(hull.volume > 0, "Hull volume");
    check(std::is_sorted(hull.vertices.begin(), hull.vertices.end(),
                         [](Vec3 a, Vec3 b) { return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z); }),
          "Canonical vertex order");
    for (const auto& face : hull.faces) {
        near(face.normal.length(), 1, 1e-9, "Unit face normal");
        check(face.count >= 3, "Polygon face");
        Vec3 first =
            hull.vertices[static_cast<std::size_t>(hull.faceVertices[static_cast<std::size_t>(face.first)])];
        Vec3 second = hull.vertices[static_cast<std::size_t>(
            hull.faceVertices[static_cast<std::size_t>(face.first + 1)])];
        Vec3 third = hull.vertices[static_cast<std::size_t>(
            hull.faceVertices[static_cast<std::size_t>(face.first + 2)])];
        check((second - first).cross(third - first).dot(face.normal) > -1e-12,
              "Face winding matches the normal");
        check(face.normal.dot(first) > 0, "Normal faces away from the centre of mass");
        for (Vec3 p : hull.vertices) {
            check(face.normal.dot(p) <= face.offset + 1e-12, "Vertex beyond a face plane");
        }
    }
    for (const auto& edge : hull.edges) {
        auto runs = [&](int faceIndex, int from, int to) {
            const auto& face = hull.faces[static_cast<std::size_t>(faceIndex)];
            for (int k = 0; k < face.count; ++k) {
                if (hull.faceVertices[static_cast<std::size_t>(face.first + k)] == from
                    && hull.faceVertices[static_cast<std::size_t>(face.first + (k + 1) % face.count)] == to) {
                    return true;
                }
            }
            return false;
        };
        check(runs(edge.face0, edge.v0, edge.v1), "face0 runs v0 -> v1");
        check(runs(edge.face1, edge.v1, edge.v0), "face1 runs v1 -> v0");
    }
    if (source) {
        for (Vec3 p : *source) {
            Vec3 local = hull.toBody(p);
            for (const auto& face : hull.faces) {
                check(face.normal.dot(local) <= face.offset + tolerance, "Input point outside the hull");
            }
        }
    }
}

Body hullBody(std::shared_ptr<const ConvexHull> hull, Vec3 position, double mass = 1, Quat orientation = {}) {
    Body body;
    setConvexHull(body, std::move(hull));
    body.position = position;
    body.orientation = orientation;
    body.mass = mass;
    return body;
}

Body boxBody(Vec3 size, Vec3 position, double mass = 1, Quat orientation = {}) {
    Body body;
    body.size = size;
    body.position = position;
    body.mass = mass;
    body.orientation = orientation;
    return body;
}

std::shared_ptr<const ConvexHull> cubeHull(double edge = 1) {
    return buildConvexHull(boxPoints({edge, edge, edge}));
}

// Regular n-gon prism about Y with a top and bottom polygon.
std::shared_ptr<const ConvexHull> prismHull(int sides, double radius, double height) {
    std::vector<Vec3> points;
    for (int k = 0; k < sides; ++k) {
        double angle = 2 * pi * k / sides;
        for (double y : {-height / 2, height / 2}) {
            points.push_back({radius * std::cos(angle), y, radius * std::sin(angle)});
        }
    }
    return buildConvexHull(points);
}

ConvexPoints pointsOf(const std::vector<Vec3>& v, Vec3 position = {}, Quat orientation = {}) {
    return {v.data(), static_cast<int>(v.size()), position, Mat3::rotation(orientation)};
}

} // namespace

// ---- Quickhull ---------------------------------------------------------------------------------

TEST_CASE("quickhull: cube points give 8 vertices and 6 quad faces") {
    auto points = boxPoints({2, 2, 2}, {5, -3, 1});
    auto hull = buildConvexHull(points);
    checkHull(*hull, &points);
    check(hull->vertices.size() == 8 && hull->faces.size() == 6 && hull->edges.size() == 12, "Cube topology");
    for (const auto& face : hull->faces) {
        check(face.count == 4, "Merged quad");
        near(face.offset, 1, 1e-12, "Face offset");
    }
    nearVec(hull->sourceOffset, {5, -3, 1}, 1e-12, "Centre of mass in source coordinates");
    near(hull->minWidth, 2, 1e-12, "Minimal width");
    near(hull->radius, std::sqrt(3.0), 1e-12, "Radius");
}

TEST_CASE("quickhull: interior, duplicate, on-face and on-edge points are dropped") {
    auto points = boxPoints({2, 2, 2});
    auto corners = points;
    points.insert(points.end(), corners.begin(), corners.end()); // Duplicates.
    for (int i = -4; i <= 4; ++i) {
        for (int k = -4; k <= 4; ++k) {
            points.push_back({i * 0.25, k * 0.25, 1}); // On a face.
            points.push_back({i * 0.25, 1, 1});        // On an edge.
            points.push_back({i * 0.2, k * 0.2, 0.3}); // Inside.
        }
    }
    auto hull = buildConvexHull(points);
    checkHull(*hull);
    check(hull->vertices.size() == 8 && hull->faces.size() == 6, "Only the corners survive");
    near(hull->volume, 8, 1e-9, "Volume");
    near(hull->approximationError, 0, 1e-12, "Exact hull");
}

TEST_CASE("quickhull: random clouds contain every input point") {
    std::mt19937 random(11);
    for (int run = 0; run < 200; ++run) {
        int count = 4 + static_cast<int>(random() % 50);
        auto cloud = randomCloud(random, count, 0.1 + 0.05 * (run % 20));
        auto hull = buildConvexHull(cloud);
        checkHull(*hull, hull->approximationError > 0 ? nullptr : &cloud, 1e-6);
    }
}

TEST_CASE("quickhull: large clouds are capped at 64 vertices and report the error") {
    std::mt19937 random(5);
    std::normal_distribution<double> gauss;
    std::vector<Vec3> sphere;
    for (int i = 0; i < 5000; ++i) {
        sphere.push_back(Vec3{gauss(random), gauss(random), gauss(random)}.normalized());
    }
    auto hull = buildConvexHull(sphere);
    checkHull(*hull);
    check(static_cast<int>(hull->vertices.size()) <= maxConvexHullVertices, "Vertex cap");
    check(hull->vertices.size() >= 60, "The cap is used, not undershot");
    check(hull->approximationError > 0 && hull->approximationError < 0.15, "Approximation error reported");
    near(hull->volume, 4.0 / 3 * pi, 0.6, "Simplified sphere keeps most of the volume");

    HullBuildOptions few;
    few.maxVertices = 12;
    auto small = buildConvexHull(sphere, few);
    checkHull(*small);
    check(small->vertices.size() <= 12, "Requested cap");
}

TEST_CASE("quickhull: tolerance scales with the input extent") {
    for (double scale : {1e-3, 1.0, 1e3}) {
        std::vector<Vec3> points;
        for (Vec3 p : boxPoints({1, 2, 3})) {
            points.push_back(p * scale);
        }
        // Noise far below the tolerance on a face centre must not add a vertex.
        points.push_back(Vec3{0, 0, 1.5 * (1 + 1e-9)} * scale);
        auto hull = buildConvexHull(points);
        check(hull->vertices.size() == 8, "Scale-independent coplanarity");
        near(hull->volume / (6 * scale * scale * scale), 1, 1e-9, "Scaled volume");
    }
}

TEST_CASE("quickhull: lattices, tessellated meshes, far offsets and noise do not break it") {
    auto accept = [](const std::vector<Vec3>& points, double extent) {
        auto hull = buildConvexHull(points);
        checkHull(*hull);
        if (hull->approximationError == 0) {
            for (Vec3 p : points) {
                Vec3 local = hull->toBody(p);
                for (const auto& face : hull->faces) {
                    check(face.normal.dot(local) <= face.offset + 1e-5 * extent,
                          "Input point outside the hull");
                }
            }
        }
        return hull;
    };
    std::vector<Vec3> lattice;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                lattice.push_back(Vec3{i * 0.2, j * 0.2, k * 0.2});
            }
        }
    }
    auto cube = accept(lattice, 1);
    check(cube->vertices.size() == 8, "Lattice is a cube");
    near(cube->volume, 1, 1e-9, "Lattice volume");

    std::vector<Vec3> globe; // Latitude-longitude sphere: repeated poles, rings of coplanar points.
    for (int i = 0; i <= 12; ++i) {
        for (int j = 0; j < 24; ++j) {
            double polar = pi * i / 12;
            double around = 2 * pi * j / 24;
            globe.push_back(
                {std::sin(polar) * std::cos(around), std::cos(polar), std::sin(polar) * std::sin(around)});
        }
    }
    accept(globe, 2);

    std::vector<Vec3> can; // 48-sided cylinder: more vertices than the cap, big coplanar caps.
    for (int k = 0; k < 48; ++k) {
        double angle = 2 * pi * k / 48;
        for (double y : {-0.5, 0.0, 0.5}) {
            can.push_back({std::cos(angle), y, std::sin(angle)});
        }
    }
    accept(can, 2);

    std::mt19937 random(77);
    std::uniform_real_distribution<double> unit(-1, 1);
    for (double noise : {1e-3, 1e-5, 1e-6, 1e-7, 1e-9, 1e-12}) {
        std::vector<Vec3> rough = lattice;
        for (Vec3& p : rough) {
            p += Vec3{unit(random), unit(random), unit(random)} * noise;
        }
        accept(rough, 1);
    }
    for (double offset : {1e3, 1e6}) {
        std::vector<Vec3> far = globe;
        for (Vec3& p : far) {
            p += Vec3{offset, -offset, 0.5 * offset};
        }
        auto hull = buildConvexHull(far);
        checkHull(*hull);
        near(hull->sourceOffset.x, offset, 1e-3, "Centre of mass far from the origin");
    }
    // A sliver: thin but not flat.
    std::vector<Vec3> sliver = boxPoints({2, 1e-3, 1});
    auto thin = accept(sliver, 2);
    near(thin->minWidth, 1e-3, 1e-9, "Thin hull width");
}

TEST_CASE("quickhull: degenerate inputs are rejected with an error") {
    rejects([] { (void)buildConvexHull({}); });
    rejects([] { (void)buildConvexHull({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}); });
    rejects([] { (void)buildConvexHull(std::vector<Vec3>(10, Vec3{1, 2, 3})); });
    rejects([] { (void)buildConvexHull({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {3, 3, 3}, {4, 4, 4}}); });
    rejects([] { (void)buildConvexHull({{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}, {0.5, 0, 0.5}}); });
    rejects([] { (void)buildConvexHull({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, std::nan("")}}); });
    HullBuildOptions bad;
    bad.maxVertices = 3;
    rejects([&] { (void)buildConvexHull(boxPoints({1, 1, 1}), bad); });
}

TEST_CASE("quickhull: building is deterministic and the frame is usable") {
    std::mt19937 random(2);
    auto cloud = randomCloud(random, 100);
    auto a = buildConvexHull(cloud);
    auto b = buildConvexHull(cloud);
    check(a->vertices.size() == b->vertices.size(), "Same size");
    for (std::size_t i = 0; i < a->vertices.size(); ++i) {
        check(a->vertices[i].x == b->vertices[i].x && a->vertices[i].y == b->vertices[i].y
                  && a->vertices[i].z == b->vertices[i].z,
              "Bit-identical vertices");
    }
    // Every body-frame vertex maps back onto an input point.
    for (Vec3 v : a->vertices) {
        Vec3 source = a->toSource(v);
        bool found =
            std::any_of(cloud.begin(), cloud.end(), [&](Vec3 p) { return (p - source).length() < 1e-12; });
        check(found, "Hull vertex comes from the input");
    }
    // placeBody and the visual transform are inverse to each other.
    Vec3 meshPosition{1, 2, 3};
    Quat meshOrientation = Quat::euler({0.3, -0.4, 1.1});
    Vec3 position;
    Quat orientation;
    a->placeBody(meshPosition, meshOrientation, position, orientation);
    Vec3 meshPoint{0.2, -0.1, 0.5};
    Vec3 viaMesh = meshPosition + meshOrientation.rotate(meshPoint);
    Vec3 viaVisual =
        position + orientation.rotate(a->visualPosition() + a->visualOrientation().rotate(meshPoint));
    nearVec(viaVisual, viaMesh, 1e-12, "Visual mesh lands on the source pose");
}

// ---- Mass properties ------------------------------------------------------------------------------

TEST_CASE("mass properties: hull cuboid matches the analytic box") {
    Vec3 size{1, 2, 3};
    auto hull = buildConvexHull(boxPoints(size, {4, 5, 6}));
    near(hull->volume, 6, 1e-12, "Volume");
    Body hullShape = hullBody(hull, {}, 12);
    Body box = boxBody(size, {}, 12);
    nearVec(hullShape.principalInertia(), box.principalInertia(), 1e-10, "Inertia");
    nearVec(hull->sourceOffset, {4, 5, 6}, 1e-12, "Centroid");
    near(std::abs(hull->sourceRotation.w), 1, 1e-12, "Aligned box keeps the source axes");
}

TEST_CASE("mass properties: rotated cuboid is found on its principal axes") {
    Quat turn = Quat::euler({0.4, 0.9, -0.7});
    std::vector<Vec3> points;
    for (Vec3 p : boxPoints({1, 2, 3})) {
        points.push_back(turn.rotate(p) + Vec3{1, 1, 1});
    }
    auto hull = buildConvexHull(points);
    // The axes are matched to the source axes, so the order of the moments is the rotation's business.
    auto sorted = [](Vec3 v) {
        std::array<double, 3> s{v.x, v.y, v.z};
        std::sort(s.begin(), s.end());
        return Vec3{s[0], s[1], s[2]};
    };
    Vec3 expected = boxBody({1, 2, 3}, {}, 1).principalInertia();
    nearVec(sorted(hull->unitInertia), sorted(expected), 1e-10, "Principal moments");
    nearVec(sorted(hull->halfExtents), {0.5, 1, 1.5}, 1e-10, "Hull is axis-aligned in the body frame");
    checkHull(*hull, &points);
}

TEST_CASE("mass properties: tetrahedron volume, centroid and inertia are analytic") {
    // Right tetrahedron (0,0,0), (a,0,0), (0,a,0), (0,0,a): V = a³/6, centroid a/4.
    double a = 2;
    std::vector<Vec3> vertices{{0, 0, 0}, {a, 0, 0}, {0, a, 0}, {0, 0, a}};
    std::vector<int> triangles{0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    PolyhedronMass mass = polyhedronMassProperties(vertices, triangles);
    near(mass.volume, a * a * a / 6, 1e-12, "Tetrahedron volume");
    nearVec(mass.centroid, Vec3{1, 1, 1} * (a / 4), 1e-12, "Tetrahedron centroid");
    // ∫x²dV = a⁵/60, ∫xy dV = a⁵/120; shift to the centroid with the parallel axis theorem.
    double second = std::pow(a, 5) / 60;
    double product = std::pow(a, 5) / 120;
    double c = a / 4;
    double diagonal = 2 * second - mass.volume * 2 * c * c;
    double off = -(product - mass.volume * c * c);
    near(mass.inertia.r0.x, diagonal, 1e-12, "Ixx");
    near(mass.inertia.r1.y, diagonal, 1e-12, "Iyy");
    near(mass.inertia.r2.z, diagonal, 1e-12, "Izz");
    near(mass.inertia.r0.y, off, 1e-12, "Ixy");
    near(mass.inertia.r1.z, off, 1e-12, "Iyz");
    near(mass.inertia.r0.z, off, 1e-12, "Izx");

    // The hull of the same points: eigenvalues of that tensor are diagonal - off (twice) and diagonal + 2
    // off.
    auto hull = buildConvexHull(vertices);
    std::array<double, 3> moments{hull->unitInertia.x, hull->unitInertia.y, hull->unitInertia.z};
    std::sort(moments.begin(), moments.end());
    double m = mass.volume;
    near(moments[0], (diagonal - off) / m, 1e-10, "Double moment");
    near(moments[1], (diagonal - off) / m, 1e-10, "Double moment");
    near(moments[2], (diagonal + 2 * off) / m, 1e-10, "Largest moment");
    checkHull(*hull);
}

TEST_CASE("mass properties: Jacobi diagonalization reconstructs the matrix") {
    std::mt19937 random(3);
    std::uniform_real_distribution<double> unit(-1, 1);
    for (int run = 0; run < 100; ++run) {
        Mat3 m{{unit(random), 0, 0}, {0, unit(random), 0}, {0, 0, unit(random)}};
        m.r0.y = m.r1.x = unit(random);
        m.r0.z = m.r2.x = unit(random);
        m.r1.z = m.r2.y = unit(random);
        Vec3 values;
        Mat3 v;
        diagonalizeSymmetric(m, values, v);
        near(v.determinant(), 1, 1e-10, "Proper rotation");
        Mat3 back = v * Mat3::diagonal(values) * v.transposed();
        nearVec(back.r0, m.r0, 1e-10, "Row 0");
        nearVec(back.r1, m.r1, 1e-10, "Row 1");
        nearVec(back.r2, m.r2, 1e-10, "Row 2");
    }
}

TEST_CASE("quatFromMatrix inverts Mat3::rotation in every branch") {
    for (Quat q :
         {Quat{}, Quat::axisAngle({1, 0, 0}, 3), Quat::axisAngle({0, 1, 0}, 3), Quat::axisAngle({0, 0, 1}, 3),
          Quat::euler({0.2, 0.3, 0.4}), Quat::axisAngle({1, 1, 1}, 2.9)}) {
        Quat back = quatFromMatrix(Mat3::rotation(q));
        Vec3 v{0.3, -0.7, 0.2};
        nearVec(back.rotate(v), q.rotate(v), 1e-12, "Same rotation");
    }
}

// ---- Ray casts -----------------------------------------------------------------------------------

TEST_CASE("hull ray cast: cube agrees with the box ray cast, rays miss, start inside, respect range") {
    Quat turn = Quat::euler({0.3, 0.5, 0.7});
    Body hull = hullBody(cubeHull(1), {1, 2, 3}, 1, turn);
    Body box = boxBody({1, 1, 1}, {1, 2, 3}, 1, turn);
    std::mt19937 random(9);
    std::uniform_real_distribution<double> unit(-1, 1);
    int hits = 0;
    for (int i = 0; i < 500; ++i) {
        Vec3 origin = Vec3{1, 2, 3} + Vec3{unit(random), unit(random), unit(random)} * 3;
        Vec3 target = Vec3{1, 2, 3} + Vec3{unit(random), unit(random), unit(random)} * 0.9;
        Vec3 direction = (target - origin).normalized();
        RayHit a = raycastBody(hull, origin, direction, 100);
        RayHit b = raycastBody(box, origin, direction, 100);
        check(a.hit == b.hit, "Hull and box disagree about a hit");
        if (a.hit) {
            ++hits;
            near(a.distance, b.distance, 1e-9, "Distance");
            if (b.distance > 0) {
                nearVec(a.normal, b.normal, 1e-9, "Normal");
            }
        }
    }
    check(hits > 100, "Enough hits sampled");
    check(!raycastBody(hull, {1, 5, 3}, {0, 1, 0}, 10).hit, "Ray pointing away");
    check(!raycastBody(hull, {1, 5, 3}, {0, -1, 0}, 1).hit, "Out of range");
    RayHit inside = raycastBody(hull, {1, 2, 3}, {0, 0, 1}, 10);
    check(inside.hit && inside.distance == 0, "Start inside hits at distance 0");
    nearVec(inside.normal, {0, 0, -1}, 1e-12, "Initial overlap normal");
}

TEST_CASE("hull ray cast: rangefinder and lidar measure a known pyramid") {
    // Square pyramid apex up, base 2 x 2 at y = 0, apex at y = 1: a side's normal is (1, 1, 0)/sqrt 2.
    std::vector<Vec3> points{{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}, {0, 1, 0}};
    auto hull = buildConvexHull(points);
    Vec3 position;
    Quat orientation;
    hull->placeBody({0, 0, 0}, {}, position, orientation);
    World world;
    Body pyramid = hullBody(hull, position, 0, orientation);
    world.addBody(pyramid);
    // Horizontal ray at y = 0.5 from +X: the side surface is at x = 0.5.
    RayHit hit = raycast(world, {3, 0.5, 0}, {-1, 0, 0}, 10, noBodyIndex, false);
    check(hit.hit && hit.body == 0, "Pyramid hit");
    near(hit.distance, 2.5, 1e-9, "Distance to the sloped face");
    nearVec(hit.normal, Vec3{1, 1, 0}.normalized(), 1e-9, "Sloped face normal");
    // Downwards on the apex side.
    hit = raycast(world, {0.25, 3, 0}, {0, -1, 0}, 10, noBodyIndex, false);
    near(hit.distance, 2.25, 1e-9, "Height of the slope at x = 0.25");

    // Through the sensor pipeline: the rangefinder looks along +X at the same slope from the other side.
    Body mount = boxBody({0.1, 0.1, 0.1}, {-3, 0.5, 0}, 0);
    int base = world.addBody(mount);
    Sensor range;
    range.name = "range";
    range.type = SensorType::Rangefinder;
    range.body = base;
    range.noise = {};
    range.rate = 0;
    range.maxRange = 10;
    world.sensors.push_back(range);
    Sensor lidar = range;
    lidar.name = "lidar";
    lidar.type = SensorType::Lidar2D;
    lidar.beams = 3;
    lidar.fov = 0.2;
    world.sensors.push_back(lidar);
    stepSimulation(world);
    near(world.sensors[0].state.value[0], 2.5, 1e-9, "Rangefinder distance");
    check(world.sensors[1].state.value.size() == 3, "Lidar beams");
    near(world.sensors[1].state.value[1], 2.5, 1e-9, "Lidar centre beam");
}

// ---- GJK / EPA --------------------------------------------------------------------------------------

TEST_CASE("gjk: distances between known shapes") {
    auto box = boxPoints({2, 2, 2});
    GjkResult faces = gjkDistance(pointsOf(box), pointsOf(box, {5, 0, 0}));
    check(!faces.overlapping, "Boxes apart");
    near(faces.distance, 3, 1e-9, "Face-face distance");
    near(faces.pointA.x, 1, 1e-9, "Witness on A");
    near(faces.pointB.x, 4, 1e-9, "Witness on B");

    GjkResult corners = gjkDistance(pointsOf(box), pointsOf(box, {3, 3, 3}));
    near(corners.distance, std::sqrt(3.0), 1e-9, "Corner-corner distance");
    nearVec(corners.pointA, {1, 1, 1}, 1e-9, "Corner witness");

    Vec3 point{0, 3, 0};
    GjkResult toPoint =
        gjkDistance(pointsOf(box, {}, Quat::axisAngle({0, 0, 1}, pi / 4)), {&point, 1, {}, {}});
    near(toPoint.distance, 3 - std::sqrt(2.0), 1e-9, "Point above a turned box's edge");

    // Two crossed edges: distance between skew segments.
    std::vector<Vec3> rodA{{-1, 0, 0}, {1, 0, 0}};
    std::vector<Vec3> rodB{{0, 1, -1}, {0, 1, 1}};
    GjkResult rods = gjkDistance(pointsOf(rodA), pointsOf(rodB));
    near(rods.distance, 1, 1e-9, "Skew segments");
    nearVec(rods.pointA, {0, 0, 0}, 1e-9, "Segment witness");

    std::vector<Vec3> single{{0, 0, 0}};
    near(gjkDistance(pointsOf(single), pointsOf(single, {3, 4, 0})).distance, 5, 1e-12, "Two points");
}

TEST_CASE("gjk: agrees with brute force on random polytopes") {
    std::mt19937 random(21);
    std::uniform_real_distribution<double> unit(-1, 1);
    for (int run = 0; run < 300; ++run) {
        auto a = randomCloud(random, 12, 0.6);
        auto b = randomCloud(random, 12, 0.6);
        Vec3 offset = Vec3{unit(random), unit(random), unit(random)}.normalized() * 2.5;
        Quat turn = Quat::euler({unit(random), unit(random), unit(random)});
        GjkResult r = gjkDistance(pointsOf(a), pointsOf(b, offset, turn));
        check(!r.overlapping, "Well separated");
        near((r.pointA - r.pointB).length(), r.distance, 1e-9, "Witness distance");
        // The separating plane through the witnesses must have all of A and B on its two sides.
        Vec3 n = (r.pointB - r.pointA) / r.distance;
        for (Vec3 p : a) {
            check(n.dot(p - r.pointA) <= 1e-7, "A beyond the separating plane");
        }
        for (Vec3 p : b) {
            check(n.dot(offset + turn.rotate(p) - r.pointB) >= -1e-7, "B before the separating plane");
        }
    }
}

TEST_CASE("gjk: overlap, containment, touching and coincident shapes") {
    auto box = boxPoints({2, 2, 2});
    check(gjkDistance(pointsOf(box), pointsOf(box, {1, 0.5, 0.2})).overlapping, "Overlap");
    check(gjkDistance(pointsOf(box), pointsOf(box)).overlapping, "Coincident");
    auto small = boxPoints({0.1, 0.1, 0.1});
    check(gjkDistance(pointsOf(box), pointsOf(small, {0.2, 0.1, 0})).overlapping, "Contained");
    GjkResult touching = gjkDistance(pointsOf(box), pointsOf(box, {2, 0, 0}));
    check(touching.overlapping || touching.distance < 1e-9, "Touching faces");
}

TEST_CASE("epa: penetration of overlapping boxes matches the separating axis result") {
    auto box = boxPoints({2, 2, 2});
    // Overlap 0.3 along X: the minimal separating translation.
    ConvexPoints a = pointsOf(box);
    ConvexPoints b = pointsOf(box, {1.7, 0.1, -0.2});
    GjkResult gjk = gjkDistance(a, b);
    check(gjk.overlapping, "Boxes overlap");
    EpaResult epa = epaPenetration(a, b, gjk);
    check(epa.valid, "EPA converges");
    near(epa.depth, 0.3, 1e-8, "Depth");
    nearVec(epa.normal, {1, 0, 0}, 1e-8, "Moving B along +X separates them");
}

TEST_CASE("epa: random box pairs agree with SAT over face and edge axes") {
    std::mt19937 random(31);
    std::uniform_real_distribution<double> unit(-1, 1);
    auto box = boxPoints({1, 1.4, 0.8});
    std::array<Vec3, 3> axes{Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}};
    int compared = 0;
    for (int run = 0; run < 400; ++run) {
        Vec3 offset = Vec3{unit(random), unit(random), unit(random)} * 0.8;
        Quat turn = Quat::euler({unit(random) * 2, unit(random) * 2, unit(random) * 2});
        ConvexPoints a = pointsOf(box);
        ConvexPoints b = pointsOf(box, offset, turn);
        // SAT reference.
        std::vector<Vec3> candidates(axes.begin(), axes.end());
        for (Vec3 axis : axes) {
            candidates.push_back(turn.rotate(axis));
        }
        for (Vec3 p : axes) {
            for (Vec3 q : axes) {
                Vec3 c = p.cross(turn.rotate(q));
                if (c.length() > 1e-6) {
                    candidates.push_back(c.normalized());
                }
            }
        }
        double depth = 1e300;
        for (Vec3 axis : candidates) {
            for (Vec3 n : {axis, -axis}) {
                double reachA = n.dot(a.support(n));
                double reachB = n.dot(b.support(-n));
                depth = std::min(depth, reachA - reachB);
            }
        }
        GjkResult gjk = gjkDistance(a, b);
        check(gjk.overlapping == (depth > 0), "GJK agrees with SAT about the overlap");
        if (!gjk.overlapping) {
            continue;
        }
        EpaResult epa = epaPenetration(a, b, gjk);
        check(epa.valid, "EPA converges");
        near(epa.depth, depth, 1e-6, "Penetration depth");
        // Separating along the result must really clear the shapes (to within the tolerance).
        ConvexPoints moved = b;
        moved.position += epa.normal * (epa.depth + 1e-6);
        check(!gjkDistance(a, moved).overlapping, "Normal * depth separates the shapes");
        ++compared;
    }
    check(compared > 100, "Enough overlapping pairs");
}

TEST_CASE("epa: degenerate starts (origin on a face, point inside a hull, flat difference)") {
    auto box = boxPoints({2, 2, 2});
    // Centre of A exactly on B's vertex lattice: GJK's first support hits the origin plane.
    ConvexPoints a = pointsOf(box);
    ConvexPoints b = pointsOf(box);
    GjkResult same = gjkDistance(a, b);
    EpaResult coincident = epaPenetration(a, b, same);
    check(coincident.valid, "Coincident boxes");
    near(coincident.depth, 2, 1e-8, "Coincident boxes separate by a full edge");

    Vec3 point{0.1, 0.2, 0.7};
    ConvexPoints inner{&point, 1, {}, {}};
    GjkResult gjk = gjkDistance(a, inner);
    check(gjk.overlapping, "Point inside");
    EpaResult epa = epaPenetration(a, inner, gjk);
    check(epa.valid, "Point inside converges");
    near(epa.depth, 0.3, 1e-8, "Distance to the nearest face");
    nearVec(epa.normal, {0, 0, 1}, 1e-8, "Nearest face normal");

    std::vector<Vec3> flat{{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}};
    Vec3 origin{};
    ConvexPoints plate = pointsOf(flat);
    ConvexPoints onPlate{&origin, 1, {}, {}};
    GjkResult onFlat = gjkDistance(plate, onPlate);
    check(onFlat.overlapping, "Point on a flat set");
    check(!epaPenetration(plate, onPlate, onFlat).valid, "No volume: reported, no crash");
}

// ---- Contacts ---------------------------------------------------------------------------------------

TEST_CASE("hull on the ground: a cube rests on 4 points like a box") {
    World w;
    w.addBody(hullBody(cubeHull(0.5), {0, 0.24, 0}, 2));
    w.step();
    check(w.contacts.size() == 4, "Four ground points");
    for (const auto& c : w.contacts) {
        check(c.touchesGround(), "Ground");
        nearVec(c.normal, {0, 1, 0}, 1e-12, "Ground normal");
    }
}

TEST_CASE("hull on the ground: polygon faces reduce to at most 4 stable extreme points") {
    World w;
    int body = w.addBody(hullBody(prismHull(8, 0.3, 0.2), {0, 0.1, 0}, 3));
    std::uint32_t featureSum = 0;
    for (int i = 0; i < 1200; ++i) {
        w.step();
        check(w.contacts.size() >= 3 && w.contacts.size() <= 4, "3-4 contacts on an octagon");
        if (i == 600) {
            featureSum = 0;
            for (const auto& c : w.contacts) {
                featureSum += c.feature;
            }
        }
        if (i > 600) {
            std::uint32_t sum = 0;
            for (const auto& c : w.contacts) {
                sum += c.feature;
            }
            check(sum == featureSum, "Contact vertices stay the same while resting");
        }
    }
    const Body& b = w.bodies[static_cast<std::size_t>(body)];
    near(b.position.y, 0.1, 2e-3, "Rest height");
    check(b.velocity.length() < 1e-3 && b.angularVelocity.length() < 1e-3, "At rest");
    near(b.position.x, 0, 1e-4, "No drift X");
    near(b.position.z, 0, 1e-4, "No drift Z");
}

TEST_CASE("hull on the ground: lands on a stable face from any tilt") {
    std::mt19937 random(17);
    for (int run = 0; run < 10; ++run) {
        World w;
        auto hull = buildConvexHull(randomCloud(random, 30, 0.2));
        int body = w.addBody(
            hullBody(hull, {0, 0.6, 0}, 1000 * hull->volume, Quat::euler({0.5 * run, 0.3 * run, 0.1 * run})));
        for (int i = 0; i < 1800; ++i) {
            w.step();
            check(w.bodies[0].position.finite(), "Finite");
        }
        const Body& b = w.bodies[static_cast<std::size_t>(body)];
        check(b.velocity.length() < 0.02, "Settled");
        check(w.contacts.size() >= 3, "A face rests on the ground");
        check(b.position.y > 0, "Above the ground");
    }
}

TEST_CASE("hull stack stays put") {
    World w;
    double edge = 0.4;
    int count = 6;
    for (int i = 0; i < count; ++i) {
        w.addBody(hullBody(prismHull(6, edge, 0.2), {0, 0.1 + 0.2 * i, 0}, 2));
    }
    for (int i = 0; i < 2400; ++i) {
        w.step();
    }
    for (int i = 0; i < count; ++i) {
        const Body& b = w.bodies[static_cast<std::size_t>(i)];
        near(b.position.x, 0, 0.01, "Stack X");
        near(b.position.z, 0, 0.01, "Stack Z");
        near(b.position.y, 0.1 + 0.2 * i, 0.01, "Stack height");
    }
}

TEST_CASE("hull-hull contacts: face manifold with stable features, edge contact, separation") {
    {
        World w;
        w.addBody(hullBody(cubeHull(1), {0, 0.5, 0}, 0));
        w.addBody(hullBody(cubeHull(0.6), {0, 1.29, 0}, 1));
        w.step();
        check(w.contacts.size() == 4, "Face-face manifold");
        for (const auto& c : w.contacts) {
            near(std::abs(c.normal.y), 1, 1e-9, "Face normal");
            near(c.depth, 0.01, 2e-3, "Depth");
        }
        auto features = w.contacts;
        w.step();
        check(w.contacts.size() == 4, "Manifold kept");
        for (std::size_t i = 0; i < 4; ++i) {
            check(features[i].feature == w.contacts[i].feature, "Stable feature ids");
            check(w.contacts[i].normalImpulse > 0, "Warm started and active");
        }
    }
    {
        // Crossed roof edges: one point.
        World w;
        Quat ridgeX = Quat::axisAngle({1, 0, 0}, pi / 4);
        Quat ridgeZ = Quat::axisAngle({0, 0, 1}, pi / 4);
        double half = std::sqrt(0.5);
        w.addBody(hullBody(cubeHull(1), {0, 2, 0}, 0, ridgeX));
        w.addBody(hullBody(cubeHull(1), {0, 2 + 2 * half - 0.005, 0}, 1, ridgeZ));
        w.step();
        check(w.contacts.size() == 1, "Edge-edge is one point");
        near(std::abs(w.contacts[0].normal.y), 1, 1e-6, "Edge normal");
        near(w.contacts[0].depth, 0.005, 1e-3, "Edge depth");
    }
    {
        World w;
        w.addBody(hullBody(cubeHull(1), {0, 5, 0}, 0));
        w.addBody(hullBody(cubeHull(1), {1.2, 5, 0}, 1, Quat::axisAngle({0, 1, 0}, 0.2)));
        w.gravity = {};
        w.step();
        check(w.contacts.empty(), "Separated hulls");
    }
}

TEST_CASE("hull contacts agree with box contacts") {
    // Cube hull vs box must behave like box vs box, for any relative pose.
    std::mt19937 random(41);
    std::uniform_real_distribution<double> unit(-1, 1);
    int touching = 0;
    for (int run = 0; run < 300; ++run) {
        Vec3 offset = Vec3{unit(random), unit(random), unit(random)} * 0.9;
        Quat turnA = Quat::euler({unit(random), unit(random), unit(random)});
        Quat turnB = Quat::euler({unit(random), unit(random), unit(random)});
        World boxes;
        boxes.gravity = {};
        boxes.addBody(boxBody({1, 1, 1}, {0, 5, 0}, 1, turnA));
        boxes.addBody(boxBody({1, 1, 1}, Vec3{0, 5, 0} + offset, 1, turnB));
        World hulls;
        hulls.gravity = {};
        hulls.addBody(hullBody(cubeHull(), {0, 5, 0}, 1, turnA));
        hulls.addBody(hullBody(cubeHull(), Vec3{0, 5, 0} + offset, 1, turnB));
        boxes.step();
        hulls.step();
        check(boxes.contacts.empty() == hulls.contacts.empty(), "Same overlap verdict");
        if (boxes.contacts.empty()) {
            continue;
        }
        ++touching;
        double deepestBox = 0;
        double deepestHull = 0;
        for (const auto& c : boxes.contacts) {
            deepestBox = std::max(deepestBox, c.depth);
        }
        for (const auto& c : hulls.contacts) {
            deepestHull = std::max(deepestHull, c.depth);
        }
        near(deepestHull, deepestBox, 1e-6, "Same depth");
        // Near-ties between axes may resolve differently; identical manifolds must agree on the normal.
        if (boxes.contacts.size() == hulls.contacts.size()) {
            near(std::abs(hulls.contacts[0].normal.dot(boxes.contacts[0].normal)), 1, 0.2, "Similar normal");
        }
    }
    check(touching > 50, "Enough touching pairs");
}

TEST_CASE("hull against box and cylinder: rests on them with a manifold") {
    {
        World w;
        w.addBody(boxBody({2, 0.2, 2}, {0, 0.1, 0}, 0));
        int hull = w.addBody(hullBody(prismHull(6, 0.3, 0.3), {0, 0.35, 0}, 2));
        for (int i = 0; i < 960; ++i) {
            w.step();
        }
        const Body& b = w.bodies[static_cast<std::size_t>(hull)];
        near(b.position.y, 0.35, 3e-3, "Hull on a box");
        check(w.contacts.size() >= 3, "Hull-box manifold");
    }
    {
        World w;
        Body table;
        table.shape = Shape::Cylinder;
        table.size = {2, 2, 0.2};
        table.position = {0, 0.1, 0};
        table.orientation = Quat::axisAngle({1, 0, 0}, pi / 2); // Cap up.
        table.mass = 0;
        w.addBody(table);
        int hull = w.addBody(hullBody(cubeHull(0.4), {0, 0.4, 0}, 2));
        for (int i = 0; i < 960; ++i) {
            w.step();
        }
        const Body& b = w.bodies[static_cast<std::size_t>(hull)];
        near(b.position.y, 0.4, 3e-3, "Hull on a cylinder cap");
        check(w.contacts.size() >= 3, "Hull-cylinder manifold");
    }
    {
        // A wheel rolling down onto a hull block: contact normal from the lateral prism faces.
        World w;
        w.gravity = {};
        w.addBody(hullBody(cubeHull(1), {0, 5, 0}, 0));
        Body wheel;
        wheel.shape = Shape::Cylinder;
        wheel.size = {0.5, 0.5, 0.3};
        wheel.position = {0, 5.74, 0};
        w.addBody(wheel);
        w.step();
        check(!w.contacts.empty(), "Cylinder lateral contact on a hull");
        near(std::abs(w.contacts[0].normal.y), 1, 0.02, "Normal up");
    }
}

TEST_CASE("sphere against hull: closest point, all features, centre inside") {
    Quat turn = Quat::euler({0.2, 0.4, 0.6});
    auto expect = [&](Vec3 localCentre, double radius, double depth, Vec3 localNormal) {
        World w;
        w.gravity = {};
        Body block = hullBody(cubeHull(2), {1, 5, 2}, 0, turn);
        w.addBody(block);
        Body ball;
        ball.shape = Shape::Sphere;
        ball.size = Vec3{1, 1, 1} * (2 * radius);
        ball.position = block.point(localCentre);
        w.addBody(ball);
        Body twin = ball;
        w.step();
        check(w.contacts.size() == 1, "One sphere contact");
        const Contact& c = w.contacts[0];
        near(c.depth, depth, 1e-7, "Sphere depth");
        Vec3 n = turn.rotate(localNormal);
        double sign = c.a == 1 ? 1 : -1; // Normal points from B towards A: towards the sphere if it is A.
        nearVec(c.normal * sign, n, 1e-7, "Sphere normal");
    };
    expect({0, 1.4, 0}, 0.5, 0.1, {0, 1, 0});                                      // Face.
    expect({1.2, 1.2, 0}, 0.5, 0.5 - std::sqrt(0.08), Vec3{1, 1, 0}.normalized()); // Edge.
    expect({1.1, 1.1, 1.1}, 0.5, 0.5 - std::sqrt(0.03), Vec3{1, 1, 1}.normalized());
    expect({0.2, 0.7, 0.1}, 0.25, 0.25 + 0.3, {0, 1, 0}); // Centre inside: EPA.
    {
        World w;
        w.addBody(hullBody(cubeHull(1), {0, 5, 0}, 0));
        Body ball;
        ball.shape = Shape::Sphere;
        ball.size = {0.2, 0.2, 0.2};
        ball.position = {0.7, 5.7, 0};
        w.addBody(ball);
        w.step();
        check(w.contacts.empty(), "Near a corner but not touching");
    }
}

TEST_CASE("ball rolls off a hull roof and settles on the ground") {
    World w;
    std::vector<Vec3> roof{{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}, {-1, 0.5, 0}, {1, 0.5, 0}};
    auto hull = buildConvexHull(roof);
    Vec3 position;
    Quat orientation;
    hull->placeBody({}, {}, position, orientation);
    w.addBody(hullBody(hull, position, 0, orientation));
    Body ball;
    ball.shape = Shape::Sphere;
    ball.size = {0.2, 0.2, 0.2};
    ball.position = {0, 1.2, 0.05};
    int index = w.addBody(ball);
    for (int i = 0; i < 2400; ++i) {
        w.step();
    }
    const Body& b = w.bodies[static_cast<std::size_t>(index)];
    near(b.position.y, 0.1, 5e-3, "Ball on the ground");
    check(std::abs(b.position.z) > 1, "Ball rolled off the slope");
}

TEST_CASE("hull pile: 30 random hulls settle without explosions or tunnelling") {
    World w;
    std::mt19937 random(23);
    for (int i = 0; i < 30; ++i) {
        auto hull = buildConvexHull(randomCloud(random, 8 + i, 0.15));
        w.addBody(hullBody(hull, {(i % 3) * 0.4 - 0.4, 0.3 + (i / 9) * 0.4, ((i / 3) % 3) * 0.4 - 0.4},
                           1000 * hull->volume, Quat::euler({0.3 * i, 0.2 * i, 0.1 * i})));
    }
    for (int i = 0; i < 1800; ++i) {
        w.step();
    }
    for (const auto& b : w.bodies) {
        check(b.position.finite(), "Finite");
        check(b.position.y > -0.01 && b.position.y < 1.5, "Inside the pile");
        check(b.velocity.length() < 0.5, "Settling");
    }
}

// ---- Validation & persistence --------------------------------------------------------------------

TEST_CASE("world validation requires hull data") {
    World w;
    Body body;
    body.shape = Shape::ConvexHull;
    rejects([&] { w.addBody(body); });
    check(w.bodies.empty(), "Rejected body is not kept");
    rejects([] {
        Body none;
        setConvexHull(none, nullptr);
    });
}

TEST_CASE("robot format: hulls round-trip through version 6 byte for byte") {
    std::mt19937 random(13);
    World w;
    auto hull = buildConvexHull(randomCloud(random, 200, 0.3));
    auto scaled = std::make_shared<ConvexHull>(*hull);
    scaled->sourcePath = "meshes/rock.stl";
    scaled->sourceScale = {0.001, 0.002, 0.003};
    Body body = hullBody(scaled, {1, 2, 3}, 4, Quat::euler({0.1, 0.2, 0.3}));
    body.visuals.push_back(
        {"meshes/rock.stl", {0.001, 0.002, 0.003}, scaled->visualPosition(), scaled->visualOrientation()});
    body.ccd = true;
    w.addBody(body);
    w.addBody(hullBody(cubeHull(1), {3, 0.6, 0}, 2));
    w.continuousCollision = false;

    std::stringstream first;
    writeScene(w, first);
    check(first.str().rfind("ROBOTICS_SCENE 6", 0) == 0, "Version 6");
    World loaded = readScene(first);
    check(loaded.bodies.size() == 2 && !loaded.continuousCollision && loaded.bodies[0].ccd
              && !loaded.bodies[1].ccd,
          "Settings restored");
    const ConvexHull& a = *w.bodies[0].hull;
    const ConvexHull& b = *loaded.bodies[0].hull;
    check(a.vertices.size() == b.vertices.size(), "Vertex count");
    for (std::size_t i = 0; i < a.vertices.size(); ++i) {
        check(a.vertices[i].x == b.vertices[i].x && a.vertices[i].y == b.vertices[i].y
                  && a.vertices[i].z == b.vertices[i].z,
              "Bit-exact vertices in the same order");
    }
    check(a.faces.size() == b.faces.size(), "Same faces");
    near(a.volume, b.volume, 1e-12, "Same volume");
    nearVec(a.unitInertia, b.unitInertia, 1e-12, "Same inertia");
    check(a.sourceOffset.x == b.sourceOffset.x && a.sourceRotation.w == b.sourceRotation.w,
          "Same source frame");
    check(b.sourcePath == "meshes/rock.stl" && b.sourceScale.y == 0.002, "Mesh provenance");
    check(loaded.bodies[0].shape == Shape::ConvexHull && loaded.bodies[0].size.x == w.bodies[0].size.x,
          "Shape and bounds");

    std::stringstream second;
    writeScene(loaded, second);
    check(first.str() == second.str(), "Saving a loaded hull scene is byte-identical");

    // Same physics after the reload.
    World before = w;
    before.continuousCollision = loaded.continuousCollision = true;
    for (int i = 0; i < 240; ++i) {
        before.step();
        loaded.step();
    }
    nearVec(before.bodies[0].position, loaded.bodies[0].position, 1e-6, "Same trajectory");
}

TEST_CASE("robot format: every hull in a random scene survives") {
    std::mt19937 random(19);
    for (int run = 0; run < 100; ++run) {
        World w;
        auto hull = buildConvexHull(randomCloud(random, 6 + run, 0.5));
        w.addBody(hullBody(hull, {0, 2, 0}));
        std::stringstream text;
        writeScene(w, text);
        World loaded = readScene(text);
        check(loaded.bodies[0].hull->vertices.size() == hull->vertices.size(), "Vertex count");
        std::stringstream again;
        writeScene(loaded, again);
        check(text.str() == again.str(), "Stable");
    }
}

TEST_CASE("robot format: old worlds keep their version, damaged hull records are refused") {
    World plain;
    plain.addBody(boxBody({1, 1, 1}, {0, 1, 0}));
    std::stringstream text;
    writeScene(plain, text);
    check(text.str().rfind("ROBOTICS_SCENE 2", 0) == 0, "Hull-free worlds stay at the oldest version");

    rejects([] {
        std::stringstream v5("ROBOTICS_SCENE 5\nGRAVITY 0 -9.81 0 \nITERATIONS 24\nSOLVER 4 1\nBODIES 1\n"
                             "BODY \"h\" 3 1 1 1 1 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0.65 0.05\n"
                             "VISUALS 0 0\nJOINTS 0\nSENSORS 0\nEND\n");
        (void)readScene(v5);
    });
    World w;
    w.addBody(hullBody(cubeHull(1), {0, 2, 0}));
    std::stringstream good;
    writeScene(w, good);
    auto damaged = [&](const std::string& from, const std::string& to) {
        std::string text2 = good.str();
        auto where = text2.find(from);
        check(where != std::string::npos, "Test pattern present");
        text2.replace(where, from.size(), to);
        std::stringstream in(text2);
        rejects([&] { (void)readScene(in); });
    };
    damaged("HULL", "HALL");
    damaged(" 8\n", " 65\n");                    // Over the vertex cap.
    damaged(" 8\n", " 3\n");                     // Too few vertices, then garbage.
    damaged("-0.5 -0.5 -0.5", "-1.5 -0.5 -0.5"); // No longer centred on the centre of mass.
    damaged("-0.5 -0.5 -0.5", "nan 0 0");
}

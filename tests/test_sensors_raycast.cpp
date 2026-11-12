// Ray casts against boxes, spheres, cylinders and the ground; range sensors built on them.
#include "test_harness.hpp"

#include "robotics/sensors/raycast.hpp"
#include "robotics/sensors/sensor_system.hpp"

using namespace robotics;
using namespace test;

namespace {

Body shape(Shape kind, Vec3 size, Vec3 position = {}, Quat orientation = {}) {
    Body body;
    body.shape = kind;
    body.size = size;
    body.position = position;
    body.orientation = orientation;
    body.mass = 0;
    return body;
}

} // namespace

TEST_CASE("raycast box: axis-aligned, rotated, miss, max distance") {
    Body box = shape(Shape::Box, {2, 1, 1});
    RayHit hit = raycastBody(box, {5, 0, 0}, {-1, 0, 0}, 100);
    check(hit.hit, "Box hit");
    near(hit.distance, 4, 1e-12, "Box face distance");
    near(hit.normal.x, 1, 1e-12, "Box face normal");
    check(!raycastBody(box, {5, 0.6, 0}, {-1, 0, 0}, 100).hit, "Ray above the box misses");
    check(!raycastBody(box, {5, 0, 0}, {-1, 0, 0}, 3.9).hit, "Hit beyond max distance ignored");
    check(!raycastBody(box, {5, 0, 0}, {1, 0, 0}, 100).hit, "Box behind the ray ignored");
    // Unit cube turned 45 degrees about Y: the ray meets the vertical edge at x = sqrt(2) / 2.
    Body turned = shape(Shape::Box, {1, 1, 1}, {}, Quat::axisAngle({0, 1, 0}, pi / 4));
    hit = raycastBody(turned, {5, 0, 0}, {-1, 0, 0}, 100);
    near(hit.distance, 5 - std::sqrt(0.5), 1e-9, "Rotated box edge");
    // Off-centre ray onto the rotated face: plane x + z = sqrt(2)/2 (world), at z = 0.2 -> x = 0.507.
    hit = raycastBody(turned, {5, 0, 0.2}, {-1, 0, 0}, 100);
    near(hit.point.x, std::sqrt(0.5) - 0.2, 1e-9, "Rotated box face");
    near(hit.normal.x, std::sqrt(0.5), 1e-9, "Rotated box face normal");
    // Translated box, oblique ray.
    Body moved = shape(Shape::Box, {1, 1, 1}, {3, 2, 0});
    Vec3 direction = Vec3{2.5, 2, 0}.normalized();
    hit = raycastBody(moved, {0, 0, 0}, direction, 100);
    check(hit.hit && std::abs(hit.point.x - 2.5) < 1e-9 && std::abs(hit.point.y - 2) < 1e-9,
          "Oblique ray hits the box's -X face");
}

TEST_CASE("raycast sphere and ground") {
    Body ball = shape(Shape::Sphere, {2, 2, 2}, {0, 3, 0});
    RayHit hit = raycastBody(ball, {0, 10, 0}, {0, -1, 0}, 100);
    near(hit.distance, 6, 1e-12, "Sphere top");
    near(hit.normal.y, 1, 1e-12, "Sphere normal");
    hit = raycastBody(ball, {-5, 3.5, 0}, {1, 0, 0}, 100);
    near(hit.point.x, -std::sqrt(0.75), 1e-12, "Sphere chord");
    check(!raycastBody(ball, {-5, 4.1, 0}, {1, 0, 0}, 100).hit, "Sphere miss");
    hit = raycastGround({0, 2, 0}, Vec3{1, -1, 0}.normalized(), 100);
    near(hit.distance, 2 * std::sqrt(2.0), 1e-12, "Ground at 45 degrees");
    check(hit.body == groundBodyIndex && hit.normal.y == 1, "Ground hit identity");
    check(!raycastGround({0, 2, 0}, {0, 1, 0}, 100).hit, "Upward ray misses the ground");
}

TEST_CASE("raycast cylinder: side, caps, tilted") {
    // Diameter 1 (radius 0.5), width 2 along local Z.
    Body cylinder = shape(Shape::Cylinder, {1, 1, 2});
    RayHit hit = raycastBody(cylinder, {5, 0, 0.9}, {-1, 0, 0}, 100);
    near(hit.distance, 4.5, 1e-12, "Cylinder side");
    near(hit.normal.x, 1, 1e-12, "Cylinder side normal");
    hit = raycastBody(cylinder, {5, 0.3, 0}, {-1, 0, 0}, 100);
    near(hit.point.x, 0.4, 1e-12, "Cylinder side chord");
    check(!raycastBody(cylinder, {5, 0, 1.1}, {-1, 0, 0}, 100).hit, "Beyond the cap misses");
    hit = raycastBody(cylinder, {0.2, 0.1, 6}, {0, 0, -1}, 100);
    near(hit.distance, 5, 1e-12, "Through the +Z cap");
    near(hit.normal.z, 1, 1e-12, "Cap normal");
    hit = raycastBody(cylinder, {0, 0, -4}, {0, 0, 1}, 100);
    near(hit.distance, 3, 1e-12, "Through the -Z cap");
    check(!raycastBody(cylinder, {0.6, 0, 6}, {0, 0, -1}, 100).hit, "Parallel ray outside the radius misses");
    // Oblique ray entering through the cap rim region: from (0, 0, 3) towards (0.4, 0, 0).
    Vec3 direction = Vec3{0.4, 0, -3}.normalized();
    hit = raycastBody(cylinder, {0, 0, 3}, direction, 100);
    near(hit.point.z, 1, 1e-12, "Oblique cap entry");
    // Wheel lying on its side (axis along world X after rotating local Z about Y).
    Body wheel = shape(Shape::Cylinder, {1, 1, 0.4}, {0, 1, 0}, Quat::axisAngle({0, 1, 0}, pi / 2));
    hit = raycastBody(wheel, {0, 5, 0}, {0, -1, 0}, 100);
    near(hit.distance, 3.5, 1e-9, "Rotated cylinder side from above");
    hit = raycastBody(wheel, {3, 1, 0}, {-1, 0, 0}, 100);
    near(hit.distance, 2.8, 1e-9, "Rotated cylinder cap");
}

TEST_CASE("raycast from inside a shape reports distance zero") {
    for (Shape kind : {Shape::Box, Shape::Sphere, Shape::Cylinder}) {
        Body body = shape(kind, {1, 1, 1}, {2, 2, 2});
        RayHit hit = raycastBody(body, {2.1, 2, 2}, {1, 0, 0}, 10);
        check(hit.hit && hit.distance == 0, "Inside ray must hit at 0");
        near(hit.normal.x, -1, 1e-12, "Inside normal opposes the ray");
    }
    check(raycastGround({0, -1, 0}, {0, 1, 0}, 10).hit, "Below the ground is occluded");
}

TEST_CASE("world raycast picks the nearest body and skips the ignored one") {
    World world;
    world.gravity = {};
    world.addBody(shape(Shape::Box, {1, 1, 1}, {0, 1, 0}));
    world.addBody(shape(Shape::Box, {1, 1, 1}, {3, 1, 0}));
    world.addBody(shape(Shape::Sphere, {1, 1, 1}, {6, 1, 0}));
    RayHit hit = raycast(world, {0, 1, 0}, {1, 0, 0}, 100);
    check(hit.body == 0 && hit.distance == 0, "Ray from inside body 0");
    hit = raycast(world, {0, 1, 0}, {1, 0, 0}, 100, 0);
    check(hit.body == 1, "Nearest other body");
    near(hit.distance, 2.5, 1e-12, "Distance to body 1");
    hit = raycast(world, {0, 1, 0}, {0, -1, 0}, 100, 0);
    check(hit.hit && hit.body == groundBodyIndex, "Ground below");
    near(hit.distance, 1, 1e-12, "Ground distance");
    check(!raycast(world, {0, 1, 0}, {0, -1, 0}, 100, 0, false).hit, "Ground can be excluded");
}

TEST_CASE("lidar excludes its own body and matches the analytic wall") {
    World world;
    world.gravity = {};
    Body base = shape(Shape::Box, {0.6, 0.4, 0.6}, {0, 1, 0});
    world.addBody(base);
    world.addBody(shape(Shape::Box, {0.2, 2, 20}, {3.1, 1, 0})); // Wall face at x = 3.
    Sensor lidar = makeSensor(SensorType::Lidar2D, 0);
    lidar.noise = {};
    lidar.rate = 0;
    lidar.beams = 91;
    lidar.fov = pi / 2;
    lidar.maxRange = 10;
    world.sensors.push_back(lidar);
    Sensor range = makeSensor(SensorType::Rangefinder, 0);
    range.noise = {};
    range.rate = 0;
    world.sensors.push_back(range);
    stepSimulation(world);
    const auto& scan = world.sensors[0].state.value;
    check(scan.size() == 91, "Beam count");
    for (int k = 0; k < 91; ++k) {
        double angle = -pi / 4 + (pi / 2) * k / 90;
        near(scan[static_cast<std::size_t>(k)], 3 / std::cos(angle), 1e-9, "Lidar range to the wall");
    }
    near(world.sensors[1].state.value[0], 3, 1e-9, "Rangefinder ignores its own body");
    check(world.sensors[0].state.rayHits[45] == 1, "Hit flag for visualization");
    // Beyond max range: the reading is maxRange and the beam reports no hit.
    world.sensors[1].maxRange = 2;
    world.sensors[1].state = {};
    stepSimulation(world);
    near(world.sensors[1].state.value[0], 2, 1e-12, "No return reads max range");
    check(world.sensors[1].state.rayHits[0] == 0, "No-hit flag");
}

TEST_CASE("noisy lidar: no-return beams read exactly max range") {
    World world;
    world.gravity = {};
    Body base;
    base.mass = 0;
    base.position = {0, 1, 0};
    world.addBody(base);
    Sensor lidar = makeSensor(SensorType::Lidar2D, 0);
    lidar.rate = 0;
    lidar.noise.stddev = 0.2;
    world.sensors.push_back(lidar);
    for (int i = 0; i < 20; ++i) {
        stepSimulation(world);
        for (double range : world.sensors[0].state.value) {
            check(range == lidar.maxRange, "Nothing in range must read maxRange without noise");
        }
    }
}

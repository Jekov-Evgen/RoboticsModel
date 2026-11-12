// Integration, collision detection and contact response.
#include "test_harness.hpp"

using namespace robotics;
using namespace test;

TEST_CASE("free fall matches semi-implicit integration") {
    World w;
    w.addBody(sphere({0, 100, 0}));
    advance(w, 240);
    near(w.bodies[0].velocity.y, -9.81, 1e-9, "Free-fall velocity");
    // Semi-implicit Euler over n = 240 * substeps substeps of h = 1 / n: y = y0 - g (1 + h) / 2 after 1 s.
    // (Before substepping n was 240; the exact value changed with the substep count, not the method.)
    const double n = 240.0 * w.substeps;
    near(w.bodies[0].position.y, 100 - 9.81 * ((n + 1) / n) / 2, 1e-8, "Free-fall position");
}

TEST_CASE("static body does not move") {
    World w;
    Body b;
    b.mass = 0;
    b.position = {0, 4, 0};
    w.addBody(b);
    advance(w, 100);
    near(w.bodies[0].position.y, 4, 1e-12, "Static body moved");
}

TEST_CASE("sphere rests on ground") {
    World w;
    w.addBody(sphere({0, 3, 0}));
    advance(w, 1200);
    near(w.bodies[0].position.y, 0.5, 0.005, "Ground height");
    near(w.bodies[0].velocity.y, 0, 0.02, "Rest velocity");
}

TEST_CASE("oriented box rests on ground") {
    World w;
    Body b;
    b.position = {0, 2, 0};
    b.orientation = Quat::euler({0.2, 0, 0.35});
    w.addBody(b);
    advance(w, 1800);
    for (auto v : w.bodies[0].vertices()) {
        check(v.y > -0.015, "Box penetrates ground");
    }
    check(w.bodies[0].velocity.length() < 0.1, "Box did not settle");
}

TEST_CASE("sphere collision conserves linear momentum") {
    World w;
    w.gravity = {};
    auto a = sphere({-0.49, 3, 0});
    auto b = sphere({0.49, 3, 0});
    a.velocity = {1, 0, 0};
    b.velocity = {-1, 0, 0};
    a.restitution = b.restitution = 1;
    w.addBody(a);
    w.addBody(b);
    w.step();
    near((w.bodies[0].velocity + w.bodies[1].velocity).length(), 0, 1e-9, "Momentum changed");
    check(w.bodies[0].velocity.x < 0 && w.bodies[1].velocity.x > 0, "Spheres did not bounce");
}

TEST_CASE("sphere versus rotated box") {
    World w;
    w.gravity = {};
    Body box;
    box.mass = 0;
    box.size = {2, 1, 1};
    box.position = {0, 3, 0};
    box.orientation = Quat::axisAngle({0, 0, 1}, 0.4);
    w.addBody(box);
    auto ball = sphere(box.point({0, 0.8, 0}));
    ball.velocity = box.orientation.rotate({0, -1, 0});
    w.addBody(ball);
    Vec3 up = box.orientation.rotate({0, 1, 0});
    w.step();
    check(!w.contacts.empty(), "Missing sphere-box contact");
    // The contact normal points from B (sphere) to A (box), i.e. along the box's -Y face normal.
    check(w.contacts.front().normal.dot(up) < -0.999, "Wrong contact normal");
    // Split impulses: the 0.2 m overlap is removed by moving the sphere out along +Y, while its 1 m/s
    // approach (below the restitution threshold) is stopped, not reversed by the position correction.
    // (Before split impulses the Baumgarte push left the sphere with a positive velocity along +Y.)
    check(w.bodies[1].velocity.dot(up) > -1e-9, "Sphere still approaching the box");
    check((w.bodies[1].position - box.point({0, 0.8, 0})).dot(up) > 0, "Sphere not pushed out along the normal");
}

TEST_CASE("OBB separating axes") {
    World w;
    w.gravity = {};
    Body a;
    a.position = {0, 4, 0};
    a.size = {1, 1, 1};
    a.orientation = Quat::euler({0.1, 0.2, 0.4});
    w.addBody(a);
    a.position.x = 0.8;
    w.addBody(a);
    w.step();
    check(!w.contacts.empty(), "Missing box-box collision");
    w.bodies[1].position.x = 5;
    w.step();
    check(w.contacts.empty(), "False box-box collision");
}

TEST_CASE("cylinder ground contact and inertia") {
    World w;
    Body wheel;
    wheel.shape = Shape::Cylinder;
    wheel.size = {0.8, 0.8, 0.2};
    wheel.mass = 2;
    wheel.position = {0, 2, 0};
    w.addBody(wheel);
    advance(w, 1200);
    near(w.bodies[0].position.y, 0.4, 0.005, "Cylinder did not rest on its radius");
    near(wheel.inverseInertia({0, 0, 1}).z, 1 / (0.5 * 2 * 0.4 * 0.4), 1e-10, "Wrong cylinder axial inertia");
    w = World{};
    wheel.orientation = Quat::axisAngle({1, 0, 0}, pi / 2);
    w.addBody(wheel);
    advance(w, 1200);
    near(w.bodies[0].position.y, 0.1, 0.005, "Cylinder cap contact failed");
}

TEST_CASE("sphere-cylinder uses circular surface") {
    World w;
    w.gravity = {};
    Body cylinder;
    cylinder.mass = 0;
    cylinder.shape = Shape::Cylinder;
    cylinder.size = {1, 1, 0.4};
    cylinder.position = {0, 3, 0};
    w.addBody(cylinder);
    w.addBody(sphere({0.4, 3.4, 0}, 0.1));
    w.step();
    check(w.contacts.empty(), "Cylinder bounds caused false sphere contact");
    w.bodies[1].position = {0.5, 3, 0};
    w.step();
    check(!w.contacts.empty(), "Sphere-cylinder rim contact missing");
}

TEST_CASE("contact impulses remain readable after step") {
    World w;
    w.addBody(sphere({0, 0.49, 0}));
    w.step();
    check(!w.contacts.empty(), "Resting sphere has no ground contact");
    const Contact& contact = w.contacts.front();
    check(contact.touchesGround() && contact.normalImpulse > 0, "Normal impulse not retained after step");
    near(contact.totalImpulse().y, contact.normalImpulse + contact.tangentImpulse.y, 1e-12, "Total impulse");
}

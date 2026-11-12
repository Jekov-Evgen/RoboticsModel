// Continuous collision: tunnelling, thin plates, spinning rods, ghost hits and inertness for slow bodies.
#include "test_harness.hpp"

#include "../src/physics/broadphase.hpp"
#include "../src/physics/ccd.hpp"

#include <algorithm>
#include <random>

using namespace robotics;
using namespace robotics::detail;
using namespace test;

namespace {

Body ball(double diameter, Vec3 position, Vec3 velocity, double mass = 0.05) {
    Body b;
    b.shape = Shape::Sphere;
    b.size = Vec3{1, 1, 1} * diameter;
    b.position = position;
    b.velocity = velocity;
    b.mass = mass;
    b.restitution = 0;
    return b;
}

Body plate(Vec3 size, Vec3 position, double mass = 0) {
    Body b;
    b.size = size;
    b.position = position;
    b.mass = mass;
    return b;
}

std::shared_ptr<const ConvexHull> octahedron(double radius) {
    return buildConvexHull(
        {{radius, 0, 0}, {-radius, 0, 0}, {0, radius, 0}, {0, -radius, 0}, {0, 0, radius}, {0, 0, -radius}});
}

} // namespace

TEST_CASE("ccd: 2 cm ball at 50 m/s stops at a 1 cm static plate") {
    for (double start : {1.0, 0.973, 0.931, 0.9, 0.85}) { // Different phases against the step length.
        World w;
        w.gravity = {};
        w.addBody(plate({2, 2, 0.01}, {0, 3, 0}));
        int b = w.addBody(ball(0.02, {0, 3, -start}, {0, 0, 50}));
        for (int i = 0; i < 240; ++i) {
            w.step();
            check(w.bodies[static_cast<std::size_t>(b)].position.z < 0, "Ball tunnelled through the plate");
        }
        near(w.bodies[static_cast<std::size_t>(b)].position.z, -0.015, 2e-3, "Rests on the plate surface");
    }
}

TEST_CASE("ccd: without it the same ball tunnels") {
    World w;
    w.gravity = {};
    w.continuousCollision = false;
    w.addBody(plate({2, 2, 0.01}, {0, 3, 0}));
    int b = w.addBody(ball(0.02, {0, 3, -0.973}, {0, 0, 50}));
    for (int i = 0; i < 120; ++i) {
        w.step();
    }
    check(w.bodies[static_cast<std::size_t>(b)].position.z > 1, "The scene really is a tunnelling case");
}

TEST_CASE("ccd: restitution works through a speculative contact") {
    World w;
    w.gravity = {};
    w.addBody(plate({2, 2, 0.01}, {0, 3, 0}));
    Body b = ball(0.02, {0, 3, -0.973}, {0, 0, 50});
    b.restitution = 0.5;
    w.bodies[0].restitution = 0.5;
    w.addBody(b);
    int steps = 0;
    while (w.bodies[1].velocity.z > 0 && steps < 100) {
        w.step();
        ++steps;
    }
    check(steps < 100, "Ball bounced");
    near(w.bodies[1].velocity.z, -25, 1, "Rebound speed e·v");
    check(!w.contacts.empty() && w.contacts[0].normalImpulse > 0 && w.contacts[0].depth == 0,
          "The impact shows as a contact with its impulse");
}

TEST_CASE("ccd: both bodies moving, oblique and against every shape") {
    std::vector<Body> targets;
    targets.push_back(plate({2, 0.01, 2}, {0, 3, 0}));
    Body cylinder = plate({1.5, 1.5, 0.02}, {0, 3, 0});
    cylinder.shape = Shape::Cylinder;
    cylinder.orientation = Quat::axisAngle({1, 0, 0}, pi / 2);
    targets.push_back(cylinder);
    Body hull;
    setConvexHull(hull, buildConvexHull({{-1, -0.005, -1},
                                         {1, -0.005, -1},
                                         {1, -0.005, 1},
                                         {-1, -0.005, 1},
                                         {-1, 0.005, -1},
                                         {1, 0.005, -1},
                                         {1, 0.005, 1},
                                         {-1, 0.005, 1}}));
    hull.position = {0, 3, 0};
    hull.mass = 0;
    targets.push_back(hull);
    for (const Body& target : targets) {
        World w;
        w.gravity = {};
        w.addBody(target);
        int b = w.addBody(ball(0.02, {-0.31, 4, 0.13}, {15, -48, -5}));
        for (int i = 0; i < 120; ++i) {
            w.step();
            check(w.bodies[static_cast<std::size_t>(b)].position.y > 3, "Oblique ball went through");
        }
    }
    // A fast plate swats a resting ball.
    World w;
    w.gravity = {};
    Body moving = plate({1, 0.01, 1}, {0, 5, 0}, 20);
    moving.velocity = {0, -60, 0};
    w.addBody(moving);
    int b = w.addBody(ball(0.05, {0.1, 4.1, 0}, {}, 0.1));
    for (int i = 0; i < 120 && w.bodies[0].position.y > 2; ++i) {
        w.step();
        check(w.bodies[static_cast<std::size_t>(b)].position.y < w.bodies[0].position.y,
              "Plate passed the ball");
    }
}

TEST_CASE("ccd: small fast hull and box do not tunnel, and thin shapes block them") {
    World w;
    w.gravity = {};
    w.addBody(plate({3, 3, 0.01}, {0, 3, 0}));
    Body rock;
    setConvexHull(rock, octahedron(0.02));
    rock.position = {0, 3, -1};
    rock.velocity = {0, 0, 80};
    rock.mass = 0.05;
    int r = w.addBody(rock);
    Body chip = plate({0.02, 0.02, 0.02}, {1, 3, -1}, 0.05);
    chip.velocity = {0, 0, 80};
    chip.orientation = Quat::euler({0.3, 0.2, 0.1});
    int c = w.addBody(chip);
    for (int i = 0; i < 240; ++i) {
        w.step();
        check(w.bodies[static_cast<std::size_t>(r)].position.z < 0, "Hull went through");
        check(w.bodies[static_cast<std::size_t>(c)].position.z < 0, "Box went through");
    }
}

TEST_CASE("ccd: fast-spinning thin rod does not pass a thin plate") {
    for (double omega : {60.0, 150.0, 300.0}) {
        for (double phase : {0.0, 0.4, 1.1}) {
            World w;
            w.gravity = {};
            w.addBody(plate({4, 0.01, 4}, {0, 3, 0}));
            Body rod = plate({1, 0.02, 0.02}, {0, 3.6, 0}, 1);
            rod.orientation = Quat::axisAngle({0, 0, 1}, phase);
            rod.angularVelocity = {0, 0, omega};
            rod.velocity = {0, -2, 0};
            int r = w.addBody(rod);
            double deepest = 10;
            for (int i = 0; i < 480; ++i) {
                w.step();
                const Body& b = w.bodies[static_cast<std::size_t>(r)];
                check(b.position.y > 3, "Rod centre went through the plate");
                for (double end : {-0.5, 0.5}) {
                    deepest = std::min(deepest, b.point({end, 0, 0}).y);
                }
            }
            check(deepest > 3 - 0.06, "Rod tip stays at the plate");
            const Body& b = w.bodies[static_cast<std::size_t>(r)];
            check(b.position.finite() && b.velocity.length() < 100, "No energy from the impact");
        }
    }
}

TEST_CASE("ccd: random fast bodies never cross a closed thin shell") {
    // A box room of 1 cm walls; everything thrown around inside it must stay inside, with no energy gain.
    std::mt19937 random(61);
    std::uniform_real_distribution<double> unit(-1, 1);
    World w;
    w.gravity = {};
    const double half = 1;
    const Vec3 centre{0, 3, 0};
    for (int axis = 0; axis < 3; ++axis) {
        for (double side : {-1.0, 1.0}) {
            Vec3 size{2.2, 2.2, 2.2};
            size[axis] = 0.01;
            Vec3 at = centre;
            at[axis] += side * (half + 0.005);
            w.addBody(plate(size, at));
        }
    }
    const int walls = static_cast<int>(w.bodies.size());
    double energy = 0;
    for (int i = 0; i < 24; ++i) {
        Vec3 velocity = Vec3{unit(random), unit(random), unit(random)}.normalized() * 60;
        Vec3 at = centre + Vec3{unit(random), unit(random), unit(random)} * 0.8;
        Body body;
        if (i % 3 == 0) {
            body = ball(0.02, at, velocity);
        } else if (i % 3 == 1) {
            body = plate({0.03, 0.03, 0.03}, at, 0.05);
            body.angularVelocity = Vec3{unit(random), unit(random), unit(random)} * 50;
        } else {
            setConvexHull(body, octahedron(0.02));
            body.position = at;
            body.mass = 0.05;
            body.angularVelocity = Vec3{unit(random), unit(random), unit(random)} * 50;
        }
        body.velocity = velocity;
        body.restitution = 0.8;
        energy += 0.5 * body.mass * velocity.length2();
        w.addBody(body);
    }
    for (auto& b : w.bodies) {
        b.restitution = 0.8;
    }
    for (int step = 0; step < 720; ++step) {
        w.step();
        double total = 0;
        for (int i = walls; i < static_cast<int>(w.bodies.size()); ++i) {
            const Body& b = w.bodies[static_cast<std::size_t>(i)];
            Vec3 d = b.position - centre;
            check(std::max({std::abs(d.x), std::abs(d.y), std::abs(d.z)}) < half, "A body left the room");
            total += 0.5 * b.mass * b.velocity.length2();
        }
        check(total <= energy * 1.01, "Impacts must not add linear kinetic energy");
    }
}

TEST_CASE("ccd: no ghost hit beside a plate or a corner") {
    // Grazing past the plate's edge: the plate's plane is crossed, the plate is not.
    World w;
    w.gravity = {};
    w.addBody(plate({1, 0.01, 1}, {0, 30, 0}));
    int b = w.addBody(ball(0.02, {0.6, 30.1, 0}, {0, -50, 0}));
    for (int i = 0; i < 24; ++i) {
        w.step();
        check(w.contacts.empty(), "No contact while passing beside the plate");
    }
    near(w.bodies[static_cast<std::size_t>(b)].velocity.y, -50, 1e-9, "Velocity untouched");
    near(w.bodies[static_cast<std::size_t>(b)].velocity.x, 0, 1e-12, "No sideways deflection");

    // Flying parallel over the plate, closer than one step of motion.
    World flyby;
    flyby.gravity = {};
    flyby.addBody(plate({2, 0.01, 2}, {0, 3, 0}));
    int f = flyby.addBody(ball(0.02, {-2, 3.03, 0}, {50, 0, 0}));
    for (int i = 0; i < 40; ++i) {
        flyby.step();
        check(flyby.contacts.empty(), "No contact while flying over");
    }
    near(flyby.bodies[static_cast<std::size_t>(f)].velocity.x, 50, 1e-9, "Fly-by keeps its speed");
}

TEST_CASE("ccd: fast bodies stop at the ground") {
    World w;
    w.addBody(ball(0.02, {0, 0.5, 0}, {0, -200, 0}));
    Body chip = plate({0.05, 0.05, 0.05}, {1, 0.4, 0}, 0.1);
    chip.velocity = {0, -150, 0};
    chip.restitution = 0;
    w.addBody(chip);
    for (int i = 0; i < 120; ++i) {
        w.step();
        check(w.bodies[0].position.y > 0.005, "Ball sank into the ground");
        check(w.bodies[1].position.y > 0.01, "Box sank into the ground");
    }
    near(w.bodies[0].position.y, 0.01, 1.5e-3, "Ball rests on the surface");
    near(w.bodies[1].position.y, 0.025, 1.5e-3, "Box rests on the surface");
}

TEST_CASE("ccd: flag covers slow small bodies and thin moving plates") {
    BodyMotion slow = predictMotion(ball(0.5, {}, {1, 0, 0}), 1.0 / 240, {});
    check(!slow.active, "A slow body is not swept");
    Body flagged = ball(0.5, {}, {1, 0, 0});
    flagged.ccd = true;
    check(predictMotion(flagged, 1.0 / 240, {}).active, "Body::ccd forces it");
    check(predictMotion(ball(0.02, {}, {50, 0, 0}), 1.0 / 240, {}).active,
          "A fast body is swept automatically");
    Body fixed = ball(0.02, {}, {50, 0, 0}, 0);
    check(!predictMotion(fixed, 1.0 / 240, {}).active, "Static bodies never are");
    Body spinning = ball(0.02, {}, {});
    spinning.angularVelocity = {1000, 0, 0};
    check(!predictMotion(spinning, 1.0 / 240, {}).active, "Spinning spheres do not sweep anything");
    Body wheel = plate({0.3, 0.3, 0.05}, {}, 1);
    wheel.shape = Shape::Cylinder;
    wheel.angularVelocity = {0, 0, 100};
    check(!predictMotion(wheel, 1.0 / 240, {}).active, "Nor do cylinders about their axis");
}

TEST_CASE("ccd: swept broad phase finds the pair across the gap and keeps brute-force pairs") {
    std::vector<Body> bodies{plate({1, 0.01, 1}, {0, 3, 0}), ball(0.02, {0, 3.5, 0}, {0, -200, 0})};
    check(broadphasePairs(bodies, {}).empty(), "Static bounds do not overlap");
    BodyMotion motion = predictMotion(bodies[1], 1.0 / 240, {});
    std::vector<Aabb> swept{worldBounds(bodies[0]), sweptBounds(bodies[1], motion, 1.0 / 240)};
    auto pairs = broadphasePairs(bodies, {}, &swept);
    check(pairs.size() == 1 && pairs[0] == BodyPair(0, 1), "Swept bounds overlap");

    Impact impact;
    BodyMotion still = predictMotion(bodies[0], 1.0 / 240, {});
    check(pairTimeOfImpact(bodies[1], motion, bodies[0], still, 1.0 / 240, impact), "Impact found");
    near(impact.time, (0.5 - 0.005 - 0.01) / 200, 1e-5, "Time of impact");
    near(impact.normal.y, 1, 1e-9, "Normal from the plate towards the ball");
}

TEST_CASE("ccd: resting stacks are unaffected, also when every body is flagged") {
    auto stack = [](bool flagged, bool continuous) {
        World w;
        w.continuousCollision = continuous;
        for (int i = 0; i < 8; ++i) {
            Body box = plate({0.4, 0.2, 0.4}, {0.005 * (i % 2), 0.1 + 0.2 * i, 0}, 1);
            box.ccd = flagged;
            w.addBody(box);
        }
        for (int i = 0; i < 2400; ++i) {
            w.step();
        }
        return w;
    };
    World plain = stack(false, false);
    World enabled = stack(false, true);
    World flagged = stack(true, true);
    for (std::size_t i = 0; i < plain.bodies.size(); ++i) {
        check(plain.bodies[i].position.y == enabled.bodies[i].position.y
                  && plain.bodies[i].position.x == enabled.bodies[i].position.x,
              "Slow bodies are bit-identical with continuous collision on");
        near(flagged.bodies[i].position.y, plain.bodies[i].position.y, 2e-3, "Flagged stack height");
        near(flagged.bodies[i].position.x, plain.bodies[i].position.x, 2e-3, "Flagged stack position");
        check(flagged.bodies[i].velocity.length() < 0.01, "Flagged stack at rest");
    }
}

TEST_CASE("ccd: a fast link on a joint is stopped by a plate without tearing the joint") {
    World w;
    w.gravity = {};
    Body base = plate({0.1, 0.1, 0.1}, {0, 2, 0}, 0);
    int ib = w.addBody(base);
    Body link = plate({0.8, 0.03, 0.03}, {0.45, 2, 0}, 1);
    link.angularVelocity = {0, 0, -60};
    link.velocity = {0, -27, 0};
    int il = w.addBody(link);
    w.connect(ib, il, JointType::Hinge, {0.05, 2, 0}, {0, 0, 1});
    w.addBody(plate({2, 0.01, 2}, {0.5, 1.4, 0}));
    double drift = 0;
    for (int i = 0; i < 240; ++i) {
        w.step();
        drift = std::max(drift, w.maxAnchorError());
        check(w.bodies[static_cast<std::size_t>(il)].point({0.4, 0, 0}).y > 1.3,
              "Link swung through the plate");
    }
    check(drift < 0.01, "Joint stays together through the impact");
}

TEST_CASE("ccd: joint-connected fast links keep the joint exclusion") {
    World w;
    w.gravity = {};
    Body a = plate({0.3, 0.02, 0.02}, {0, 2, 0}, 1);
    Body b = plate({0.3, 0.02, 0.02}, {0.3, 2, 0}, 1);
    a.angularVelocity = {0, 0, 200};
    b.angularVelocity = {0, 0, -200};
    int ia = w.addBody(a);
    int ib = w.addBody(b);
    w.connect(ia, ib, JointType::Ball, {0.15, 2, 0});
    for (int i = 0; i < 240; ++i) {
        w.step();
        check(w.contacts.empty(), "Connected bodies never collide");
    }
    check(w.maxAnchorError() < 1e-3, "Joint holds");
}

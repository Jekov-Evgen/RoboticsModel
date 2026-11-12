// Contact manifolds, stacking and warm starting.
#include "test_harness.hpp"

#include <algorithm>
#include <cstddef>

using namespace robotics;
using namespace test;

namespace {

Body box(Vec3 position, Vec3 size = {0.5, 0.5, 0.5}, double mass = 1) {
    Body body;
    body.position = position;
    body.size = size;
    body.mass = mass;
    return body;
}

Body cylinder(Vec3 position, Quat orientation, double diameter = 0.6, double width = 0.4) {
    Body body;
    body.shape = Shape::Cylinder;
    body.size = {diameter, diameter, width};
    body.position = position;
    body.orientation = orientation;
    return body;
}

std::size_t contactsBetween(const World& world, int a, int b) {
    return static_cast<std::size_t>(
        std::count_if(world.contacts.begin(), world.contacts.end(),
                      [&](const Contact& c) { return (c.a == a && c.b == b) || (c.a == b && c.b == a); }));
}

// Largest displacement of any body between two snapshots.
double maxDrift(const std::vector<Vec3>& before, const World& world) {
    double drift = 0;
    for (std::size_t i = 0; i < before.size(); ++i) {
        drift = std::max(drift, (world.bodies[i].position - before[i]).length());
    }
    return drift;
}

std::vector<Vec3> positions(const World& world) {
    std::vector<Vec3> result;
    for (const auto& body : world.bodies) {
        result.push_back(body.position);
    }
    return result;
}

double maxSpeed(const World& world) {
    double speed = 0;
    for (const auto& body : world.bodies) {
        speed = std::max(speed, body.velocity.length() + body.angularVelocity.length() * 0.25);
    }
    return speed;
}

} // namespace

TEST_CASE("box resting on a box face has a four point manifold") {
    World w;
    w.gravity = {};
    w.addBody(box({0, 1, 0}, {2, 1, 2}, 0)); // Static table, top at y = 1.5.
    w.addBody(box({0.1, 1.74, 0.05}));       // 1 cm penetration.
    w.step();
    check(contactsBetween(w, 0, 1) == 4, "Box face contact must have 4 points");
    for (const auto& c : w.contacts) {
        near(std::abs(c.normal.y), 1, 1e-9, "Face normal (from B to A: table is A, so it points down)");
        near(c.depth, 0.01, 1e-3, "Manifold depth");
    }
    // Box rotated 45 degrees about Y on a larger box: clipped octagon reduced to 4 points.
    w.bodies[1].orientation = Quat::axisAngle({0, 1, 0}, pi / 4);
    w.contacts.clear();
    w.step();
    check(contactsBetween(w, 0, 1) == 4, "Rotated box must keep 4 points");
    // Ground contact of a resting box: 4 corners.
    World g;
    g.addBody(box({0, 0.245, 0}));
    g.step();
    check(g.contacts.size() == 4, "Box on ground must have 4 points");
}

TEST_CASE("edge-edge box contact produces one point between the edges") {
    World w;
    w.gravity = {};
    Body lower = box({0, 0, 0}, {1, 1, 1}, 0);
    lower.orientation = Quat::axisAngle({1, 0, 0}, pi / 4); // Edge along X on top at y = 0.7071.
    w.addBody(lower);
    Body upper = box({0, 1.40, 0}, {1, 1, 1});
    upper.orientation = Quat::axisAngle({0, 0, 1}, pi / 4); // Edge along Z at the bottom.
    w.addBody(upper);
    w.step();
    check(contactsBetween(w, 0, 1) == 1, "Crossed edges must give a single contact");
    const Contact& c = w.contacts.front();
    near(c.point.x, 0, 1e-6, "Edge contact x");
    near(c.point.z, 0, 1e-6, "Edge contact z");
    near(std::abs(c.normal.y), 1, 1e-6, "Edge contact normal");
}

TEST_CASE("cylinder manifolds on its side and on its cap") {
    // Ground: lying on the side = 2 rim points (a line), standing on a cap = 4 points.
    World side;
    side.addBody(cylinder({0, 0.295, 0}, Quat::axisAngle({0, 1, 0}, 0.3)));
    side.step();
    check(side.contacts.size() == 2, "Lying cylinder on ground needs 2 points");
    World cap;
    cap.addBody(cylinder({0, 0.195, 0}, Quat::axisAngle({1, 0, 0}, pi / 2)));
    cap.step();
    check(cap.contacts.size() == 4, "Standing cylinder on ground needs 4 points");

    // On a box: flat lateral face -> 4 points; cap -> 4 points.
    World onBox;
    onBox.gravity = {};
    onBox.addBody(box({0, 0.5, 0}, {2, 1, 2}, 0));
    // Rotate about the axis by half a facet so a lateral face points straight down.
    Quat lying = Quat::axisAngle({0, 0, 1}, pi / 32 + pi / 2);
    onBox.addBody(cylinder({0, 1.295, 0}, lying));
    onBox.step();
    check(contactsBetween(onBox, 0, 1) == 4, "Cylinder lying on a box needs 4 points");
    onBox.bodies[1].orientation = Quat::axisAngle({1, 0, 0}, pi / 2);
    onBox.bodies[1].position = {0, 1.195, 0};
    onBox.contacts.clear();
    onBox.step();
    check(contactsBetween(onBox, 0, 1) == 4, "Cylinder standing on a box needs 4 points");
}

TEST_CASE("cylinder settles stably on its cap and on a box") {
    World w;
    w.addBody(box({0, 0.5, 0}, {2, 1, 2}, 0));
    w.addBody(cylinder({0, 1.25, 0}, Quat::axisAngle({1, 0, 0}, pi / 2)));
    w.addBody(cylinder({1.5, 0.35, 1.5}, Quat::axisAngle({1, 0, 0}, pi / 2)));
    advance(w, 240);
    auto start = positions(w);
    advance(w, 4 * 240);
    check(maxDrift(start, w) < 0.002, "Standing cylinders drifted");
    near(w.bodies[1].position.y, 1.2, 0.003, "Cylinder cap on box height");
    near(w.bodies[2].position.y, 0.2, 0.003, "Cylinder cap on ground height");
}

TEST_CASE("stack of eight boxes settles and stays") {
    World w;
    for (int i = 0; i < 8; ++i) {
        w.addBody(box({0.01 * (i % 2), 0.25 + 0.5 * i, 0}));
    }
    advance(w, 240);
    auto settled = positions(w);
    advance(w, 10 * 240);
    double drift = maxDrift(settled, w);
    check(drift < 0.01, ("Stack drifted " + std::to_string(drift) + " m").c_str());
    check(maxSpeed(w) < 0.02, "Stack still moving");
    near(w.bodies[7].position.y, 0.25 + 0.5 * 7, 0.02, "Top box height");
}

TEST_CASE("pyramid of ten boxes settles and stays") {
    World w;
    int rows = 4;
    for (int row = 0; row < rows; ++row) {
        for (int k = 0; k < rows - row; ++k) {
            double x = (k - (rows - row - 1) * 0.5) * 0.52;
            w.addBody(box({x, 0.25 + 0.5 * row, 0}));
        }
    }
    check(w.bodies.size() == 10, "Pyramid size");
    advance(w, 240);
    auto settled = positions(w);
    advance(w, 10 * 240);
    double drift = maxDrift(settled, w);
    check(drift < 0.01, ("Pyramid drifted " + std::to_string(drift) + " m").c_str());
    check(maxSpeed(w) < 0.02, "Pyramid still moving");
}

TEST_CASE("warm starting reduces the resting residual at low iteration counts") {
    auto residual = [](bool warm) {
        World w;
        w.iterations = 4;
        w.substeps = 1;
        w.warmStarting = warm;
        for (int i = 0; i < 4; ++i) {
            w.addBody(box({0, 0.25 + 0.5 * i, 0}));
        }
        advance(w, 2 * 240);
        double sum = 0;
        for (int i = 0; i < 240; ++i) {
            w.step();
            for (const auto& body : w.bodies) {
                sum += body.velocity.length();
            }
        }
        return sum / (240 * 4);
    };
    double warm = residual(true);
    double cold = residual(false);
    check(warm < 0.2 * cold,
          ("Warm start residual " + std::to_string(warm) + " vs cold " + std::to_string(cold)).c_str());
    check(warm < 1e-3, "Warm-started stack is not at rest");
}

TEST_CASE("contacts keep feature ids and warm start across steps") {
    World w;
    w.addBody(box({0, 0.249, 0}));
    advance(w, 60);
    auto before = w.contacts;
    w.step();
    check(before.size() == 4 && w.contacts.size() == 4, "Resting box contacts");
    for (std::size_t i = 0; i < 4; ++i) {
        check(before[i].feature == w.contacts[i].feature, "Feature ids changed while resting");
        near(w.contacts[i].normalImpulse, before[i].normalImpulse, 0.05 * before[i].normalImpulse + 1e-6,
             "Resting impulse not steady");
    }
    double total = 0;
    for (const auto& c : w.contacts) {
        total += c.normalImpulse;
    }
    near(total / World::defaultTimeStep, 9.81, 0.05, "Contact forces carry the weight");
}

TEST_CASE("deep initial penetration is resolved without launching the body") {
    World w;
    w.addBody(box({0, 0.15, 0})); // 10 cm in the ground.
    double peak = 0;
    for (int i = 0; i < 480; ++i) {
        w.step();
        peak = std::max(peak, w.bodies[0].velocity.y);
    }
    near(w.bodies[0].position.y, 0.25, 0.003, "Box pushed out to rest height");
    check(peak < 1.0, "Penetration recovery launched the box");
}

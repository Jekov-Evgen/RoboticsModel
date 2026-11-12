// Long chains, mass ratios, motorized arms and position correction without energy gain.
#include "test_harness.hpp"

#include <cmath>
#include <string>

using namespace robotics;
using namespace test;

namespace {

// `links` boxes hanging from a static base by `type` joints, initially horizontal (along +X) so the
// chain swings down under gravity. Masses alternate light/heavy with the given ratio.
World horizontalChain(int links, double massRatio, JointType type) {
    World w;
    Body base;
    base.mass = 0;
    base.size = {0.2, 0.2, 0.2};
    base.position = {0, 5, 0};
    w.addBody(base);
    for (int i = 0; i < links; ++i) {
        Body link;
        link.size = {0.4, 0.08, 0.08};
        link.mass = i % 2 == 0 ? 0.25 : 0.25 * massRatio;
        link.position = {0.2 + 0.4 * i, 5, 0};
        w.addBody(link);
        w.connect(i, i + 1, type, {0.4 * i, 5, 0}, {0, 0, 1});
    }
    return w;
}

double peakAnchorError(World& w, int steps) {
    double peak = 0;
    for (int i = 0; i < steps; ++i) {
        w.step();
        peak = std::max(peak, w.maxAnchorError());
    }
    return peak;
}

} // namespace

TEST_CASE("eight-link swinging chain keeps anchor error below 1 mm") {
    for (JointType type : {JointType::Hinge, JointType::Ball}) {
        World w = horizontalChain(8, 1, type);
        double peak = peakAnchorError(w, 5 * 240);
        check(peak < 1e-3, ("Chain anchor error " + std::to_string(peak) + " m").c_str());
    }
}

TEST_CASE("chain with mass ratio 1:20 swings stably within 1 mm") {
    for (int links : {6, 8}) {
        World w = horizontalChain(links, 20, JointType::Hinge);
        double peak = peakAnchorError(w, 5 * 240);
        w.validate();
        check(peak < 1e-3, ("Mass-ratio chain anchor error " + std::to_string(peak) + " m").c_str());
        // Heavy end link hanging: the chain must have swung down, not exploded.
        check(w.bodies.back().position.y < 5 && w.bodies.back().position.y > 5 - 0.4 * links - 0.1,
              "Chain left its reach");
    }
}

TEST_CASE("motorized six-joint arm holds its pose") {
    World w;
    Body base;
    base.mass = 0;
    base.size = {0.4, 0.4, 0.4};
    base.position = {0, 0.2, 0};
    w.addBody(base);
    const double lean = 0.5;
    const Vec3 direction{std::sin(lean), std::cos(lean), 0};
    Vec3 anchor{0, 0.4, 0};
    for (int i = 0; i < 6; ++i) {
        Body link;
        link.mass = i == 5 ? 0.25 : 5.0 - 0.8 * i; // Mass ratio 1:20 between the base link and the tool.
        link.size = {0.12, 0.4, 0.12};
        link.position = anchor + direction * 0.2;
        link.orientation = Quat::axisAngle({0, 0, 1}, -lean);
        int index = w.addBody(link);
        Vec3 axis = i == 0 ? Vec3{0, 1, 0} : (i % 2 == 0 ? Vec3{1, 0, 0} : Vec3{0, 0, 1});
        Joint& joint =
            w.joints[static_cast<std::size_t>(w.connect(index - 1, index, JointType::Hinge, anchor, axis))];
        joint.maxTorque = 300;
        anchor += direction * 0.4;
    }
    Vec3 tip = w.bodies.back().position;
    double peak = peakAnchorError(w, 5 * 240);
    check(peak < 1e-3, ("Arm anchor error " + std::to_string(peak) + " m").c_str());
    check((w.bodies.back().position - tip).length() < 0.005, "Arm sagged under gravity");
}

TEST_CASE("joint outside its limit is pushed back without gaining speed") {
    // Reviewer case: slider at 0 with limits [10, 11]. Plain Baumgarte turned the 5 m/s correction into
    // real velocity and overshot to the upper limit.
    World w;
    w.gravity = {};
    Body rail;
    rail.mass = 0;
    rail.position = {0, 4, 0};
    w.addBody(rail);
    Body carriage;
    carriage.position = {0, 4, 0};
    w.addBody(carriage);
    w.connect(0, 1, JointType::Prismatic, {0, 4, 0}, {1, 0, 0});
    w.joints[0].setLimits(10, 11);
    double peak = 0;
    for (int i = 0; i < 6 * 240; ++i) {
        w.step();
        peak = std::max(peak, w.joints[0].angle);
    }
    check(peak <= 11, "Limit correction overshot to the far limit");
    check(w.joints[0].angle >= 10 - 1e-3, "Limit violation was not corrected");
    check(w.bodies[1].velocity.length() < 0.01, "Limit correction left kinetic energy");

    // 1 cm hinge limit violation and 1 cm anchor error: corrected, final speed negligible.
    World h;
    h.gravity = {};
    h.addBody(rail);
    Body arm;
    arm.position = {0.5, 4, 0};
    h.addBody(arm);
    h.connect(0, 1, JointType::Hinge, {0, 4, 0}, {0, 0, 1});
    h.joints[0].setLimits(0.02, 0.5);
    h.bodies[1].position.y += 0.01;
    advance(h, 240);
    check(h.maxAnchorError() < 1e-4, "Anchor error not corrected");
    check(h.bodies[1].velocity.length() < 0.01 && h.bodies[1].angularVelocity.length() < 0.02,
          "Anchor/limit correction left kinetic energy");
    check(h.joints[0].angle > 0.02 - 1e-3 && h.joints[0].angle < 0.5, "Hinge outside limits");
}

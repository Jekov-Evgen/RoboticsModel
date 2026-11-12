// Built-in demo robots.
#include "test_harness.hpp"

#include <sstream>

using namespace robotics;
using namespace test;

TEST_CASE("demo survives ten seconds") {
    auto w = makeDemo();
    double peak = 0;
    for (int i = 0; i < 2400; ++i) {
        w.step();
        peak = std::max(peak, w.maxAnchorError());
    }
    w.validate();
    check(peak < 0.08, "Demo joints unstable");
}

TEST_CASE("rover drives turns brakes and remains editable") {
    auto w = makeRoverDemo();
    check(w.bodies.size() == 7 && w.joints.size() == 6, "Rover assembly incomplete");
    w.bodies[1].name = "custom wheel";
    check(driveRover(w, 3, 0) == 4, "Renaming broke drive controls");
    advance(w, 720);
    check(w.bodies[0].position.x > 2, "Rover did not travel");
    check(w.maxAnchorError() < 0.01, "Rover anchor drift");
    driveRover(w, 0, 0);
    advance(w, 240);
    check(w.bodies[0].velocity.length() < 0.1, "Rover did not brake");
    driveRover(w, 0, 2);
    advance(w, 720);
    double yaw = rotationVector(w.bodies[0].orientation).y;
    if (std::abs(yaw) <= 0.2) {
        throw std::runtime_error("Differential drive did not turn; yaw=" + std::to_string(yaw));
    }
    check(w.bodies[0].angularVelocity.y > 0.01, "Positive turn command should turn left");
    w.joints[0].controller.type = ControllerType::PI;
    w.joints[0].motorSpeed = 5;
    check(driveRover(w, 1, 0) == 3 && w.joints[0].motorSpeed == 5, "Keyboard overrode feedback controller");
    std::stringstream data;
    writeScene(w, data);
    auto loaded = readScene(data);
    check(loaded.bodies[1].shape == Shape::Cylinder, "Wheel shape lost on reload");
    loaded.removeBody(1);
    loaded.validate();
    check(loaded.bodies.size() == 6, "Wheel cannot be removed");
}

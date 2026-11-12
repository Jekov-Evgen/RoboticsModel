// Prismatic (slider) joints, joint range limits and their persistence.
#include "test_harness.hpp"

#include <filesystem>
#include <limits>
#include <sstream>
#include <string>

using namespace robotics;
using namespace test;

namespace {

// Static base at (0, 4, 0) and a unit-mass box child connected at `anchor` with the given type and axis.
World twoBodies(JointType type, Vec3 axis, Vec3 childPosition = {0, 4, 0}, Vec3 anchor = {0, 4, 0}) {
    World world;
    Body base;
    base.mass = 0;
    base.position = {0, 4, 0};
    world.addBody(base);
    Body child;
    child.size = {0.4, 0.2, 0.3};
    child.position = childPosition;
    world.addBody(child);
    world.connect(0, 1, type, anchor, axis);
    return world;
}

// Distance of the child's anchor from the slide line through the parent's anchor.
double perpendicularDrift(const World& world, const Joint& joint) {
    const Body& a = world.bodies[static_cast<std::size_t>(joint.a)];
    const Body& b = world.bodies[static_cast<std::size_t>(joint.b)];
    Vec3 axis = a.orientation.rotate(joint.axisA);
    Vec3 offset = b.point(joint.anchorB) - a.point(joint.anchorA);
    return (offset - axis * offset.dot(axis)).length();
}

double relativeRotation(const World& world, const Joint& joint) {
    const Body& a = world.bodies[static_cast<std::size_t>(joint.a)];
    const Body& b = world.bodies[static_cast<std::size_t>(joint.b)];
    return rotationVector((a.orientation * joint.reference) * b.orientation.conjugate()).length();
}

std::string firstLine(const std::string& text) {
    return text.substr(0, text.find('\n'));
}

} // namespace

TEST_CASE("prismatic joint keeps the child on a tilted axis under gravity") {
    // 45 degree axis in the XY plane, anchor off the child's center so gravity also produces torque.
    Vec3 axis = Vec3{1, 1, 0.3}.normalized();
    World w = twoBodies(JointType::Prismatic, axis, {0.3, 4, 0.2}, {0, 4, 0});
    for (auto& body : w.bodies) {
        body.position.y += 40; // Room to slide for 3 s without reaching the floor.
    }
    w.bodies[1].angularVelocity = {0.5, -0.4, 0.8}; // Initial spin must be absorbed by the lock.
    double peakDrift = 0;
    double peakRotation = 0;
    for (int i = 0; i < 3 * 240; ++i) {
        w.step();
        peakDrift = std::max(peakDrift, perpendicularDrift(w, w.joints[0]));
        peakRotation = std::max(peakRotation, relativeRotation(w, w.joints[0]));
    }
    w.validate();
    check(peakDrift < 2e-3, "Prismatic child left the axis");
    check(peakRotation < 0.01, "Prismatic child rotated");
    check(w.maxAnchorError() < 2e-3, "maxAnchorError must ignore sliding along the axis");
    // Frictionless slide: displacement follows g * sin(elevation) for the 3 s run.
    double along = -9.81 * axis.y;
    near(w.joints[0].position(), 0.5 * along * 9, 0.05 * std::abs(0.5 * along * 9), "Slide distance");
    Vec3 offset = w.bodies[1].point(w.joints[0].anchorB) - w.bodies[0].point(w.joints[0].anchorA);
    near(w.joints[0].angle, offset.dot(axis), 1e-12, "Encoder is the displacement along the axis");
}

TEST_CASE("prismatic speed motor reaches its speed within the force limit") {
    World w = twoBodies(JointType::Prismatic, {1, 0, 0});
    w.gravity = {};
    Joint& joint = w.joints[0];
    joint.motorSpeed = 0.5;
    joint.maxTorque = 2; // Newtons.
    w.step();
    check(w.bodies[1].velocity.x <= 2.0 / 240 + 1e-9, "Linear motor exceeded its force limit");
    advance(w, 240);
    near(w.bodies[1].velocity.x, 0.5, 1e-3, "Linear motor target speed");
    near(w.bodies[1].velocity.y, 0, 1e-6, "Linear motor moved off-axis");
    check(!w.joints[0].history.empty(), "Slider telemetry missing");
    near(w.joints[0].history.back().speed, 0.5, 1e-3, "Telemetry speed is m/s along the axis");
}

TEST_CASE("prismatic limits stop the slider against a motor and gravity") {
    for (double direction : {1.0, -1.0}) {
        World w = twoBodies(JointType::Prismatic, Vec3{1, -1, 0}.normalized());
        Joint& joint = w.joints[0];
        joint.motorSpeed = 2 * direction;
        joint.maxTorque = 60;
        joint.setLimits(-0.1, 0.3);
        double peak = -1e9;
        double low = 1e9;
        for (int i = 0; i < 480; ++i) {
            w.step();
            peak = std::max(peak, w.joints[0].angle);
            low = std::min(low, w.joints[0].angle);
        }
        double limit = direction > 0 ? 0.3 : -0.1;
        near(w.joints[0].angle, limit, 2e-3, "Slider did not rest at its limit");
        check(peak < 0.3 + 5e-3 && low > -0.1 - 5e-3, "Slider overshot its limit");
        check(std::abs(w.bodies[1].velocity.length()) < 1e-2, "Slider still moving at its limit");
        check(direction > 0 ? w.joints[0].upperLimitImpulse > 0 : w.joints[0].lowerLimitImpulse > 0,
              "Limit impulse not reported");
    }
}

TEST_CASE("hinge limits stop a motor and gravity") {
    World w = twoBodies(JointType::Hinge, {0, 0, 1}, {0.6, 4, 0});
    Joint& joint = w.joints[0];
    joint.motorSpeed = 3;
    joint.maxTorque = 20;
    joint.setLimits(-0.5, 0.8);
    double peak = 0;
    for (int i = 0; i < 480; ++i) {
        w.step();
        peak = std::max(peak, w.joints[0].angle);
    }
    near(w.joints[0].angle, 0.8, 0.01, "Hinge did not rest at the upper limit");
    check(peak < 0.8 + 0.02, "Hinge overshot the upper limit");
    w.joints[0].motorSpeed = -3;
    advance(w, 480);
    near(w.joints[0].angle, -0.5, 0.01, "Hinge did not rest at the lower limit");
    // Without a motor, gravity alone presses the arm into the lower limit.
    w.joints[0].maxTorque = 0;
    advance(w, 240);
    near(w.joints[0].angle, -0.5, 0.01, "Gravity pushed the hinge through its limit");
}

TEST_CASE("hinge limits beyond one turn use the continuous encoder") {
    World w = twoBodies(JointType::Hinge, {0, 0, 1}, {0.6, 4, 0});
    w.gravity = {};
    Joint& joint = w.joints[0];
    joint.motorSpeed = 4;
    joint.maxTorque = 10;
    joint.setLimits(-1, 4.0); // Upper limit past +pi.
    advance(w, 3 * 240);
    near(w.joints[0].angle, 4.0, 0.02, "Hinge did not pass pi and stop at 4 rad");
    w.joints[0].motorSpeed = -4;
    advance(w, 3 * 240);
    near(w.joints[0].angle, -1, 0.02, "Hinge did not return to the lower limit");
    // Limit wider than a full turn: the joint makes more than one revolution before stopping.
    w.joints[0].setLimits(-1, 9);
    w.joints[0].motorSpeed = 6;
    advance(w, 4 * 240);
    near(w.joints[0].angle, 9, 0.03, "Multi-turn limit");
}

TEST_CASE("position PID drives a vertical slider to a step") {
    World w = twoBodies(JointType::Prismatic, {0, 1, 0});
    Joint& joint = w.joints[0];
    joint.maxTorque = 40;
    joint.controller.type = ControllerType::PID;
    joint.controller.signal = ReferenceSignal::Step;
    joint.controller.amplitude = 0.3;
    joint.controller.startTime = 0.2;
    joint.controller.kp = 80;
    joint.controller.ki = 40;
    joint.controller.kd = 16;
    joint.controller.integralLimit = 20;
    advance(w, 5 * 240);
    near(w.joints[0].angle, 0.3, 0.01, "PID slider did not track the step against gravity");
    const ControlSample& last = w.joints[0].history.back();
    near(last.target, 0.3, 1e-12, "Reference in metres");
    near(last.effort, 9.81, 0.5, "Steady force holds the carriage's weight");
    check(perpendicularDrift(w, w.joints[0]) < 1e-3, "PID slider left its axis");
}

TEST_CASE("linear actuator demo tracks its step and respects limits") {
    World w = makeLinearActuatorDemo();
    double slideMax = 0;
    double flapMax = 0;
    for (int i = 0; i < 4 * 240; ++i) {
        w.step();
        slideMax = std::max(slideMax, w.joints[0].angle);
        flapMax = std::max(flapMax, std::abs(w.joints[1].angle));
    }
    w.validate();
    near(w.joints[0].angle, 0.4, 0.03, "Actuator demo step");
    check(slideMax <= 0.5 + 5e-3, "Actuator demo exceeded the slider limit");
    check(flapMax <= 35 * pi / 180 + 0.02, "Flap exceeded its angle limit");
    check(w.maxAnchorError() < 0.01, "Actuator demo drift");
}

TEST_CASE("scene format v3 round-trips prismatic joints and limits") {
    World w = makeLinearActuatorDemo();
    w.joints[1].limitsEnabled = false; // Disabled limits keep their values.
    std::stringstream data;
    writeScene(w, data);
    check(firstLine(data.str()) == "ROBOTICS_SCENE 3", "New features must be written as version 3");
    World loaded = readScene(data);
    check(loaded.joints.size() == 2 && loaded.joints[0].type == JointType::Prismatic, "Slider type lost");
    check(loaded.joints[0].limitsEnabled && loaded.joints[0].lowerLimit == 0
              && loaded.joints[0].upperLimit == 0.5,
          "Slider limits lost");
    check(!loaded.joints[1].limitsEnabled && loaded.joints[1].upperLimit == w.joints[1].upperLimit,
          "Disabled hinge limits lost");
    check(loaded.joints[0].controller.kp == 60 && loaded.joints[0].maxTorque == 20, "Slider actuator lost");
    std::stringstream again;
    writeScene(loaded, again);
    check(again.str() == data.str(), "Second round trip changed the file");

    // Same simulation after reload.
    advance(w, 240);
    advance(loaded, 240);
    near(loaded.bodies[2].position.x, w.bodies[2].position.x, 1e-12, "Reloaded slider simulates differently");
}

TEST_CASE("worlds without new joint features are still written as version 2") {
    // The rover is not here: its 32 solver iterations need version 4 (tests/test_visual_format.cpp).
    for (World w : {makeDemo(), makeControlDemo()}) {
        std::stringstream data;
        writeScene(w, data);
        check(firstLine(data.str()) == "ROBOTICS_SCENE 2", "Plain world must stay version 2");
        check(data.str().find("LIMITS") == std::string::npos, "Version 2 has no LIMITS records");
        World loaded = readScene(data);
        check(loaded.joints.size() == w.joints.size(), "Version 2 reload failed");
    }
    // A prismatic joint cannot appear in a version 2 file.
    World slider = twoBodies(JointType::Prismatic, {1, 0, 0});
    std::stringstream data;
    writeScene(slider, data);
    std::string text = data.str();
    text.replace(0, firstLine(text).size(), "ROBOTICS_SCENE 2");
    text.erase(text.find("LIMITS"), text.find('\n', text.find("LIMITS")) - text.find("LIMITS") + 1);
    rejects([&] {
        std::stringstream in(text);
        (void)readScene(in);
    });
}

TEST_CASE("example files of every format version still load") {
    namespace fs = std::filesystem;
    int count = 0;
    for (const auto& entry : fs::directory_iterator(fs::path(ROBOTICS_SOURCE_DIR) / "examples")) {
        if (entry.path().extension() != ".robot") {
            continue;
        }
        World w = loadScene(entry.path().string());
        advance(w, 60);
        ++count;
    }
    check(count >= 4, "Examples missing");
    World pendulum = loadScene(std::string(ROBOTICS_SOURCE_DIR) + "/examples/spatial_arm.robot");
    for (const auto& joint : pendulum.joints) {
        check(!joint.limitsEnabled, "Old files must load without limits");
    }
    World actuator = loadScene(std::string(ROBOTICS_SOURCE_DIR) + "/examples/linear_actuator.robot");
    check(actuator.joints.size() == 2 && actuator.joints[0].type == JointType::Prismatic
              && actuator.joints[0].limitsEnabled && actuator.joints[1].limitsEnabled,
          "linear_actuator.robot content");
}

TEST_CASE("limits validation rejects inverted and non-finite ranges") {
    World w = twoBodies(JointType::Hinge, {0, 0, 1});
    rejects([&] { w.joints[0].setLimits(1, -1); });
    check(!w.joints[0].limitsEnabled, "Rejected setLimits must not enable limits");
    rejects([&] { w.joints[0].setLimits(0, std::numeric_limits<double>::infinity()); });
    w.joints[0].setLimits(0.2, 0.2); // A zero-width range is valid (locks the joint).
    w.validate();
    w.joints[0].lowerLimit = 0.5;
    rejects([&] { w.validate(); });
    w.joints[0].lowerLimit = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { w.validate(); });

    World ball = twoBodies(JointType::Ball, {0, 0, 1});
    ball.joints[0].setLimits(-1, 1);
    rejects([&] { ball.validate(); });

    std::stringstream data;
    writeScene(makeLinearActuatorDemo(), data);
    std::string text = data.str();
    std::size_t at = text.find("LIMITS 1 0 0.5");
    check(at != std::string::npos, "LIMITS record format");
    text.replace(at, 14, "LIMITS 1 0.6 0.5");
    rejects([&] {
        std::stringstream in(text);
        (void)readScene(in);
    });
}

TEST_CASE("rover driving ignores slider joints") {
    World w = makeRoverDemo();
    Body wheel;
    wheel.shape = Shape::Cylinder;
    wheel.size = {0.3, 0.3, 0.1};
    wheel.position = {0, 1.2, 0};
    int index = w.addBody(wheel);
    w.connect(0, index, JointType::Prismatic, wheel.position, {0, 1, 0});
    check(driveRover(w, 2, 0) == 4, "driveRover commanded a slider");
    check(w.joints.back().motorSpeed == 0, "Slider motor changed by rover drive");
}

// .robot format round trips, compatibility and validation.
#include "test_harness.hpp"

#include <sstream>

using namespace robotics;
using namespace test;

TEST_CASE("scene round trip") {
    auto original = makeDemo();
    advance(original, 10);
    std::stringstream stream;
    writeScene(original, stream);
    auto restored = readScene(stream);
    std::stringstream a;
    std::stringstream b;
    writeScene(original, a);
    writeScene(restored, b);
    check(a.str() == b.str(), "Scene fields lost");
    advance(restored, 10);
}

TEST_CASE("invalid scenes rejected") {
    rejects([] {
        std::stringstream s("ROBOTICS_SCENE 99");
        (void)readScene(s);
    });
    rejects([] {
        std::stringstream s("ROBOTICS_SCENE 1 GRAVITY 0 -9.81 0 BODIES -1");
        (void)readScene(s);
    });
    rejects([] {
        World w;
        Body b;
        b.mass = -1;
        w.addBody(b);
    });
    rejects([] {
        World w;
        w.step(0);
    });
    rejects([] {
        World w;
        w.addBody(Body{});
        w.connect(0, 0, JointType::Hinge, {});
    });
    rejects([] {
        auto w = makeDemo();
        w.joints[0].b = 500;
        w.validate();
    });
}

TEST_CASE("controller configuration round trip and state reset") {
    auto w = makeControlDemo();
    advance(w, 720);
    std::stringstream data;
    writeScene(w, data);
    auto restored = readScene(data);
    std::stringstream a;
    std::stringstream b;
    writeScene(w, a);
    writeScene(restored, b);
    check(a.str() == b.str(), "Controller settings lost");
    check(restored.joints[0].history.empty() && !restored.joints[0].controllerState.initialized,
          "Transient state should not load");
    double angle = w.joints[0].angle;
    w.resetControllers();
    near(w.joints[0].angle, angle, 1e-12, "Reset lost angle calibration");
    check(w.joints[0].history.empty(), "Reset did not clear history");
}

TEST_CASE("legacy version 1 and invalid controller settings") {
    std::stringstream legacy("ROBOTICS_SCENE 1 GRAVITY 0 -9.81 0 BODIES 0 JOINTS 0 END");
    check(readScene(legacy).bodies.empty(), "Version 1 rejected");
    rejects([] {
        auto w = makeControlDemo();
        w.joints[0].controller.ki = -1;
        w.validate();
    });
    rejects([] {
        auto w = makeControlDemo();
        w.joints[0].controller.type = static_cast<ControllerType>(99);
        w.validate();
    });
    rejects([] {
        auto w = makeControlDemo();
        w.joints[0].type = JointType::Ball;
        w.validate();
    });
    rejects([] {
        auto w = makeControlDemo();
        w.joints[0].controller.derivativeFilter = -0.1;
        w.validate();
    });
}

TEST_CASE("example scenes load") {
    for (const char* path :
         {"examples/pendulum.robot", "examples/pid_servo.robot", "examples/spatial_arm.robot"}) {
        std::string resolved = std::string(ROBOTICS_SOURCE_DIR) + "/" + path;
        auto world = loadScene(resolved);
        check(!world.bodies.empty(), "Example scene is empty");
        advance(world, 24);
    }
}

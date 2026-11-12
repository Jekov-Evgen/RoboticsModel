// Joint constraints, motors and encoders.
#include "test_harness.hpp"

#include <sstream>

using namespace robotics;
using namespace test;

TEST_CASE("ball joint anchor stays attached in 3D") {
    World w;
    Body a;
    a.mass = 0;
    a.position = {0, 4, 0};
    w.addBody(a);
    Body b;
    b.position = {0, 3, 0};
    b.velocity = {1, 0, 0.7};
    w.addBody(b);
    w.connect(0, 1, JointType::Ball, {0, 4, 0});
    advance(w, 1200);
    check(w.maxAnchorError() < 0.015, "Ball joint drift");
}

TEST_CASE("fixed joint preserves relative orientation") {
    World w;
    Body a;
    a.mass = 0;
    a.position = {0, 4, 0};
    w.addBody(a);
    Body b;
    b.position = {1, 4, 0};
    w.addBody(b);
    w.connect(0, 1, JointType::Fixed, {0, 4, 0});
    w.bodies[1].angularVelocity = {1, 2, 3};
    advance(w, 480);
    check(w.maxAnchorError() < 0.01, "Fixed anchor drift");
    check(rotationVector(w.bodies[1].orientation).length() < 0.01, "Fixed orientation drift");
}

TEST_CASE("hinge motor speed and torque limit") {
    World w;
    w.gravity = {};
    Body a;
    a.mass = 0;
    a.position = {0, 4, 0};
    w.addBody(a);
    Body b;
    b.position = a.position;
    w.addBody(b);
    int j = w.connect(0, 1, JointType::Hinge, a.position, {0, 1, 0});
    w.joints[j].motorSpeed = 2;
    w.joints[j].maxTorque = 0.1;
    w.step();
    check(w.bodies[1].angularVelocity.y <= 0.1 / 240 * 24 + 1e-9, "Motor exceeded torque limit");
    advance(w, 1200);
    near(w.bodies[1].angularVelocity.y, 2, 0.005, "Motor target speed");
    near(w.bodies[1].angularVelocity.x, 0, 1e-8, "Unwanted angular axis");
}

TEST_CASE("joint reaction impulse holds a hanging body") {
    World w;
    Body a;
    a.mass = 0;
    a.position = {0, 4, 0};
    w.addBody(a);
    Body b;
    b.mass = 2;
    b.position = {0, 3, 0};
    w.addBody(b);
    w.connect(0, 1, JointType::Ball, {0, 4, 0});
    advance(w, 240);
    // Body B hangs at rest: the joint pushes B up, i.e. pulls A down.
    near(-w.joints[0].reactionImpulse.y / World::defaultTimeStep, 2 * 9.81, 0.05, "Reaction force");
}

TEST_CASE("deleting body removes and reindexes joints") {
    auto w = makeDemo();
    w.removeBody(1);
    w.validate();
    check(w.bodies.size() == 4 && w.joints.size() == 1, "Invalid removal");
    check(w.joints[0].a == 1 && w.joints[0].b == 2, "Invalid index remap");
}

TEST_CASE("encoder unwrap and multirevolution serialization") {
    auto w = servo(ControllerType::SpeedMotor);
    auto& j = w.joints[0];
    j.motorSpeed = 20;
    j.maxTorque = 1000;
    advance(w, 1200);
    near(j.angle, 100, 0.01, "Positive rotations wrapped");
    std::stringstream data;
    writeScene(w, data);
    World restored = readScene(data);
    advance(restored, 240);
    near(restored.joints[0].angle, 120, 0.01, "Encoder turns lost on reload");
    j.motorSpeed = -20;
    advance(w, 480);
    // Tolerance 0.05 rad (was 0.01): reversing 20 -> -20 rad/s at the 1000 N·m limit takes ~1/600 s, during
    // which the substepped solver decelerates continuously instead of within one step. The physically
    // exact offset from the ideal instant reversal is ~+0.033 rad; the encoder unwrap itself is exact.
    near(j.angle, 60, 0.05, "Negative encoder rotation");
}

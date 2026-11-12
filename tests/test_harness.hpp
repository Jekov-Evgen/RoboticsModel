#pragma once
// Minimal self-registering test harness (no third-party framework).
//
//   #include "test_harness.hpp"
//   TEST_CASE("my feature works") { check(1 + 1 == 2, "arithmetic"); }
//
// Add a new tests/test_*.cpp file; Makefile and CMake pick it up automatically.

#include "robotics/demo_scenes.hpp"
#include "robotics/io/robot_format.hpp"
#include "robotics/world.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace test {

struct TestCase {
    const char* name;
    std::function<void()> body;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> body) { registry().push_back({name, std::move(body)}); }
};

inline void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

inline void near(double actual, double expected, double tolerance, const char* message) {
    if (!(std::abs(actual - expected) < tolerance)) {
        throw std::runtime_error(std::string(message) + " (actual " + std::to_string(actual) + ", expected "
                                 + std::to_string(expected) + ")");
    }
}

// Expects `action` to throw std::exception (invalid input must be rejected).
template <class Action>
void rejects(Action action) {
    bool rejected = false;
    try {
        action();
    } catch (const std::exception&) {
        rejected = true;
    }
    check(rejected, "Expected invalid input to be rejected");
}

// ---- Scene builders shared by several test files ----------------------------------------------

inline robotics::Body sphere(robotics::Vec3 position, double diameter = 1) {
    robotics::Body body;
    body.shape = robotics::Shape::Sphere;
    body.size = {diameter, diameter, diameter};
    body.position = position;
    return body;
}

inline void advance(robotics::World& world, int steps) {
    for (int i = 0; i < steps; ++i) {
        world.step();
    }
    world.validate();
}

// Gravity-free hinge servo: static base, unit box arm, Z axis, gains kp=2 ki=3 kd=0.5, target 1.
inline robotics::World servo(robotics::ControllerType type,
                             robotics::ControlVariable variable = robotics::ControlVariable::Position) {
    robotics::World world;
    world.gravity = {};
    robotics::Body base;
    base.mass = 0;
    base.position = {0, 4, 0};
    world.addBody(base);
    robotics::Body arm;
    arm.position = base.position;
    world.addBody(arm);
    world.connect(0, 1, robotics::JointType::Hinge, base.position, {0, 0, 1});
    auto& joint = world.joints[0];
    joint.maxTorque = 3;
    joint.controller.type = type;
    joint.controller.variable = variable;
    joint.controller.kp = 2;
    joint.controller.ki = 3;
    joint.controller.kd = 0.5;
    joint.controller.target = 1;
    return world;
}

} // namespace test

#define TEST_CONCAT_INNER(a, b) a##b
#define TEST_CONCAT(a, b) TEST_CONCAT_INNER(a, b)
#define TEST_CASE_IMPL(name, function)                                                                       \
    static void function();                                                                                  \
    static const test::Registrar TEST_CONCAT(function, _registrar){name, &function};                         \
    static void function()
#define TEST_CASE(name) TEST_CASE_IMPL(name, TEST_CONCAT(test_case_, __LINE__))

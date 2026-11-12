// Feedback controllers, reference signals and telemetry.
#include "test_harness.hpp"

#include <sstream>

using namespace robotics;
using namespace test;

TEST_CASE("PD and PID position control converge") {
    for (auto type : {ControllerType::PD, ControllerType::PID}) {
        auto w = servo(type);
        advance(w, 2400);
        near(w.joints[0].angle, 1, 0.015, "Position did not converge");
        check(std::abs(w.joints[0].history.back().speed) < 0.03, "Joint did not settle");
    }
}

TEST_CASE("PI removes velocity error under constant load") {
    for (auto type : {ControllerType::P, ControllerType::PI}) {
        auto w = servo(type, ControlVariable::Velocity);
        for (int i = 0; i < 2400; ++i) {
            w.bodies[1].torque = {0, 0, -0.5};
            w.step();
        }
        double expected = type == ControllerType::PI ? 1 : 0.75;
        near(w.bodies[1].angularVelocity.z, expected, 0.015, "Steady-state velocity under load");
    }
}

TEST_CASE("controller family P I D contributions") {
    for (auto type : {ControllerType::P, ControllerType::PI, ControllerType::PD, ControllerType::PID}) {
        ControllerConfig c;
        c.type = type;
        c.target = 2;
        c.kp = 3;
        c.ki = 2;
        c.kd = 4;
        c.derivativeFilter = 0;
        ControllerState state;
        auto s = evaluateController(c, state, 0, 0.5, 0.25, 100, 0.1);
        bool hasI = type == ControllerType::PI || type == ControllerType::PID;
        bool hasD = type == ControllerType::PD || type == ControllerType::PID;
        near(s.effort, 4.5 + (hasI ? 0.3 : 0) - (hasD ? 1 : 0), 1e-12, "Wrong controller transfer law");
    }
}

TEST_CASE("torque controller does not also run speed motor") {
    auto w = servo(ControllerType::P);
    auto& j = w.joints[0];
    j.controller.kp = 100;
    j.controller.target = 10;
    j.maxTorque = 0.1;
    j.motorSpeed = -100;
    w.step();
    near(w.bodies[1].angularVelocity.z, 0.1 / 240 * 24, 1e-10, "Torque bound or motor conflict");
    near(j.history.back().effort, 0.1, 1e-12, "Wrong torque telemetry");
    check(j.history.back().saturated, "Missing saturation flag");
}

TEST_CASE("anti-windup and integral bound") {
    ControllerConfig c;
    c.type = ControllerType::PID;
    c.kp = 2;
    c.ki = 20;
    c.target = 100;
    ControllerState state;
    for (int i = 0; i < 1000; ++i) {
        evaluateController(c, state, i * 0.01, 0, 0, 1, 0.01);
    }
    near(state.integral, 0, 1e-12, "Integrator wound up during saturation");
    c.target = 0;
    near(evaluateController(c, state, 10, 0, 0, 1, 0.01).effort, 0, 1e-12, "Saturation recovery failed");
    c.target = 1;
    c.kp = 0;
    c.integralLimit = 0.2;
    for (int i = 0; i < 100; ++i) {
        evaluateController(c, state, 11, 0, 0, 10, 0.01);
    }
    near(state.integral, 0.2, 1e-12, "I term limit not respected");
    evaluateController(c, state, 12, 0, 0, 0, 0.01);
    near(state.integral, 0, 1e-12, "Disabled actuator retains integrator");
}

TEST_CASE("derivative filter and no setpoint kick") {
    ControllerConfig c;
    c.type = ControllerType::PD;
    c.kp = 0;
    c.kd = 1;
    c.derivativeFilter = 0.1;
    ControllerState state;
    evaluateController(c, state, 0, 0, 0, 100, 0.01);
    c.target = 100;
    near(evaluateController(c, state, 0.01, 0, 0, 100, 0.01).d, 0, 1e-12,
         "Derivative kick on setpoint change");
    auto sample = evaluateController(c, state, 0.02, 0, 10, 100, 0.01);
    check(sample.d < 0 && sample.d > -2, "Derivative filter not applied");
}

TEST_CASE("relay signs and deadband") {
    ControllerConfig c;
    c.type = ControllerType::Relay;
    c.target = 1;
    c.deadband = 0.1;
    ControllerState state;
    near(evaluateController(c, state, 0, 0, 0, 3, 0.01).effort, 3, 1e-12, "Relay positive output");
    near(evaluateController(c, state, 0, 2, 0, 3, 0.01).effort, -3, 1e-12, "Relay negative output");
    near(evaluateController(c, state, 0, 1.05, 0, 3, 0.01).effort, 0, 1e-12, "Relay deadband");
}

TEST_CASE("constant step and sine references") {
    ControllerConfig c;
    c.target = 0.2;
    c.amplitude = 0.8;
    c.startTime = 1;
    c.frequency = 0.5;
    near(c.reference(3), 0.2, 1e-12, "Constant reference");
    c.signal = ReferenceSignal::Step;
    near(c.reference(0.9), 0.2, 1e-12, "Step before delay");
    near(c.reference(1), 1, 1e-12, "Step at delay");
    c.signal = ReferenceSignal::Sine;
    near(c.reference(1), 0.2, 1e-12, "Sine phase at start");
    near(c.reference(1.5), 1, 1e-12, "Sine amplitude");
}

TEST_CASE("bounded correctly timestamped telemetry and CSV") {
    auto w = servo(ControllerType::PID);
    advance(w, 2500);
    const auto& h = w.joints[0].history;
    check(h.size() == Joint::historyCapacity, "Unbounded history");
    near(h.back().time, w.time - 1.0 / 240, 1e-9, "Telemetry timestamp off by one step");
    near(h.back().error, h.back().target - h.back().measured, 1e-12, "Telemetry inconsistent");
    std::stringstream csv;
    writeControlCsvHeader(csv);
    writeControlCsvRow(csv, 0, h.back());
    check(csv.str().find("torque_nm,p_nm,i_nm,d_nm") != std::string::npos, "Missing CSV channels");
}

TEST_CASE("PID demonstration step response settles") {
    auto w = makeControlDemo();
    advance(w, 2400);
    near(w.joints[0].angle, 1, 0.03, "PID demo failed to settle");
    check(w.maxAnchorError() < 0.01, "PID demo joint drift");
}

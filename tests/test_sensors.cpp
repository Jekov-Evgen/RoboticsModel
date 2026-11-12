// Sensor models: noise statistics, determinism, quantization, timing, IMU, force/torque, contact,
// controller feedback and .robot persistence. No test depends on exact physics trajectories.
#include "test_harness.hpp"

#include "robotics/sensors/sensor_system.hpp"

#include <sstream>

using namespace robotics;
using namespace test;

namespace {

// Static base with a hinged arm in zero gravity: the joint stays at angle 0 unless driven.
World hingeBench(double motorSpeed = 0) {
    World world;
    world.gravity = {};
    Body base;
    base.mass = 0;
    base.position = {0, 3, 0};
    world.addBody(base);
    Body arm;
    arm.position = {0, 3, 0};
    world.addBody(arm);
    world.connect(0, 1, JointType::Hinge, {0, 3, 0}, {0, 0, 1});
    world.joints[0].motorSpeed = motorSpeed;
    world.joints[0].maxTorque = motorSpeed != 0 ? 100 : 0;
    return world;
}

Sensor idealEncoder() {
    Sensor sensor = makeSensor(SensorType::JointEncoder, 1);
    sensor.rate = 0;
    sensor.encoderCounts = 0;
    sensor.noise = {};
    return sensor;
}

struct Stats {
    double mean{};
    double sigma{};
};

Stats statistics(const std::vector<double>& values) {
    Stats s;
    for (double v : values) {
        s.mean += v;
    }
    s.mean /= static_cast<double>(values.size());
    for (double v : values) {
        s.sigma += (v - s.mean) * (v - s.mean);
    }
    s.sigma = std::sqrt(s.sigma / static_cast<double>(values.size() - 1));
    return s;
}

std::vector<double> encoderReadings(World world, int steps) {
    std::vector<double> values;
    for (int i = 0; i < steps; ++i) {
        stepSimulation(world);
        values.push_back(world.sensors[0].state.value[0]);
    }
    return values;
}

} // namespace

TEST_CASE("deterministic normal generator has unit statistics") {
    std::mt19937_64 rng(42);
    std::vector<double> values;
    for (int i = 0; i < 200000; ++i) {
        values.push_back(sampleNormal(rng));
    }
    Stats s = statistics(values);
    near(s.mean, 0, 0.01, "Normal mean");
    near(s.sigma, 1, 0.01, "Normal sigma");
    std::mt19937_64 a(7);
    std::mt19937_64 b(7);
    for (int i = 0; i < 100; ++i) {
        check(sampleNormal(a) == sampleNormal(b), "Same seed, same sequence");
    }
    std::mt19937_64 reference(1);
    double first = sampleUniform(reference);
    check(first > 0 && first < 1, "Uniform in (0, 1)");
}

TEST_CASE("sensor noise: mean, sigma, bias and seed determinism") {
    World world = hingeBench();
    Sensor encoder = idealEncoder();
    encoder.noise.stddev = 0.05;
    encoder.noise.bias = 0.2;
    world.sensors.push_back(encoder);
    std::vector<double> values = encoderReadings(world, 6000);
    Stats s = statistics(values);
    near(s.mean, 0.2, 0.004, "Biased mean");
    near(s.sigma, 0.05, 0.003, "Noise sigma");
    check(encoderReadings(world, 300) == encoderReadings(world, 300), "Same seed must reproduce readings");
    World other = world;
    other.sensors[0].seed = 99;
    check(encoderReadings(world, 300) != encoderReadings(other, 300), "Another seed changes the noise");
    // Bias random walk: the spread of the walk grows like sigma * sqrt(t).
    World walk = hingeBench();
    Sensor drifting = idealEncoder();
    drifting.noise.biasWalk = 0.1;
    std::vector<double> finals;
    for (std::uint64_t seed = 1; seed <= 200; ++seed) {
        drifting.seed = seed;
        walk.sensors = {drifting};
        std::vector<double> run = encoderReadings(walk, 240); // 1 s.
        finals.push_back(run.back());
    }
    near(statistics(finals).sigma, 0.1, 0.02, "Random walk sigma after 1 s");
}

TEST_CASE("sensor quantization: resolution and encoder counts") {
    World world = hingeBench(1.0);
    Sensor encoder = idealEncoder();
    encoder.encoderCounts = 16; // 16 counts per revolution.
    world.sensors.push_back(encoder);
    const double count = 2 * pi / 16;
    for (int i = 0; i < 400; ++i) {
        stepSimulation(world);
        double value = world.sensors[0].state.value[0];
        near(std::remainder(value, count), 0, 1e-9, "Reading is a whole number of counts");
        check(std::abs(value - world.sensors[0].state.truth[0]) <= count / 2 + 1e-12, "Within half a count");
    }
    World range = hingeBench();
    Sensor plain = idealEncoder();
    plain.noise.bias = 0.123;
    plain.noise.resolution = 0.05;
    range.sensors.push_back(plain);
    stepSimulation(range);
    near(range.sensors[0].state.value[0], 0.1, 1e-12, "Rounded to the nearest step");
}

TEST_CASE("sensor timing: rate, sample-and-hold and latency") {
    World world = hingeBench(2.0);
    Sensor encoder = idealEncoder();
    encoder.rate = 10;
    encoder.latency = 3;
    world.sensors.push_back(encoder);
    std::vector<double> truths;
    std::vector<double> outputs;
    int fresh = 0;
    double held = 0;
    for (int i = 0; i < 240 * 2; ++i) {
        stepSimulation(world);
        const auto& state = world.sensors[0].state;
        if (state.fresh) {
            ++fresh;
            truths.push_back(state.truth[0]);
            outputs.push_back(state.value[0]);
            held = state.value[0];
        } else {
            check(state.value[0] == held, "Output held between samples");
        }
    }
    check(fresh >= 19 && fresh <= 21, "10 Hz over 2 s");
    check(world.sensors[0].state.samples == static_cast<std::uint64_t>(fresh), "Sample counter");
    for (std::size_t k = 3; k < outputs.size(); ++k) {
        check(outputs[k] == truths[k - 3], "Output delayed by exactly three samples");
    }
    check(outputs[0] == truths[0] && outputs[2] == truths[0], "Delay line starts with the first sample");
    // Every-step rate.
    World fast = hingeBench();
    fast.sensors.push_back(idealEncoder());
    for (int i = 0; i < 10; ++i) {
        stepSimulation(fast);
        check(fast.sensors[0].state.fresh, "Rate 0 samples every step");
    }
}

TEST_CASE("sensor dropout repeats the previous measurement") {
    World world = hingeBench(1.0);
    Sensor encoder = idealEncoder();
    encoder.dropout = 0.5;
    world.sensors.push_back(encoder);
    for (int i = 0; i < 2000; ++i) {
        stepSimulation(world);
    }
    double fraction = static_cast<double>(world.sensors[0].state.dropped) / 2000.0;
    near(fraction, 0.5, 0.05, "Dropout fraction");
}

TEST_CASE("IMU: at rest, free fall, spinning disc, mount rotation") {
    World world;
    Body stand;
    stand.mass = 0;
    stand.position = {0, 1, 0};
    world.addBody(stand);
    Body falling = sphere({5, 50, 0});
    falling.mass = 1;
    world.addBody(falling);
    Sensor imu = makeSensor(SensorType::Imu, 0);
    imu.rate = 0;
    imu.noise = {};
    imu.noise2 = {};
    imu.orientationNoise = 0;
    world.sensors.push_back(imu);
    imu.body = 1;
    world.sensors.push_back(imu);
    imu.body = 0;
    imu.orientation = Quat::axisAngle({0, 0, 1}, pi / 2); // Sensor +X points up.
    world.sensors.push_back(imu);
    for (int i = 0; i < 20; ++i) {
        stepSimulation(world);
    }
    const auto& rest = world.sensors[0].state.value;
    near(rest[0], 0, 1e-9, "At rest: no horizontal specific force");
    near(rest[1], 9.81, 1e-9, "At rest: +g up");
    near(rest[6], 1, 1e-12, "At rest: identity orientation");
    const auto& fall = world.sensors[1].state.value;
    check(Vec3{fall[0], fall[1], fall[2]}.length() < 0.05, "Free fall reads ~0");
    near(world.sensors[2].state.value[0], 9.81, 1e-9, "Rotated mount: +g along sensor +X");

    // Disc spinning about Y at 4 rad/s in zero gravity; IMU 0.5 m from the axis.
    World disc;
    disc.gravity = {};
    Body wheel = sphere({0, 10, 0}, 1.2);
    wheel.angularVelocity = {0, 4, 0};
    disc.addBody(wheel);
    Sensor spin = makeSensor(SensorType::Imu, 0);
    spin.rate = 0;
    spin.noise = {};
    spin.noise2 = {};
    spin.orientationNoise = 0;
    spin.position = {0.5, 0, 0};
    disc.sensors.push_back(spin);
    for (int i = 0; i < 100; ++i) {
        stepSimulation(disc);
    }
    const auto& reading = disc.sensors[0].state.value;
    near(reading[0], -16 * 0.5, 0.1, "Centripetal acceleration towards the axis (sensor frame)");
    near(reading[2], 0, 0.15, "No tangential acceleration");
    near(reading[4], 4, 1e-6, "Gyroscope reads the spin rate");
}

TEST_CASE("force/torque sensor reads m*g for a hanging mass at rest") {
    World world;
    Body base;
    base.mass = 0;
    base.position = {0, 3, 0};
    world.addBody(base);
    Body mass;
    mass.mass = 2;
    mass.size = {0.3, 0.6, 0.3};
    mass.position = {0, 2.2, 0};
    world.addBody(mass);
    world.connect(0, 1, JointType::Hinge, {0, 2.5, 0}, {0, 0, 1});
    Sensor wrench = makeSensor(SensorType::ForceTorque, 1);
    wrench.noise = {};
    wrench.noise2 = {};
    wrench.rate = 0;
    world.sensors.push_back(wrench);
    for (int i = 0; i < 480; ++i) {
        stepSimulation(world);
    }
    const auto& value = world.sensors[0].state.value;
    near(value[1], 2 * 9.81, 2 * 9.81 * 0.03, "Joint carries the weight upwards");
    check(std::abs(value[0]) < 0.3 && std::abs(value[2]) < 0.3, "No sideways force at rest");
    check(Vec3{value[3], value[4], value[5]}.length() < 0.3, "No torque about the anchor at rest");
    check(sensorJoint(world, world.sensors[0]) == 0, "Sensor resolves its joint");
}

TEST_CASE("contact sensor: touching and total normal force") {
    World world;
    Body box;
    box.mass = 3;
    box.size = {0.5, 0.5, 0.5};
    box.position = {0, 0.25, 0};
    world.addBody(box);
    Body flying = box;
    flying.position = {3, 20, 0};
    world.addBody(flying);
    Sensor touch = makeSensor(SensorType::Contact, 0);
    touch.rate = 0;
    world.sensors.push_back(touch);
    touch.body = 1;
    world.sensors.push_back(touch);
    for (int i = 0; i < 240; ++i) {
        stepSimulation(world);
    }
    check(world.sensors[0].state.value[0] == 1, "Resting box touches the ground");
    near(world.sensors[0].state.value[1], 3 * 9.81, 3 * 9.81 * 0.05, "Normal force = weight");
    check(world.sensors[1].state.value[0] == 0 && world.sensors[1].state.value[1] == 0, "Airborne box");
}

TEST_CASE("encoder feedback reaches the PID controller") {
    World world = makeNoisyPidDemo();
    check(world.sensors[0].feedsController, "Noisy PID demo feeds its controller");
    bool differs = false;
    for (int i = 0; i < 240 * 4; ++i) {
        stepSimulation(world);
        const Joint& joint = world.joints[0];
        if (i > 10) {
            check(joint.controllerState.sensorFeedback, "Feedback flag set");
            const ControlSample& sample = joint.history.back();
            differs = differs || std::abs(sample.measured - sample.angle) > 1e-6;
        }
    }
    check(differs, "Controller saw the sensor, not the ideal angle");
    // Still tracks the step roughly despite noise and latency (no exact trajectory).
    check(std::abs(world.joints[0].angle - 1) < 0.35, "Noisy loop still near the target");
    // Removing the sensor restores ideal feedback after the next update.
    world.sensors.clear();
    stepSimulation(world);
    check(!world.joints[0].controllerState.sensorFeedback, "Ideal feedback restored");
}

TEST_CASE("sensor persistence: v5 round trip, oldest sufficient version, body removal") {
    World world = makeSensorRoverDemo();
    world.sensors[1].noise.biasWalk = 0.25;
    world.sensors[1].latency = 4;
    world.sensors[1].dropout = 0.125;
    world.sensors[1].seed = 123456789012345ULL;
    std::stringstream first;
    writeScene(world, first);
    check(first.str().rfind("ROBOTICS_SCENE 5", 0) == 0, "Sensors need version 5");
    World restored = readScene(first);
    check(restored.sensors.size() == world.sensors.size(), "Sensor count");
    check(restored.sensors[1].seed == 123456789012345ULL && restored.sensors[1].noise.biasWalk == 0.25,
          "Sensor fields");
    std::stringstream second;
    writeScene(restored, second);
    check(first.str() == second.str(), "Lossless round trip");
    for (int i = 0; i < 24; ++i) {
        stepSimulation(restored);
    }
    std::stringstream plain;
    writeScene(makeRoverDemo(), plain);
    check(plain.str().rfind("ROBOTICS_SCENE 5", 0) != 0, "No sensors -> older version");
    // Old (version 2) files still load without sensors.
    std::stringstream old("ROBOTICS_SCENE 2 GRAVITY 0 -9.81 0 BODIES 0 JOINTS 0 END");
    check(readScene(old).sensors.empty(), "Version 2 loads");
    // Removing a body drops its sensors and renumbers the rest.
    World edited = makeSensorRoverDemo();
    std::size_t before = edited.sensors.size();
    edited.removeBody(1); // A wheel with an encoder.
    check(edited.sensors.size() == before - 1, "Wheel encoder removed");
    for (const auto& sensor : edited.sensors) {
        check(sensor.body >= 0 && sensor.body < static_cast<int>(edited.bodies.size()), "Indices shifted");
    }
    check(edited.sensors[0].body == 5, "Lidar follows the renumbered sensor head");
    validateSensors(edited);
}

TEST_CASE("invalid sensors rejected") {
    World world = hingeBench();
    auto bad = [&](auto edit) {
        World copy = world;
        Sensor sensor = idealEncoder();
        edit(sensor);
        copy.sensors.push_back(sensor);
        rejects([&] { validateSensors(copy); });
        rejects([&] {
            std::stringstream out;
            writeScene(copy, out);
        });
    };
    bad([](Sensor& s) { s.body = 7; });
    bad([](Sensor& s) { s.rate = -1; });
    bad([](Sensor& s) { s.latency = -2; });
    bad([](Sensor& s) { s.dropout = 1; });
    bad([](Sensor& s) { s.noise.stddev = -0.1; });
    bad([](Sensor& s) { s.maxRange = s.minRange; });
    bad([](Sensor& s) { s.beams = 0; });
    bad([](Sensor& s) {
        s.type = SensorType::Imu;
        s.feedsController = true;
    });
    bad([](Sensor& s) { s.orientation = {2, 0, 0, 0}; });
    std::stringstream truncated(
        "ROBOTICS_SCENE 5 GRAVITY 0 -9.81 0 ITERATIONS 24 SOLVER 4 1 BODIES 0 JOINTS 0 SENSORS 1 "
        "SENSOR \"x\" 2 0 END");
    rejects([&] { (void)readScene(truncated); });
}

TEST_CASE("format v5 stores substeps and warm starting only when non-default") {
    World world = makeDemo();
    std::stringstream plain;
    writeScene(world, plain);
    check(plain.str().find("SOLVER") == std::string::npos && plain.str().rfind("ROBOTICS_SCENE 5", 0) != 0,
          "Default solver settings keep the older version");
    world.substeps = 8;
    world.warmStarting = false;
    std::stringstream first;
    writeScene(world, first);
    check(first.str().rfind("ROBOTICS_SCENE 5", 0) == 0, "Solver settings need version 5");
    World restored = readScene(first);
    check(restored.substeps == 8 && !restored.warmStarting, "Solver settings round trip");
    std::stringstream second;
    writeScene(restored, second);
    check(first.str() == second.str(), "Lossless solver round trip");
    World warmOnly = makeDemo();
    warmOnly.warmStarting = false;
    std::stringstream third;
    writeScene(warmOnly, third);
    World warmRestored = readScene(third);
    check(!warmRestored.warmStarting && warmRestored.substeps == World{}.substeps, "Warm starting alone");
    auto rejectsScene = [](const std::string& text, const char* expected) {
        bool rejected = false;
        try {
            std::stringstream in(text);
            (void)readScene(in);
        } catch (const std::exception& error) {
            rejected = std::string(error.what()).find(expected) != std::string::npos;
        }
        check(rejected, expected);
    };
    rejectsScene(
        "ROBOTICS_SCENE 5 GRAVITY 0 -9.81 0 ITERATIONS 24 SOLVER 0 1 BODIES 0 JOINTS 0 SENSORS 0 END",
        "substeps must be in [1, 64]");
    rejectsScene(
        "ROBOTICS_SCENE 5 GRAVITY 0 -9.81 0 ITERATIONS 24 SOLVER 65 1 BODIES 0 JOINTS 0 SENSORS 0 END",
        "substeps must be in [1, 64]");
    rejectsScene(
        "ROBOTICS_SCENE 5 GRAVITY 0 -9.81 0 ITERATIONS 24 SOLVER 4 2 BODIES 0 JOINTS 0 SENSORS 0 END",
        "warm-starting flag");
    rejectsScene("ROBOTICS_SCENE 5 GRAVITY 0 -9.81 0 ITERATIONS 24 BODIES 0 JOINTS 0 SENSORS 0 END",
                 "SOLVER");
}

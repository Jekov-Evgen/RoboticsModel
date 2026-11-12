#include "robotics/demo_scenes.hpp"

#include "robotics/sensors/sensor_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace robotics {
namespace {

// Wheel speed command bound (rad/s) for driveRover().
constexpr double maxWheelSpeed = 12.0;

} // namespace

World makeDemo() {
    World world;

    Body base;
    base.name = "base";
    base.mass = 0;
    base.size = {1.1, 0.4, 1.1};
    base.position = {0, 0.2, 0};
    world.addBody(base);

    Body arm;
    arm.name = "upper_arm";
    arm.size = {0.28, 1.5, 0.28};
    arm.mass = 2;
    arm.position = {0, 1.15, 0};
    world.addBody(arm);
    int shoulder = world.connect(0, 1, JointType::Hinge, {0, 0.4, 0}, {0, 0, 1});
    world.joints[static_cast<std::size_t>(shoulder)].motorSpeed = 0.35;
    world.joints[static_cast<std::size_t>(shoulder)].maxTorque = 35;

    arm.name = "forearm";
    arm.size = {1.1, 0.22, 0.22};
    arm.mass = 1;
    arm.position = {0.55, 1.9, 0};
    world.addBody(arm);
    int elbow = world.connect(1, 2, JointType::Hinge, {0, 1.9, 0}, {0, 0, 1});
    world.joints[static_cast<std::size_t>(elbow)].motorSpeed = -0.55;
    world.joints[static_cast<std::size_t>(elbow)].maxTorque = 12;

    Body payload;
    payload.name = "tool";
    payload.shape = Shape::Sphere;
    payload.size = {0.35, 0.35, 0.35};
    payload.mass = 0.35;
    payload.position = {1.2, 1.9, 0};
    world.addBody(payload);
    world.connect(2, 3, JointType::Fixed, {1.1, 1.9, 0});

    Body ball;
    ball.name = "free_ball";
    ball.shape = Shape::Sphere;
    ball.size = {0.65, 0.65, 0.65};
    ball.position = {-1.8, 2.5, 0.5};
    ball.restitution = 0.6;
    world.addBody(ball);
    return world;
}

World makeControlDemo() {
    World world;

    Body base;
    base.name = "servo_base";
    base.mass = 0;
    base.size = {0.3, 1, 0.3};
    base.position = {0, 0.5, 0};
    world.addBody(base);

    Body arm;
    arm.name = "controlled_arm";
    arm.size = {1.2, 0.16, 0.16};
    arm.position = {0.6, 1, 0};
    world.addBody(arm);

    int index = world.connect(0, 1, JointType::Hinge, {0, 1, 0}, {0, 1, 0});
    Joint& joint = world.joints[static_cast<std::size_t>(index)];
    joint.maxTorque = 8;
    joint.controller.type = ControllerType::PID;
    joint.controller.signal = ReferenceSignal::Step;
    joint.controller.integralLimit = 2;
    return world;
}

World makeLinearActuatorDemo() {
    World world;

    Body post;
    post.name = "rail_post";
    post.mass = 0;
    post.size = {0.2, 1.44, 0.2};
    post.position = {-0.4, 0.72, 0};
    world.addBody(post);

    Body rail;
    rail.name = "rail";
    rail.mass = 0;
    rail.size = {1.6, 0.12, 0.3};
    rail.position = {0.3, 1.5, 0};
    int railIndex = world.addBody(rail);

    Body carriage;
    carriage.name = "carriage";
    carriage.mass = 1;
    carriage.size = {0.3, 0.2, 0.4};
    carriage.position = {0, 1.66, 0};
    int carriageIndex = world.addBody(carriage);
    int slider = world.connect(railIndex, carriageIndex, JointType::Prismatic, carriage.position, {1, 0, 0});
    Joint& actuator = world.joints[static_cast<std::size_t>(slider)];
    actuator.maxTorque = 20; // Max force, N.
    actuator.setLimits(0, 0.5);
    actuator.controller.type = ControllerType::PID;
    actuator.controller.signal = ReferenceSignal::Step;
    actuator.controller.target = 0;
    actuator.controller.amplitude = 0.4; // Step to 0.4 m.
    actuator.controller.startTime = 0.5;
    actuator.controller.kp = 60;
    actuator.controller.ki = 10;
    actuator.controller.kd = 12;
    actuator.controller.integralLimit = 5;
    actuator.controller.deadband = 0.005;

    Body flap;
    flap.name = "flap";
    flap.mass = 0.2;
    flap.size = {0.08, 0.6, 0.06};
    flap.position = {0, 1.36, 0.3};
    int flapIndex = world.addBody(flap);
    int hinge = world.connect(carriageIndex, flapIndex, JointType::Hinge, {0, 1.66, 0.3}, {0, 0, 1});
    world.joints[static_cast<std::size_t>(hinge)].setLimits(-35 * pi / 180, 35 * pi / 180);
    return world;
}

World makeRoverDemo() {
    World world;
    world.iterations = 32;

    Body chassis;
    chassis.name = "rover_chassis";
    chassis.mass = 5;
    chassis.size = {1.8, 0.28, 1.0};
    chassis.position = {0, 0.7, 0};
    world.addBody(chassis);

    for (int i = 0; i < 4; ++i) {
        const bool front = i < 2;
        const bool left = i % 2 != 0;
        Body wheel;
        wheel.name = std::string("wheel_") + (front ? "front_" : "rear_") + (left ? "left" : "right");
        wheel.shape = Shape::Cylinder;
        wheel.size = {0.76, 0.76, 0.24};
        wheel.mass = 0.6;
        wheel.friction = 0.9;
        wheel.restitution = 0;
        wheel.position = {front ? 0.62 : -0.62, 0.4, left ? -0.65 : 0.65};
        int index = world.addBody(wheel);
        int joint = world.connect(0, index, JointType::Hinge, wheel.position, {0, 0, 1});
        world.joints[static_cast<std::size_t>(joint)].maxTorque = 8;
    }

    Body deck;
    deck.name = "rover_deck";
    deck.mass = 0.4;
    deck.size = {1.35, 0.14, 0.8};
    deck.position = {-0.1, 0.91, 0};
    int deckIndex = world.addBody(deck);
    world.connect(0, deckIndex, JointType::Fixed, deck.position);

    Body head;
    head.name = "rover_sensor";
    head.mass = 0.2;
    head.size = {0.32, 0.22, 0.58};
    head.position = {0.48, 1.09, 0};
    int headIndex = world.addBody(head);
    world.connect(deckIndex, headIndex, JointType::Fixed, head.position);

    // Paused starter: Run enables physics; W/A/S/D choose a persistent drive command.
    driveRover(world, 0, 0);
    return world;
}

World makeSensorRoverDemo() {
    World world = makeRoverDemo();
    constexpr int chassis = 0;
    constexpr int head = 6; // rover_sensor
    // Obstacles for the lidar and camera.
    auto obstacle = [&](const char* name, Shape shape, Vec3 size, Vec3 position, double yaw) {
        Body body;
        body.name = name;
        body.shape = shape;
        body.mass = 0;
        body.size = size;
        body.position = position;
        body.orientation = Quat::axisAngle({0, 1, 0}, yaw);
        world.addBody(body);
    };
    // Tall enough to cross the lidar's scan plane (about 1.23 m above the ground).
    obstacle("wall", Shape::Box, {0.3, 1.8, 4.0}, {4.5, 0.9, 0}, 0);
    obstacle("crate", Shape::Box, {0.8, 1.5, 0.8}, {2.6, 0.75, -1.8}, 0.5);
    obstacle("pillar", Shape::Cylinder, {0.6, 0.6, 2.0}, {2.2, 1.0, 1.9}, 0);
    world.bodies.back().orientation = Quat::axisAngle({1, 0, 0}, pi / 2); // Stand the pillar upright.
    obstacle("boulder", Shape::Sphere, {2.6, 2.6, 2.6}, {-3.6, 1.3, 1.6}, 0);

    Sensor lidar = makeSensor(SensorType::Lidar2D, head);
    lidar.name = "lidar";
    lidar.position = {0, 0.14, 0};
    lidar.beams = 241;
    lidar.fov = 2 * pi * 300 / 360;
    world.sensors.push_back(lidar);

    Sensor imu = makeSensor(SensorType::Imu, chassis);
    imu.name = "imu";
    imu.seed = 2;
    world.sensors.push_back(imu);

    Sensor camera = makeSensor(SensorType::Camera, head);
    camera.name = "front_camera";
    camera.position = {0.17, 0, 0};
    camera.orientation = Quat::axisAngle({0, 0, 1}, -8 * pi / 180); // Tilted slightly down.
    world.sensors.push_back(camera);

    for (int wheel = 1; wheel <= 4; ++wheel) {
        Sensor encoder = makeSensor(SensorType::JointEncoder, wheel);
        encoder.name = world.bodies[static_cast<std::size_t>(wheel)].name + "_encoder";
        encoder.encoderCounts = 1024;
        encoder.rate = 100;
        world.sensors.push_back(encoder);
    }
    return world;
}

World makeNoisyPidDemo() {
    World world = makeControlDemo();
    Sensor encoder = makeSensor(SensorType::JointEncoder, 1);
    encoder.name = "arm_encoder";
    encoder.rate = 200;
    encoder.latency = 6; // 30 ms at 200 Hz.
    encoder.encoderCounts = 256;
    encoder.noise.stddev = 0.02;
    encoder.feedsController = true;
    world.sensors.push_back(encoder);
    Sensor torque = makeSensor(SensorType::ForceTorque, 1);
    torque.name = "arm_wrench";
    world.sensors.push_back(torque);
    return world;
}

int driveRover(World& world, double forward, double turn) {
    if (!std::isfinite(forward) || !std::isfinite(turn)) {
        throw std::runtime_error("Invalid rover command");
    }
    int count = 0;
    for (auto& joint : world.joints) {
        const auto& wheel = world.bodies[static_cast<std::size_t>(joint.b)];
        if (joint.type != JointType::Hinge || wheel.shape != Shape::Cylinder || !joint.usesSpeedMotor()) {
            continue;
        }
        bool left = joint.anchorA.z < 0;
        joint.motorSpeed = std::clamp(-forward + (left ? turn : -turn), -maxWheelSpeed, maxWheelSpeed);
        ++count;
    }
    return count;
}

bool isWheelJoint(const World& world, const Joint& joint) {
    if (joint.type != JointType::Hinge || joint.limitsEnabled) {
        return false;
    }
    const Body& parent = world.bodies[static_cast<std::size_t>(joint.a)];
    const Body& wheel = world.bodies[static_cast<std::size_t>(joint.b)];
    if (wheel.shape != Shape::Cylinder) {
        return false;
    }
    Vec3 axis = parent.orientation.rotate(joint.axisA).normalized();
    Vec3 spin = wheel.orientation.rotate({0, 0, 1});
    return std::abs(axis.dot(spin)) > 0.95 && std::abs(axis.y) < 0.3;
}

} // namespace robotics

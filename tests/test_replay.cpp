// Recording ring buffer, restoring simulation state, plot channels and CSV.

#include "robotics/replay/channels.hpp"
#include "robotics/replay/recording.hpp"
#include "robotics/sensors/sensor_system.hpp"
#include "test_harness.hpp"

#include <iostream>
#include <sstream>

using namespace robotics;
using test::check;
using test::near;

namespace {

Recording record(World& world, int steps, double seconds = 60) {
    Recording recording(seconds);
    recording.start(world);
    for (int i = 0; i < steps; ++i) {
        stepSimulation(world);
        recording.afterStep(world);
    }
    return recording;
}

} // namespace

TEST_CASE("recording stores frames at 60 Hz with periodic keyframes") {
    World world = makeRoverDemo();
    Recording recording = record(world, 240);
    check(recording.size() == 61, "One second: 60 frames plus the start");
    near(recording.endTime(), world.time, 1e-12, "Last frame is current");
    near(recording.frame(30).time, 0.5, 1e-9, "Frame spacing");
    check(recording.frame(0).keyframe && recording.frame(30).keyframe && !recording.frame(29).keyframe,
          "Keyframe interval");
    check(recording.frame(30).keyframe->joints[0].history.empty(), "Keyframes strip telemetry");
    check(recording.indexAtTime(0.5) == 30 && recording.indexAtTime(-1) == 0
              && recording.indexAtTime(99) == 60,
          "Time lookup");
    check(recording.indexAtTime(0.5 + 0.4 / 60) == 30 && recording.indexAtTime(0.5 + 0.6 / 60) == 31,
          "Closest frame");
    recording.clear();
    check(recording.empty(), "Clear");
}

TEST_CASE("recording ring buffer drops the oldest blocks") {
    World world = makeControlDemo();
    Recording recording = record(world, 240 * 9, 5);
    near(recording.endTime(), world.time, 1e-12, "Newest frame kept");
    check(recording.endTime() - recording.startTime() <= 5.51, "Older frames dropped");
    check(recording.endTime() - recording.startTime() >= 4.999, "Configured length kept");
    check(recording.frame(0).keyframe != nullptr, "Front stays restorable");
    recording.setDuration(60);
    test::rejects([&] { recording.setDuration(1); });
}

TEST_CASE("view shows a frame without touching the live world") {
    World world = makeDemo();
    Recording recording = record(world, 480);
    Vec3 live = world.bodies[1].position;
    std::size_t index = recording.indexAtTime(1);
    World view = recording.view(index);
    near(view.time, 1, 1e-9, "View time");
    near((view.bodies[1].position - recording.frame(index).bodies[1].position).length(), 0, 1e-15, "Pose");
    check(view.bodies[1].name == world.bodies[1].name, "Configuration comes along");
    check((world.bodies[1].position - live).length() == 0, "Live world untouched");
}

TEST_CASE("restore continues exactly like the original run") {
    for (World world : {makeRoverDemo(), makeSensorRoverDemo(), makeNoisyPidDemo()}) {
        driveRover(world, 3, 0.5);
        Recording recording = record(world, 480);
        std::size_t index = recording.indexAtTime(1) + 7; // Between keyframes.
        check(!recording.frame(index).keyframe, "Test must exercise the re-simulation");
        World restored = recording.restore(index, world);
        near(restored.time, recording.frame(index).time, 1e-12, "Time restored");
        // Full state: stepping on reproduces the recorded future, contacts, controllers and noise included.
        std::size_t next = index + 20;
        std::uint64_t steps = recording.frame(next).step - recording.frame(index).step;
        for (std::uint64_t s = 0; s < steps; ++s) {
            stepSimulation(restored);
        }
        for (std::size_t b = 0; b < restored.bodies.size(); ++b) {
            near((restored.bodies[b].position - recording.frame(next).bodies[b].position).length(), 0, 1e-9,
                 "Restored run diverged");
        }
        for (std::size_t s = 0; s < restored.sensors.size(); ++s) {
            if (restored.sensors[s].state.valid && !restored.sensors[s].state.value.empty()) {
                near(restored.sensors[s].state.value[0],
                     recording.frame(next).sensors[s * Sensor::historyChannels], 1e-9,
                     "Sensor noise diverged");
            }
        }
    }
}

TEST_CASE("restore replays driving commands and cuts telemetry at the frame") {
    World world = makeRoverDemo();
    Recording recording(60);
    recording.start(world);
    for (int i = 0; i < 480; ++i) {
        if (i == 250) { // Between keyframes at 240 and 360 steps... commands change mid-block.
            driveRover(world, 4, 0);
        }
        stepSimulation(world);
        recording.afterStep(world);
    }
    std::size_t index = recording.indexAtTime(1.3);
    World restored = recording.restore(index, world);
    near(restored.joints[0].motorSpeed, world.joints[0].motorSpeed, 1e-12, "Command replayed");
    check(!restored.joints[0].history.empty(), "History kept");
    check(restored.joints[0].history.back().time <= recording.frame(index).time, "History cut");
    check(restored.joints[0].history.size() < world.joints[0].history.size(), "Future removed");
    // Continuing after truncation keeps a consistent timeline.
    recording.truncateAfter(index);
    check(recording.size() == index + 1, "Future frames dropped");
    for (int i = 0; i < 240; ++i) {
        stepSimulation(restored);
        recording.afterStep(restored);
    }
    check(recording.size() == index + 1 + 60, "Recording continues");
    near(recording.frame(index + 1).time - recording.frame(index).time, 1.0 / 60, 1e-9, "No gap");
    std::size_t keyframes = 0;
    for (std::size_t i = 0; i < recording.size(); ++i) {
        keyframes += recording.frame(i).keyframe ? 1 : 0;
    }
    check(keyframes == (recording.size() + Recording::framesPerKeyframe - 1) / Recording::framesPerKeyframe,
          "Keyframe cadence continues");
}

TEST_CASE("recording memory stays modest") {
    World world = makeSensorRoverDemo();
    driveRover(world, 3, 0);
    Recording recording = record(world, 240 * 10);
    double perSecond = static_cast<double>(recording.memoryBytes()) / 10;
    std::cout << "recording memory, sensor rover: " << perSecond / 1e6 << " MB/s (" << world.bodies.size()
              << " bodies, " << world.sensors.size() << " sensors)\n";
    check(perSecond < 4e6, "More than 4 MB per second");
}

TEST_CASE("channels cover scene, parts, joints and sensors") {
    World world = makeSensorRoverDemo();
    std::vector<Channel> channels = listChannels(world);
    auto has = [&](const std::string& query) {
        return std::any_of(channels.begin(), channels.end(),
                           [&](const Channel& c) { return channelMatches(c, query); });
    };
    check(has("scene kinetic") && has("scene potential") && has("contact count"), "Scene channels");
    check(has("rover_chassis position y") && has("CHASSIS SPEED") && has("chassis yaw")
              && has("chassis angular"),
          "Part channels (case-insensitive words)");
    check(has("wheel_front_left joint angle") && has("motor torque") && has("reaction force"),
          "Joint channels");
    check(has("imu accel_x") && has("lidar"), "Sensor channels");
    check(!has("no such thing"), "Search filters");
    // Static bodies only expose their pose.
    for (const Channel& c : channels) {
        if (c.source == ChannelSource::Body && world.bodies[static_cast<std::size_t>(c.index)].isStatic()) {
            check(c.unit == "m" || c.unit == "°", "Static bodies have no velocity channels");
        }
    }
}

TEST_CASE("channel values match the recorded state") {
    World world;
    Body ball = test::sphere({0, 5, 0}, 0.5);
    ball.mass = 2;
    ball.velocity = {3, 0, 4};
    ball.angularVelocity = {0, 2, 0};
    ball.orientation = Quat::axisAngle({0, 0, 1}, pi / 6);
    world.addBody(ball);
    Recording recording;
    recording.start(world);
    const RecordingFrame& frame = recording.frame(0);
    auto value = [&](BodyQuantity q) {
        return channelValue(bodyChannel(world, 0, q), world, frame);
    };
    near(value(BodyQuantity::PositionY), 5, 1e-12, "Position");
    near(value(BodyQuantity::Speed), 5, 1e-12, "Speed");
    near(value(BodyQuantity::AngularSpeed), 2, 1e-12, "Angular speed");
    near(value(BodyQuantity::Yaw), 30, 1e-9, "Yaw in degrees");
    double inertia = world.bodies[0].principalInertia().y;
    near(value(BodyQuantity::KineticEnergy), 0.5 * 2 * 25 + 0.5 * inertia * 4, 1e-9, "Kinetic energy");
    near(value(BodyQuantity::PotentialEnergy), 2 * 9.81 * 5, 1e-9, "Potential energy");
    Channel total{ChannelSource::Scene, -1, static_cast<int>(SceneQuantity::TotalEnergy), "Scene", "", "J"};
    near(channelValue(total, world, frame),
         value(BodyQuantity::KineticEnergy) + value(BodyQuantity::PotentialEnergy), 1e-9, "Total energy");
}

TEST_CASE("energy is conserved in recorded free flight") {
    World world;
    world.addBody(test::sphere({0, 50, 0}));
    world.bodies[0].angularVelocity = {0, 3, 0};
    Recording recording = record(world, 240);
    Channel total{ChannelSource::Scene, -1, static_cast<int>(SceneQuantity::TotalEnergy), "Scene", "", "J"};
    double first = channelValue(total, world, recording.frame(0));
    double last = channelValue(total, world, recording.frame(recording.size() - 1));
    near(last / first, 1, 2e-3, "Free flight keeps its energy");
}

TEST_CASE("joint channels read coordinate, effort and reaction") {
    World world = test::servo(ControllerType::PID);
    world.gravity = {0, -9.81, 0};
    Recording recording = record(world, 240);
    const RecordingFrame& frame = recording.frame(recording.size() - 1);
    near(channelValue(jointChannel(world, 0, JointQuantity::Coordinate), world, frame), world.joints[0].angle,
         1e-12, "Coordinate");
    near(channelValue(jointChannel(world, 0, JointQuantity::Speed), world, frame),
         world.joints[0].history.back().speed, 1e-12, "Speed");
    check(std::abs(channelValue(jointChannel(world, 0, JointQuantity::Effort), world, frame)) > 0, "Effort");
    near(channelValue(jointChannel(world, 0, JointQuantity::ReactionForce), world, frame), 9.81, 2,
         "Holds weight");
}

TEST_CASE("channels rebind by name after edits") {
    World world = makeRoverDemo();
    Channel channel = bodyChannel(world, 2, BodyQuantity::Speed);
    world.removeBody(1);
    check(rebindChannel(channel, world) && channel.index == 1, "Index follows the part");
    world.removeBody(1);
    check(!rebindChannel(channel, world), "Removed part");
}

TEST_CASE("channel CSV has one column per channel and one row per frame") {
    World world = makeRoverDemo();
    Recording recording = record(world, 40);
    std::vector<Channel> channels{bodyChannel(world, 0, BodyQuantity::PositionY),
                                  jointChannel(world, 0, JointQuantity::Effort)};
    std::stringstream csv;
    writeChannelCsv(csv, recording, world, channels);
    std::string line;
    std::getline(csv, line);
    check(line
              == "time,\"rover_chassis · position Y (m)\",\"rover_chassis → wheel_front_right · motor torque "
                 "(N·m)\"",
          "Header");
    int rows = 0;
    while (std::getline(csv, line)) {
        check(std::count(line.begin(), line.end(), ',') == 2, "Columns");
        ++rows;
    }
    check(rows == static_cast<int>(recording.size()), "Rows");
}

#include "robotics/replay/recording.hpp"

#include "robotics/sensors/sensor_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace robotics {
namespace {

// Copy of the world without the telemetry that dominates its size.
std::shared_ptr<const World> makeKeyframe(const World& world) {
    auto keyframe = std::make_shared<World>(world);
    for (auto& joint : keyframe->joints) {
        joint.history.clear();
    }
    for (auto& sensor : keyframe->sensors) {
        sensor.state.history.clear();
    }
    return keyframe;
}

std::size_t worldBytes(const World& world) {
    std::size_t bytes =
        sizeof(World) + world.bodies.capacity() * sizeof(Body) + world.joints.capacity() * sizeof(Joint)
        + world.contacts.capacity() * sizeof(Contact) + world.sensors.capacity() * sizeof(Sensor);
    for (const auto& sensor : world.sensors) {
        const SensorState& state = sensor.state;
        bytes += (state.value.capacity() + state.truth.capacity() + state.walk.capacity()) * sizeof(double)
                 + state.rayEnds.capacity() * sizeof(Vec3) + state.rayHits.capacity();
        for (const auto& entry : state.pipeline) {
            bytes += entry.capacity() * sizeof(double);
        }
    }
    return bytes;
}

} // namespace

Recording::Recording(double seconds, double physicsStep) : physicsStep_(physicsStep) {
    setDuration(seconds);
}

void Recording::setDuration(double seconds) {
    if (!std::isfinite(seconds) || seconds < minDuration) {
        throw std::runtime_error("Recording length must be at least 5 seconds");
    }
    duration_ = seconds;
    trim();
}

void Recording::clear() {
    frames_.clear();
    steps_ = 0;
    framesSinceKeyframe_ = 0;
}

void Recording::start(const World& world) {
    clear();
    capture(world);
}

void Recording::afterStep(const World& world) {
    ++steps_;
    if (steps_ % stepsPerFrame == 0) {
        capture(world);
    }
}

void Recording::capture(const World& world) {
    RecordingFrame frame;
    frame.time = world.time;
    frame.step = steps_;
    frame.contacts = static_cast<int>(world.contacts.size());
    frame.bodies.reserve(world.bodies.size());
    for (const Body& body : world.bodies) {
        frame.bodies.push_back({body.position, body.orientation, body.velocity, body.angularVelocity});
    }
    frame.joints.reserve(world.joints.size());
    for (const Joint& joint : world.joints) {
        RecordedJoint recorded;
        recorded.coordinate = joint.angle;
        if (!joint.history.empty()) {
            recorded.speed = joint.history.back().speed;
            recorded.effort = joint.history.back().effort;
        }
        recorded.reactionForce = joint.reactionImpulse.length() / physicsStep_;
        recorded.reactionTorque = joint.reactionAngularImpulse.length() / physicsStep_;
        recorded.motorSpeed = joint.motorSpeed;
        frame.joints.push_back(recorded);
    }
    frame.sensors.assign(world.sensors.size() * Sensor::historyChannels,
                         std::numeric_limits<double>::quiet_NaN());
    for (std::size_t s = 0; s < world.sensors.size(); ++s) {
        const SensorState& state = world.sensors[s].state;
        if (state.valid) {
            std::copy_n(state.value.begin(), std::min(state.value.size(), Sensor::historyChannels),
                        frame.sensors.begin() + static_cast<std::ptrdiff_t>(s * Sensor::historyChannels));
        }
    }
    if (framesSinceKeyframe_ == 0) {
        frame.keyframe = makeKeyframe(world);
    }
    framesSinceKeyframe_ = (framesSinceKeyframe_ + 1) % framesPerKeyframe;
    frames_.push_back(std::move(frame));
    trim();
}

void Recording::trim() {
    const double physicsPerFrame = physicsStep_ * stepsPerFrame;
    const auto capacity = static_cast<std::size_t>(std::ceil(duration_ / physicsPerFrame));
    while (frames_.size() > capacity + framesPerKeyframe) {
        // Drop the block up to the next keyframe; the front stays restorable.
        do {
            frames_.pop_front();
        } while (!frames_.empty() && !frames_.front().keyframe);
    }
}

std::size_t Recording::indexAtTime(double time) const {
    auto next = std::lower_bound(frames_.begin(), frames_.end(), time,
                                 [](const RecordingFrame& frame, double t) { return frame.time < t; });
    if (next == frames_.end()) {
        return frames_.size() - 1;
    }
    auto index = static_cast<std::size_t>(next - frames_.begin());
    if (index > 0 && time - frames_[index - 1].time < next->time - time) {
        --index;
    }
    return index;
}

void Recording::applyFrame(World& world, std::size_t index) const {
    const RecordingFrame& frame = frames_.at(index);
    world.time = frame.time;
    for (std::size_t i = 0; i < world.bodies.size() && i < frame.bodies.size(); ++i) {
        Body& body = world.bodies[i];
        const RecordedBody& recorded = frame.bodies[i];
        body.position = recorded.position;
        body.orientation = recorded.orientation;
        body.velocity = recorded.velocity;
        body.angularVelocity = recorded.angularVelocity;
    }
    for (std::size_t j = 0; j < world.joints.size() && j < frame.joints.size(); ++j) {
        world.joints[j].angle = frame.joints[j].coordinate;
    }
}

World Recording::view(std::size_t index) const {
    std::size_t key = index;
    while (key > 0 && !frames_.at(key).keyframe) {
        --key;
    }
    World world = *frames_.at(key).keyframe;
    applyFrame(world, index);
    world.contacts.clear();
    for (auto& sensor : world.sensors) {
        if (sensor.type == SensorType::Lidar2D || sensor.type == SensorType::Rangefinder) {
            sensor.state.valid = false; // The rays of the keyframe do not match this frame's pose.
        }
    }
    return world;
}

World Recording::restore(std::size_t index, const World& live) const {
    std::size_t key = index;
    while (key > 0 && !frames_.at(key).keyframe) {
        --key;
    }
    World world = *frames_.at(key).keyframe;
    for (std::size_t f = key + 1; f <= index; ++f) {
        const RecordingFrame& frame = frames_[f];
        for (std::size_t j = 0; j < world.joints.size() && j < frame.joints.size(); ++j) {
            world.joints[j].motorSpeed = frame.joints[j].motorSpeed;
        }
        for (std::uint64_t s = frames_[f - 1].step; s < frame.step; ++s) {
            stepSimulation(world, physicsStep_);
        }
    }
    applyFrame(world, index);
    const double time = frames_[index].time;
    // Keep what the plots have seen up to this moment. The re-simulated steps already added their samples.
    if (live.joints.size() == world.joints.size()) {
        for (std::size_t j = 0; j < world.joints.size(); ++j) {
            world.joints[j].history = live.joints[j].history;
            auto& history = world.joints[j].history;
            while (!history.empty() && history.back().time > time) {
                history.pop_back();
            }
        }
    }
    if (live.sensors.size() == world.sensors.size()) {
        for (std::size_t s = 0; s < world.sensors.size(); ++s) {
            world.sensors[s].state.history = live.sensors[s].state.history;
            auto& history = world.sensors[s].state.history;
            while (!history.empty() && history.back().time > time) {
                history.pop_back();
            }
        }
    }
    return world;
}

void Recording::truncateAfter(std::size_t index) {
    if (index + 1 >= frames_.size()) {
        return;
    }
    frames_.resize(index + 1);
    steps_ = frames_.back().step;
    framesSinceKeyframe_ = 0;
    for (std::size_t i = index + 1; i-- > 0 && !frames_[i].keyframe;) {
        ++framesSinceKeyframe_;
    }
    framesSinceKeyframe_ = (framesSinceKeyframe_ + 1) % framesPerKeyframe;
}

std::size_t Recording::memoryBytes() const {
    std::size_t bytes = 0;
    for (const auto& frame : frames_) {
        bytes += sizeof(RecordingFrame) + frame.bodies.capacity() * sizeof(RecordedBody)
                 + frame.joints.capacity() * sizeof(RecordedJoint)
                 + frame.sensors.capacity() * sizeof(double);
        if (frame.keyframe) {
            bytes += worldBytes(*frame.keyframe);
        }
    }
    return bytes;
}

} // namespace robotics

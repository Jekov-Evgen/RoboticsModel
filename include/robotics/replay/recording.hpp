#pragma once
// Recording of a simulation run: a bounded history of light frames (poses, velocities, joint and sensor
// readings) with a full World snapshot every framesPerKeyframe frames. Frames are enough to display and plot
// any moment; Recording::restore() rebuilds the complete simulation state (contacts, controllers, sensors) of
// any frame from the keyframe before it.

#include "robotics/world.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

namespace robotics {

struct RecordedBody {
    Vec3 position;
    Quat orientation;
    Vec3 velocity;
    Vec3 angularVelocity;
};

struct RecordedJoint {
    double coordinate{};     // Joint::angle (rad or m).
    double speed{};          // rad/s or m/s; 0 for joints without a free coordinate.
    double effort{};         // N·m or N applied by the motor or controller.
    double reactionForce{};  // Magnitude of the constraint force (N).
    double reactionTorque{}; // Magnitude of the constraint torque (N·m).
    double motorSpeed{};     // Command, replayed by Recording::restore().
};

struct RecordingFrame {
    double time{};
    std::uint64_t step{}; // Physics steps since the recording began.
    int contacts{};
    std::vector<RecordedBody> bodies;
    std::vector<RecordedJoint> joints;
    // Sensor::historyChannels values per sensor; NaN where the sensor has no reading.
    std::vector<double> sensors;
    std::shared_ptr<const World> keyframe; // Set on keyframes; telemetry histories stripped.
};

class Recording {
public:
    static constexpr int stepsPerFrame = 4; // 60 frames per second at the default 240 Hz physics step.
    static constexpr int framesPerKeyframe =
        30; // A keyframe every 0.5 s; restore() re-simulates at most that.
    static constexpr double defaultDuration = 60;
    static constexpr double minDuration = 5;

    explicit Recording(double seconds = defaultDuration, double physicsStep = World::defaultTimeStep);

    // Length of the ring buffer. The oldest frames are dropped in whole keyframe blocks, so the first frame
    // can always be restored.
    void setDuration(double seconds);

    [[nodiscard]] double duration() const { return duration_; }

    void clear();
    void start(const World& world);     // Clears and stores the first frame.
    void afterStep(const World& world); // Call after every physics step; stores every stepsPerFrame-th.

    [[nodiscard]] bool empty() const { return frames_.empty(); }

    [[nodiscard]] std::size_t size() const { return frames_.size(); }

    [[nodiscard]] const RecordingFrame& frame(std::size_t index) const { return frames_.at(index); }

    [[nodiscard]] double startTime() const { return frames_.empty() ? 0 : frames_.front().time; }

    [[nodiscard]] double endTime() const { return frames_.empty() ? 0 : frames_.back().time; }

    // Frame closest to `time` (clamped to the recording). Requires a non-empty recording.
    [[nodiscard]] std::size_t indexAtTime(double time) const;

    // World showing a frame for display: bodies, joint coordinates and time as recorded, the rest (including
    // configuration) from the keyframe before it. Range sensors show no stale rays.
    [[nodiscard]] World view(std::size_t index) const;

    // Complete simulation state at frame `index`: the keyframe before it re-simulated up to the frame with
    // the recorded motor commands. Telemetry histories are taken from `live`, cut at the frame's time.
    [[nodiscard]] World restore(std::size_t index, const World& live) const;

    // Drops every frame after `index`, so recording can continue from a restored state.
    void truncateAfter(std::size_t index);

    // Approximate heap use of all frames and keyframes.
    [[nodiscard]] std::size_t memoryBytes() const;

private:
    void capture(const World& world);
    void applyFrame(World& world, std::size_t index) const;
    void trim();

    std::deque<RecordingFrame> frames_;
    double duration_;
    double physicsStep_;
    std::uint64_t steps_{};
    int framesSinceKeyframe_{};
};

} // namespace robotics

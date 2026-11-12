#pragma once
// Sensor models attached to bodies: configuration (saved in .robot files) and runtime state.
//
// Data only. The simulation (sampling, noise, latency, ray casting) lives in src/sensors; see
// robotics/sensors/sensor_system.hpp. World stores its sensors in World::sensors so that copies of a
// World (editor undo, Reset snapshots, save/load) carry them along; World::step() itself never reads them.

#include "robotics/math.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <string_view>
#include <vector>

namespace robotics {

// Stored in .robot files by numeric value: append new types at the end only.
enum class SensorType {
    JointEncoder,
    JointVelocity,
    Imu,
    ForceTorque,
    Contact,
    Rangefinder,
    Lidar2D,
    Camera
};

[[nodiscard]] constexpr bool isKnownSensorType(SensorType type) {
    switch (type) {
    case SensorType::JointEncoder:
    case SensorType::JointVelocity:
    case SensorType::Imu:
    case SensorType::ForceTorque:
    case SensorType::Contact:
    case SensorType::Rangefinder:
    case SensorType::Lidar2D:
    case SensorType::Camera:
        return true;
    }
    return false;
}

[[nodiscard]] constexpr std::string_view sensorTypeName(SensorType type) {
    switch (type) {
    case SensorType::JointEncoder:
        return "Encoder";
    case SensorType::JointVelocity:
        return "Joint velocity";
    case SensorType::Imu:
        return "IMU";
    case SensorType::ForceTorque:
        return "Force/torque";
    case SensorType::Contact:
        return "Contact";
    case SensorType::Rangefinder:
        return "Rangefinder";
    case SensorType::Lidar2D:
        return "2D lidar";
    case SensorType::Camera:
        return "Camera";
    }
    return "Unknown";
}

// Joint sensors measure the joint whose child (Joint::b) is the sensor's body.
[[nodiscard]] constexpr bool isJointSensorType(SensorType type) {
    return type == SensorType::JointEncoder || type == SensorType::JointVelocity
           || type == SensorType::ForceTorque;
}

// Error model of one channel group, applied in this order on every sample:
//   measured = quantize(truth + bias + walk + stddev * N(0, 1)),  walk += biasWalk * sqrt(period) * N(0, 1)
// Units follow the channel (rad, m, m/s², N, ...).
struct NoiseModel {
    double stddev{};     // Gaussian white noise sigma.
    double bias{};       // Constant offset.
    double biasWalk{};   // Bias random walk: sigma growth per sqrt(second).
    double resolution{}; // Quantization step (rounding to the nearest multiple); 0 = continuous.

    [[nodiscard]] bool operator==(const NoiseModel&) const = default;
};

// Runtime state; reset (cleared) on Reset/load. Not saved in .robot files.
struct SensorState {
    // One sample of the plotted history: the delayed output of the first historyChannels channels.
    struct Sample {
        double time{};
        std::vector<double> values;
    };

    std::mt19937_64 rng;      // Seeded with Sensor::seed on the first update (std::mt19937_64 is portable).
    bool initialized{};       // rng seeded and timing started.
    double nextSampleTime{};  // World time of the next sample (sample-and-hold in between).
    double outputTime{};      // World time at which `value` was produced.
    std::vector<double> walk; // Current bias random-walk offset per channel.
    std::deque<std::vector<double>> pipeline; // Latency buffer: newest measurement at the back.
    std::vector<double> value; // Output seen by consumers (noisy, quantized, delayed, held between samples).
    std::vector<double> truth; // Ideal value of the latest sample (no noise, no delay).
    bool valid{};              // `value` holds a reading (false before the first sample or without a target).
    bool fresh{};              // The last update produced a new sample (CSV logging, camera rendering).
    std::uint64_t samples{};
    std::uint64_t dropped{};
    std::deque<Sample> history; // Bounded by Sensor::historyCapacity.

    // Model internals.
    Vec3 previousVelocity{}; // IMU: mount-point velocity at the previous physics step.
    bool hasPreviousVelocity{};
    Vec3 acceleration{};       // IMU: mount-point acceleration of the last step (world space).
    double speedEstimate{};    // Encoder: finite-difference speed of the output (unit/s).
    Vec3 rayOrigin{};          // Range sensors: world-space rays of the latest scan, for visualization.
    std::vector<Vec3> rayEnds; // End point of every beam (hit point, or the point at max range).
    std::vector<char> rayHits; // 1 where the beam hit something within range.
};

struct Sensor {
    static constexpr std::size_t historyCapacity = 1200; // Plot samples kept per sensor.
    static constexpr std::size_t historyChannels = 12;   // Channels kept per history sample.
    static constexpr int maxLatency = 1000;              // Samples.
    static constexpr int maxBeams = 2048;
    static constexpr int maxImageSize = 1024; // Camera pixels per side.

    std::string name{"sensor"};
    SensorType type{SensorType::Imu};
    int body{}; // Mount body index. Joint sensors: child body of the measured joint.

    // Mount pose in the body frame. Sensor frame: +X forward (range sensors and camera look along +X),
    // +Y up, +Z right. Lidar beams sweep the X-Z plane, positive angles turn towards -Z (left).
    Vec3 position{};
    Quat orientation{};

    // Timing and error model.
    double rate{100};      // Samples per second; 0 = every physics step.
    int latency{};         // Output delay in samples (delay line of `latency` samples).
    double dropout{};      // Probability in [0, 1) that a sample is lost (the previous measurement repeats).
    std::uint64_t seed{1}; // Noise generator seed: equal seeds and inputs give identical readings.
    NoiseModel noise;      // Encoder, velocity, range, accelerometer, force, contact force.
    NoiseModel noise2;     // Gyroscope (rad/s) and torque (N·m).
    double orientationNoise{}; // IMU orientation estimate: sigma of the error rotation per axis (rad).

    // Type-specific parameters.
    double encoderCounts{}; // Encoder: counts per revolution (hinge) or per metre (slider); 0 = continuous.
    bool feedsController{}; // Encoder / joint velocity: the joint's feedback controller uses this reading.
    double minRange{0.05};  // Range sensors (m). Returns closer than this are invalid.
    double maxRange{8};     // Range sensors (m); invalid returns read maxRange. Camera: depth clip.
    double fov{4.71238898038}; // Lidar: horizontal field of view; camera: vertical field of view (rad).
    int beams{180};            // Lidar beams.
    int width{160};            // Camera image width (pixels).
    int height{120};           // Camera image height (pixels).

    SensorState state;

    [[nodiscard]] bool isJointSensor() const { return isJointSensorType(type); }

    // Number of output channels (0 for the camera, whose image is rendered by the editor).
    [[nodiscard]] std::size_t channelCount() const {
        switch (type) {
        case SensorType::JointEncoder:
        case SensorType::JointVelocity:
        case SensorType::Rangefinder:
            return 1;
        case SensorType::Contact:
            return 2;
        case SensorType::ForceTorque:
            return 6;
        case SensorType::Imu:
            return 10;
        case SensorType::Lidar2D:
            return beams > 0 ? static_cast<std::size_t>(beams) : 0;
        case SensorType::Camera:
            return 0;
        }
        return 0;
    }
};

} // namespace robotics

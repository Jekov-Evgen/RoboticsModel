#pragma once
// Sensor simulation: sampling, noise, latency and the measurement models of World::sensors.
// Reference: docs/SENSORS.md.
//
// Sensors are not part of World::step(). Call updateSensors() after every step (or use stepSimulation()):
//
//   robotics::stepSimulation(world);           // world.step() + robotics::updateSensors(world, dt)
//   double angle = world.sensors[0].state.value[0];
//
// Encoder and joint-velocity sensors with Sensor::feedsController hand their reading to the joint's feedback
// controller for the next step (ControllerState::sensorFeedback), so noise, quantization and latency reach
// the control loop.

#include "robotics/sensors/sensor.hpp"
#include "robotics/world.hpp"

#include <iosfwd>
#include <random>
#include <string>
#include <vector>

namespace robotics {

struct SensorPose {
    Vec3 position{};
    Quat orientation{};
};

// A sensor with sensible defaults for its type, mounted at the centre of `body`.
[[nodiscard]] Sensor makeSensor(SensorType type, int body);

// Advances every sensor after World::step(dt): takes the samples that are due, updates held outputs and the
// controller feedback of joints with feedsController sensors. `dt` is the step that was just taken.
void updateSensors(World& world, double dt = World::defaultTimeStep);

// world.step(dt) followed by updateSensors(world, dt).
void stepSimulation(World& world, double dt = World::defaultTimeStep);

// Clears the runtime state of every sensor (outputs, delay lines, noise generators, history) and the
// controller feedback taken from sensors.
void resetSensors(World& world);

// Throws std::runtime_error describing the first invalid sensor setting (index, ranges, rates).
void validateSensors(const World& world);
void validateSensor(const Sensor& sensor, std::size_t bodyCount);

// Joint measured by a joint sensor: the first joint whose child is the sensor's body (encoders and joint
// velocity sensors need a hinge or slider). -1 if there is none or the sensor is not a joint sensor.
[[nodiscard]] int sensorJoint(const World& world, const Sensor& sensor);

// World pose of the sensor frame (body pose * mount pose).
[[nodiscard]] SensorPose sensorPose(const World& world, const Sensor& sensor);

// Lidar beam k in the sensor frame (unit vector in the X-Z plane).
[[nodiscard]] Vec3 lidarBeamDirection(const Sensor& sensor, int beam);

// Channel names and units, e.g. "accel_x" / "m/s^2". Size == Sensor::channelCount().
[[nodiscard]] std::vector<std::string> sensorChannelNames(const World& world, const Sensor& sensor);
[[nodiscard]] std::string sensorChannelUnit(const World& world, const Sensor& sensor, std::size_t channel);

// Deterministic standard normal sample (Box-Muller on the raw 64-bit output), identical on every platform,
// unlike std::normal_distribution.
[[nodiscard]] double sampleNormal(std::mt19937_64& rng);
[[nodiscard]] double sampleUniform(std::mt19937_64& rng); // In (0, 1).

// Long-format CSV of sensor outputs: one row per channel of every sensor that sampled in the last update.
//   time,sensor,name,type,channel,value,truth
void writeSensorCsvHeader(std::ostream& out);
void writeSensorCsvRows(std::ostream& out, const World& world);

} // namespace robotics

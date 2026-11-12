#pragma once
// Plot channels: every scalar quantity of a simulation that can be read from a recorded frame.

#include "robotics/replay/recording.hpp"

#include <iosfwd>
#include <string>
#include <vector>

namespace robotics {

enum class ChannelSource { Scene, Body, Joint, Sensor };
enum class SceneQuantity { KineticEnergy, PotentialEnergy, TotalEnergy, ContactCount };
enum class BodyQuantity {
    PositionX,
    PositionY,
    PositionZ,
    VelocityX,
    VelocityY,
    VelocityZ,
    Speed,
    AngularVelocityX,
    AngularVelocityY,
    AngularVelocityZ,
    AngularSpeed,
    Roll,
    Pitch,
    Yaw,
    KineticEnergy,
    PotentialEnergy
};
enum class JointQuantity { Coordinate, Speed, Effort, ReactionForce, ReactionTorque };

struct Channel {
    ChannelSource source{ChannelSource::Scene};
    int index{-1};            // Body, joint or sensor index; -1 for scene quantities.
    int quantity{};           // SceneQuantity, BodyQuantity, JointQuantity, or the sensor channel.
    std::string owner;        // Name of the part, joint or sensor ("Scene" for scene quantities).
    std::string quantityName; // E.g. "position Y".
    std::string unit;

    [[nodiscard]] std::string label() const { return owner + " · " + quantityName; }
};

// Every channel of the world: scene totals, then parts, joints and sensors. Sensors expose their first
// Sensor::historyChannels channels.
[[nodiscard]] std::vector<Channel> listChannels(const World& world);

[[nodiscard]] Channel bodyChannel(const World& world, int body, BodyQuantity quantity);
[[nodiscard]] Channel jointChannel(const World& world, int joint, JointQuantity quantity);

// Case-insensitive: every word of the query occurs in the label or unit.
[[nodiscard]] bool channelMatches(const Channel& channel, const std::string& query);

// Value in a frame recorded from `model`; NaN if the frame does not contain it. Angles are in degrees.
[[nodiscard]] double channelValue(const Channel& channel, const World& model, const RecordingFrame& frame);

// Finds the same quantity in an edited world (by owner name) and updates the index; false if it is gone.
[[nodiscard]] bool rebindChannel(Channel& channel, const World& world);

// Wide CSV: a time column, then one column per channel ("owner · quantity (unit)"), one row per frame.
void writeChannelCsv(std::ostream& out, const Recording& recording, const World& model,
                     const std::vector<Channel>& channels);

} // namespace robotics

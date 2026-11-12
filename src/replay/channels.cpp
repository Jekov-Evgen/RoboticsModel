#include "robotics/replay/channels.hpp"

#include "robotics/sensors/sensor_system.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <ostream>
#include <sstream>

namespace robotics {
namespace {

constexpr double nan = std::numeric_limits<double>::quiet_NaN();

struct QuantityInfo {
    const char* name;
    const char* unit;
};

constexpr std::array<QuantityInfo, 4> sceneQuantities{{{"total kinetic energy", "J"},
                                                       {"total potential energy", "J"},
                                                       {"total energy", "J"},
                                                       {"contact count", ""}}};
constexpr std::array<QuantityInfo, 16> bodyQuantities{{{"position X", "m"},
                                                       {"position Y", "m"},
                                                       {"position Z", "m"},
                                                       {"velocity X", "m/s"},
                                                       {"velocity Y", "m/s"},
                                                       {"velocity Z", "m/s"},
                                                       {"speed", "m/s"},
                                                       {"angular velocity X", "rad/s"},
                                                       {"angular velocity Y", "rad/s"},
                                                       {"angular velocity Z", "rad/s"},
                                                       {"angular speed", "rad/s"},
                                                       {"roll", "°"},
                                                       {"pitch", "°"},
                                                       {"yaw", "°"},
                                                       {"kinetic energy", "J"},
                                                       {"potential energy", "J"}}};

std::string jointOwner(const World& world, const Joint& joint) {
    return world.bodies[static_cast<std::size_t>(joint.a)].name + " → "
           + world.bodies[static_cast<std::size_t>(joint.b)].name;
}

double kineticEnergy(const Body& body, const RecordedBody& state) {
    if (body.isStatic()) {
        return 0;
    }
    Vec3 inertia = body.principalInertia();
    Vec3 w = state.orientation.conjugate().rotate(state.angularVelocity);
    return 0.5 * body.mass * state.velocity.length2() + 0.5 * componentProduct(inertia, w).dot(w);
}

double potentialEnergy(const World& model, const Body& body, const RecordedBody& state) {
    return body.isStatic() ? 0 : -body.mass * model.gravity.dot(state.position);
}

std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

void writeCsvField(std::ostream& out, const std::string& text) {
    out << '"';
    for (char c : text) {
        out << (c == '"' ? "\"\"" : std::string(1, c));
    }
    out << '"';
}

} // namespace

Channel bodyChannel(const World& world, int body, BodyQuantity quantity) {
    const QuantityInfo& info = bodyQuantities[static_cast<std::size_t>(quantity)];
    return {ChannelSource::Body,
            body,
            static_cast<int>(quantity),
            world.bodies.at(static_cast<std::size_t>(body)).name,
            info.name,
            info.unit};
}

Channel jointChannel(const World& world, int index, JointQuantity quantity) {
    const Joint& joint = world.joints.at(static_cast<std::size_t>(index));
    bool linear = joint.type == JointType::Prismatic;
    Channel channel{ChannelSource::Joint,     index, static_cast<int>(quantity),
                    jointOwner(world, joint), "",    ""};
    switch (quantity) {
    case JointQuantity::Coordinate:
        channel.quantityName = linear ? "joint position" : "joint angle";
        channel.unit = linear ? "m" : "rad";
        break;
    case JointQuantity::Speed:
        channel.quantityName = "joint speed";
        channel.unit = linear ? "m/s" : "rad/s";
        break;
    case JointQuantity::Effort:
        channel.quantityName = linear ? "motor force" : "motor torque";
        channel.unit = linear ? "N" : "N·m";
        break;
    case JointQuantity::ReactionForce:
        channel.quantityName = "reaction force";
        channel.unit = "N";
        break;
    case JointQuantity::ReactionTorque:
        channel.quantityName = "reaction torque";
        channel.unit = "N·m";
        break;
    }
    return channel;
}

std::vector<Channel> listChannels(const World& world) {
    std::vector<Channel> channels;
    for (std::size_t q = 0; q < sceneQuantities.size(); ++q) {
        channels.push_back({ChannelSource::Scene, -1, static_cast<int>(q), "Scene", sceneQuantities[q].name,
                            sceneQuantities[q].unit});
    }
    for (int i = 0; i < static_cast<int>(world.bodies.size()); ++i) {
        for (std::size_t q = 0; q < bodyQuantities.size(); ++q) {
            auto quantity = static_cast<BodyQuantity>(q);
            bool pose = quantity <= BodyQuantity::PositionZ
                        || (quantity >= BodyQuantity::Roll && quantity <= BodyQuantity::Yaw);
            if (pose || !world.bodies[static_cast<std::size_t>(i)].isStatic()) {
                channels.push_back(bodyChannel(world, i, quantity));
            }
        }
    }
    for (int j = 0; j < static_cast<int>(world.joints.size()); ++j) {
        bool actuated = world.joints[static_cast<std::size_t>(j)].isActuated();
        for (JointQuantity quantity :
             {JointQuantity::Coordinate, JointQuantity::Speed, JointQuantity::Effort}) {
            if (actuated) {
                channels.push_back(jointChannel(world, j, quantity));
            }
        }
        channels.push_back(jointChannel(world, j, JointQuantity::ReactionForce));
        channels.push_back(jointChannel(world, j, JointQuantity::ReactionTorque));
    }
    for (int s = 0; s < static_cast<int>(world.sensors.size()); ++s) {
        const Sensor& sensor = world.sensors[static_cast<std::size_t>(s)];
        std::vector<std::string> names = sensorChannelNames(world, sensor);
        for (std::size_t c = 0; c < names.size() && c < Sensor::historyChannels; ++c) {
            channels.push_back({ChannelSource::Sensor, s, static_cast<int>(c), sensor.name, names[c],
                                sensorChannelUnit(world, sensor, c)});
        }
    }
    return channels;
}

bool channelMatches(const Channel& channel, const std::string& query) {
    std::string text = lower(channel.label() + " " + channel.unit);
    std::istringstream words(lower(query));
    std::string word;
    while (words >> word) {
        if (text.find(word) == std::string::npos) {
            return false;
        }
    }
    return true;
}

double channelValue(const Channel& channel, const World& model, const RecordingFrame& frame) {
    auto index = static_cast<std::size_t>(channel.index);
    switch (channel.source) {
    case ChannelSource::Scene: {
        auto quantity = static_cast<SceneQuantity>(channel.quantity);
        if (quantity == SceneQuantity::ContactCount) {
            return frame.contacts;
        }
        double kinetic = 0;
        double potential = 0;
        for (std::size_t i = 0; i < model.bodies.size() && i < frame.bodies.size(); ++i) {
            kinetic += kineticEnergy(model.bodies[i], frame.bodies[i]);
            potential += potentialEnergy(model, model.bodies[i], frame.bodies[i]);
        }
        return quantity == SceneQuantity::KineticEnergy
                   ? kinetic
                   : (quantity == SceneQuantity::PotentialEnergy ? potential : kinetic + potential);
    }
    case ChannelSource::Body: {
        if (channel.index < 0 || index >= frame.bodies.size() || index >= model.bodies.size()) {
            return nan;
        }
        const RecordedBody& body = frame.bodies[index];
        constexpr double degrees = 180 / pi;
        switch (static_cast<BodyQuantity>(channel.quantity)) {
        case BodyQuantity::PositionX:
            return body.position.x;
        case BodyQuantity::PositionY:
            return body.position.y;
        case BodyQuantity::PositionZ:
            return body.position.z;
        case BodyQuantity::VelocityX:
            return body.velocity.x;
        case BodyQuantity::VelocityY:
            return body.velocity.y;
        case BodyQuantity::VelocityZ:
            return body.velocity.z;
        case BodyQuantity::Speed:
            return body.velocity.length();
        case BodyQuantity::AngularVelocityX:
            return body.angularVelocity.x;
        case BodyQuantity::AngularVelocityY:
            return body.angularVelocity.y;
        case BodyQuantity::AngularVelocityZ:
            return body.angularVelocity.z;
        case BodyQuantity::AngularSpeed:
            return body.angularVelocity.length();
        case BodyQuantity::Roll:
            return body.orientation.eulerAngles().x * degrees;
        case BodyQuantity::Pitch:
            return body.orientation.eulerAngles().y * degrees;
        case BodyQuantity::Yaw:
            return body.orientation.eulerAngles().z * degrees;
        case BodyQuantity::KineticEnergy:
            return kineticEnergy(model.bodies[index], body);
        case BodyQuantity::PotentialEnergy:
            return potentialEnergy(model, model.bodies[index], body);
        }
        return nan;
    }
    case ChannelSource::Joint: {
        if (channel.index < 0 || index >= frame.joints.size()) {
            return nan;
        }
        const RecordedJoint& joint = frame.joints[index];
        switch (static_cast<JointQuantity>(channel.quantity)) {
        case JointQuantity::Coordinate:
            return joint.coordinate;
        case JointQuantity::Speed:
            return joint.speed;
        case JointQuantity::Effort:
            return joint.effort;
        case JointQuantity::ReactionForce:
            return joint.reactionForce;
        case JointQuantity::ReactionTorque:
            return joint.reactionTorque;
        }
        return nan;
    }
    case ChannelSource::Sensor: {
        std::size_t slot = index * Sensor::historyChannels + static_cast<std::size_t>(channel.quantity);
        return channel.index >= 0 && slot < frame.sensors.size() ? frame.sensors[slot] : nan;
    }
    }
    return nan;
}

bool rebindChannel(Channel& channel, const World& world) {
    if (channel.source == ChannelSource::Scene) {
        return true;
    }
    for (const Channel& candidate : listChannels(world)) {
        if (candidate.source == channel.source && candidate.quantity == channel.quantity
            && candidate.owner == channel.owner) {
            channel = candidate;
            return true;
        }
    }
    return false;
}

void writeChannelCsv(std::ostream& out, const Recording& recording, const World& model,
                     const std::vector<Channel>& channels) {
    out << "time";
    for (const Channel& channel : channels) {
        out << ',';
        writeCsvField(out,
                      channel.unit.empty() ? channel.label() : channel.label() + " (" + channel.unit + ")");
    }
    out << '\n';
    auto precision = out.precision(10);
    for (std::size_t f = 0; f < recording.size(); ++f) {
        const RecordingFrame& frame = recording.frame(f);
        out << frame.time;
        for (const Channel& channel : channels) {
            out << ',';
            double value = channelValue(channel, model, frame);
            if (std::isfinite(value)) {
                out << value;
            }
        }
        out << '\n';
    }
    out.precision(precision);
}

} // namespace robotics

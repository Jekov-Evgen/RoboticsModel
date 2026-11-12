#include "robotics/sensors/sensor_system.hpp"

#include "robotics/sensors/raycast.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace robotics {
namespace {

// Tolerance when comparing the world clock with the next sample time (steps are 1/240 s).
constexpr double sampleTimeTolerance = 1e-9;
constexpr double maxSensorRate = 100000;

bool isBodyIndex(int index, std::size_t count) {
    return index >= 0 && index < static_cast<int>(count);
}

// Joint coordinate rate: rad/s for hinges, m/s for sliders (same definition as the encoder in actuators).
double jointSpeed(const World& world, const Joint& joint) {
    const Body& a = world.bodies[static_cast<std::size_t>(joint.a)];
    const Body& b = world.bodies[static_cast<std::size_t>(joint.b)];
    Vec3 axis = a.orientation.rotate(joint.axisA);
    if (joint.type == JointType::Prismatic) {
        Vec3 point = b.point(joint.anchorB);
        return (b.pointVelocity(point) - a.pointVelocity(point)).dot(axis);
    }
    return (b.angularVelocity - a.angularVelocity).dot(axis);
}

// Noise group of a channel: 0 = Sensor::noise, 1 = Sensor::noise2, -1 = none.
int noiseGroup(const Sensor& sensor, std::size_t channel) {
    switch (sensor.type) {
    case SensorType::Imu:
        return channel < 3 ? 0 : (channel < 6 ? 1 : -1);
    case SensorType::ForceTorque:
        return channel < 3 ? 0 : 1;
    case SensorType::Contact:
        return channel == 0 ? -1 : 0;
    default:
        return 0;
    }
}

double quantize(double value, double resolution) {
    return resolution > 0 ? std::round(value / resolution) * resolution : value;
}

// ---- Measurement models: ideal values in the sensor's channel layout --------------------------------

std::vector<double> measureJoint(const World& world, const Sensor& sensor, double dt) {
    int index = sensorJoint(world, sensor);
    if (index < 0) {
        return {};
    }
    const Joint& joint = world.joints[static_cast<std::size_t>(index)];
    if (sensor.type == SensorType::JointEncoder) {
        return {joint.angle};
    }
    if (sensor.type == SensorType::JointVelocity) {
        return {jointSpeed(world, joint)};
    }
    // Force/torque: wrench the parent transmits to the child (body B) through the joint, averaged over the
    // step, about the joint anchor, in the sensor frame. The solver stores impulses acting on A.
    const Body& a = world.bodies[static_cast<std::size_t>(joint.a)];
    Vec3 axis = a.orientation.rotate(joint.axisA);
    Vec3 force = -joint.reactionImpulse / dt;
    Vec3 torque = -joint.reactionAngularImpulse / dt;
    if (joint.isActuated()) {
        double along = (-joint.motorImpulse + joint.lowerLimitImpulse - joint.upperLimitImpulse) / dt;
        if (!joint.usesSpeedMotor() && !joint.history.empty()) {
            along += joint.history.back().effort; // Feedback controller effort, applied as an impulse pair.
        }
        if (joint.type == JointType::Prismatic) {
            force += axis * along;
        } else {
            torque += axis * along;
        }
    }
    Quat frame = sensorPose(world, sensor).orientation.conjugate();
    Vec3 f = frame.rotate(force);
    Vec3 t = frame.rotate(torque);
    return {f.x, f.y, f.z, t.x, t.y, t.z};
}

std::vector<double> measureImu(const World& world, const Sensor& sensor) {
    SensorPose pose = sensorPose(world, sensor);
    const Body& body = world.bodies[static_cast<std::size_t>(sensor.body)];
    Quat inverse = pose.orientation.conjugate();
    // Accelerometer: specific force = acceleration of the mount point minus gravity.
    Vec3 specific = inverse.rotate(sensor.state.acceleration - world.gravity);
    Vec3 gyro = inverse.rotate(body.angularVelocity);
    Quat q = pose.orientation;
    return {specific.x, specific.y, specific.z, gyro.x, gyro.y, gyro.z, q.w, q.x, q.y, q.z};
}

std::vector<double> measureContact(const World& world, const Sensor& sensor, double dt) {
    bool touching = false;
    double normalForce = 0;
    for (const auto& contact : world.contacts) {
        if (contact.a == sensor.body || contact.b == sensor.body) {
            touching = true;
            normalForce += contact.normalImpulse / dt;
        }
    }
    return {touching ? 1.0 : 0.0, normalForce};
}

std::vector<double> measureRange(const World& world, Sensor& sensor) {
    SensorPose pose = sensorPose(world, sensor);
    int beams = sensor.type == SensorType::Rangefinder ? 1 : sensor.beams;
    std::vector<double> ranges(static_cast<std::size_t>(beams), sensor.maxRange);
    auto& state = sensor.state;
    state.rayOrigin = pose.position;
    state.rayEnds.assign(static_cast<std::size_t>(beams), pose.position);
    state.rayHits.assign(static_cast<std::size_t>(beams), 0);
    for (int k = 0; k < beams; ++k) {
        Vec3 local = sensor.type == SensorType::Rangefinder ? Vec3{1, 0, 0} : lidarBeamDirection(sensor, k);
        Vec3 direction = pose.orientation.rotate(local);
        RayHit hit = raycast(world, pose.position, direction, sensor.maxRange, sensor.body);
        auto index = static_cast<std::size_t>(k);
        if (hit.hit && hit.distance >= sensor.minRange) {
            ranges[index] = hit.distance;
            state.rayHits[index] = 1;
            state.rayEnds[index] = hit.point;
        } else {
            state.rayEnds[index] = pose.position + direction * sensor.maxRange;
        }
    }
    return ranges;
}

std::vector<double> measure(const World& world, Sensor& sensor, double dt) {
    switch (sensor.type) {
    case SensorType::JointEncoder:
    case SensorType::JointVelocity:
    case SensorType::ForceTorque:
        return measureJoint(world, sensor, dt);
    case SensorType::Imu:
        return measureImu(world, sensor);
    case SensorType::Contact:
        return measureContact(world, sensor, dt);
    case SensorType::Rangefinder:
    case SensorType::Lidar2D:
        return measureRange(world, sensor);
    case SensorType::Camera:
        return {};
    }
    return {};
}

// ---- Error model and timing ---------------------------------------------------------------------------

// `countResolution`: extra quantization of encoder counts (rad or m per count), 0 for other sensors.
std::vector<double> corrupt(Sensor& sensor, const std::vector<double>& truth, double period,
                            double countResolution) {
    auto& state = sensor.state;
    state.walk.resize(truth.size(), 0.0);
    std::vector<double> measured = truth;
    for (std::size_t c = 0; c < truth.size(); ++c) {
        int group = noiseGroup(sensor, c);
        if (group < 0) {
            continue;
        }
        const NoiseModel& model = group == 0 ? sensor.noise : sensor.noise2;
        if (model.biasWalk > 0) {
            state.walk[c] += model.biasWalk * std::sqrt(period) * sampleNormal(state.rng);
        }
        double value = truth[c] + model.bias + state.walk[c];
        if (model.stddev > 0) {
            value += model.stddev * sampleNormal(state.rng);
        }
        measured[c] = quantize(value, std::max(model.resolution, countResolution));
        if (sensor.type == SensorType::Rangefinder || sensor.type == SensorType::Lidar2D) {
            // No return stays exactly maxRange; noisy returns stay within [0, maxRange].
            measured[c] =
                truth[c] >= sensor.maxRange ? sensor.maxRange : std::clamp(measured[c], 0.0, sensor.maxRange);
        }
    }
    if (sensor.type == SensorType::Imu && measured.size() == 10) {
        Quat q{truth[6], truth[7], truth[8], truth[9]};
        if (sensor.orientationNoise > 0) {
            Vec3 error{sampleNormal(state.rng), sampleNormal(state.rng), sampleNormal(state.rng)};
            error = error * sensor.orientationNoise;
            q = (q * Quat::axisAngle(error, error.length())).normalized();
        }
        measured[6] = q.w;
        measured[7] = q.x;
        measured[8] = q.y;
        measured[9] = q.z;
    }
    return measured;
}

void recordSample(Sensor& sensor, std::vector<double> truth, double time, double period,
                  double countResolution) {
    auto& state = sensor.state;
    bool lost = sensor.dropout > 0 && sampleUniform(state.rng) < sensor.dropout;
    std::vector<double> measured = corrupt(sensor, truth, period, countResolution);
    if (lost) {
        ++state.dropped;
        if (!state.pipeline.empty()) {
            measured = state.pipeline.back(); // A lost sample repeats the previous measurement.
        }
    }
    state.pipeline.push_back(std::move(measured));
    while (state.pipeline.size() > static_cast<std::size_t>(sensor.latency) + 1) {
        state.pipeline.pop_front();
    }
    const std::vector<double>& output = state.pipeline.front(); // Oldest = delayed by `latency` samples.
    if (sensor.type == SensorType::JointEncoder) {
        double previous = state.valid && !state.value.empty() ? state.value[0] : output[0];
        double elapsed = time - state.outputTime;
        state.speedEstimate = state.valid && elapsed > 0 ? (output[0] - previous) / elapsed : 0;
    }
    state.value = output;
    state.truth = std::move(truth);
    state.outputTime = time;
    state.valid = true;
    state.fresh = true;
    ++state.samples;
    SensorState::Sample entry;
    entry.time = time;
    entry.values.assign(state.value.begin(), state.value.begin()
                                                 + static_cast<std::ptrdiff_t>(
                                                     std::min(state.value.size(), Sensor::historyChannels)));
    state.history.push_back(std::move(entry));
    if (state.history.size() > Sensor::historyCapacity) {
        state.history.pop_front();
    }
}

// Per-step bookkeeping that must run every step regardless of the sample rate.
void trackMotion(const World& world, Sensor& sensor, double dt) {
    if (sensor.type != SensorType::Imu) {
        return;
    }
    const Body& body = world.bodies[static_cast<std::size_t>(sensor.body)];
    Vec3 point = body.point(sensor.position);
    Vec3 velocity = body.pointVelocity(point);
    auto& state = sensor.state;
    if (state.hasPreviousVelocity) {
        state.acceleration = (velocity - state.previousVelocity) / dt;
    } else {
        // First step: centripetal part only (no history for the finite difference yet).
        Vec3 r = point - body.position;
        state.acceleration = body.angularVelocity.cross(body.angularVelocity.cross(r));
    }
    state.previousVelocity = velocity;
    state.hasPreviousVelocity = true;
}

void updateSensor(World& world, Sensor& sensor, double dt) {
    auto& state = sensor.state;
    state.fresh = false;
    if (!isBodyIndex(sensor.body, world.bodies.size())) {
        state.valid = false;
        return;
    }
    if (!state.initialized) {
        state.rng.seed(sensor.seed);
        state.nextSampleTime = world.time;
        state.outputTime = world.time;
        state.initialized = true;
    }
    trackMotion(world, sensor, dt);
    bool everyStep = sensor.rate <= 0;
    if (!everyStep && world.time + sampleTimeTolerance < state.nextSampleTime) {
        return; // Sample and hold.
    }
    double period = everyStep ? dt : 1 / sensor.rate;
    if (!everyStep) {
        state.nextSampleTime += period;
        if (state.nextSampleTime <= world.time + sampleTimeTolerance) {
            state.nextSampleTime = world.time + period; // Fell behind (e.g. rate above the step rate).
        }
    }
    if (sensor.type == SensorType::Camera) {
        state.fresh = true; // The editor renders the image; the core only keeps the frame clock.
        state.valid = true;
        state.outputTime = world.time;
        ++state.samples;
        return;
    }
    std::vector<double> truth = measure(world, sensor, dt);
    if (truth.empty()) {
        state.valid = false;
        return;
    }
    double countResolution = 0;
    if (sensor.type == SensorType::JointEncoder && sensor.encoderCounts > 0) {
        bool slider =
            world.joints[static_cast<std::size_t>(sensorJoint(world, sensor))].type == JointType::Prismatic;
        countResolution = (slider ? 1.0 : 2 * pi) / sensor.encoderCounts;
    }
    recordSample(sensor, std::move(truth), world.time, period, countResolution);
}

void writeName(std::ostream& out, const std::string& name) {
    out << '"';
    for (char c : name) {
        out << c;
        if (c == '"') {
            out << '"';
        }
    }
    out << '"';
}

} // namespace

double sampleUniform(std::mt19937_64& rng) {
    // 53 random bits mapped to the open interval (0, 1).
    return (static_cast<double>(rng() >> 11) + 0.5) * 0x1.0p-53;
}

double sampleNormal(std::mt19937_64& rng) {
    double u1 = sampleUniform(rng);
    double u2 = sampleUniform(rng);
    return std::sqrt(-2 * std::log(u1)) * std::cos(2 * pi * u2);
}

Sensor makeSensor(SensorType type, int body) {
    Sensor sensor;
    sensor.type = type;
    sensor.body = body;
    switch (type) {
    case SensorType::JointEncoder:
        sensor.name = "encoder";
        sensor.rate = 1000;
        sensor.encoderCounts = 4096; // Per revolution (hinge) or per metre (slider).
        break;
    case SensorType::JointVelocity:
        sensor.name = "joint_velocity";
        sensor.rate = 500;
        sensor.noise.stddev = 0.01;
        break;
    case SensorType::Imu:
        sensor.name = "imu";
        sensor.rate = 200;
        sensor.noise.stddev = 0.05;   // m/s^2
        sensor.noise2.stddev = 0.005; // rad/s
        sensor.noise2.biasWalk = 0.0005;
        sensor.orientationNoise = 0.002;
        break;
    case SensorType::ForceTorque:
        sensor.name = "force_torque";
        sensor.rate = 500;
        sensor.noise.stddev = 0.1;
        sensor.noise2.stddev = 0.01;
        break;
    case SensorType::Contact:
        sensor.name = "contact";
        sensor.rate = 240;
        break;
    case SensorType::Rangefinder:
        sensor.name = "rangefinder";
        sensor.rate = 50;
        sensor.noise.stddev = 0.01;
        sensor.minRange = 0.02;
        sensor.maxRange = 6;
        break;
    case SensorType::Lidar2D:
        sensor.name = "lidar";
        sensor.rate = 10;
        sensor.noise.stddev = 0.01;
        sensor.noise.resolution = 0.001;
        sensor.minRange = 0.1;
        sensor.maxRange = 8;
        sensor.beams = 180;
        sensor.fov = 1.5 * pi;
        break;
    case SensorType::Camera:
        sensor.name = "camera";
        sensor.rate = 15;
        sensor.fov = 60 * pi / 180;
        sensor.maxRange = 20;
        sensor.width = 160;
        sensor.height = 120;
        break;
    }
    return sensor;
}

int sensorJoint(const World& world, const Sensor& sensor) {
    if (!sensor.isJointSensor()) {
        return -1;
    }
    bool needsActuated = sensor.type != SensorType::ForceTorque;
    for (int i = 0; i < static_cast<int>(world.joints.size()); ++i) {
        const Joint& joint = world.joints[static_cast<std::size_t>(i)];
        if (joint.b == sensor.body && (!needsActuated || joint.isActuated())) {
            return i;
        }
    }
    return -1;
}

SensorPose sensorPose(const World& world, const Sensor& sensor) {
    if (!isBodyIndex(sensor.body, world.bodies.size())) {
        return {sensor.position, sensor.orientation};
    }
    const Body& body = world.bodies[static_cast<std::size_t>(sensor.body)];
    if (sensor.type == SensorType::ForceTorque) {
        // Measured at the joint anchor on the child.
        int index = sensorJoint(world, sensor);
        if (index >= 0) {
            const Joint& joint = world.joints[static_cast<std::size_t>(index)];
            return {body.point(joint.anchorB), body.orientation * sensor.orientation};
        }
    }
    return {body.point(sensor.position), body.orientation * sensor.orientation};
}

Vec3 lidarBeamDirection(const Sensor& sensor, int beam) {
    int beams = std::max(1, sensor.beams);
    bool fullCircle = sensor.fov >= 2 * pi - 1e-9;
    double step = beams == 1 ? 0 : sensor.fov / (fullCircle ? beams : beams - 1);
    double angle = beams == 1 ? 0 : -sensor.fov / 2 + step * beam;
    return {std::cos(angle), 0, -std::sin(angle)};
}

void updateSensors(World& world, double dt) {
    if (!std::isfinite(dt) || dt <= 0) {
        throw std::runtime_error("Sensor update needs a positive dt");
    }
    for (auto& joint : world.joints) {
        joint.controllerState.sensorFeedback = false;
    }
    for (auto& sensor : world.sensors) {
        updateSensor(world, sensor, dt);
    }
    // Encoders feed position (and a finite-difference speed); velocity sensors override the speed.
    for (const auto& sensor : world.sensors) {
        if (!sensor.feedsController || !sensor.state.valid || sensor.state.value.empty()
            || sensor.type != SensorType::JointEncoder) {
            continue;
        }
        int index = sensorJoint(world, sensor);
        if (index >= 0) {
            auto& state = world.joints[static_cast<std::size_t>(index)].controllerState;
            state.sensorFeedback = true;
            state.feedbackPosition = sensor.state.value[0];
            state.feedbackSpeed = sensor.state.speedEstimate;
        }
    }
    for (const auto& sensor : world.sensors) {
        if (!sensor.feedsController || !sensor.state.valid || sensor.state.value.empty()
            || sensor.type != SensorType::JointVelocity) {
            continue;
        }
        int index = sensorJoint(world, sensor);
        if (index >= 0) {
            Joint& joint = world.joints[static_cast<std::size_t>(index)];
            auto& state = joint.controllerState;
            if (!state.sensorFeedback) {
                state.sensorFeedback = true;
                state.feedbackPosition = joint.angle; // Ideal position when only the speed is sensed.
            }
            state.feedbackSpeed = sensor.state.value[0];
        }
    }
}

void stepSimulation(World& world, double dt) {
    world.step(dt);
    updateSensors(world, dt);
}

void resetSensors(World& world) {
    for (auto& sensor : world.sensors) {
        sensor.state = {};
    }
    for (auto& joint : world.joints) {
        joint.controllerState.sensorFeedback = false;
    }
}

void validateSensor(const Sensor& sensor, std::size_t bodyCount) {
    auto fail = [&](const char* what) {
        throw std::runtime_error("Invalid sensor " + sensor.name + ": " + what);
    };
    if (!isKnownSensorType(sensor.type)) {
        fail("unknown type");
    }
    if (sensor.name.empty()) {
        throw std::runtime_error("A sensor needs a name");
    }
    if (!isBodyIndex(sensor.body, bodyCount)) {
        fail("body index out of range");
    }
    const Quat& q = sensor.orientation;
    double norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (!sensor.position.finite() || !std::isfinite(norm) || std::abs(norm - 1) > 1e-5) {
        fail("mount pose must be finite with a unit quaternion");
    }
    if (!std::isfinite(sensor.rate) || sensor.rate < 0 || sensor.rate > maxSensorRate) {
        fail("rate must be in [0, 100000] Hz");
    }
    if (sensor.latency < 0 || sensor.latency > Sensor::maxLatency) {
        fail("latency must be 0..1000 samples");
    }
    if (!std::isfinite(sensor.dropout) || sensor.dropout < 0 || sensor.dropout >= 1) {
        fail("dropout probability must be in [0, 1)");
    }
    for (const NoiseModel* model : {&sensor.noise, &sensor.noise2}) {
        if (!std::isfinite(model->stddev) || model->stddev < 0 || !std::isfinite(model->bias)
            || !std::isfinite(model->biasWalk) || model->biasWalk < 0 || !std::isfinite(model->resolution)
            || model->resolution < 0) {
            fail("noise sigma, bias walk and resolution must be finite and non-negative");
        }
    }
    if (!std::isfinite(sensor.orientationNoise) || sensor.orientationNoise < 0
        || !std::isfinite(sensor.encoderCounts) || sensor.encoderCounts < 0) {
        fail("orientation noise and encoder counts must be non-negative");
    }
    if (!std::isfinite(sensor.minRange) || !std::isfinite(sensor.maxRange) || sensor.minRange < 0
        || sensor.maxRange <= sensor.minRange) {
        fail("range needs 0 <= min < max");
    }
    if (!std::isfinite(sensor.fov) || sensor.fov <= 0 || sensor.fov > 2 * pi + 1e-9) {
        fail("field of view must be in (0, 360] degrees");
    }
    if (sensor.type == SensorType::Camera && sensor.fov >= pi - 1e-6) {
        fail("camera field of view must be below 180 degrees");
    }
    if (sensor.beams < 1 || sensor.beams > Sensor::maxBeams) {
        fail("lidar beams must be 1..2048");
    }
    if (sensor.width < 1 || sensor.height < 1 || sensor.width > Sensor::maxImageSize
        || sensor.height > Sensor::maxImageSize) {
        fail("camera image must be 1..1024 pixels per side");
    }
    if (sensor.feedsController && sensor.type != SensorType::JointEncoder
        && sensor.type != SensorType::JointVelocity) {
        fail("only encoders and joint velocity sensors can feed a controller");
    }
}

void validateSensors(const World& world) {
    for (const auto& sensor : world.sensors) {
        validateSensor(sensor, world.bodies.size());
    }
}

std::vector<std::string> sensorChannelNames(const World& world, const Sensor& sensor) {
    (void)world;
    switch (sensor.type) {
    case SensorType::JointEncoder:
        return {"position"};
    case SensorType::JointVelocity:
        return {"velocity"};
    case SensorType::Imu:
        return {"accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y",
                "gyro_z",  "quat_w",  "quat_x",  "quat_y", "quat_z"};
    case SensorType::ForceTorque:
        return {"force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"};
    case SensorType::Contact:
        return {"in_contact", "normal_force"};
    case SensorType::Rangefinder:
        return {"range"};
    case SensorType::Lidar2D: {
        std::vector<std::string> names;
        for (int k = 0; k < sensor.beams; ++k) {
            names.push_back("range_" + std::to_string(k));
        }
        return names;
    }
    case SensorType::Camera:
        return {};
    }
    return {};
}

std::string sensorChannelUnit(const World& world, const Sensor& sensor, std::size_t channel) {
    bool slider = false;
    int joint = sensorJoint(world, sensor);
    if (joint >= 0) {
        slider = world.joints[static_cast<std::size_t>(joint)].type == JointType::Prismatic;
    }
    switch (sensor.type) {
    case SensorType::JointEncoder:
        return slider ? "m" : "rad";
    case SensorType::JointVelocity:
        return slider ? "m/s" : "rad/s";
    case SensorType::Imu:
        return channel < 3 ? "m/s^2" : (channel < 6 ? "rad/s" : "");
    case SensorType::ForceTorque:
        return channel < 3 ? "N" : "N*m";
    case SensorType::Contact:
        return channel == 0 ? "" : "N";
    case SensorType::Rangefinder:
    case SensorType::Lidar2D:
        return "m";
    case SensorType::Camera:
        return "";
    }
    return "";
}

void writeSensorCsvHeader(std::ostream& out) {
    out << "time,sensor,name,type,channel,value,truth\n";
}

void writeSensorCsvRows(std::ostream& out, const World& world) {
    auto precision = out.precision(12);
    for (std::size_t i = 0; i < world.sensors.size(); ++i) {
        const Sensor& sensor = world.sensors[i];
        if (!sensor.state.fresh || !sensor.state.valid) {
            continue;
        }
        std::vector<std::string> names = sensorChannelNames(world, sensor);
        for (std::size_t c = 0; c < sensor.state.value.size() && c < names.size(); ++c) {
            out << sensor.state.outputTime << ',' << i << ',';
            writeName(out, sensor.name);
            out << ',' << sensorTypeName(sensor.type) << ',' << names[c] << ',' << sensor.state.value[c]
                << ',' << (c < sensor.state.truth.size() ? sensor.state.truth[c] : 0.0) << '\n';
        }
    }
    out.precision(precision);
}

} // namespace robotics

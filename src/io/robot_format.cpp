#include "robotics/io/robot_format.hpp"

#include "robotics/sensors/sensor_system.hpp"

#include "atomic_file.hpp"
#include "text_reader.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <istream>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace robotics {
namespace {

constexpr std::size_t maxBodies = 10000;
constexpr std::size_t maxJoints = 30000;
constexpr std::size_t maxVisualsPerBody = 1000;
constexpr std::size_t maxSensors = 10000;

// First version with prismatic joints and the LIMITS record.
constexpr int limitsFormatVersion = 3;

// Joint data that only version 3+ can store.
bool needsLimitsFormat(const Joint& joint) {
    return joint.type == JointType::Prismatic || joint.limitsEnabled || joint.lowerLimit != 0
           || joint.upperLimit != 0;
}

// First version with visual meshes, hidden primitives and the ITERATIONS record.
constexpr int visualsFormatVersion = 4;

bool needsVisualsFormat(const World& world) {
    if (world.iterations != World{}.iterations) {
        return true;
    }
    for (const auto& body : world.bodies) {
        if (!body.visuals.empty() || body.primitiveHidden) {
            return true;
        }
    }
    return false;
}

// First version with the SENSORS block and the SOLVER record (World::substeps, World::warmStarting).
constexpr int sensorsFormatVersion = 5;

bool needsSensorsFormat(const World& world) {
    const World defaults;
    return !world.sensors.empty() || world.substeps != defaults.substeps
           || world.warmStarting != defaults.warmStarting;
}

// First version with convex hull bodies (HULL record) and continuous collision (Body::ccd in the CCD record,
// World::continuousCollision in the SOLVER record).
constexpr int hullsFormatVersion = 6;

bool needsHullsFormat(const World& world) {
    if (world.continuousCollision != World{}.continuousCollision) {
        return true;
    }
    return std::any_of(world.bodies.begin(), world.bodies.end(),
                       [](const Body& body) { return body.shape == Shape::ConvexHull || body.ccd; });
}

// Oldest version that stores the world losslessly. Worlds without prismatic joints, limits, visual meshes or
// sensors and with the default solver iteration count are written as version 2, byte-identical to the
// previous release and readable by it; version 3 adds limits; version 4 visual meshes and a non-default
// iteration count (so e.g. the rover demo, iterations = 32, needs version 4 and older builds cannot open it);
// version 5 sensors and non-default substeps / warm starting; version 6 convex hulls and continuous
// collision settings.
int formatVersionFor(const World& world) {
    if (needsHullsFormat(world)) {
        return hullsFormatVersion;
    }
    if (needsSensorsFormat(world)) {
        return sensorsFormatVersion;
    }
    if (needsVisualsFormat(world)) {
        return visualsFormatVersion;
    }
    for (const auto& joint : world.joints) {
        if (needsLimitsFormat(joint)) {
            return limitsFormatVersion;
        }
    }
    return 2;
}

// ---- Writing ---------------------------------------------------------------------------------

void writeVec(std::ostream& out, Vec3 v) {
    out << v.x << ' ' << v.y << ' ' << v.z << ' ';
}

void writeQuat(std::ostream& out, Quat q) {
    out << q.w << ' ' << q.x << ' ' << q.y << ' ' << q.z << ' ';
}

// Mesh paths inside `baseDirectory` (the .robot file's folder) are written relative to it, so a scene
// folder with its meshes can be moved as a whole. Other paths stay as they are.
std::string storedMeshPath(const std::string& path, const std::filesystem::path& baseDirectory) {
    namespace fs = std::filesystem;
    fs::path file(path);
    if (baseDirectory.empty() || !file.is_absolute()) {
        return path;
    }
    fs::path relative = file.lexically_normal().lexically_relative(baseDirectory.lexically_normal());
    if (relative.empty()) {
        return path;
    }
    for (const auto& part : relative) {
        if (part == "..") {
            return path;
        }
    }
    return relative.generic_string();
}

// A hull is stored as its body-frame vertices, not as a mesh reference: at most 64 points, valid without the
// mesh file, and immune to the mesh changing after the import. The source fields only keep the link to the
// mesh frame for the editor.
void writeHull(std::ostream& out, const ConvexHull& hull, const std::filesystem::path& baseDirectory) {
    out << "HULL " << std::quoted(hull.sourcePath.empty() ? hull.sourcePath
                                                          : storedMeshPath(hull.sourcePath, baseDirectory))
        << ' ';
    writeVec(out, hull.sourceScale);
    writeVec(out, hull.sourceOffset);
    writeQuat(out, hull.sourceRotation);
    out << hull.vertices.size() << '\n';
    for (Vec3 vertex : hull.vertices) {
        writeVec(out, vertex);
        out << '\n';
    }
}

void writeBody(std::ostream& out, const Body& body, int version, const std::filesystem::path& baseDirectory) {
    out << "BODY " << std::quoted(body.name) << ' ' << static_cast<int>(body.shape) << ' ' << body.mass
        << ' ';
    writeVec(out, body.size);
    writeVec(out, body.position);
    writeQuat(out, body.orientation);
    writeVec(out, body.velocity);
    writeVec(out, body.angularVelocity);
    out << body.friction << ' ' << body.restitution << '\n';
    if (version >= visualsFormatVersion) {
        out << "VISUALS " << body.primitiveHidden << ' ' << body.visuals.size() << '\n';
        for (const auto& visual : body.visuals) {
            out << "MESH " << std::quoted(storedMeshPath(visual.path, baseDirectory)) << ' ';
            writeVec(out, visual.scale);
            writeVec(out, visual.position);
            writeQuat(out, visual.orientation);
            out << '\n';
        }
    }
    if (version >= hullsFormatVersion) {
        out << "CCD " << body.ccd << '\n';
        if (body.shape == Shape::ConvexHull) {
            writeHull(out, *body.hull, baseDirectory);
        }
    }
}

void writeJoint(std::ostream& out, const Joint& joint, int version) {
    out << "JOINT " << static_cast<int>(joint.type) << ' ' << joint.a << ' ' << joint.b << ' ';
    writeVec(out, joint.anchorA);
    writeVec(out, joint.anchorB);
    writeVec(out, joint.axisA);
    writeVec(out, joint.axisB);
    writeQuat(out, joint.reference);
    out << joint.motorSpeed << ' ' << joint.maxTorque << '\n';

    const auto& c = joint.controller;
    out << "CONTROL " << static_cast<int>(c.type) << ' ' << static_cast<int>(c.variable) << ' '
        << static_cast<int>(c.signal) << ' ' << c.target << ' ' << c.amplitude << ' ' << c.frequency << ' '
        << c.startTime << ' ' << c.kp << ' ' << c.ki << ' ' << c.kd << ' ' << c.integralLimit << ' '
        << c.derivativeFilter << ' ' << c.deadband << '\n';
    out << "ENCODER " << joint.angle << ' ' << joint.lastWrappedAngle << ' ' << joint.encoderInitialized
        << '\n';
    if (version >= limitsFormatVersion) {
        out << "LIMITS " << joint.limitsEnabled << ' ' << joint.lowerLimit << ' ' << joint.upperLimit << '\n';
    }
}

// One line per sensor: identity, mount pose, timing, the two noise models and the type parameters.
void writeNoise(std::ostream& out, const NoiseModel& noise) {
    out << noise.stddev << ' ' << noise.bias << ' ' << noise.biasWalk << ' ' << noise.resolution << ' ';
}

void writeSensor(std::ostream& out, const Sensor& sensor) {
    out << "SENSOR " << std::quoted(sensor.name) << ' ' << static_cast<int>(sensor.type) << ' ' << sensor.body
        << ' ';
    writeVec(out, sensor.position);
    writeQuat(out, sensor.orientation);
    out << sensor.rate << ' ' << sensor.latency << ' ' << sensor.dropout << ' ' << sensor.seed << ' ';
    writeNoise(out, sensor.noise);
    writeNoise(out, sensor.noise2);
    out << sensor.orientationNoise << ' ' << sensor.encoderCounts << ' ' << sensor.feedsController << ' '
        << sensor.minRange << ' ' << sensor.maxRange << ' ' << sensor.fov << ' ' << sensor.beams << ' '
        << sensor.width << ' ' << sensor.height << '\n';
}

// ---- Parsing (text -> records), no World logic ------------------------------------------------

struct ParsedScene {
    int version{};
    Vec3 gravity{};
    int iterations{World{}.iterations};
    int substeps{World{}.substeps};
    bool warmStarting{World{}.warmStarting};
    bool continuousCollision{World{}.continuousCollision};
    std::vector<Body> bodies;
    std::vector<Joint> joints;
    std::vector<Sensor> sensors;
};

void readNoise(std::istream& in, NoiseModel& noise) {
    in >> noise.stddev >> noise.bias >> noise.biasWalk >> noise.resolution;
}

Sensor parseSensor(std::istream& in) {
    detail::expectToken(in, "SENSOR");
    Sensor sensor;
    int type{};
    in >> std::quoted(sensor.name) >> type >> sensor.body;
    sensor.type = static_cast<SensorType>(type);
    detail::readVec(in, sensor.position);
    detail::readQuat(in, sensor.orientation);
    in >> sensor.rate >> sensor.latency >> sensor.dropout >> sensor.seed;
    readNoise(in, sensor.noise);
    readNoise(in, sensor.noise2);
    in >> sensor.orientationNoise >> sensor.encoderCounts >> sensor.feedsController >> sensor.minRange
        >> sensor.maxRange >> sensor.fov >> sensor.beams >> sensor.width >> sensor.height;
    if (!in) {
        throw std::runtime_error("Malformed sensor record");
    }
    return sensor;
}

VisualMesh parseVisual(std::istream& in) {
    detail::expectToken(in, "MESH");
    VisualMesh visual;
    in >> std::quoted(visual.path);
    detail::readVec(in, visual.scale);
    detail::readVec(in, visual.position);
    detail::readQuat(in, visual.orientation);
    if (!in || visual.path.empty() || !visual.scale.finite() || !visual.position.finite()) {
        throw std::runtime_error("Malformed visual mesh record");
    }
    if (std::abs(visual.scale.x) < 1e-12 || std::abs(visual.scale.y) < 1e-12
        || std::abs(visual.scale.z) < 1e-12) {
        throw std::runtime_error("Visual mesh scale must not be zero");
    }
    const Quat& q = visual.orientation;
    double norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-5) {
        throw std::runtime_error("Visual mesh orientation must be a unit quaternion");
    }
    return visual;
}

// Rebuilds the topology and mass properties; a list that is no convex body in a principal frame is rejected.
std::shared_ptr<const ConvexHull> parseHull(std::istream& in) {
    detail::expectToken(in, "HULL");
    std::string path;
    Vec3 scale;
    Vec3 offset;
    Quat rotation;
    in >> std::quoted(path);
    detail::readVec(in, scale);
    detail::readVec(in, offset);
    detail::readQuat(in, rotation);
    double norm = rotation.w * rotation.w + rotation.x * rotation.x + rotation.y * rotation.y
                  + rotation.z * rotation.z;
    if (!in || !scale.finite() || !offset.finite() || !(std::abs(norm - 1) <= 1e-5)) {
        throw std::runtime_error("Malformed hull record");
    }
    std::size_t count = detail::readCount(in, static_cast<std::size_t>(maxConvexHullVertices));
    std::vector<Vec3> vertices(count);
    for (auto& vertex : vertices) {
        detail::readVec(in, vertex);
    }
    if (!in) {
        throw std::runtime_error("Malformed hull vertices");
    }
    HullBuildOptions options;
    options.preserveFrame = true;
    auto hull = std::make_shared<ConvexHull>(*buildConvexHull(vertices, options));
    if (hull->vertices.size() != count) {
        throw std::runtime_error("Hull vertices are not in convex position");
    }
    hull->sourcePath = path;
    hull->sourceScale = scale;
    hull->sourceOffset = offset;
    hull->sourceRotation = rotation;
    return hull;
}

Body parseBody(std::istream& in, int version) {
    detail::expectToken(in, "BODY");
    Body body;
    int shape{};
    in >> std::quoted(body.name) >> shape >> body.mass;
    body.shape = static_cast<Shape>(shape);
    detail::readVec(in, body.size);
    detail::readVec(in, body.position);
    detail::readQuat(in, body.orientation);
    detail::readVec(in, body.velocity);
    detail::readVec(in, body.angularVelocity);
    in >> body.friction >> body.restitution;
    if (!in) {
        throw std::runtime_error("Malformed body record");
    }
    if (version >= visualsFormatVersion) {
        detail::expectToken(in, "VISUALS");
        in >> body.primitiveHidden;
        if (!in) {
            throw std::runtime_error("Malformed visuals record");
        }
        std::size_t count = detail::readCount(in, maxVisualsPerBody);
        for (std::size_t i = 0; i < count; ++i) {
            body.visuals.push_back(parseVisual(in));
        }
    }
    if (version >= hullsFormatVersion) {
        detail::expectToken(in, "CCD");
        in >> body.ccd;
        if (!in) {
            throw std::runtime_error("Malformed CCD record");
        }
        if (body.shape == Shape::ConvexHull) {
            setConvexHull(body, parseHull(in));
        }
    } else if (body.shape == Shape::ConvexHull) {
        throw std::runtime_error("Convex hull bodies require scene format version 6");
    }
    return body;
}

void parseControllerRecord(std::istream& in, Joint& joint) {
    detail::expectToken(in, "CONTROL");
    auto& c = joint.controller;
    int controller{};
    int variable{};
    int signal{};
    in >> controller >> variable >> signal >> c.target >> c.amplitude >> c.frequency >> c.startTime >> c.kp
        >> c.ki >> c.kd >> c.integralLimit >> c.derivativeFilter >> c.deadband;
    c.type = static_cast<ControllerType>(controller);
    c.variable = static_cast<ControlVariable>(variable);
    c.signal = static_cast<ReferenceSignal>(signal);
    if (!in) {
        throw std::runtime_error("Malformed controller record");
    }
    detail::expectToken(in, "ENCODER");
    in >> joint.angle >> joint.lastWrappedAngle >> joint.encoderInitialized;
    if (!in) {
        throw std::runtime_error("Malformed encoder record");
    }
}

Joint parseJoint(std::istream& in, int version) {
    detail::expectToken(in, "JOINT");
    Joint joint;
    int type{};
    in >> type >> joint.a >> joint.b;
    joint.type = static_cast<JointType>(type);
    detail::readVec(in, joint.anchorA);
    detail::readVec(in, joint.anchorB);
    detail::readVec(in, joint.axisA);
    detail::readVec(in, joint.axisB);
    detail::readQuat(in, joint.reference);
    in >> joint.motorSpeed >> joint.maxTorque;
    if (!in) {
        throw std::runtime_error("Malformed joint record");
    }
    if (version < limitsFormatVersion && joint.type == JointType::Prismatic) {
        throw std::runtime_error("Prismatic joints require scene format version 3");
    }
    if (version >= 2) {
        parseControllerRecord(in, joint);
    }
    if (version >= limitsFormatVersion) {
        detail::expectToken(in, "LIMITS");
        in >> joint.limitsEnabled >> joint.lowerLimit >> joint.upperLimit;
        if (!in) {
            throw std::runtime_error("Malformed limits record");
        }
    }
    return joint;
}

ParsedScene parseScene(std::istream& in) {
    ParsedScene scene;
    detail::expectToken(in, "ROBOTICS_SCENE");
    if (!(in >> scene.version) || scene.version < robotFormatOldestVersion
        || scene.version > robotFormatVersion) {
        throw std::runtime_error("Unsupported scene version");
    }
    detail::expectToken(in, "GRAVITY");
    detail::readVec(in, scene.gravity);
    if (scene.version >= visualsFormatVersion) {
        detail::expectToken(in, "ITERATIONS");
        if (!(in >> scene.iterations)) {
            throw std::runtime_error("Malformed iterations record");
        }
        if (scene.iterations < World::minIterations || scene.iterations > World::maxIterations) {
            throw std::runtime_error("Solver iterations must be in [" + std::to_string(World::minIterations)
                                     + ", " + std::to_string(World::maxIterations) + "], got "
                                     + std::to_string(scene.iterations));
        }
    }
    if (scene.version >= sensorsFormatVersion) {
        detail::expectToken(in, "SOLVER");
        int warm = 1;
        if (!(in >> scene.substeps >> warm)) {
            throw std::runtime_error("Malformed solver record");
        }
        if (scene.substeps < World::minSubsteps || scene.substeps > World::maxSubsteps) {
            throw std::runtime_error("Solver substeps must be in [" + std::to_string(World::minSubsteps)
                                     + ", " + std::to_string(World::maxSubsteps) + "], got "
                                     + std::to_string(scene.substeps));
        }
        if (warm != 0 && warm != 1) {
            throw std::runtime_error("Solver warm-starting flag must be 0 or 1");
        }
        scene.warmStarting = warm == 1;
    }
    if (scene.version >= hullsFormatVersion) {
        detail::expectToken(in, "CONTINUOUS");
        int continuous = 1;
        if (!(in >> continuous) || (continuous != 0 && continuous != 1)) {
            throw std::runtime_error("Malformed continuous collision record");
        }
        scene.continuousCollision = continuous == 1;
    }

    detail::expectToken(in, "BODIES");
    std::size_t count = detail::readCount(in, maxBodies);
    for (std::size_t i = 0; i < count; ++i) {
        scene.bodies.push_back(parseBody(in, scene.version));
    }

    detail::expectToken(in, "JOINTS");
    count = detail::readCount(in, maxJoints);
    for (std::size_t i = 0; i < count; ++i) {
        scene.joints.push_back(parseJoint(in, scene.version));
    }

    if (scene.version >= sensorsFormatVersion) {
        detail::expectToken(in, "SENSORS");
        count = detail::readCount(in, maxSensors);
        for (std::size_t i = 0; i < count; ++i) {
            scene.sensors.push_back(parseSensor(in));
        }
    }

    detail::expectToken(in, "END");
    std::string extra;
    if (in >> extra) {
        throw std::runtime_error("Unexpected data after END");
    }
    return scene;
}

// ---- World construction -----------------------------------------------------------------------

// Records are restored verbatim (local frames, encoder state), so no addBody()/connect() here:
// those would re-derive frames from world poses.
World buildWorld(ParsedScene scene) {
    World world;
    world.gravity = scene.gravity;
    world.iterations = scene.iterations;
    world.substeps = scene.substeps;
    world.warmStarting = scene.warmStarting;
    world.continuousCollision = scene.continuousCollision;
    world.bodies = std::move(scene.bodies);
    world.joints = std::move(scene.joints);
    world.sensors = std::move(scene.sensors);
    world.validate();
    validateSensors(world);
    return world;
}

void writeSceneTo(const World& world, std::ostream& out, const std::filesystem::path& baseDirectory) {
    world.validate();
    validateSensors(world);
    const int version = formatVersionFor(world);
    out << std::setprecision(17) << "ROBOTICS_SCENE " << version << "\nGRAVITY ";
    writeVec(out, world.gravity);
    if (version >= visualsFormatVersion) {
        out << "\nITERATIONS " << world.iterations;
    }
    if (version >= sensorsFormatVersion) {
        out << "\nSOLVER " << world.substeps << ' ' << world.warmStarting;
    }
    if (version >= hullsFormatVersion) {
        out << "\nCONTINUOUS " << world.continuousCollision;
    }
    out << "\nBODIES " << world.bodies.size() << '\n';
    for (const auto& body : world.bodies) {
        writeBody(out, body, version, baseDirectory);
    }
    out << "JOINTS " << world.joints.size() << '\n';
    for (const auto& joint : world.joints) {
        writeJoint(out, joint, version);
    }
    if (version >= sensorsFormatVersion) {
        out << "SENSORS " << world.sensors.size() << '\n';
        for (const auto& sensor : world.sensors) {
            writeSensor(out, sensor);
        }
    }
    out << "END\n";
    if (!out) {
        throw std::runtime_error("Failed to write scene");
    }
}

std::filesystem::path sceneDirectory(const std::string& scenePath) {
    std::filesystem::path directory = std::filesystem::path(scenePath).parent_path();
    return std::filesystem::absolute(directory.empty() ? std::filesystem::path(".") : directory);
}

} // namespace

void writeScene(const World& world, std::ostream& out) {
    writeSceneTo(world, out, {});
}

World readScene(std::istream& in) {
    return buildWorld(parseScene(in));
}

void saveScene(const World& world, const std::string& path) {
    world.validate();
    const std::filesystem::path base = sceneDirectory(path);
    detail::writeFileAtomically(path, [&](std::ostream& out) { writeSceneTo(world, out, base); });
}

World loadScene(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Cannot open scene: " + path);
    }
    World world = readScene(in);
    // Relative mesh paths are relative to the scene file.
    const std::filesystem::path base = sceneDirectory(path);
    for (auto& body : world.bodies) {
        for (auto& visual : body.visuals) {
            if (std::filesystem::path(visual.path).is_relative()) {
                visual.path = (base / visual.path).lexically_normal().string();
            }
        }
        if (body.hull && !body.hull->sourcePath.empty()
            && std::filesystem::path(body.hull->sourcePath).is_relative()) {
            auto hull = std::make_shared<ConvexHull>(*body.hull);
            hull->sourcePath = (base / hull->sourcePath).lexically_normal().string();
            body.hull = std::move(hull);
        }
    }
    return world;
}

} // namespace robotics

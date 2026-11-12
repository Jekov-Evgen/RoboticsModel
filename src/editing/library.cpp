#include "robotics/editing/library.hpp"

#include "robotics/editing/assembly.hpp"
#include "robotics/sensors/sensor_system.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace robotics {
namespace {

constexpr double degrees = pi / 180;
// Cylinder local Z along assembly +Y.
const Quat cylinderUp = Quat::axisAngle({1, 0, 0}, -pi / 2);

Body box(const std::string& name, Vec3 size, Vec3 position, double mass) {
    Body body;
    body.name = name;
    body.size = size;
    body.position = position;
    body.mass = mass;
    return body;
}

Body cylinder(const std::string& name, double diameter, double width, Vec3 position, double mass) {
    Body body = box(name, {diameter, diameter, width}, position, mass);
    body.shape = Shape::Cylinder;
    body.orientation = cylinderUp;
    return body;
}

Body ball(const std::string& name, double diameter, Vec3 position, double mass) {
    Body body = box(name, {diameter, diameter, diameter}, position, mass);
    body.shape = Shape::Sphere;
    return body;
}

void positionServo(Joint& joint, double maxEffort, double kp, double ki, double kd) {
    joint.maxTorque = maxEffort;
    joint.controller.type = ControllerType::PID;
    joint.controller.variable = ControlVariable::Position;
    joint.controller.signal = ReferenceSignal::Constant;
    joint.controller.kp = kp;
    joint.controller.ki = ki;
    joint.controller.kd = kd;
    joint.controller.integralLimit = maxEffort / 2;
    joint.controller.deadband = 0.005;
}

const std::vector<LibraryMount> topMounts{{{0, 1, 0}, {0, 1, 0}},
                                          {{0.6, 1, 0}, {0, 1, 0}},
                                          {{-0.6, 1, 0}, {0, 1, 0}},
                                          {{1, 0, 0}, {1, 0, 0}},
                                          {{-1, 0, 0}, {-1, 0, 0}}};
const std::vector<LibraryMount> frontMounts{{{1, 0, 0}, {1, 0, 0}},
                                            {{0, 1, 0}, {0, 1, 0}},
                                            {{-1, 0, 0}, {-1, 0, 0}},
                                            {{0, 0, 1}, {0, 0, 1}},
                                            {{0, 0, -1}, {0, 0, -1}}};

LibraryItem drivenWheel() {
    LibraryItem item;
    item.id = "driven_wheel";
    item.title = "Driven wheel module";
    item.summary = "Wheel on a motorized hinge, drivable with WASD. Attaches at a free corner of the part's "
                   "side, with a horizontal axis.";
    item.category = "Drive";
    World& w = item.assembly;
    w.addBody(box("wheel_hub", {0.22, 0.1, 0.22}, {0, 0.05, 0}, 0.3));
    Body wheel = cylinder("wheel", 0.6, 0.16, {0, 0.18, 0}, 0.6);
    wheel.friction = 0.9;
    wheel.restitution = 0;
    int index = w.addBody(wheel);
    w.joints[static_cast<std::size_t>(w.connect(0, index, JointType::Hinge, wheel.position, {0, 1, 0}))]
        .maxTorque = 8;
    item.replaceRootWithHost = true;
    item.mounts = {{{0.7, -0.5, 1}, {0, 0, 1}},  {{0.7, -0.5, -1}, {0, 0, -1}},
                   {{-0.7, -0.5, 1}, {0, 0, 1}}, {{-0.7, -0.5, -1}, {0, 0, -1}},
                   {{0, -0.5, 1}, {0, 0, 1}},    {{0, -0.5, -1}, {0, 0, -1}}};
    return item;
}

LibraryItem servoJoint() {
    LibraryItem item;
    item.id = "servo_joint";
    item.title = "Servo joint";
    item.summary = "Hinge with a position PID and ±90° limits, with a link to build on. Set the target "
                   "angle in Control Lab.";
    item.category = "Actuators";
    World& w = item.assembly;
    w.addBody(box("servo_base", {0.18, 0.14, 0.18}, {0, 0.07, 0}, 0.4));
    int link = w.addBody(box("servo_link", {0.08, 0.44, 0.08}, {0, 0.34, 0}, 0.3));
    Joint& hinge =
        w.joints[static_cast<std::size_t>(w.connect(0, link, JointType::Hinge, {0, 0.12, 0}, {0, 0, 1}))];
    hinge.setLimits(-90 * degrees, 90 * degrees);
    positionServo(hinge, 8, 14, 2, 2);
    item.mounts = topMounts;
    return item;
}

LibraryItem linearActuator() {
    LibraryItem item;
    item.id = "linear_actuator";
    item.title = "Linear actuator";
    item.summary = "Slider with a position PID and a 0–0.3 m stroke. Set the target position in Control Lab.";
    item.category = "Actuators";
    World& w = item.assembly;
    w.addBody(box("actuator_body", {0.14, 0.42, 0.14}, {0, 0.21, 0}, 0.5));
    int rod = w.addBody(box("actuator_rod", {0.07, 0.42, 0.07}, {0, 0.5, 0}, 0.2));
    Joint& slider =
        w.joints[static_cast<std::size_t>(w.connect(0, rod, JointType::Prismatic, {0, 0.5, 0}, {0, 1, 0}))];
    slider.setLimits(0, 0.3);
    positionServo(slider, 30, 60, 10, 12);
    item.mounts = topMounts;
    return item;
}

LibraryItem gripper() {
    LibraryItem item;
    item.id = "gripper";
    item.title = "Two-finger gripper";
    item.summary = "Palm with two opposing sliders. Each finger closes 0–6 cm; set their target in Control "
                   "Lab.";
    item.category = "Actuators";
    World& w = item.assembly;
    w.addBody(box("gripper_palm", {0.34, 0.08, 0.14}, {0, 0.04, 0}, 0.4));
    for (double side : {-1.0, 1.0}) {
        int finger = w.addBody(
            box(side < 0 ? "finger_left" : "finger_right", {0.04, 0.2, 0.1}, {side * 0.14, 0.18, 0}, 0.1));
        Vec3 anchor{side * 0.14, 0.1, 0};
        // Positive coordinate closes the gripper.
        Joint& slider = w.joints[static_cast<std::size_t>(
            w.connect(0, finger, JointType::Prismatic, anchor, {-side, 0, 0}))];
        slider.setLimits(0, 0.06);
        positionServo(slider, 15, 40, 4, 4);
    }
    item.mounts = frontMounts;
    return item;
}

LibraryItem caster() {
    LibraryItem item;
    item.id = "caster_ball";
    item.title = "Caster ball";
    item.summary = "Free-rolling ball on a ball joint. Attaches under the part to support it.";
    item.category = "Drive";
    World& w = item.assembly;
    w.addBody(box("caster_mount", {0.16, 0.06, 0.16}, {0, 0.03, 0}, 0.1));
    Body sphere = ball("caster_ball", 0.2, {0, 0.14, 0}, 0.2);
    sphere.friction = 0.2;
    sphere.restitution = 0;
    int index = w.addBody(sphere);
    w.connect(0, index, JointType::Ball, sphere.position);
    item.replaceRootWithHost = true;
    item.mounts = {{{-0.7, -1, 0}, {0, -1, 0}}, {{0.7, -1, 0}, {0, -1, 0}}, {{0, -1, 0}, {0, -1, 0}}};
    return item;
}

LibraryItem chassisPlate() {
    LibraryItem item;
    item.id = "chassis_plate";
    item.title = "Chassis plate";
    item.summary = "Flat base to build a robot on. Add wheel modules, a caster and sensors to it.";
    item.category = "Structure";
    item.assembly.addBody(box("chassis_plate", {1.2, 0.1, 0.7}, {0, 0.05, 0}, 4));
    item.mounts = topMounts;
    return item;
}

// Foot plate and pole shared by the masts; returns the pole's index.
int mast(World& w, const std::string& name) {
    w.addBody(box(name + "_foot", {0.24, 0.04, 0.24}, {0, 0.02, 0}, 0.3));
    int pole = w.addBody(box(name + "_pole", {0.05, 0.4, 0.05}, {0, 0.24, 0}, 0.15));
    w.connect(0, pole, JointType::Fixed, {0, 0.04, 0});
    return pole;
}

LibraryItem lidarMast() {
    LibraryItem item;
    item.id = "lidar_mast";
    item.title = "Lidar mast";
    item.summary = "Pole with a 2D lidar on top. Readings appear in the Sensors window.";
    item.category = "Sensors";
    World& w = item.assembly;
    int pole = mast(w, "lidar_mast");
    int head = w.addBody(cylinder("lidar_head", 0.16, 0.07, {0, 0.475, 0}, 0.15));
    w.connect(pole, head, JointType::Fixed, {0, 0.44, 0});
    Sensor lidar = makeSensor(SensorType::Lidar2D, head);
    lidar.name = "lidar";
    lidar.orientation = Quat::axisAngle({1, 0, 0}, pi / 2); // Undo the cylinder's tilt: scan horizontally.
    w.sensors.push_back(lidar);
    item.mounts = topMounts;
    return item;
}

LibraryItem cameraMast() {
    LibraryItem item;
    item.id = "camera_mast";
    item.title = "Camera mast";
    item.summary = "Pole with a forward-looking camera. Its picture shows in the corner of the 3D view.";
    item.category = "Sensors";
    World& w = item.assembly;
    int pole = mast(w, "camera_mast");
    int head = w.addBody(box("camera_head", {0.16, 0.1, 0.12}, {0, 0.49, 0}, 0.12));
    w.connect(pole, head, JointType::Fixed, {0, 0.44, 0});
    Sensor camera = makeSensor(SensorType::Camera, head);
    camera.name = "camera";
    camera.position = {0.09, 0, 0};
    w.sensors.push_back(camera);
    item.mounts = topMounts;
    return item;
}

LibraryItem imuBoard() {
    LibraryItem item;
    item.id = "imu_board";
    item.title = "IMU board";
    item.summary = "Small board with an accelerometer, gyroscope and orientation estimate.";
    item.category = "Sensors";
    World& w = item.assembly;
    int board = w.addBody(box("imu_board", {0.18, 0.025, 0.12}, {0, 0.0125, 0}, 0.1));
    Sensor imu = makeSensor(SensorType::Imu, board);
    imu.name = "imu";
    w.sensors.push_back(imu);
    item.mounts = topMounts;
    return item;
}

std::string withoutNumber(const std::string& name) {
    std::size_t end = name.size();
    while (end > 0 && std::isdigit(static_cast<unsigned char>(name[end - 1]))) {
        --end;
    }
    return end > 1 && end < name.size() && name[end - 1] == '_' ? name.substr(0, end - 1) : name;
}

template <class Taken>
std::string uniqueName(const std::string& name, Taken taken) {
    if (!taken(name)) {
        return name;
    }
    std::string base = withoutNumber(name);
    for (int n = 1;; ++n) {
        std::string candidate = base + "_" + std::to_string(n);
        if (!taken(candidate)) {
            return candidate;
        }
    }
}

// Rotation taking +Y to `normal`. Keeps local X for any normal in the Y-Z plane, so wheels keep rolling
// along the host's X axis.
Quat rotationFromUp(Vec3 normal) {
    Vec3 up{0, 1, 0};
    Vec3 n = normal.normalized();
    double cosine = up.dot(n);
    if (cosine < -0.999999) {
        return Quat::axisAngle({1, 0, 0}, pi);
    }
    return Quat::axisAngle(up.cross(n), std::acos(std::clamp(cosine, -1.0, 1.0)));
}

// Mount position on the host in its local frame.
Vec3 mountPoint(const Body& host, const LibraryMount& mount) {
    Vec3 half = host.size * 0.5;
    if (host.shape == Shape::Sphere) {
        return mount.surface.normalized() * half.x;
    }
    return componentProduct(mount.surface, half);
}

// First mount without a joint attached nearby; the first mount if all are taken.
const LibraryMount& freeMount(const World& world, int host, const std::vector<LibraryMount>& mounts) {
    const Body& body = world.bodies[static_cast<std::size_t>(host)];
    double clearance = std::max(0.05, 0.3 * std::min({body.size.x, body.size.y, body.size.z}));
    auto occupies = [&](Vec3 anchor, const LibraryMount& mount) {
        // A joint on this side of the part, close to the mount when seen along its normal.
        Vec3 normal = mount.normal.normalized();
        Vec3 offset = anchor - mountPoint(body, mount);
        double outward = offset.dot(normal);
        return outward > -0.01 && (offset - normal * outward).length() < clearance;
    };
    for (const LibraryMount& mount : mounts) {
        bool taken = false;
        for (const Joint& joint : world.joints) {
            taken = taken || (joint.a == host && occupies(joint.anchorA, mount))
                    || (joint.b == host && occupies(joint.anchorB, mount));
        }
        if (!taken) {
            return mount;
        }
    }
    return mounts.front();
}

} // namespace

const std::vector<LibraryItem>& builtinLibrary() {
    static const std::vector<LibraryItem> items{drivenWheel(), servoJoint(), linearActuator(),
                                                gripper(),     caster(),     chassisPlate(),
                                                lidarMast(),   cameraMast(), imuBoard()};
    return items;
}

std::string uniqueBodyName(const World& world, const std::string& name) {
    return uniqueName(name, [&](const std::string& candidate) {
        return std::any_of(world.bodies.begin(), world.bodies.end(),
                           [&](const Body& body) { return body.name == candidate; });
    });
}

std::string uniqueSensorName(const World& world, const std::string& name) {
    return uniqueName(name, [&](const std::string& candidate) {
        return std::any_of(world.sensors.begin(), world.sensors.end(),
                           [&](const Sensor& sensor) { return sensor.name == candidate; });
    });
}

LibraryInsertion insertLibraryItem(World& world, const LibraryItem& item, int host, Vec3 point) {
    if (item.assembly.bodies.empty()) {
        throw std::runtime_error("Library item has no parts");
    }
    if (host >= static_cast<int>(world.bodies.size())) {
        throw std::runtime_error("Invalid part to attach to");
    }
    if (host >= 0 && item.mounts.empty()) {
        throw std::runtime_error("This library item cannot be attached to a part");
    }
    const bool attached = host >= 0;
    const bool skipRoot = attached && item.replaceRootWithHost;
    World result = world; // Strong guarantee: `world` only changes when everything succeeded.
    Vec3 origin = point;
    Quat rotation;
    if (attached) {
        const Body& body = result.bodies[static_cast<std::size_t>(host)];
        const LibraryMount& mount = freeMount(result, host, item.mounts);
        origin = body.point(mountPoint(body, mount));
        rotation = (body.orientation * rotationFromUp(mount.normal)).normalized();
    }
    auto toWorld = [&](Vec3 local) {
        return origin + rotation.rotate(local);
    };

    // Assembly body index -> world body index.
    std::vector<int> map(item.assembly.bodies.size(), -1);
    for (std::size_t i = skipRoot ? 1 : 0; i < item.assembly.bodies.size(); ++i) {
        Body body = item.assembly.bodies[i];
        body.name = uniqueBodyName(result, body.name);
        body.position = toWorld(body.position);
        body.orientation = (rotation * body.orientation).normalized();
        body.velocity = {};
        body.angularVelocity = {};
        map[i] = result.addBody(std::move(body));
    }
    if (skipRoot) {
        map[0] = host;
    }
    LibraryInsertion insertion;
    insertion.root = map[skipRoot ? 1 : 0];
    insertion.bodies = static_cast<int>(item.assembly.bodies.size()) - (skipRoot ? 1 : 0);
    for (const Joint& source : item.assembly.joints) {
        auto a = static_cast<std::size_t>(source.a);
        auto b = static_cast<std::size_t>(source.b);
        Joint joint = source;
        joint.history.clear();
        joint.controllerState = {};
        if (skipRoot && (source.a == 0 || source.b == 0)) {
            // Rebuild against the host, then keep the template's motor, controller and limits.
            const Body& templateA = item.assembly.bodies[a];
            Vec3 anchor = toWorld(templateA.point(source.anchorA));
            Vec3 axis = rotation.rotate(templateA.orientation.rotate(source.axisA));
            Joint rebuilt = result.joints[static_cast<std::size_t>(
                result.connect(map[a], map[b], source.type, anchor, axis))];
            joint.anchorA = rebuilt.anchorA;
            joint.anchorB = rebuilt.anchorB;
            joint.axisA = rebuilt.axisA;
            joint.axisB = rebuilt.axisB;
            joint.reference = rebuilt.reference;
            result.joints.pop_back();
        }
        joint.a = map[a];
        joint.b = map[b];
        result.joints.push_back(std::move(joint));
        if (skipRoot && (source.a == 0 || source.b == 0)) {
            insertion.attachJoint = static_cast<int>(result.joints.size()) - 1;
        }
    }
    for (const Sensor& source : item.assembly.sensors) {
        int body = map[static_cast<std::size_t>(source.body)];
        if (skipRoot && source.body == 0) {
            continue;
        }
        Sensor sensor = source;
        sensor.body = body;
        sensor.state = {};
        sensor.name = uniqueSensorName(result, sensor.name);
        result.sensors.push_back(std::move(sensor));
    }
    if (attached && !skipRoot) {
        insertion.attachJoint = result.connect(host, insertion.root, item.attachType,
                                               toWorld(item.attachAnchor), rotation.rotate(item.attachAxis));
    }
    if (attached) {
        // E.g. wheels on a plate lying on the floor: lift the whole robot so nothing starts in the ground.
        int top = host;
        for (std::size_t guard = 0; guard < result.joints.size(); ++guard) {
            auto parent = std::find_if(result.joints.begin(), result.joints.end(),
                                       [&](const Joint& joint) { return joint.b == top; });
            if (parent == result.joints.end()) {
                break;
            }
            top = parent->a;
        }
        double lowest = assemblyLowestPoint(result, top);
        if (lowest < 0 && !result.bodies[static_cast<std::size_t>(top)].isStatic()) {
            moveAssembly(result, top, {0, -lowest, 0});
        }
    } else {
        double lowest = 1e300;
        for (int i = insertion.root; i < static_cast<int>(result.bodies.size()); ++i) {
            lowest = std::min(lowest, lowestPoint(result.bodies[static_cast<std::size_t>(i)]));
        }
        for (int i = insertion.root; i < static_cast<int>(result.bodies.size()); ++i) {
            result.bodies[static_cast<std::size_t>(i)].position.y -= lowest;
        }
    }
    result.validate();
    validateSensors(result);
    world = std::move(result);
    return insertion;
}

World extractAssembly(const World& world, int root) {
    std::vector<char> members = assemblyMembers(world, root);
    std::vector<int> map(world.bodies.size(), -1);
    World result;
    result.gravity = world.gravity;
    result.iterations = world.iterations;
    result.substeps = world.substeps;
    result.warmStarting = world.warmStarting;
    auto add = [&](std::size_t index) {
        Body body = world.bodies[index];
        body.velocity = {};
        body.angularVelocity = {};
        body.force = {};
        body.torque = {};
        map[index] = result.addBody(std::move(body));
    };
    add(static_cast<std::size_t>(root));
    for (std::size_t i = 0; i < world.bodies.size(); ++i) {
        if (members[i] && map[i] < 0) {
            add(i);
        }
    }
    for (const Joint& source : world.joints) {
        int a = map[static_cast<std::size_t>(source.a)];
        int b = map[static_cast<std::size_t>(source.b)];
        if (a < 0 || b < 0) {
            continue;
        }
        Joint joint = source;
        joint.a = a;
        joint.b = b;
        joint.history.clear();
        joint.controllerState = {};
        result.joints.push_back(std::move(joint));
    }
    for (const Sensor& source : world.sensors) {
        int body = map[static_cast<std::size_t>(source.body)];
        if (body >= 0) {
            Sensor sensor = source;
            sensor.body = body;
            sensor.state = {};
            result.sensors.push_back(std::move(sensor));
        }
    }
    return result;
}

LibraryItem customLibraryItem(const std::string& title, World assembly) {
    if (assembly.bodies.empty()) {
        throw std::runtime_error("Library item has no parts");
    }
    LibraryItem item;
    item.id = "custom:" + title;
    item.title = title;
    item.summary = std::to_string(assembly.bodies.size()) + (assembly.bodies.size() == 1 ? " part" : " parts")
                   + " · saved by you. Attaches with a fixed joint.";
    item.category = "My parts";
    item.custom = true;
    double lowest = assemblyLowestPoint(assembly, 0);
    Vec3 shift{-assembly.bodies[0].position.x, -lowest, -assembly.bodies[0].position.z};
    for (Body& body : assembly.bodies) {
        body.position += shift;
    }
    item.attachAnchor = {0, 0, 0};
    item.assembly = std::move(assembly);
    item.mounts = topMounts;
    return item;
}

} // namespace robotics

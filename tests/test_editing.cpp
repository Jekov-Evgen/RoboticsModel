// Assembly edits, gizmo math and the part library (all headless).

#include "robotics/editing/assembly.hpp"
#include "robotics/editing/gizmo.hpp"
#include "robotics/editing/library.hpp"
#include "robotics/sensors/sensor_system.hpp"
#include "test_harness.hpp"

#include <set>
#include <sstream>

using namespace robotics;
using test::check;
using test::near;

namespace {

GizmoView lookingAt(Vec3 eye, Vec3 target) {
    GizmoView view;
    view.eye = eye;
    view.forward = (target - eye).normalized();
    view.right = view.forward.cross({0, 1, 0}).normalized();
    view.up = view.right.cross(view.forward);
    view.focal = 2.2;
    view.width = 800;
    view.height = 600;
    return view;
}

// Base (static) -> arm (hinge) -> forearm (slider) with a sibling free body: exercises crossing joints.
World chain() {
    World world;
    world.gravity = {};
    Body base;
    base.name = "base";
    base.mass = 0;
    base.position = {0, 1, 0};
    world.addBody(base);
    Body arm;
    arm.name = "arm";
    arm.position = {1, 1, 0};
    int armIndex = world.addBody(arm);
    world.connect(0, armIndex, JointType::Hinge, {0.5, 1, 0}, {0, 0, 1});
    Body forearm;
    forearm.name = "forearm";
    forearm.position = {2, 1, 0};
    int foreIndex = world.addBody(forearm);
    world.connect(armIndex, foreIndex, JointType::Prismatic, {1.5, 1, 0}, {1, 0, 0});
    return world;
}

} // namespace

TEST_CASE("rotateAssembly keeps every joint type valid") {
    for (JointType type : {JointType::Fixed, JointType::Ball, JointType::Hinge, JointType::Prismatic}) {
        World world = chain();
        world.joints[0].type = type;
        world.step();
        rotateAssembly(world, 1, world.bodies[1].position, {0.3, 1, -0.2}, 1.1);
        near(world.maxAnchorError(), 0, 1e-12, "Anchors separated by the rotation");
        const Joint& j = world.joints[0];
        Vec3 axisA = world.bodies[0].orientation.rotate(j.axisA);
        Vec3 axisB = world.bodies[1].orientation.rotate(j.axisB);
        near((axisA - axisB).length(), 0, 1e-12, "Axes disagree after rotation");
        Quat relative = world.bodies[0].orientation.conjugate() * world.bodies[1].orientation;
        near(rotationVector(relative * j.reference.conjugate()).length(), 0, 1e-9, "Reference not updated");
        check(j.angle == 0 && !j.encoderInitialized, "Encoder not re-zeroed");
        // Physics agrees: nothing moves in zero gravity.
        Vec3 before = world.bodies[2].position;
        for (int i = 0; i < 120; ++i) {
            world.step();
        }
        near((world.bodies[2].position - before).length(), 0, 1e-6, "Rotated assembly did not stay put");
        near(world.joints[0].angle, 0, 1e-6, "Joint does not read zero at the rotated pose");
    }
}

TEST_CASE("rotateAssembly moves the subtree rigidly about the pivot") {
    World world = chain();
    Vec3 pivot = world.bodies[1].position;
    rotateAssembly(world, 1, pivot, {0, 1, 0}, pi / 2);
    near((world.bodies[1].position - pivot).length(), 0, 1e-12, "Pivot body moved");
    near((world.bodies[2].position - Vec3{1, 1, -1}).length(), 0, 1e-12, "Child not carried along");
    near((world.bodies[0].position - Vec3{0, 1, 0}).length(), 0, 1e-12, "Parent must stay");
    // The inner joint frames are body-local and untouched: its slide axis turned with the assembly.
    Vec3 axis = world.bodies[1].orientation.rotate(world.joints[1].axisA);
    near((axis - Vec3{0, 0, -1}).length(), 0, 1e-12, "Inner joint axis did not follow");
    near(world.maxAnchorError(), 0, 1e-12, "Joint drift after rotation");
    // Rotating the rover's chassis leaves all wheels attached; a whole turn returns to the start.
    World rover = makeRoverDemo();
    World original = rover;
    for (int k = 0; k < 4; ++k) {
        rotateAssembly(rover, 0, rover.bodies[0].position, {0, 1, 0}, pi / 2);
    }
    near(rover.maxAnchorError(), 0, 1e-9, "Rover joints broke");
    near((rover.bodies[1].position - original.bodies[1].position).length(), 0, 1e-9, "Full turn drifted");
    check(!isWheelJoint(rover, rover.joints[5]), "Fixed joint is not a wheel");
    check(isWheelJoint(rover, rover.joints[0]), "Rotation must keep wheels drivable");
}

TEST_CASE("moveAssembly and dropAssemblyToFloor") {
    World world = chain();
    moveAssembly(world, 1, {0, 0.5, 0});
    near(world.maxAnchorError(), 0, 1e-12, "Move broke the hinge");
    near(world.bodies[2].position.y, 1.5, 1e-12, "Child did not follow");
    double shift = dropAssemblyToFloor(world, 1);
    near(shift, -1.25, 1e-12, "Default 0.5 m box at 1.5 must drop by 1.25");
    near(assemblyLowestPoint(world, 1), 0, 1e-12, "Not on the floor");
    // Exact support of rotated shapes.
    Body wheel;
    wheel.shape = Shape::Cylinder;
    wheel.size = {0.6, 0.6, 0.2};
    wheel.position = {0, 2, 0};
    near(lowestPoint(wheel), 1.7, 1e-12, "Upright cylinder rests on its rim");
    wheel.orientation = Quat::axisAngle({1, 0, 0}, pi / 2);
    near(lowestPoint(wheel), 1.9, 1e-12, "Flat cylinder rests on its cap");
    Body tilted;
    tilted.size = {1, 1, 1};
    tilted.position = {0, 2, 0};
    tilted.orientation = Quat::axisAngle({0, 0, 1}, pi / 4);
    near(lowestPoint(tilted), 2 - std::sqrt(0.5), 1e-12, "Tilted box rests on its edge");
}

TEST_CASE("snapToStep") {
    near(snapToStep(0.137, 0.05), 0.15, 1e-12, "Nearest multiple");
    near(snapToStep(-0.137, 0.05), -0.15, 1e-12, "Negative values");
    near(snapToStep(0.137, 0), 0.137, 1e-15, "Step 0 disables snapping");
}

TEST_CASE("gizmo view projection and handle hit testing") {
    Vec3 center{0.2, 0.7, -0.1};
    GizmoView view = lookingAt({4, 3, 5}, center);
    GizmoPoint p;
    check(view.project(center, p), "Center in front of camera");
    near(p.x, 400, 1e-9, "Center projects to view center");
    near(p.y, 300, 1e-9, "Center projects to view center");
    near((view.ray(p) - view.forward).length(), 0, 1e-12, "Ray through the center");
    // A constant on-screen size: the arrow tip is the same distance in pixels at any camera distance.
    for (double zoom : {1.0, 3.0}) {
        GizmoView v = lookingAt(center + (Vec3{4, 3, 5} - center) * zoom, center);
        double unit = v.worldPerPoint(center);
        Vec3 screenUp = center + v.up * (gizmoArrowLength * unit);
        GizmoPoint tip;
        check(v.project(screenUp, tip), "Tip visible");
        near(tip.y - 300, gizmoArrowLength, 0.5, "Handle size must not depend on the distance");
    }
    double unit = view.worldPerPoint(center);
    auto at = [&](Vec3 world) {
        GizmoPoint q;
        check(view.project(world, q), "Handle visible");
        return q;
    };
    check(gizmoHitTest(view, center, GizmoTool::Move, at(center + Vec3{70, 0, 0} * unit))
              == GizmoHandle::AxisX,
          "X arrow");
    check(gizmoHitTest(view, center, GizmoTool::Move, at(center + Vec3{0, 80, 0} * unit))
              == GizmoHandle::AxisY,
          "Y arrow");
    check(gizmoHitTest(view, center, GizmoTool::Move, at(center + Vec3{0, 0, 70} * unit))
              == GizmoHandle::AxisZ,
          "Z arrow");
    check(gizmoHitTest(view, center, GizmoTool::Move, at(center + Vec3{43, 0, 43} * unit))
              == GizmoHandle::PlaneXZ,
          "XZ plane");
    check(gizmoHitTest(view, center, GizmoTool::Move, {5, 5}) == GizmoHandle::None, "Empty space");
    check(gizmoHitTest(view, center, GizmoTool::Select, p) == GizmoHandle::None,
          "Select tool has no handles");
    Vec3 onRing = center + Vec3{gizmoRingRadius * unit, 0, 0};
    check(gizmoHitTest(view, center, GizmoTool::Rotate, at(onRing)) != GizmoHandle::None, "Ring reachable");
    Vec3 onY = center + Vec3{std::cos(0.7), 0, std::sin(0.7)} * (gizmoRingRadius * unit);
    check(gizmoHitTest(view, center, GizmoTool::Rotate, at(onY)) == GizmoHandle::RingY, "Y ring");
}

TEST_CASE("gizmo drags follow the mouse and snap") {
    Vec3 center{0.203, 0.7, -0.1};
    GizmoView view = lookingAt({4, 3, 5}, center);
    auto at = [&](Vec3 world) {
        GizmoPoint q;
        check(view.project(world, q), "Point visible");
        return q;
    };
    // Grab the arrow away from its center: the grab offset must not make the part jump.
    Vec3 grab = center + Vec3{0.4, 0, 0};
    GizmoDrag drag = beginGizmoDrag(view, center, GizmoHandle::AxisX, at(grab));
    Vec3 same = gizmoDragPosition(drag, view, at(grab), 0);
    near((same - center).length(), 0, 1e-9, "No jump when the drag begins");
    Vec3 target = gizmoDragPosition(drag, view, at(grab + Vec3{0.75, 0, 0}), 0);
    near((target - (center + Vec3{0.75, 0, 0})).length(), 0, 1e-9, "Axis drag follows the mouse");
    Vec3 snapped = gizmoDragPosition(drag, view, at(grab + Vec3{0.75, 0, 0}), 0.25);
    near(snapped.x, 1.0, 1e-12, "X lands on the 25 cm grid");
    near(snapped.y, center.y, 1e-12, "Other axes keep their value");
    near(snapped.z, center.z, 1e-12, "Other axes keep their value");
    // Plane drag.
    GizmoDrag plane = beginGizmoDrag(view, center, GizmoHandle::PlaneXZ, at(center + Vec3{0.2, 0, 0.2}));
    Vec3 moved = gizmoDragPosition(plane, view, at(center + Vec3{0.5, 0, -0.1}), 0);
    near((moved - (center + Vec3{0.3, 0, -0.3})).length(), 0, 1e-9, "Plane drag");
    Vec3 grid = gizmoDragPosition(plane, view, at(center + Vec3{0.5, 0, -0.1}), 0.1);
    near(grid.x, 0.5, 1e-12, "Plane snaps X");
    near(grid.z, -0.4, 1e-12, "Plane snaps Z");
    near(grid.y, center.y, 1e-12, "Plane keeps Y");
    // Rotation: right-handed about +Y, accumulating past half a turn.
    auto ring = [&](double angle) {
        // Ring Y basis: e1 = Z, e2 = X.
        return at(center + Vec3{std::sin(angle), 0, std::cos(angle)} * 0.5);
    };
    GizmoDrag turn = beginGizmoDrag(view, center, GizmoHandle::RingY, ring(0));
    near(gizmoDragAngle(turn, view, ring(0.5), 0), 0.5, 1e-9, "Ring angle");
    near(gizmoDragAngle(turn, view, ring(0.5), pi / 12), pi / 6, 1e-12, "Ring snaps to 15 degrees");
    for (double a = 0.5; a < 4; a += 0.5) {
        (void)gizmoDragAngle(turn, view, ring(a), 0);
    }
    near(gizmoDragAngle(turn, view, ring(4), 0), 4, 1e-9, "Angles accumulate past pi");
    // Handles seen end-on cannot be used; a parallel ray keeps the last target.
    GizmoView down = lookingAt({0, 5, 0.001}, {0, 0, 0});
    check(!gizmoHandleUsable(down, {0, 0, 0}, GizmoHandle::AxisY), "End-on arrow");
    check(!gizmoHandleUsable(lookingAt({5, 0, 0}, {0, 0, 0}), {0, 0, 0}, GizmoHandle::PlaneXZ),
          "Edge-on plane");
}

TEST_CASE("library items insert with unique names and stay stable") {
    for (const LibraryItem& item : builtinLibrary()) {
        // Free placement twice: names stay unique and nothing explodes.
        World world;
        LibraryInsertion first = insertLibraryItem(world, item, -1, {1, 0, 2});
        LibraryInsertion second = insertLibraryItem(world, item, -1, {-2, 0, 0});
        check(first.attachJoint < 0 && second.bodies == first.bodies, "Free insertion");
        std::set<std::string> names;
        for (const Body& body : world.bodies) {
            check(names.insert(body.name).second, "Duplicate body name");
        }
        near(assemblyLowestPoint(world, first.root), 0, 1e-9, "Rests on the floor");
        for (int i = 0; i < 240; ++i) {
            stepSimulation(world);
        }
        world.validate();
        for (const Body& body : world.bodies) {
            check(body.velocity.length() < 5 && body.position.length() < 10, item.title.c_str());
        }
        check(world.maxAnchorError() < 0.01, item.title.c_str());
    }
}

TEST_CASE("library items attach to a selected part") {
    for (const LibraryItem& item : builtinLibrary()) {
        World world = makeRoverDemo();
        std::size_t bodies = world.bodies.size();
        std::size_t joints = world.joints.size();
        Vec3 chassis = world.bodies[0].position;
        LibraryInsertion insertion = insertLibraryItem(world, item, 0);
        check(insertion.attachJoint >= 0, "Item must attach");
        check(world.bodies.size() == bodies + static_cast<std::size_t>(insertion.bodies), "Body count");
        check(world.joints.size() > joints, "No joint added");
        near(world.maxAnchorError(), 0, 1e-9, "Attached item must start with valid joints");
        near((world.bodies[0].position - chassis).length(), 0, 1e-15, "Host must not move");
        for (const Sensor& sensor : world.sensors) {
            check(sensor.body >= 0 && sensor.body < static_cast<int>(world.bodies.size()), "Sensor body");
        }
        for (int i = 0; i < 240; ++i) {
            stepSimulation(world);
        }
        world.validate();
        check(world.maxAnchorError() < 0.02, item.title.c_str());
        for (const Body& body : world.bodies) {
            check(body.velocity.length() < 8, item.title.c_str());
        }
    }
}

TEST_CASE("driven wheels attach as horizontal hinges and drive the host") {
    const LibraryItem* wheel = nullptr;
    const LibraryItem* plate = nullptr;
    const LibraryItem* caster = nullptr;
    for (const LibraryItem& item : builtinLibrary()) {
        wheel = item.id == "driven_wheel" ? &item : wheel;
        plate = item.id == "chassis_plate" ? &item : plate;
        caster = item.id == "caster_ball" ? &item : caster;
    }
    check(wheel && plate && caster, "Items exist");
    World world;
    int chassis = insertLibraryItem(world, *plate, -1, {0, 0, 0}).root;
    LibraryInsertion left = insertLibraryItem(world, *wheel, chassis);
    near(assemblyLowestPoint(world, chassis), 0, 1e-9, "A plate on the floor is lifted onto its wheels");
    LibraryInsertion right = insertLibraryItem(world, *wheel, chassis);
    insertLibraryItem(world, *caster, chassis);
    near(assemblyLowestPoint(world, chassis), 0, 1e-9, "Robot stands on the floor");
    check(left.bodies == 1, "Hub replaced by the host");
    check(world.joints[static_cast<std::size_t>(left.attachJoint)].a == chassis, "Wheel hinged to the host");
    check(isWheelJoint(world, world.joints[static_cast<std::size_t>(left.attachJoint)]), "Not a drive wheel");
    check(isWheelJoint(world, world.joints[static_cast<std::size_t>(right.attachJoint)]),
          "Not a drive wheel");
    near(world.bodies[static_cast<std::size_t>(left.root)].position.z
             + world.bodies[static_cast<std::size_t>(right.root)].position.z,
         0, 1e-12, "Second wheel uses the opposite side");
    double x = world.bodies[static_cast<std::size_t>(chassis)].position.x;
    driveRover(world, 3, 0);
    for (int i = 0; i < 480; ++i) {
        stepSimulation(world);
    }
    check(std::abs(world.bodies[static_cast<std::size_t>(chassis)].position.x - x) > 0.3,
          "Robot did not drive");
}

TEST_CASE("assemblies extract, save and reinsert") {
    World rover = makeSensorRoverDemo();
    int deck = 5;
    World part = extractAssembly(rover, deck);
    check(part.bodies.size() == 2 && part.bodies[0].name == "rover_deck", "Root first, children included");
    check(part.joints.size() == 1 && part.joints[0].a == 0 && part.joints[0].b == 1, "Joints remapped");
    bool lidar = false;
    for (const Sensor& sensor : part.sensors) {
        lidar = lidar || (sensor.type == SensorType::Lidar2D && sensor.body == 1);
    }
    check(lidar, "Sensors of the assembly come along");
    std::stringstream file;
    writeScene(part, file);
    LibraryItem item = customLibraryItem("Sensor deck", readScene(file));
    near(assemblyLowestPoint(item.assembly, 0), 0, 1e-9, "Custom item origin on its lowest point");
    World world = makeRoverDemo();
    LibraryInsertion insertion = insertLibraryItem(world, item, 0);
    check(world.bodies[static_cast<std::size_t>(insertion.root)].name == "rover_deck_1", "Renamed");
    check(world.joints[static_cast<std::size_t>(insertion.attachJoint)].type == JointType::Fixed,
          "Fixed joint");
    near(world.maxAnchorError(), 0, 1e-9, "Attached anchors");
    test::rejects([&] {
        World copy;
        (void)insertLibraryItem(copy, item, 3);
    });
}

TEST_CASE("uniqueBodyName") {
    World world;
    Body body;
    body.name = "wheel";
    world.addBody(body);
    check(uniqueBodyName(world, "arm") == "arm", "Free name stays");
    check(uniqueBodyName(world, "wheel") == "wheel_1", "First copy");
    body.name = "wheel_1";
    world.addBody(body);
    check(uniqueBodyName(world, "wheel") == "wheel_2", "Second copy");
    check(uniqueBodyName(world, "wheel_1") == "wheel_2", "Numeric suffix is replaced");
}

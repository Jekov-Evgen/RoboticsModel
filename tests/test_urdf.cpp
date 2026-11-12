// URDF importer: joint mapping, kinematic tree poses, mesh handling, warnings and error messages.
#include "test_harness.hpp"

#include "robotics/io/urdf.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace robotics;
using namespace test;

namespace {

std::string examplePath(const std::string& relative) {
    return std::string(ROBOTICS_SOURCE_DIR) + "/examples/urdf/" + relative;
}

UrdfImportResult importText(const std::string& text, const UrdfImportOptions& options = {}) {
    std::istringstream in(text);
    return importUrdf(in, options);
}

int bodyNamed(const World& world, const std::string& name) {
    for (std::size_t i = 0; i < world.bodies.size(); ++i) {
        if (world.bodies[i].name == name) {
            return static_cast<int>(i);
        }
    }
    throw std::runtime_error("No body named " + name);
}

bool hasWarning(const UrdfImportResult& result, const std::string& fragment) {
    return std::any_of(result.warnings.begin(), result.warnings.end(),
                       [&](const std::string& w) { return w.find(fragment) != std::string::npos; });
}

// Error message of a failing import (empty if it did not fail).
std::string importError(const std::string& text) {
    try {
        (void)importText(text);
    } catch (const std::exception& e) {
        return e.what();
    }
    return {};
}

void nearVec(Vec3 actual, Vec3 expected, double tolerance, const char* message) {
    near((actual - expected).length(), 0, tolerance, message);
}

} // namespace

TEST_CASE("urdf two-link arm: revolute limits, effort and zero-configuration poses") {
    UrdfImportOptions options;
    options.fixBase = true;
    auto result = loadUrdf(examplePath("two_link_arm.urdf"), options);
    const World& w = result.world;
    check(result.robotName == "two_link_arm", "Robot name");
    check(w.bodies.size() == 3 && w.joints.size() == 2, "Arm needs 3 bodies (tool0 merged) and 2 joints");
    int base = bodyNamed(w, "base_link");
    int upper = bodyNamed(w, "upper_arm");
    int fore = bodyNamed(w, "forearm");
    check(w.bodies[base].mass == 0, "fixBase makes the root static");
    near(w.bodies[upper].mass, 1, 1e-12, "Upper arm mass");
    // Z-up URDF -> Y-up world: URDF (x, y, z) = world (x, z, -y).
    nearVec(w.bodies[base].position, {0, 0.05, 0}, 1e-9, "Base pose");
    nearVec(w.bodies[upper].position, {0, 0.35, 0}, 1e-9, "Upper arm pose");
    nearVec(w.bodies[fore].position, {0, 0.8, 0}, 1e-9, "Forearm pose");
    check(w.bodies[upper].shape == Shape::Cylinder && w.bodies[upper].size.z == 0.5
              && w.bodies[upper].size.x == 0.08,
          "Cylinder length -> size.z, radius -> diameter");
    // The cylinder axis (local Z) must point up in the world.
    nearVec(w.bodies[upper].orientation.rotate({0, 0, 1}), {0, 1, 0}, 1e-9, "Cylinder axis");

    const Joint& shoulder = w.joints[0];
    check(shoulder.type == JointType::Hinge && shoulder.a == base && shoulder.b == upper,
          "Shoulder topology");
    check(shoulder.limitsEnabled && shoulder.lowerLimit == -1.57 && shoulder.upperLimit == 1.57,
          "Revolute limits");
    check(shoulder.maxTorque == 30, "Effort -> maxTorque");
    nearVec(w.bodies[base].point(shoulder.anchorA), {0, 0.1, 0}, 1e-9, "Shoulder anchor");
    // URDF axis +Y maps to world -Z.
    nearVec(w.bodies[base].orientation.rotate(shoulder.axisA), {0, 0, -1}, 1e-9, "Shoulder axis");

    // rpy yaw 90 deg: the elbow's local Y is the parent's -X.
    const Joint& elbow = w.joints[1];
    check(elbow.limitsEnabled && elbow.maxTorque == 15, "Elbow limits");
    nearVec(w.bodies[upper].orientation.rotate(elbow.axisA), {-1, 0, 0}, 1e-9, "Elbow axis after rpy");
    nearVec(w.bodies[upper].point(elbow.anchorA), {0, 0.6, 0}, 1e-9, "Elbow anchor");
    check(w.maxAnchorError() < 1e-9, "Joints closed at import");

    // Gravity and limits: the arm keeps running without breaking constraints.
    World running = w;
    advance(running, 240);
    check(running.maxAnchorError() < 0.01, "Imported arm drifts");
    check(running.joints[0].angle >= -1.57 - 0.05 && running.joints[0].angle <= 1.57 + 0.05,
          "Shoulder limit holds");
}

TEST_CASE("urdf rpy is fixed-axis roll-pitch-yaw") {
    const char* text = R"(<robot name="r">
      <link name="a"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
      <joint name="j" type="fixed"><parent link="a"/><child link="b"/>
        <origin xyz="1 2 3" rpy="0.3 -0.2 0.7"/></joint>
      <link name="b"><collision><geometry><box size="0.1 0.2 0.3"/></geometry></collision></link>
    </robot>)";
    UrdfImportOptions options;
    options.zUpToYUp = false;
    options.liftAboveGround = false;
    auto result = importText(text, options);
    const Body& b = result.world.bodies[1];
    nearVec(b.position, {1, 2, 3}, 1e-12, "Origin xyz");
    // R = Rz(yaw) Ry(pitch) Rx(roll), so Rx acts first.
    Quat expected =
        Quat::axisAngle({0, 0, 1}, 0.7) * Quat::axisAngle({0, 1, 0}, -0.2) * Quat::axisAngle({1, 0, 0}, 0.3);
    for (Vec3 v : {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}}) {
        nearVec(b.orientation.rotate(v), expected.rotate(v), 1e-12, "rpy convention");
    }
    // Explicit check of one column: roll then pitch then yaw of the X axis.
    double cy = std::cos(0.7);
    double sy = std::sin(0.7);
    double cp = std::cos(-0.2);
    double sp = std::sin(-0.2);
    nearVec(b.orientation.rotate({1, 0, 0}), {cy * cp, sy * cp, -sp}, 1e-12, "First column of R");
}

TEST_CASE("urdf differential drive: continuous wheels, caster and reference frames") {
    auto result = loadUrdf(examplePath("diff_drive.urdf"));
    const World& w = result.world;
    check(w.bodies.size() == 4 && w.joints.size() == 3,
          "Diff drive: base, caster, 2 wheels; caster weld + 2 wheel hinges");
    int base = bodyNamed(w, "base_link");
    int caster = bodyNamed(w, "base_link_collision_2");
    int left = bodyNamed(w, "left_wheel");
    int right = bodyNamed(w, "right_wheel");
    check(w.bodies[caster].shape == Shape::Sphere && w.bodies[caster].size.x == 0.08, "Caster sphere");
    // Mass 4 kg split by volume between the two collision shapes, plus the laser frame's 0.1 kg.
    near(w.bodies[base].mass + w.bodies[caster].mass, 4.1, 1e-9, "Link mass split and merged sensor mass");
    check(w.bodies[left].shape == Shape::Cylinder && w.bodies[left].size.x == 0.16
              && w.bodies[left].size.z == 0.04,
          "Wheel cylinder");
    // Left (URDF +Y) is world -Z; wheels touch the floor; the cylinder axis is lateral.
    check(w.bodies[left].position.z < 0 && w.bodies[right].position.z > 0, "Wheel sides");
    near(w.bodies[left].position.y, 0.08, 1e-9, "Wheel height");
    near(std::abs(w.bodies[left].orientation.rotate({0, 0, 1}).z), 1, 1e-9, "Wheel axis is lateral");
    near(w.bodies[caster].position.y, 0.04, 1e-9, "Caster touches the floor");
    int hinges = 0;
    for (const auto& j : w.joints) {
        if (j.type == JointType::Hinge) {
            ++hinges;
            check(!j.limitsEnabled && j.maxTorque == 5, "Continuous joints: no limits, effort from <limit>");
        } else {
            check(j.type == JointType::Fixed && j.a == base && j.b == caster, "Caster weld");
        }
    }
    check(hinges == 2, "Two wheel hinges");
    check(hasWarning(result, "several collision shapes"), "Multi-collision link reported");

    // Drive: equal wheel speeds move the robot along +X without breaking joints.
    World driving = w;
    for (auto& j : driving.joints) {
        if (j.type == JointType::Hinge) {
            j.motorSpeed = 6; // About URDF +Y = forward for both wheels.
        }
    }
    advance(driving, 480);
    check(driving.bodies[base].position.x > 0.2, "Diff drive did not move forward");
    check(driving.maxAnchorError() < 0.01, "Diff drive joints drift");
}

TEST_CASE("urdf gripper: prismatic fingers with limits, unsupported features reported") {
    UrdfImportOptions options;
    options.fixBase = true;
    auto result = loadUrdf(examplePath("gripper.urdf"), options);
    const World& w = result.world;
    check(w.bodies.size() == 3 && w.joints.size() == 2, "Gripper bodies/joints");
    for (const auto& j : w.joints) {
        check(j.type == JointType::Prismatic && j.limitsEnabled && j.lowerLimit == 0 && j.upperLimit == 0.05
                  && j.maxTorque == 20,
              "Prismatic finger mapping");
    }
    nearVec(w.bodies[0].orientation.rotate(w.joints[0].axisA), {-1, 0, 0}, 1e-9, "Finger axis");
    check(hasWarning(result, "mimic"), "Mimic joint reported");
    check(hasWarning(result, "damping"), "Dynamics reported");
    near(w.bodies[0].position.y, 0.03, 1e-9, "Palm rests on the floor");

    // Close both fingers with their motors: limits stop them at 5 cm.
    World closing = w;
    closing.gravity = {};
    for (auto& j : closing.joints) {
        j.motorSpeed = 0.2;
    }
    advance(closing, 480);
    for (const auto& j : closing.joints) {
        check(j.angle <= 0.05 + 0.005 && j.angle > 0.03, "Finger limit");
    }
}

TEST_CASE("urdf meshes: convex hull collision by default, visuals stay on the mesh") {
    UrdfImportOptions options;
    options.fixBase = true;
    auto result = loadUrdf(examplePath("demo_meshes/urdf/mesh_arm.urdf"), options);
    const World& w = result.world;
    check(w.bodies.size() == 2 && w.joints.size() == 1, "Same structure as with boxes");
    check(!hasWarning(result, "bounding boxes"), "No box approximation");
    const Body& base = w.bodies[0];
    check(base.shape == Shape::ConvexHull && base.hull, "Mesh collision becomes a hull");
    // base.stl (scale 0.001): octagon of radius 80 mm up to z = 12 mm, tapering to radius 55 mm at z = 60 mm;
    // the step at z = 18 mm lies inside the hull. Regular octagon area: 2·sqrt(2)·r².
    double wide = 2 * std::sqrt(2.0) * 0.08 * 0.08;
    double narrow = 2 * std::sqrt(2.0) * 0.055 * 0.055;
    double volume = wide * 0.012 + 0.048 / 3 * (wide + narrow + std::sqrt(wide * narrow));
    near(base.hull->volume, volume, 2e-6, "Hull volume respects the scale");
    check(base.hull->vertices.size() == 24, "Three octagonal rings");
    check(base.hull->sourcePath.find("base.stl") != std::string::npos && base.hull->sourceScale.x == 0.001,
          "Hull remembers its mesh");
    double lowest = 1e9;
    double highest = -1e9;
    for (Vec3 v : base.hull->vertices) {
        lowest = std::min(lowest, base.point(v).y);
        highest = std::max(highest, base.point(v).y);
    }
    near(lowest, 0, 1e-6, "Mesh base on the floor (URDF Z up is world Y up)");
    near(highest, 0.06, 1e-6, "Mesh top");
    // The visual mesh must sit where the bounding-box import puts it, whatever the principal frame is.
    options.meshCollision = UrdfMeshCollision::BoundingBox;
    auto boxes = loadUrdf(examplePath("demo_meshes/urdf/mesh_arm.urdf"), options);
    for (std::size_t i = 0; i < w.bodies.size(); ++i) {
        const Body& hull = w.bodies[i];
        const Body& box = boxes.world.bodies[i];
        check(hull.visuals.size() == box.visuals.size(), "Same visuals");
        for (std::size_t k = 0; k < hull.visuals.size(); ++k) {
            for (Vec3 p : {Vec3{}, Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}}) {
                Vec3 viaHull = hull.point(hull.visuals[k].position + hull.visuals[k].orientation.rotate(p));
                Vec3 viaBox = box.point(box.visuals[k].position + box.visuals[k].orientation.rotate(p));
                nearVec(viaHull, viaBox, 1e-9, "Visual pose does not depend on the collision shape");
            }
        }
    }
    // A hull body on the joint works like the box did: the arm hangs on its hinge.
    World moving = w;
    for (int i = 0; i < 240; ++i) {
        moving.step();
    }
    check(moving.maxAnchorError() < 1e-3, "Joint between hull bodies holds");
}

TEST_CASE("urdf meshes: flat collision meshes fall back to boxes, floating hulls are lifted off the floor") {
    auto folder = std::filesystem::temp_directory_path() / "robotics_urdf_hull";
    std::filesystem::create_directories(folder);
    std::ofstream(folder / "flat.obj") << "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n";
    std::ofstream(folder / "tetra.obj") << "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";
    std::string urdf = R"(<robot name="r">
      <link name="flat"><inertial><mass value="1"/></inertial>
        <collision><geometry><mesh filename="flat.obj"/></geometry></collision></link>
      <link name="rock"><inertial><mass value="2"/></inertial>
        <collision><origin xyz="0 0 1" rpy="0.3 0.2 0.1"/><geometry><mesh filename="tetra.obj" scale="2 -1 1"/></geometry></collision></link>
      <joint name="j" type="fixed"><parent link="flat"/><child link="rock"/></joint>
    </robot>)";
    std::istringstream text(urdf);
    auto result = importUrdf(text, {}, (folder / "robot.urdf").string());
    const World& w = result.world;
    check(w.bodies.size() == 2, "Two bodies");
    check(w.bodies[0].shape == Shape::Box, "Flat mesh stays a box");
    check(hasWarning(result, "no volume"), "Fallback reported");
    const Body& rock = w.bodies[1];
    check(rock.shape == Shape::ConvexHull, "Tetrahedron becomes a hull");
    near(rock.hull->volume, 2.0 / 6, 1e-9, "Mirroring scale keeps the volume positive");
    near(rock.mass, 2, 1e-12, "Mass from the URDF link");
    double lowest = 1e9;
    for (const auto& body : w.bodies) {
        if (body.shape == Shape::ConvexHull) {
            for (Vec3 v : body.hull->vertices) {
                lowest = std::min(lowest, body.point(v).y);
            }
        }
    }
    check(lowest >= -1e-9, "Lifted by the true lowest vertex of the hull");
    std::filesystem::remove_all(folder);
}

TEST_CASE("urdf package:// meshes: bounding-box collision, visuals and relative paths") {
    UrdfImportOptions options;
    options.fixBase = true;
    options.meshCollision = UrdfMeshCollision::BoundingBox;
    auto result = loadUrdf(examplePath("demo_meshes/urdf/mesh_arm.urdf"), options);
    const World& w = result.world;
    check(w.bodies.size() == 2 && w.joints.size() == 1, "Mesh arm: tool merged into the arm");
    const Body& base = w.bodies[0];
    check(base.shape == Shape::Box, "Mesh collision becomes a box");
    // base.stl spans +-80 mm (octagon, flat sides) and 0..60 mm, scaled by 0.001.
    double across = 0.16 * std::cos(pi / 8);
    nearVec(base.size, {across, across, 0.06}, 1e-6, "Bounding box size respects scale");
    near(base.position.y, 0.03, 1e-6, "Bounding box centre");
    check(hasWarning(result, "bounding boxes"), "Bounding-box approximation reported");
    check(base.visuals.size() == 1 && base.visuals[0].path.find("base.stl") != std::string::npos
              && base.visuals[0].scale.x == 0.001,
          "Base visual mesh");
    // The mesh frame sits 30 mm below the box centre.
    nearVec(base.visuals[0].position, {0, 0, -0.03}, 1e-6, "Visual offset in the body frame");
    const Body& arm = w.bodies[1];
    check(arm.visuals.size() == 2, "Arm draws its OBJ and the merged tool's ASCII STL");
    check(arm.visuals[1].path.find("tip_ascii.stl") != std::string::npos, "Relative visual path");
    for (const auto& visual : arm.visuals) {
        check(std::filesystem::exists(visual.path), "Resolved mesh path exists");
    }
    // The tool mesh frame is at the arm tip: 0.4 m along the arm (URDF X = world X).
    nearVec(arm.point(arm.visuals[1].position), {0.4, 0.1, 0}, 1e-6, "Tool visual pose");
    check(w.joints[0].type == JointType::Hinge && w.joints[0].maxTorque == 12, "Shoulder yaw");
    nearVec(base.orientation.rotate(w.joints[0].axisA), {0, 1, 0}, 1e-9, "URDF Z axis is world up");

    // package:// resolution, search paths and failures.
    std::string urdf = examplePath("demo_meshes/urdf/mesh_arm.urdf");
    check(!resolveUrdfMeshPath("package://demo_meshes/meshes/arm.obj", urdf).empty(),
          "package:// upward search");
    check(resolveUrdfMeshPath("package://missing_pkg/meshes/arm.obj", urdf).empty(), "Unknown package");
    check(
        !resolveUrdfMeshPath("package://demo_meshes/meshes/arm.obj", "/tmp/elsewhere.urdf", {examplePath("")})
             .empty(),
        "Search path as package parent");
}

TEST_CASE("urdf links without collision, missing meshes and floating joints") {
    const char* text = R"(<?xml version="1.0"?>
    <robot name="odd">
      <link name="world"/>
      <joint name="anchor" type="fixed"><parent link="world"/><child link="base"/></joint>
      <link name="base">
        <collision><origin xyz="0 0 0.1"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
      </link>
      <joint name="spin" type="continuous"><parent link="base"/><child link="hub"/>
        <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/></joint>
      <link name="hub"><visual><geometry><mesh filename="meshes/nowhere.stl"/></geometry></visual></link>
      <joint name="j2" type="revolute"><parent link="hub"/><child link="paddle"/>
        <origin xyz="0.1 0 0"/><axis xyz="1 0 0"/></joint>
      <link name="paddle">
        <visual><geometry><mesh filename="paddle.dae"/></geometry></visual>
        <collision><geometry><sphere radius="0.05"/></geometry></collision>
      </link>
      <joint name="loose" type="floating"><parent link="base"/><child link="ball"/>
        <origin xyz="1 0 1"/></joint>
      <link name="ball"><inertial><mass value="0"/></inertial>
        <collision><geometry><sphere radius="0.1"/></geometry></collision></link>
    </robot>)";
    auto result = importText(text);
    const World& w = result.world;
    // "world" is a static reference: base is welded-static; hub gets a hidden placeholder.
    int base = bodyNamed(w, "base");
    int hub = bodyNamed(w, "hub");
    int ball = bodyNamed(w, "ball");
    check(w.bodies[base].mass == 0, "Links fixed to 'world' are static");
    check(w.bodies[hub].primitiveHidden && w.bodies[hub].shape == Shape::Sphere, "Placeholder body");
    check(w.bodies[hub].mass == 0.01 && w.bodies[ball].mass == 0.01, "Missing/zero mass -> 0.01 kg");
    check(w.joints.size() == 2, "Floating joint skipped, fixed joint to world not needed");
    check(w.joints[0].maxTorque == 10 && !w.joints[0].limitsEnabled,
          "Continuous without <limit>: default effort");
    check(!w.joints[1].limitsEnabled && w.joints[1].maxTorque == 10, "Revolute without <limit>");
    check(hasWarning(result, "not found") && hasWarning(result, "nowhere.stl"), "Missing mesh reported");
    check(hasWarning(result, "unsupported format") && hasWarning(result, "paddle.dae"), "DAE reported");
    check(hasWarning(result, "floating"), "Floating joint reported");
    check(hasWarning(result, "without mass"), "Massless links reported");
    check(hasWarning(result, "placeholder"), "Placeholder reported");
    check(hasWarning(result, "without <limit>"), "Missing limit reported");
}

TEST_CASE("urdf import options: spawn offset, no lift, effort default") {
    UrdfImportOptions options;
    options.spawnOffset = {2, 1, -3};
    options.defaultEffort = 3;
    auto lifted = loadUrdf(examplePath("diff_drive.urdf"), options);
    int base = bodyNamed(lifted.world, "base_link");
    nearVec(lifted.world.bodies[base].position, {2, 1 + 0.12, -3}, 1e-9, "Spawn offset");

    const char* sunk = R"(<robot name="s"><link name="a">
        <collision><geometry><box size="1 1 1"/></geometry></collision></link></robot>)";
    near(importText(sunk).world.bodies[0].position.y, 0.5, 1e-12, "Lifted onto the floor");
    options = {};
    options.liftAboveGround = false;
    near(importText(sunk, options).world.bodies[0].position.y, 0, 1e-12, "No lift when disabled");
    rejects([] {
        UrdfImportOptions bad;
        bad.minimumMass = 0;
        (void)importText("<robot name='x'><link name='a'/></robot>", bad);
    });
}

TEST_CASE("urdf malformed input gives clear errors with line numbers") {
    struct Case {
        const char* text;
        const char* expected;
    };

    const Case cases[] = {
        {"<robot name=\"r\">\n  <link name=\"a\">\n</robot>", "line 3"},
        {"<robot name=\"r\">\n  <link name=\"a\">\n</robot>", "does not match <link> opened on line 2"},
        {"<robot name=\"r\">\n<link name=\"a\"/>\n", "never closed"},
        {"<robot name=\"r\">\n<link name=a/>\n</robot>", "must be in quotes"},
        {"<robot name=\"r\">\n<link name=\"a\" name=\"b\"/>\n</robot>", "appears twice"},
        {"<robot name=\"r\"><link name=\"a&b\"/></robot>", "&amp;"},
        {"<robot name=\"r\"><!-- unterminated\n<link name=\"a\"/></robot>", "comment started on line 1"},
        {"", "empty"},
        {"<robot name=\"r\"><link name=\"a\"/></robot>\n<extra/>", "after the closing </robot>"},
        {"<model/>", "expected <robot>"},
        {"<robot name=\"r\"/>", "no <link>"},
        {"<robot name=\"r\">\n<link name=\"a\"/>\n<link name=\"a\"/></robot>",
         "line 3: link name 'a' is used twice"},
        {"<robot name=\"r\"><link name=\"a\"/>\n<joint name=\"j\" type=\"revolute\"><parent link=\"a\"/>"
         "<child link=\"ghost\"/></joint></robot>",
         "line 2: joint 'j' refers to child link 'ghost'"},
        {"<robot name=\"r\"><link name=\"a\"/><link name=\"b\"/>\n<joint name=\"j\" type=\"hinge\">"
         "<parent link=\"a\"/><child link=\"b\"/></joint></robot>",
         "unknown type 'hinge'"},
        {"<robot name=\"r\"><link name=\"a\"/><link name=\"b\"/>"
         "<joint name=\"j1\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/></joint>\n"
         "<joint name=\"j2\" type=\"fixed\"><parent link=\"b\"/><child link=\"a\"/></joint></robot>",
         "loop"},
        {"<robot name=\"r\">\n<link name=\"a\"><collision><origin xyz=\"1 2\"/><geometry><box size=\"1 1 "
         "1\"/>"
         "</geometry></collision></link></robot>",
         "line 2: <origin xyz=\"1 2\"> must contain 3 numbers"},
        {"<robot name=\"r\"><link name=\"a\"><collision><geometry><box size=\"1 x "
         "1\"/></geometry></collision>"
         "</link></robot>",
         "'x' is not a valid number"},
        {"<robot name=\"r\"><link name=\"a\"><collision><geometry><sphere radius=\"-1\"/></geometry>"
         "</collision></link></robot>",
         "radius must be greater than zero"},
        {"<robot name=\"r\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n<xacro:property name=\"w\" "
         "value=\"1\"/>"
         "</robot>",
         "xacro"},
        {"<robot name=\"r\"><link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"revolute\">"
         "<parent link=\"a\"/><child link=\"b\"/><limit lower=\"1\" upper=\"-1\" effort=\"1\" "
         "velocity=\"1\"/>"
         "</joint></robot>",
         "lower> is greater than upper"},
    };
    for (const Case& c : cases) {
        std::string error = importError(c.text);
        if (error.find(c.expected) == std::string::npos) {
            throw std::runtime_error("Expected an error containing \"" + std::string(c.expected)
                                     + "\", got \"" + error + "\"");
        }
    }
    // Errors from files name the file.
    std::string error;
    try {
        (void)loadUrdf(std::string(ROBOTICS_SOURCE_DIR) + "/tests/data/malformed_unclosed.urdf");
    } catch (const std::exception& e) {
        error = e.what();
    }
    check(error.find("malformed_unclosed.urdf, line 9") != std::string::npos, "File name and line in error");
    rejects([] { (void)loadUrdf("/nonexistent/robot.urdf"); });
}

TEST_CASE("xml entities, CDATA and comments are handled") {
    const char* text = R"(<?xml version="1.0" encoding="UTF-8"?>
    <!DOCTYPE robot [ <!ELEMENT robot ANY> ]>
    <!-- leading comment -->
    <robot name="a&amp;b &#x41;&#66; &lt;&gt;&quot;&apos;">
      <link name='single'><![CDATA[ <not a tag> ]]></link>
      <?custom instruction?>
    </robot>)";
    auto result = importText(text);
    check(result.robotName == "a&b AB <>\"'", "Entities decoded");
}

TEST_CASE("wheel predicate: diff-drive wheels yes, cylindrical arm links no") {
    World drive = loadUrdf(examplePath("diff_drive.urdf")).world;
    int wheels = 0;
    for (const auto& j : drive.joints) {
        wheels += isWheelJoint(drive, j) ? 1 : 0;
    }
    check(wheels == 2, "Diff drive has two wheels");
    // A vertical cylinder on a limited revolute (like a UR5 link) is not a wheel, nor is a horizontal
    // cylinder on an unlimited hinge whose axis is not its own spin axis.
    const char* arm = R"(<robot name="arm">
      <link name="base"><collision><geometry><box size="0.2 0.2 0.1"/></geometry></collision></link>
      <joint name="pan" type="revolute"><parent link="base"/><child link="column"/><origin xyz="0 0 0.1"/>
        <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
      <link name="column"><collision><origin xyz="0 0 0.2"/><geometry><cylinder radius="0.05" length="0.4"/>
        </geometry></collision></link>
      <joint name="spin" type="continuous"><parent link="column"/><child link="bar"/><origin xyz="0 0 0.4"/>
        <axis xyz="1 0 0"/></joint>
      <link name="bar"><collision><origin rpy="1.5708 0 0"/><geometry><cylinder radius="0.03" length="0.3"/>
        </geometry></collision></link>
    </robot>)";
    UrdfImportOptions options;
    options.fixBase = true;
    World w = importText(arm, options).world;
    for (const auto& j : w.joints) {
        check(!isWheelJoint(w, j), "Arm joint misdetected as a wheel");
    }
    World rover = makeRoverDemo();
    for (const auto& j : rover.joints) {
        check(isWheelJoint(rover, j) == (j.type == JointType::Hinge), "Rover wheels");
    }
}

TEST_CASE("fixed bases are not lifted; parts below the floor are reported") {
    const char* text = R"(<robot name="r">
      <link name="base"><collision><origin xyz="0 0 0.05"/><geometry><box size="0.2 0.2 0.1"/></geometry>
        </collision></link>
      <joint name="j" type="revolute"><parent link="base"/><child link="hang"/><origin xyz="0.3 0 0.05"/>
        <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
      <link name="hang"><collision><origin xyz="0 0 -0.1"/><geometry><box size="0.05 0.05 0.2"/></geometry>
        </collision></link>
    </robot>)";
    UrdfImportOptions options;
    options.fixBase = true;
    auto fixed = importText(text, options);
    near(fixed.world.bodies[0].position.y, 0.05, 1e-12, "Fixed base stays on the floor");
    check(hasWarning(fixed, "below the floor"), "Parts below the floor reported");
    auto free = importText(text);
    near(free.world.bodies[0].position.y, 0.05 + 0.15, 1e-12, "Free robot lifted");
    check(!hasWarning(free, "below the floor"), "No floor warning when lifted");
}

TEST_CASE("package:// matches package.xml names and returns absolute paths") {
    namespace fs = std::filesystem;
    fs::path root = fs::temp_directory_path() / "robotics_urdf_package_test";
    fs::remove_all(root);
    fs::create_directories(root / "checkout-folder" / "meshes");
    fs::create_directories(root / "checkout-folder" / "urdf");
    std::ofstream(root / "checkout-folder" / "package.xml") << "<package><name> my_robot </name></package>";
    std::ofstream(root / "checkout-folder" / "meshes" / "m.obj") << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
    std::string urdf = (root / "checkout-folder" / "urdf" / "r.urdf").string();
    std::string found = resolveUrdfMeshPath("package://my_robot/meshes/m.obj", urdf);
    check(!found.empty() && fs::path(found).is_absolute(), "package.xml <name> walk-up");
    found = resolveUrdfMeshPath("package://my_robot/meshes/m.obj", "/tmp/x.urdf", {root.string()});
    check(!found.empty(), "package.xml <name> inside a search folder");
    fs::path previous = fs::current_path();
    fs::current_path(root);
    found = resolveUrdfMeshPath("meshes/m.obj", "/tmp/x.urdf", {"checkout-folder"});
    fs::current_path(previous);
    check(!found.empty() && fs::path(found).is_absolute(), "Relative search path made absolute");
    fs::remove_all(root);
    check(importError("<robot name=\"&#xD800;\"><link name=\"a\"/></robot>").find("character reference")
              != std::string::npos,
          "Surrogate character reference rejected");
}

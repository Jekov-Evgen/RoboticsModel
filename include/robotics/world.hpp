#pragma once
// World: bodies, joints and the fixed-step physics pipeline.
//
// Headless core: no UI or platform dependencies. The editor, CLI, importers and (future) language
// bindings all drive the simulation exclusively through this API.

#include "robotics/body.hpp"
#include "robotics/contact.hpp"
#include "robotics/controller.hpp"
#include "robotics/joint.hpp"
#include "robotics/math.hpp"
#include "robotics/sensors/sensor.hpp"

#include <vector>

namespace robotics {

class World {
public:
    static constexpr double defaultTimeStep = 1.0 / 240.0;
    static constexpr double maxTimeStep = 1.0 / 30.0;
    static constexpr int minIterations = 1;
    static constexpr int maxIterations = 200;
    static constexpr int minSubsteps = 1;
    static constexpr int maxSubsteps = 64;

    std::vector<Body> bodies;
    std::vector<Joint> joints;
    Vec3 gravity{0, -9.81, 0};
    // Sequential-impulse velocity passes per step, spread over the substeps: every substep runs
    // ceil(iterations / substeps) passes with position correction plus one relax pass without it.
    int iterations{24};
    // Substeps per step (TGS-style): forces, constraint solve and integration repeat with h = dt /
    // substeps; collision detection runs once per step. More substeps = stiffer joints and contacts.
    int substeps{4};
    // Start every step (and substep) from the previous accumulated impulses of matching contacts and
    // joints. Disable only for experiments.
    bool warmStarting{true};
    // Continuous collision detection: bodies with Body::ccd, or moving more than a quarter of their smallest
    // extent per step, are swept over the step and stop at the moment of impact instead of tunneling.
    bool continuousCollision{true};
    double time{};

    // Contacts of the last step() with their accumulated impulses; see Contact.
    std::vector<Contact> contacts;

    // Sensors mounted on bodies (configuration + runtime state). step() ignores them; they are advanced by
    // robotics::updateSensors() / stepSimulation() (robotics/sensors/sensor_system.hpp). Stored here so that
    // copies, removeBody() and the .robot format keep them in sync with the bodies.
    std::vector<Sensor> sensors;

    // Adds a validated body (orientation is normalized) and returns its index.
    int addBody(Body body);

    // Connects bodies a (parent) and b (child) at a world-space anchor; the axis is the hinge rotation
    // axis or the prismatic slide axis. The current pose becomes the joint's zero (Joint::angle = 0).
    // Returns the joint index; configure the joint afterwards through `joints[index]`, e.g.
    //   Joint& j = world.joints[world.connect(base, link, JointType::Prismatic, anchor, axis)];
    //   j.setLimits(0, 0.5); j.maxTorque = 100; // URDF <limit lower upper effort>
    // Ball and fixed joints ignore the axis.
    int connect(int a, int b, JointType type, Vec3 worldAnchor, Vec3 worldAxis = {0, 0, 1});

    // Removes a body and its joints; joint body indices above `index` are shifted down.
    void removeBody(int index);

    // Throws std::runtime_error describing the first invalid setting.
    void validate() const;

    // Advances by dt in (0, maxTimeStep] seconds:
    // encoders -> controllers -> collision detection (broad + narrow phase, continuous collision,
    // warm-start matching)
    // -> substeps [forces + gyroscopic term -> warm start -> constraint passes -> integration
    // -> relax pass] -> restitution -> encoders.
    void step(double dt = defaultTimeStep);

    // Largest distance between the two world-space anchors of any joint (constraint drift, m).
    // For prismatic joints only the component perpendicular to the slide axis counts.
    [[nodiscard]] double maxAnchorError() const;

    // Clears integrators/history, preserves encoder calibration.
    void resetControllers();
};

} // namespace robotics

// Forward and inverse kinematics, the Jacobian and trajectory generation.

#include "robotics/kinematics/kinematics.hpp"
#include "robotics/kinematics/trajectory.hpp"
#include "test_harness.hpp"

#include <algorithm>
#include <cmath>
#include <random>

using namespace robotics;
using test::check;
using test::near;

namespace {

// A serial arm along +X: link i is `length` long and hangs from a hinge or slider about the given world axis.
struct Arm {
    World world;
    std::vector<int> joints;
    int tip{};
};

Arm makeArm(const std::vector<Vec3>& axes, const std::vector<JointType>& types, double length = 0.5) {
    Arm arm;
    Body base;
    base.mass = 0;
    base.size = {0.2, 0.2, 0.2};
    base.position = {0, 1, 0};
    int previous = arm.world.addBody(base);
    double x = 0;
    for (std::size_t i = 0; i < axes.size(); ++i) {
        Body link;
        link.size = {length, 0.06, 0.06};
        link.mass = 0.5;
        link.position = {x + length / 2, 1, 0};
        int index = arm.world.addBody(link);
        int joint = arm.world.connect(previous, index, types[i], {x, 1, 0}, axes[i]);
        arm.world.joints[static_cast<std::size_t>(joint)].maxTorque = 50;
        arm.joints.push_back(joint);
        previous = index;
        x += length;
    }
    arm.tip = previous;
    return arm;
}

Arm threeDof() {
    return makeArm({{0, 1, 0}, {0, 0, 1}, {0, 0, 1}}, {JointType::Hinge, JointType::Hinge, JointType::Hinge});
}

Arm sixDof() {
    return makeArm({{0, 1, 0}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0}, {0, 0, 1}, {1, 0, 0}}, std::vector<JointType>(6, JointType::Hinge), 0.3);
}

double distance(Vec3 a, Vec3 b) {
    return (a - b).length();
}

} // namespace

TEST_CASE("kinematics: the chain follows the joint graph from the root to the tip") {
    Arm arm = threeDof();
    Chain chain = makeChain(arm.world, arm.tip, {0.25, 0, 0});
    check(chain.root == 0 && chain.tip == arm.tip, "Root and tip");
    check(chain.joints == arm.joints && chain.dofs() == 3, "Joints in order");
    // A fixed joint is part of the chain without a coordinate.
    Body extra;
    extra.size = {0.1, 0.1, 0.1};
    extra.position = {2, 1, 0};
    int extraBody = arm.world.addBody(extra);
    arm.world.connect(arm.tip, extraBody, JointType::Fixed, {1.5, 1, 0});
    Chain longer = makeChain(arm.world, extraBody);
    check(longer.joints.size() == 4 && longer.dofs() == 3, "Fixed joint adds no coordinate");
    test::rejects([&] { (void)makeChain(arm.world, 99); });
    int ball = arm.world.addBody(extra);
    arm.world.connect(extraBody, ball, JointType::Ball, {2, 1, 0});
    test::rejects([&] { (void)makeChain(arm.world, ball); });
}

TEST_CASE("kinematics: FK at the connect pose reproduces the body pose; zero chain gives the root pose") {
    Arm arm = sixDof();
    Chain chain = makeChain(arm.world, arm.tip, {0.15, 0, 0});
    Pose tip = forwardKinematics(arm.world, chain, std::vector<double>(6, 0.0));
    const Body& body = arm.world.bodies[static_cast<std::size_t>(arm.tip)];
    check(distance(tip.position, body.point({0.15, 0, 0})) < 1e-12, "Tip position at q = 0");
    check(std::abs(tip.orientation.w * body.orientation.w + tip.orientation.x * body.orientation.x
                   + tip.orientation.y * body.orientation.y + tip.orientation.z * body.orientation.z) > 1 - 1e-12,
          "Tip orientation at q = 0");
    Chain none = makeChain(arm.world, 0);
    check(none.dofs() == 0 && distance(forwardKinematics(arm.world, none, {}).position, arm.world.bodies[0].position) < 1e-12, "Empty chain");
}

TEST_CASE("kinematics: FK agrees with the simulated arm while the joints move") {
    Arm arm = threeDof();
    arm.world.gravity = {0, 0, 0};
    // Drive the joints with PID controllers to other angles, then compare the encoder-based FK with the real pose.
    double targets[3] = {0.7, 0.5, -0.9};
    for (std::size_t i = 0; i < 3; ++i) {
        Joint& joint = arm.world.joints[static_cast<std::size_t>(arm.joints[i])];
        joint.controller.type = ControllerType::PID;
        joint.controller.target = targets[i];
        joint.controller.kp = 40;
        joint.controller.ki = 1;
        joint.controller.kd = 8;
    }
    for (int i = 0; i < 240 * 4; ++i) {
        arm.world.step();
    }
    Chain chain = makeChain(arm.world, arm.tip, {0.25, 0, 0});
    std::vector<double> q = chainCoordinates(arm.world, chain);
    near(q[0], 0.7, 0.05, "Yaw reached its target");
    Pose fk = forwardKinematics(arm.world, chain, q);
    const Body& tip = arm.world.bodies[static_cast<std::size_t>(arm.tip)];
    check(distance(fk.position, tip.point({0.25, 0, 0})) < 2e-4, "FK matches the simulated tip position");
}

TEST_CASE("kinematics: a prismatic joint extends along its axis") {
    Arm arm = makeArm({{0, 1, 0}, {1, 0, 0}}, {JointType::Hinge, JointType::Prismatic});
    Chain chain = makeChain(arm.world, arm.tip);
    Vec3 p0 = forwardKinematics(arm.world, chain, {0.0, 0.0}).position;
    Vec3 p1 = forwardKinematics(arm.world, chain, {0.0, 0.2}).position;
    check(distance(p1 - p0, {0.2, 0, 0}) < 1e-12, "Slider moves the tip along +X");
    Vec3 rotated = forwardKinematics(arm.world, chain, {pi / 2, 0.2}).position;
    check(distance(rotated - forwardKinematics(arm.world, chain, {pi / 2, 0.0}).position, {0, 0, -0.2}) < 1e-12, "Yaw turns the slide axis");
}

TEST_CASE("kinematics: the Jacobian matches finite differences of FK") {
    Arm arm = sixDof();
    Chain chain = makeChain(arm.world, arm.tip, {0.15, 0, 0});
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> angle(-1.0, 1.0);
    for (int trial = 0; trial < 20; ++trial) {
        std::vector<double> q(6);
        for (double& v : q) {
            v = angle(rng);
        }
        auto columns = jacobian(arm.world, chain, q);
        Pose base = forwardKinematics(arm.world, chain, q);
        for (std::size_t k = 0; k < 6; ++k) {
            std::vector<double> shifted = q;
            const double h = 1e-6;
            shifted[k] += h;
            Pose moved = forwardKinematics(arm.world, chain, shifted);
            Vec3 linear = (moved.position - base.position) / h;
            Vec3 angular = rotationVector(moved.orientation * base.orientation.conjugate()) / h;
            check(distance(linear, {columns[k][0], columns[k][1], columns[k][2]}) < 1e-5, "Linear column");
            check(distance(angular, {columns[k][3], columns[k][4], columns[k][5]}) < 1e-5, "Angular column");
        }
    }
}

TEST_CASE("kinematics: IK reaches random reachable targets for a 3-DOF arm and a 6-DOF pose") {
    {
        Arm arm = threeDof();
        Chain chain = makeChain(arm.world, arm.tip, {0.25, 0, 0});
        std::mt19937 rng(11);
        std::uniform_real_distribution<double> angle(-1.2, 1.2);
        int solved = 0;
        for (int trial = 0; trial < 40; ++trial) {
            std::vector<double> truth{angle(rng), angle(rng), angle(rng)};
            Vec3 target = forwardKinematics(arm.world, chain, truth).position;
            IkResult r = solveIk(arm.world, chain, target, std::nullopt, {0.1, 0.1, 0.1});
            if (r.converged && distance(forwardKinematics(arm.world, chain, r.q).position, target) < 2e-4) {
                ++solved;
            }
        }
        check(solved >= 38, "At least 95% of random reachable targets are solved");
    }
    {
        Arm arm = sixDof();
        Chain chain = makeChain(arm.world, arm.tip, {0.15, 0, 0});
        std::mt19937 rng(3);
        std::uniform_real_distribution<double> angle(-0.8, 0.8);
        int solved = 0;
        for (int trial = 0; trial < 30; ++trial) {
            std::vector<double> truth(6);
            for (double& v : truth) {
                v = angle(rng);
            }
            Pose target = forwardKinematics(arm.world, chain, truth);
            IkResult r = solveIk(arm.world, chain, target.position, target.orientation, std::vector<double>(6, 0.05));
            if (r.converged) {
                Pose reached = forwardKinematics(arm.world, chain, r.q);
                check(distance(reached.position, target.position) < 2e-4, "Position of the pose solution");
                check(r.orientationError < 2e-3, "Orientation of the pose solution");
                ++solved;
            }
        }
        check(solved >= 24, "Most reachable full poses are solved");
    }
}

TEST_CASE("kinematics: IK rejects unreachable targets and respects joint limits") {
    Arm arm = threeDof();
    Chain chain = makeChain(arm.world, arm.tip, {0.25, 0, 0});
    IkResult far = solveIk(arm.world, chain, {5, 1, 0}, std::nullopt, {0, 0, 0});
    check(!far.converged && far.positionError > 3, "A target beyond the reach is reported, not faked");
    // A limit on the elbow blocks a target that needs it.
    arm.world.joints[static_cast<std::size_t>(arm.joints[2])].setLimits(-0.1, 0.1);
    arm.world.joints[static_cast<std::size_t>(arm.joints[1])].setLimits(-0.1, 0.1);
    Vec3 high = forwardKinematics(arm.world, chain, {0.0, 1.0, 1.0}).position;
    IkResult blocked = solveIk(arm.world, chain, high, std::nullopt, {0, 0, 0});
    check(!blocked.converged, "Limits block the target");
    check(std::abs(blocked.q[1]) <= 0.1 + 1e-12 && std::abs(blocked.q[2]) <= 0.1 + 1e-12, "Coordinates stay inside the limits");
    IkOptions free;
    free.respectLimits = false;
    check(solveIk(arm.world, chain, high, std::nullopt, {0, 0, 0}, free).converged, "Without limits the target is reachable");
    test::rejects([&] { (void)solveIk(arm.world, chain, {1, 1, 0}, std::nullopt, {0, 0}); });
}

TEST_CASE("trajectory: minimum-jerk moves are rest-to-rest and respect the limits") {
    JointLimits limits{{1.0, 2.0}, {4.0, 8.0}};
    std::vector<double> from{0.0, 1.0}, to{1.5, -0.5};
    double T = minJerkDuration(from, to, limits);
    check(T > 0, "Positive duration");
    Trajectory trajectory = planPointToPoint(from, to, limits, 0.001);
    check(trajectory.duration() >= T - 1e-9 && trajectory.duration() < T + 0.002, "Duration is the limit-based minimum");
    for (std::size_t k = 0; k < 2; ++k) {
        near(trajectory.position.front()[k], from[k], 1e-12, "Start");
        near(trajectory.position.back()[k], to[k], 1e-9, "End");
        near(trajectory.velocity.front()[k], 0, 1e-9, "Starts at rest");
        near(trajectory.velocity.back()[k], 0, 1e-9, "Ends at rest");
        near(trajectory.acceleration.front()[k], 0, 1e-9, "Zero start acceleration");
    }
    double peakVelocity = 0, peakAcceleration = 0;
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        peakVelocity = std::max(peakVelocity, std::abs(trajectory.velocity[i][0]));
        peakAcceleration = std::max(peakAcceleration, std::abs(trajectory.acceleration[i][0]));
    }
    check(peakVelocity <= limits.maxVelocity[0] * 1.001 && peakAcceleration <= limits.maxAcceleration[0] * 1.001, "Limits hold");
    // The reported velocity is the derivative of the position.
    double numeric = (trajectory.position[501][0] - trajectory.position[499][0]) / (2 * trajectory.dt);
    near(numeric, trajectory.velocity[500][0], 1e-3, "Velocity is d position / dt");
    // Interpolated sample.
    Trajectory::Sample middle = trajectory.sample(trajectory.duration() / 2);
    near(middle.position[0], 0.75, 1e-3, "Halfway in time is halfway in space for minimum jerk");
    near(trajectory.sample(-1).position[0], from[0], 1e-12, "Clamped before the start");
    near(trajectory.sample(1e3).position[0], to[0], 1e-9, "Clamped after the end");
    check(planPointToPoint(from, from, limits).size() == 1, "No move gives a single sample");
    test::rejects([&] { (void)planPointToPoint(from, {1.0}, limits); });
    test::rejects([&] { (void)planPointToPoint(from, to, {{0.0, 1.0}, {1.0, 1.0}}); });
}

TEST_CASE("trajectory: waypoints are visited in order with a stop at each one") {
    JointLimits limits{{1.0}, {5.0}};
    Trajectory t = planWaypoints({{0.0}, {1.0}, {0.5}, {2.0}}, limits, 0.002);
    int stops = 0;
    for (std::size_t i = 1; i + 1 < t.size(); ++i) {
        if (std::abs(t.velocity[i][0]) < 1e-9) {
            ++stops;
        }
    }
    check(stops == 2, "Two interior waypoints with zero velocity");
    near(t.position.back()[0], 2.0, 1e-9, "Ends at the last waypoint");
    for (std::size_t i = 1; i < t.size(); ++i) {
        check(std::abs(t.position[i][0] - t.position[i - 1][0]) < 0.01, "Continuous path (no jump at the junctions)");
    }
}

TEST_CASE("trajectory: a Cartesian line stays straight, reaches the target and respects the limits") {
    Arm arm = threeDof();
    Chain chain = makeChain(arm.world, arm.tip, {0.25, 0, 0});
    std::vector<double> q0{0.3, 0.4, 0.5};
    Vec3 start = forwardKinematics(arm.world, chain, q0).position;
    Vec3 target = forwardKinematics(arm.world, chain, {-0.4, 0.9, 0.2}).position;
    JointLimits limits{{2.0, 2.0, 2.0}, {10.0, 10.0, 10.0}};
    auto trajectory = planCartesianLine(arm.world, chain, q0, target, limits, 100, 0.002);
    check(trajectory.has_value(), "The line is feasible");
    Vec3 direction = (target - start).normalized();
    double worst = 0, peakVelocity = 0, peakAcceleration = 0;
    for (std::size_t i = 0; i < trajectory->size(); ++i) {
        Vec3 p = forwardKinematics(arm.world, chain, trajectory->position[i]).position;
        Vec3 offset = p - start;
        worst = std::max(worst, (offset - direction * offset.dot(direction)).length());
        for (std::size_t k = 0; k < 3; ++k) {
            peakVelocity = std::max(peakVelocity, std::abs(trajectory->velocity[i][k]));
            peakAcceleration = std::max(peakAcceleration, std::abs(trajectory->acceleration[i][k]));
        }
    }
    check(worst < 2e-3, "The tip stays within 2 mm of the straight line");
    check(distance(forwardKinematics(arm.world, chain, trajectory->position.back()).position, target) < 5e-4, "Reaches the target");
    check(peakVelocity <= 2.0 * 1.05 && peakAcceleration <= 10.0 * 1.15, "Joint limits hold along the path");
    // An unreachable target gives no trajectory.
    check(!planCartesianLine(arm.world, chain, q0, {4, 1, 0}, limits).has_value(), "Out of reach");
}

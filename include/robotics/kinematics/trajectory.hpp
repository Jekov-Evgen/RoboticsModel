#pragma once
// Joint-space trajectory generation: minimum-jerk point-to-point moves timed by velocity and acceleration limits,
// rest-to-rest waypoint sequences, and straight-line Cartesian paths computed with inverse kinematics.
// Collision-free planning around obstacles is not part of this module.

#include "robotics/kinematics/kinematics.hpp"

#include <optional>
#include <vector>

namespace robotics {

struct JointLimits {
    std::vector<double> maxVelocity;     // per coordinate; rad/s or m/s
    std::vector<double> maxAcceleration; // per coordinate
};

// A trajectory sampled at a fixed period; sample() interpolates linearly between samples.
struct Trajectory {
    struct Sample {
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> acceleration;
    };

    double dt{0.002};
    std::vector<std::vector<double>> position; // [sample][coordinate]
    std::vector<std::vector<double>> velocity;
    std::vector<std::vector<double>> acceleration;

    [[nodiscard]] double duration() const { return position.empty() ? 0.0 : dt * static_cast<double>(position.size() - 1); }
    [[nodiscard]] std::size_t size() const { return position.size(); }
    // Clamped to [0, duration()].
    [[nodiscard]] Sample sample(double time) const;
};

// Shortest duration of a minimum-jerk rest-to-rest move that respects the limits (peak velocity 15/8 * delta / T,
// peak acceleration 10/sqrt(3) * delta / T^2).
[[nodiscard]] double minJerkDuration(const std::vector<double>& from, const std::vector<double>& to,
                                     const JointLimits& limits);

// Rest-to-rest minimum-jerk move; all coordinates share one duration so the path in joint space is straight.
[[nodiscard]] Trajectory planPointToPoint(const std::vector<double>& from, const std::vector<double>& to,
                                          const JointLimits& limits, double dt = 0.002);

// Stops at every waypoint (zero velocity and acceleration), each segment timed by the limits.
[[nodiscard]] Trajectory planWaypoints(const std::vector<std::vector<double>>& waypoints, const JointLimits& limits,
                                       double dt = 0.002);

// Moves the controlled point along a straight line in space to `target`, from the pose at `qStart`, keeping the
// tip orientation. The path is solved with IK at `pathSamples` points; returns nullopt when IK fails along it (a
// target out of reach or a limit blocks the line). Timing satisfies the limits for the sampled path.
[[nodiscard]] std::optional<Trajectory> planCartesianLine(const World& world, const Chain& chain,
                                                          const std::vector<double>& qStart, Vec3 target,
                                                          const JointLimits& limits, int pathSamples = 100,
                                                          double dt = 0.002, const IkOptions& ik = {});

} // namespace robotics

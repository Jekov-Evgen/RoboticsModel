#include "robotics/kinematics/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robotics {
namespace {

constexpr double jerkVelocityFactor = 15.0 / 8.0;      // peak of d s / d tau for s = 10 t^3 - 15 t^4 + 6 t^5
constexpr double jerkAccelerationFactor = 5.773502691896258; // 10 / sqrt(3): peak of d^2 s / d tau^2

struct Blend {
    double s;
    double ds;  // per unit of T
    double dds; // per unit of T^2
};

Blend minJerk(double tau) {
    tau = std::clamp(tau, 0.0, 1.0);
    double t2 = tau * tau;
    double t3 = t2 * tau;
    return {10 * t3 - 15 * t3 * tau + 6 * t3 * t2, 30 * t2 - 60 * t3 + 30 * t3 * tau, 60 * tau - 180 * t2 + 120 * t3};
}

void checkLimits(const JointLimits& limits, std::size_t n) {
    if (limits.maxVelocity.size() != n || limits.maxAcceleration.size() != n) {
        throw std::runtime_error("Limits need one velocity and one acceleration value per coordinate");
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (!(limits.maxVelocity[i] > 0) || !(limits.maxAcceleration[i] > 0)) {
            throw std::runtime_error("Velocity and acceleration limits must be positive");
        }
    }
}

void checkDt(double dt) {
    if (!(dt > 0) || !std::isfinite(dt)) {
        throw std::runtime_error("Trajectory sample period must be positive");
    }
}

} // namespace

Trajectory::Sample Trajectory::sample(double time) const {
    if (position.empty()) {
        throw std::runtime_error("Empty trajectory");
    }
    double t = std::clamp(time, 0.0, duration()) / dt;
    std::size_t i = std::min(static_cast<std::size_t>(t), position.size() - 1);
    std::size_t j = std::min(i + 1, position.size() - 1);
    double f = t - static_cast<double>(i);
    Sample out;
    auto lerp = [&](const std::vector<std::vector<double>>& data, std::vector<double>& dst) {
        dst.resize(data[i].size());
        for (std::size_t k = 0; k < dst.size(); ++k) {
            dst[k] = data[i][k] + (data[j][k] - data[i][k]) * f;
        }
    };
    lerp(position, out.position);
    lerp(velocity, out.velocity);
    lerp(acceleration, out.acceleration);
    return out;
}

double minJerkDuration(const std::vector<double>& from, const std::vector<double>& to, const JointLimits& limits) {
    if (from.size() != to.size()) {
        throw std::runtime_error("Start and end must have the same number of coordinates");
    }
    checkLimits(limits, from.size());
    double duration = 0;
    for (std::size_t i = 0; i < from.size(); ++i) {
        double delta = std::abs(to[i] - from[i]);
        duration = std::max({duration, jerkVelocityFactor * delta / limits.maxVelocity[i],
                             std::sqrt(jerkAccelerationFactor * delta / limits.maxAcceleration[i])});
    }
    return duration;
}

Trajectory planPointToPoint(const std::vector<double>& from, const std::vector<double>& to, const JointLimits& limits, double dt) {
    checkDt(dt);
    double duration = minJerkDuration(from, to, limits);
    std::size_t n = from.size();
    Trajectory trajectory;
    trajectory.dt = dt;
    std::size_t steps = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(duration / dt - 1e-9)));
    double total = static_cast<double>(steps) * dt;
    if (duration <= 0) {
        steps = 0;
        total = 1.0;
    }
    for (std::size_t k = 0; k <= steps; ++k) {
        Blend b = minJerk(static_cast<double>(k) / static_cast<double>(std::max<std::size_t>(steps, 1)));
        std::vector<double> q(n), qd(n), qdd(n);
        for (std::size_t i = 0; i < n; ++i) {
            double delta = to[i] - from[i];
            q[i] = from[i] + delta * b.s;
            qd[i] = delta * b.ds / total;
            qdd[i] = delta * b.dds / (total * total);
        }
        trajectory.position.push_back(std::move(q));
        trajectory.velocity.push_back(std::move(qd));
        trajectory.acceleration.push_back(std::move(qdd));
    }
    return trajectory;
}

Trajectory planWaypoints(const std::vector<std::vector<double>>& waypoints, const JointLimits& limits, double dt) {
    if (waypoints.size() < 2) {
        throw std::runtime_error("Need at least two waypoints");
    }
    Trajectory all;
    all.dt = dt;
    for (std::size_t w = 0; w + 1 < waypoints.size(); ++w) {
        Trajectory segment = planPointToPoint(waypoints[w], waypoints[w + 1], limits, dt);
        std::size_t first = w == 0 ? 0 : 1; // The end of one segment is the start of the next.
        for (std::size_t k = first; k < segment.size(); ++k) {
            all.position.push_back(std::move(segment.position[k]));
            all.velocity.push_back(std::move(segment.velocity[k]));
            all.acceleration.push_back(std::move(segment.acceleration[k]));
        }
    }
    return all;
}

std::optional<Trajectory> planCartesianLine(const World& world, const Chain& chain, const std::vector<double>& qStart, Vec3 target,
                                            const JointLimits& limits, int pathSamples, double dt, const IkOptions& ik) {
    checkDt(dt);
    std::size_t n = chain.dofs();
    checkLimits(limits, n);
    if (qStart.size() != n || pathSamples < 4) {
        throw std::runtime_error("Invalid start coordinates or too few path samples");
    }
    Vec3 start = forwardKinematics(world, chain, qStart).position;
    std::vector<std::vector<double>> path{qStart};
    for (int i = 1; i <= pathSamples; ++i) {
        double s = static_cast<double>(i) / pathSamples;
        IkResult r = solveIk(world, chain, start + (target - start) * s, std::nullopt, path.back(), ik);
        if (!r.converged) {
            return std::nullopt;
        }
        path.push_back(std::move(r.q));
    }
    // Derivatives of the joint path with respect to the path parameter s in [0, 1] (central differences).
    std::size_t m = path.size();
    double h = 1.0 / pathSamples;
    std::vector<std::vector<double>> d1(m, std::vector<double>(n, 0.0)), d2(m, std::vector<double>(n, 0.0));
    for (std::size_t i = 0; i < m; ++i) {
        std::size_t lo = i == 0 ? 0 : i - 1;
        std::size_t hi = std::min(i + 1, m - 1);
        std::size_t mid = i;
        for (std::size_t k = 0; k < n; ++k) {
            d1[i][k] = (path[hi][k] - path[lo][k]) / (static_cast<double>(hi - lo) * h);
            double forward = i + 1 < m ? path[i + 1][k] : 2 * path[i][k] - path[i - 1][k];
            double backward = i > 0 ? path[i - 1][k] : 2 * path[i][k] - path[i + 1][k];
            d2[mid][k] = (forward - 2 * path[i][k] + backward) / (h * h);
        }
    }
    double duration = 0;
    for (std::size_t i = 0; i < m; ++i) {
        for (std::size_t k = 0; k < n; ++k) {
            duration = std::max(duration, jerkVelocityFactor * std::abs(d1[i][k]) / limits.maxVelocity[k]);
            double accel = std::abs(d2[i][k]) * jerkVelocityFactor * jerkVelocityFactor + std::abs(d1[i][k]) * jerkAccelerationFactor;
            duration = std::max(duration, std::sqrt(accel / limits.maxAcceleration[k]));
        }
    }
    Trajectory trajectory;
    trajectory.dt = dt;
    if (duration <= 0) {
        trajectory.position.push_back(qStart);
        trajectory.velocity.emplace_back(n, 0.0);
        trajectory.acceleration.emplace_back(n, 0.0);
        return trajectory;
    }
    std::size_t steps = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(duration / dt - 1e-9)));
    double total = static_cast<double>(steps) * dt;
    auto interpolate = [&](const std::vector<std::vector<double>>& data, double s, std::size_t k) {
        double position = s * pathSamples;
        std::size_t i = std::min(static_cast<std::size_t>(position), m - 2);
        double f = position - static_cast<double>(i);
        return data[i][k] + (data[i + 1][k] - data[i][k]) * f;
    };
    for (std::size_t step = 0; step <= steps; ++step) {
        Blend b = minJerk(static_cast<double>(step) / static_cast<double>(steps));
        std::vector<double> q(n), qd(n), qdd(n);
        for (std::size_t k = 0; k < n; ++k) {
            double sDot = b.ds / total;
            double sDdot = b.dds / (total * total);
            q[k] = interpolate(path, b.s, k);
            double slope = interpolate(d1, b.s, k);
            double curvature = interpolate(d2, b.s, k);
            qd[k] = slope * sDot;
            qdd[k] = curvature * sDot * sDot + slope * sDdot;
        }
        trajectory.position.push_back(std::move(q));
        trajectory.velocity.push_back(std::move(qd));
        trajectory.acceleration.push_back(std::move(qdd));
    }
    return trajectory;
}

} // namespace robotics

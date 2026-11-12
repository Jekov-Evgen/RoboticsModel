#include "robotics/kinematics/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace robotics {
namespace {

struct JointFrame {
    Vec3 origin; // World-space anchor of the joint on the parent body.
    Vec3 axis;   // World-space joint axis.
};

struct Propagated {
    Pose tip;
    std::vector<JointFrame> frames; // One per actuated joint, in chain order.
};

const Joint& jointAt(const World& world, int index) {
    return world.joints[static_cast<std::size_t>(index)];
}

Propagated propagate(const World& world, const Chain& chain, const std::vector<double>& q) {
    if (q.size() != chain.dofs()) {
        throw std::runtime_error("Expected one coordinate per actuated joint of the chain");
    }
    const Body& root = world.bodies[static_cast<std::size_t>(chain.root)];
    Pose pose{root.position, root.orientation};
    Propagated out;
    std::size_t k = 0;
    for (int index : chain.joints) {
        const Joint& joint = jointAt(world, index);
        Vec3 axis = pose.orientation.rotate(joint.axisA);
        Vec3 anchor = pose.position + pose.orientation.rotate(joint.anchorA);
        Quat orientation = pose.orientation * joint.reference;
        Vec3 slide{};
        if (joint.isActuated()) {
            double coordinate = q[k++];
            out.frames.push_back({anchor, axis});
            if (joint.type == JointType::Hinge) {
                orientation = pose.orientation * Quat::axisAngle(joint.axisA, coordinate) * joint.reference;
            } else {
                slide = axis * coordinate;
            }
        }
        orientation = orientation.normalized();
        pose = {anchor + slide - orientation.rotate(joint.anchorB), orientation};
    }
    out.tip = {pose.position + pose.orientation.rotate(chain.tipPoint), pose.orientation};
    return out;
}

// Solves the n x n system A x = b in place (partial pivoting); A is row-major.
std::vector<double> solveLinear(std::vector<double> a, std::vector<double> b, std::size_t n) {
    for (std::size_t col = 0; col < n; ++col) {
        std::size_t pivot = col;
        for (std::size_t r = col + 1; r < n; ++r) {
            if (std::abs(a[r * n + col]) > std::abs(a[pivot * n + col])) {
                pivot = r;
            }
        }
        if (std::abs(a[pivot * n + col]) < 1e-14) {
            throw std::runtime_error("Singular system in inverse kinematics");
        }
        if (pivot != col) {
            for (std::size_t c = 0; c < n; ++c) {
                std::swap(a[col * n + c], a[pivot * n + c]);
            }
            std::swap(b[col], b[pivot]);
        }
        for (std::size_t r = col + 1; r < n; ++r) {
            double f = a[r * n + col] / a[col * n + col];
            for (std::size_t c = col; c < n; ++c) {
                a[r * n + c] -= f * a[col * n + c];
            }
            b[r] -= f * b[col];
        }
    }
    for (std::size_t i = n; i-- > 0;) {
        for (std::size_t c = i + 1; c < n; ++c) {
            b[i] -= a[i * n + c] * b[c];
        }
        b[i] /= a[i * n + i];
    }
    return b;
}

} // namespace

Chain makeChain(const World& world, int tipBody, Vec3 tipPoint) {
    if (tipBody < 0 || static_cast<std::size_t>(tipBody) >= world.bodies.size()) {
        throw std::runtime_error("Invalid tip body index");
    }
    Chain chain;
    chain.tip = tipBody;
    chain.tipPoint = tipPoint;
    std::set<int> seen{tipBody};
    int body = tipBody;
    while (true) {
        int found = -1;
        for (std::size_t i = 0; i < world.joints.size(); ++i) {
            if (world.joints[i].b == body) {
                if (found >= 0) {
                    throw std::runtime_error("Body has several parent joints (closed loop): not a serial chain");
                }
                found = static_cast<int>(i);
            }
        }
        if (found < 0) {
            break;
        }
        const Joint& joint = jointAt(world, found);
        if (joint.type == JointType::Ball) {
            throw std::runtime_error("Ball joints are not supported in a kinematic chain");
        }
        if (!seen.insert(joint.a).second) {
            throw std::runtime_error("Joint graph contains a loop");
        }
        chain.joints.push_back(found);
        body = joint.a;
    }
    chain.root = body;
    std::reverse(chain.joints.begin(), chain.joints.end());
    for (int index : chain.joints) {
        if (jointAt(world, index).isActuated()) {
            chain.actuated.push_back(index);
        }
    }
    return chain;
}

std::vector<double> chainCoordinates(const World& world, const Chain& chain) {
    std::vector<double> q;
    for (int index : chain.actuated) {
        q.push_back(jointAt(world, index).angle);
    }
    return q;
}

Pose forwardKinematics(const World& world, const Chain& chain, const std::vector<double>& q) {
    return propagate(world, chain, q).tip;
}

std::vector<std::array<double, 6>> jacobian(const World& world, const Chain& chain, const std::vector<double>& q) {
    Propagated p = propagate(world, chain, q);
    std::vector<std::array<double, 6>> columns;
    for (std::size_t k = 0; k < chain.actuated.size(); ++k) {
        const Joint& joint = jointAt(world, chain.actuated[k]);
        const JointFrame& frame = p.frames[k];
        Vec3 linear;
        Vec3 angular;
        if (joint.type == JointType::Hinge) {
            linear = frame.axis.cross(p.tip.position - frame.origin);
            angular = frame.axis;
        } else {
            linear = frame.axis;
        }
        columns.push_back({linear.x, linear.y, linear.z, angular.x, angular.y, angular.z});
    }
    return columns;
}

IkResult solveIk(const World& world, const Chain& chain, Vec3 targetPosition, const std::optional<Quat>& targetOrientation,
                 const std::vector<double>& seed, const IkOptions& options) {
    std::size_t n = chain.dofs();
    if (seed.size() != n) {
        throw std::runtime_error("Seed must have one coordinate per actuated joint");
    }
    if (!targetPosition.finite()) {
        throw std::runtime_error("Target position must be finite");
    }
    auto clampToLimits = [&](std::vector<double>& q) {
        if (!options.respectLimits) {
            return;
        }
        for (std::size_t i = 0; i < n; ++i) {
            const Joint& joint = jointAt(world, chain.actuated[i]);
            if (joint.limitsEnabled) {
                q[i] = std::clamp(q[i], joint.lowerLimit, joint.upperLimit);
            }
        }
    };
    IkResult result;
    result.q = seed;
    clampToLimits(result.q);
    std::size_t rows = targetOrientation ? 6 : 3;
    double lambda2 = options.damping * options.damping;

    for (int iteration = 0; iteration <= options.maxIterations; ++iteration) {
        Pose pose = forwardKinematics(world, chain, result.q);
        Vec3 positionError = targetPosition - pose.position;
        Vec3 orientationError = targetOrientation ? rotationVector(*targetOrientation * pose.orientation.conjugate()) : Vec3{};
        result.positionError = positionError.length();
        result.orientationError = orientationError.length();
        result.iterations = iteration;
        if (result.positionError < options.positionTolerance
            && (!targetOrientation || result.orientationError < options.orientationTolerance)) {
            result.converged = true;
            return result;
        }
        if (iteration == options.maxIterations || n == 0) {
            break;
        }
        // Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e.
        auto columns = jacobian(world, chain, result.q);
        std::vector<double> e{positionError.x, positionError.y, positionError.z};
        if (targetOrientation) {
            e.insert(e.end(), {orientationError.x, orientationError.y, orientationError.z});
        }
        std::vector<double> jjt(rows * rows, 0.0);
        for (std::size_t r = 0; r < rows; ++r) {
            for (std::size_t c = 0; c < rows; ++c) {
                double sum = 0;
                for (std::size_t k = 0; k < n; ++k) {
                    sum += columns[k][r] * columns[k][c];
                }
                jjt[r * rows + c] = sum + (r == c ? lambda2 : 0.0);
            }
        }
        std::vector<double> y = solveLinear(jjt, e, rows);
        std::vector<double> dq(n, 0.0);
        double largest = 0;
        for (std::size_t k = 0; k < n; ++k) {
            for (std::size_t r = 0; r < rows; ++r) {
                dq[k] += columns[k][r] * y[r];
            }
            largest = std::max(largest, std::abs(dq[k]));
        }
        double scale = largest > options.maxStep ? options.maxStep / largest : 1.0;
        std::vector<double> next = result.q;
        for (std::size_t k = 0; k < n; ++k) {
            next[k] += dq[k] * scale;
        }
        clampToLimits(next);
        double moved = 0;
        for (std::size_t k = 0; k < n; ++k) {
            moved = std::max(moved, std::abs(next[k] - result.q[k]));
        }
        result.q = next;
        if (moved < 1e-12) {
            break; // Stalled at a limit or a singularity.
        }
    }
    return result;
}

} // namespace robotics

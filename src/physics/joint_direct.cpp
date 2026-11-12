#include "joint_direct.hpp"

#include <algorithm>
#include <cmath>

namespace robotics::detail {
namespace {

// Fraction of the joint position error removed per substep by the (exact) position pass. Below 1 so
// the nonlinear error after one linearized projection does not overshoot.
constexpr double positionCorrectionFactor = 0.8;
// Clamp (m/s or rad/s) of each row's correction pseudo-velocity.
constexpr double maxCorrectionVelocity = 5.0;
// Relative diagonal regularization (constraint force mixing) for redundant rows.
constexpr double regularization = 1e-10;

constexpr std::array<Vec3, 3> worldBasis{Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}};

// In-place inverse of the leading n x n part of a row-major block (stride 6) by Gauss-Jordan elimination
// with partial pivoting. Rows with a vanishing pivot (dependent constraints) get a zero inverse.
template <class Block>
Block invert(const Block& input, int n) {
    constexpr int s = 6;
    Block a = input;
    Block inv{};
    for (int i = 0; i < n; ++i) {
        inv[static_cast<std::size_t>(i * s + i)] = 1;
    }
    double scale = 0;
    for (int i = 0; i < n; ++i) {
        scale = std::max(scale, std::abs(a[static_cast<std::size_t>(i * s + i)]));
    }
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int r = col + 1; r < n; ++r) {
            if (std::abs(a[static_cast<std::size_t>(r * s + col)])
                > std::abs(a[static_cast<std::size_t>(pivot * s + col)])) {
                pivot = r;
            }
        }
        if (!(std::abs(a[static_cast<std::size_t>(pivot * s + col)]) > 1e-12 * scale)) {
            for (int k = 0; k < n; ++k) {
                a[static_cast<std::size_t>(col * s + k)] = 0;
                inv[static_cast<std::size_t>(col * s + k)] = 0;
            }
            continue;
        }
        if (pivot != col) {
            for (int k = 0; k < n; ++k) {
                std::swap(a[static_cast<std::size_t>(col * s + k)],
                          a[static_cast<std::size_t>(pivot * s + k)]);
                std::swap(inv[static_cast<std::size_t>(col * s + k)],
                          inv[static_cast<std::size_t>(pivot * s + k)]);
            }
        }
        double d = 1 / a[static_cast<std::size_t>(col * s + col)];
        for (int k = 0; k < n; ++k) {
            a[static_cast<std::size_t>(col * s + k)] *= d;
            inv[static_cast<std::size_t>(col * s + k)] *= d;
        }
        for (int r = 0; r < n; ++r) {
            double f = a[static_cast<std::size_t>(r * s + col)];
            if (r == col || f == 0) {
                continue;
            }
            for (int k = 0; k < n; ++k) {
                a[static_cast<std::size_t>(r * s + k)] -= f * a[static_cast<std::size_t>(col * s + k)];
                inv[static_cast<std::size_t>(r * s + k)] -= f * inv[static_cast<std::size_t>(col * s + k)];
            }
        }
    }
    return inv;
}

// c (m x n) = a (m x p) * b (p x n), stride 6.
template <class Block>
Block multiply(const Block& a, const Block& b, int m, int p, int n) {
    Block c{};
    for (int i = 0; i < m; ++i) {
        for (int k = 0; k < p; ++k) {
            double f = a[static_cast<std::size_t>(i * 6 + k)];
            if (f == 0) {
                continue;
            }
            for (int j = 0; j < n; ++j) {
                c[static_cast<std::size_t>(i * 6 + j)] += f * b[static_cast<std::size_t>(k * 6 + j)];
            }
        }
    }
    return c;
}

// target (m x n) -= a (m x p) * bᵀ where b is (n x p), stride 6.
template <class Block>
void subtractProductTransposed(Block& target, const Block& a, const Block& b, int m, int p, int n) {
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            double sum = 0;
            for (int k = 0; k < p; ++k) {
                sum += a[static_cast<std::size_t>(i * 6 + k)] * b[static_cast<std::size_t>(j * 6 + k)];
            }
            target[static_cast<std::size_t>(i * 6 + j)] -= sum;
        }
    }
}

} // namespace

void DirectJointSolver::analyze(const std::vector<JointConstraint>& joints,
                                const std::vector<SolverBody>& bodies) {
    order_.clear();
    upper_.clear();
    edges_.clear();
    std::vector<int> active;
    for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
        const JointConstraint& c = joints[static_cast<std::size_t>(i)];
        if (bodies[static_cast<std::size_t>(c.a)].dynamic()
            || bodies[static_cast<std::size_t>(c.b)].dynamic()) {
            active.push_back(i);
        }
    }
    const int n = static_cast<int>(active.size());
    if (n == 0) {
        return;
    }
    // Joint graph: joints sharing a dynamic body are coupled.
    std::vector<std::vector<int>> atBody(bodies.size());
    for (int j = 0; j < n; ++j) {
        const JointConstraint& c = joints[static_cast<std::size_t>(active[static_cast<std::size_t>(j)])];
        for (int body : {c.a, c.b}) {
            if (bodies[static_cast<std::size_t>(body)].dynamic()) {
                atBody[static_cast<std::size_t>(body)].push_back(j);
            }
        }
    }
    std::vector<std::vector<int>> adjacent(static_cast<std::size_t>(n));
    for (const auto& list : atBody) {
        for (int j : list) {
            for (int k : list) {
                if (j != k) {
                    adjacent[static_cast<std::size_t>(j)].push_back(k);
                }
            }
        }
    }
    for (auto& list : adjacent) {
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end()), list.end());
    }
    // Greedy minimum degree elimination with symbolic fill (deterministic tie-break by index).
    std::vector<char> eliminated(static_cast<std::size_t>(n), 0);
    std::vector<int> position(static_cast<std::size_t>(n), -1);
    std::vector<std::vector<int>> later(static_cast<std::size_t>(n)); // By joint id.
    for (int step = 0; step < n; ++step) {
        int best = -1;
        std::size_t bestDegree = 0;
        for (int j = 0; j < n; ++j) {
            if (!eliminated[static_cast<std::size_t>(j)]
                && (best < 0 || adjacent[static_cast<std::size_t>(j)].size() < bestDegree)) {
                best = j;
                bestDegree = adjacent[static_cast<std::size_t>(j)].size();
            }
        }
        eliminated[static_cast<std::size_t>(best)] = 1;
        position[static_cast<std::size_t>(best)] = step;
        const std::vector<int> neighbors = adjacent[static_cast<std::size_t>(best)];
        later[static_cast<std::size_t>(best)] = neighbors;
        for (int k : neighbors) {
            auto& list = adjacent[static_cast<std::size_t>(k)];
            list.erase(std::remove(list.begin(), list.end(), best), list.end());
            for (int l : neighbors) {
                if (l != k && !std::binary_search(list.begin(), list.end(), l)) {
                    list.insert(std::lower_bound(list.begin(), list.end(), l), l); // Fill.
                }
            }
        }
    }
    order_.resize(static_cast<std::size_t>(n));
    upper_.assign(static_cast<std::size_t>(n), {});
    for (int j = 0; j < n; ++j) {
        order_[static_cast<std::size_t>(position[static_cast<std::size_t>(j)])] =
            active[static_cast<std::size_t>(j)];
    }
    for (int j = 0; j < n; ++j) {
        int p = position[static_cast<std::size_t>(j)];
        for (int k : later[static_cast<std::size_t>(j)]) {
            upper_[static_cast<std::size_t>(p)].push_back({position[static_cast<std::size_t>(k)], 0});
        }
    }
    for (int p = 0; p < n; ++p) {
        auto& list = upper_[static_cast<std::size_t>(p)];
        std::sort(list.begin(), list.end(),
                  [](const Upper& l, const Upper& r) { return l.position < r.position; });
        for (auto& u : list) {
            u.edge = static_cast<int>(edges_.size());
            edges_.emplace_back(p, u.position);
        }
    }
    rows_.assign(static_cast<std::size_t>(n), {});
    diagonal_.assign(static_cast<std::size_t>(n), {});
    rhs_.assign(static_cast<std::size_t>(n), {});
    blocks_.assign(edges_.size(), {});
    factors_.assign(edges_.size(), {});
}

void DirectJointSolver::buildRows(const JointConstraint& c, const std::vector<SolverBody>& bodies,
                                  Rows& rows) const {
    rows.count = 0;
    rows.a = c.a;
    rows.b = c.b;
    auto point = [&](Vec3 e, double error) {
        auto r = static_cast<std::size_t>(rows.count++);
        rows.linA[r] = e;
        rows.angA[r] = c.rA.cross(e);
        rows.linB[r] = -e;
        rows.angB[r] = -c.rB.cross(e);
        rows.error[r] = error;
        rows.kind[r] = 0;
        rows.direction[r] = e;
    };
    auto angular = [&](Vec3 n, double error) {
        auto r = static_cast<std::size_t>(rows.count++);
        rows.linA[r] = {};
        rows.angA[r] = n;
        rows.linB[r] = {};
        rows.angB[r] = -n;
        rows.error[r] = error;
        rows.kind[r] = 1;
        rows.direction[r] = n;
    };
    (void)bodies;
    switch (c.joint->type) {
    case JointType::Fixed:
        for (Vec3 e : worldBasis) {
            angular(e, c.angularError.dot(e));
        }
        for (Vec3 e : worldBasis) {
            point(e, c.pointError.dot(e));
        }
        break;
    case JointType::Ball:
        for (Vec3 e : worldBasis) {
            point(e, c.pointError.dot(e));
        }
        break;
    case JointType::Hinge:
        angular(c.t, c.angularError.dot(c.t));
        angular(c.u, c.angularError.dot(c.u));
        for (Vec3 e : worldBasis) {
            point(e, c.pointError.dot(e));
        }
        break;
    case JointType::Prismatic:
        for (Vec3 e : worldBasis) {
            angular(e, c.angularError.dot(e));
        }
        point(c.t, c.pointError.dot(c.t));
        point(c.u, c.pointError.dot(c.u));
        break;
    }
}

// Σ over shared dynamic bodies X of J_jX M_X⁻¹ J_kXᵀ (rows of j x rows of k).
DirectJointSolver::Block DirectJointSolver::coupling(const Rows& j, const Rows& k,
                                                     const std::vector<SolverBody>& bodies) const {
    Block result{};
    auto add = [&](int body, bool jIsA, bool kIsA) {
        const SolverBody& x = bodies[static_cast<std::size_t>(body)];
        for (int r = 0; r < j.count; ++r) {
            auto ri = static_cast<std::size_t>(r);
            Vec3 lin = jIsA ? j.linA[ri] : j.linB[ri];
            Vec3 ang = x.invInertia * (jIsA ? j.angA[ri] : j.angB[ri]);
            for (int s = 0; s < k.count; ++s) {
                auto si = static_cast<std::size_t>(s);
                Vec3 linK = kIsA ? k.linA[si] : k.linB[si];
                Vec3 angK = kIsA ? k.angA[si] : k.angB[si];
                result[static_cast<std::size_t>(r * maxRows + s)] +=
                    x.invMass * lin.dot(linK) + ang.dot(angK);
            }
        }
    };
    for (int body : {j.a, j.b}) {
        if (!bodies[static_cast<std::size_t>(body)].dynamic() || (body == j.b && j.a == j.b)) {
            continue;
        }
        bool jIsA = body == j.a;
        if (body == k.a) {
            add(body, jIsA, true);
        }
        if (body == k.b && k.a != k.b) {
            add(body, jIsA, false);
        }
    }
    return result;
}

void DirectJointSolver::factor(const std::vector<JointConstraint>& joints,
                               const std::vector<SolverBody>& bodies) {
    const std::size_t n = order_.size();
    for (std::size_t p = 0; p < n; ++p) {
        buildRows(joints[static_cast<std::size_t>(order_[p])], bodies, rows_[p]);
    }
    for (std::size_t p = 0; p < n; ++p) {
        diagonal_[p] = coupling(rows_[p], rows_[p], bodies);
        double trace = 0;
        for (int r = 0; r < rows_[p].count; ++r) {
            trace = std::max(trace, diagonal_[p][static_cast<std::size_t>(r * maxRows + r)]);
        }
        for (int r = 0; r < rows_[p].count; ++r) {
            diagonal_[p][static_cast<std::size_t>(r * maxRows + r)] += regularization * trace + 1e-300;
        }
    }
    for (std::size_t e = 0; e < edges_.size(); ++e) {
        auto [j, k] = edges_[e];
        blocks_[e] = coupling(rows_[static_cast<std::size_t>(k)], rows_[static_cast<std::size_t>(j)], bodies);
    }
    // Right-looking block LDLᵀ: D_j, L_kj = A_kj D_j⁻¹, A_lk -= L_lj A_kjᵀ for coupled later k <= l.
    for (std::size_t p = 0; p < n; ++p) {
        const int rj = rows_[p].count;
        diagonal_[p] = invert(diagonal_[p], rj);
        const auto& list = upper_[p];
        for (const Upper& u : list) {
            int rk = rows_[static_cast<std::size_t>(u.position)].count;
            factors_[static_cast<std::size_t>(u.edge)] =
                multiply(blocks_[static_cast<std::size_t>(u.edge)], diagonal_[p], rk, rj, rj);
        }
        for (std::size_t x = 0; x < list.size(); ++x) {
            const Upper& k = list[x];
            int rk = rows_[static_cast<std::size_t>(k.position)].count;
            subtractProductTransposed(diagonal_[static_cast<std::size_t>(k.position)],
                                      factors_[static_cast<std::size_t>(k.edge)],
                                      blocks_[static_cast<std::size_t>(k.edge)], rk, rj, rk);
            for (std::size_t y = x + 1; y < list.size(); ++y) {
                const Upper& l = list[y];
                int rl = rows_[static_cast<std::size_t>(l.position)].count;
                // Edge (k, l): k eliminated before l, stored as A_lk.
                const auto& kUpper = upper_[static_cast<std::size_t>(k.position)];
                auto found = std::find_if(kUpper.begin(), kUpper.end(),
                                          [&](const Upper& v) { return v.position == l.position; });
                subtractProductTransposed(blocks_[static_cast<std::size_t>(found->edge)],
                                          factors_[static_cast<std::size_t>(l.edge)],
                                          blocks_[static_cast<std::size_t>(k.edge)], rl, rj, rk);
            }
        }
    }
}

// Solves A x = rhs_ in place with the factorization (forward, diagonal, backward substitution).
void DirectJointSolver::substitute() {
    const std::size_t n = order_.size();
    for (std::size_t p = 0; p < n; ++p) {
        int rj = rows_[p].count;
        for (const Upper& u : upper_[p]) {
            auto& target = rhs_[static_cast<std::size_t>(u.position)];
            const Block& l = factors_[static_cast<std::size_t>(u.edge)];
            for (int r = 0; r < rows_[static_cast<std::size_t>(u.position)].count; ++r) {
                for (int s = 0; s < rj; ++s) {
                    target[static_cast<std::size_t>(r)] -=
                        l[static_cast<std::size_t>(r * maxRows + s)] * rhs_[p][static_cast<std::size_t>(s)];
                }
            }
        }
    }
    for (std::size_t p = 0; p < n; ++p) {
        std::array<double, maxRows> scaled{};
        int rj = rows_[p].count;
        for (int r = 0; r < rj; ++r) {
            for (int s = 0; s < rj; ++s) {
                scaled[static_cast<std::size_t>(r)] += diagonal_[p][static_cast<std::size_t>(r * maxRows + s)]
                                                       * rhs_[p][static_cast<std::size_t>(s)];
            }
        }
        rhs_[p] = scaled;
    }
    for (std::size_t q = n; q-- > 0;) {
        int rj = rows_[q].count;
        for (const Upper& u : upper_[q]) {
            const auto& source = rhs_[static_cast<std::size_t>(u.position)];
            const Block& l = factors_[static_cast<std::size_t>(u.edge)];
            for (int r = 0; r < rows_[static_cast<std::size_t>(u.position)].count; ++r) {
                for (int s = 0; s < rj; ++s) {
                    rhs_[q][static_cast<std::size_t>(s)] -=
                        l[static_cast<std::size_t>(r * maxRows + s)] * source[static_cast<std::size_t>(r)];
                }
            }
        }
    }
}

void DirectJointSolver::computeResponses(std::vector<JointConstraint>& joints,
                                         const std::vector<SolverBody>& bodies) {
    dv_.assign(bodies.size(), {});
    dw_.assign(bodies.size(), {});
    std::vector<int> touched;
    for (auto& c : joints) {
        c.response.clear();
        const Joint& joint = *c.joint;
        bool bounded =
            joint.isActuated() && ((joint.maxTorque > 0 && joint.usesSpeedMotor()) || joint.limitsEnabled);
        if (!bounded || order_.empty()) {
            continue;
        }
        // Jacobian of the joint speed (rate of Joint::angle); a unit impulse applies its transpose.
        Vec3 linA;
        Vec3 angA = -c.axis;
        Vec3 linB;
        Vec3 angB = c.axis;
        if (joint.type == JointType::Prismatic) {
            linA = -c.axis;
            angA = -c.rA.cross(c.axis);
            linB = c.axis;
            angB = c.rB.cross(c.axis);
        }
        const SolverBody& a = bodies[static_cast<std::size_t>(c.a)];
        const SolverBody& b = bodies[static_cast<std::size_t>(c.b)];
        touched.clear();
        touched.push_back(c.a);
        touched.push_back(c.b);
        dv_[static_cast<std::size_t>(c.a)] += linA * a.invMass;
        dw_[static_cast<std::size_t>(c.a)] += a.invInertia * angA;
        dv_[static_cast<std::size_t>(c.b)] += linB * b.invMass;
        dw_[static_cast<std::size_t>(c.b)] += b.invInertia * angB;
        // Reaction of the bilateral rows: A λ = -J Δv, Δv += M⁻¹ Jᵀ λ.
        for (std::size_t p = 0; p < order_.size(); ++p) {
            const Rows& rows = rows_[p];
            auto ia = static_cast<std::size_t>(rows.a);
            auto ib = static_cast<std::size_t>(rows.b);
            for (int r = 0; r < rows.count; ++r) {
                auto ri = static_cast<std::size_t>(r);
                rhs_[p][ri] = -(rows.linA[ri].dot(dv_[ia]) + rows.angA[ri].dot(dw_[ia])
                                + rows.linB[ri].dot(dv_[ib]) + rows.angB[ri].dot(dw_[ib]));
            }
            touched.push_back(rows.a);
            touched.push_back(rows.b);
        }
        substitute();
        for (std::size_t p = 0; p < order_.size(); ++p) {
            const Rows& rows = rows_[p];
            const SolverBody& ra = bodies[static_cast<std::size_t>(rows.a)];
            const SolverBody& rb = bodies[static_cast<std::size_t>(rows.b)];
            for (int r = 0; r < rows.count; ++r) {
                auto ri = static_cast<std::size_t>(r);
                double lambda = rhs_[p][ri];
                dv_[static_cast<std::size_t>(rows.a)] += rows.linA[ri] * (lambda * ra.invMass);
                dw_[static_cast<std::size_t>(rows.a)] += ra.invInertia * (rows.angA[ri] * lambda);
                dv_[static_cast<std::size_t>(rows.b)] += rows.linB[ri] * (lambda * rb.invMass);
                dw_[static_cast<std::size_t>(rows.b)] += rb.invInertia * (rows.angB[ri] * lambda);
            }
        }
        auto ia = static_cast<std::size_t>(c.a);
        auto ib = static_cast<std::size_t>(c.b);
        double inverseMass = linA.dot(dv_[ia]) + angA.dot(dw_[ia]) + linB.dot(dv_[ib]) + angB.dot(dw_[ib]);
        std::sort(touched.begin(), touched.end());
        touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
        if (inverseMass > 1e-12) {
            c.axialMass = 1 / inverseMass;
            for (int body : touched) {
                auto i = static_cast<std::size_t>(body);
                if (bodies[i].dynamic()) {
                    c.response.push_back({body, dv_[i], dw_[i]});
                }
            }
        }
        for (int body : touched) {
            dv_[static_cast<std::size_t>(body)] = {};
            dw_[static_cast<std::size_t>(body)] = {};
        }
    }
}

void DirectJointSolver::solve(std::vector<JointConstraint>& joints, std::vector<SolverBody>& bodies, double h,
                              SolvePass pass) {
    const std::size_t n = order_.size();
    const bool position = pass == SolvePass::Position;
    const double biasRate = position ? positionCorrectionFactor / h : 0;
    for (std::size_t p = 0; p < n; ++p) {
        const Rows& rows = rows_[p];
        const SolverBody& a = bodies[static_cast<std::size_t>(rows.a)];
        const SolverBody& b = bodies[static_cast<std::size_t>(rows.b)];
        for (int r = 0; r < rows.count; ++r) {
            auto ri = static_cast<std::size_t>(r);
            double cdot = rows.linA[ri].dot(a.velocity) + rows.angA[ri].dot(a.angularVelocity)
                          + rows.linB[ri].dot(b.velocity) + rows.angB[ri].dot(b.angularVelocity);
            double bias =
                std::clamp(rows.error[ri] * biasRate, -maxCorrectionVelocity, maxCorrectionVelocity);
            rhs_[p][ri] = -(cdot + bias);
        }
    }
    substitute();
    // Apply λ: Δv = M⁻¹ Jᵀ λ; accumulate the impulses for reporting and warm starting.
    for (std::size_t p = 0; p < n; ++p) {
        const Rows& rows = rows_[p];
        JointConstraint& c = joints[static_cast<std::size_t>(order_[p])];
        Vec3 linA;
        Vec3 angA;
        Vec3 linB;
        Vec3 angB;
        Vec3& linear = position ? c.pseudoLinear : c.linear;
        Vec3& angularSum = position ? c.pseudoAngular : c.angular;
        for (int r = 0; r < rows.count; ++r) {
            auto ri = static_cast<std::size_t>(r);
            double lambda = rhs_[p][ri];
            linA += rows.linA[ri] * lambda;
            angA += rows.angA[ri] * lambda;
            linB += rows.linB[ri] * lambda;
            angB += rows.angB[ri] * lambda;
            (rows.kind[ri] == 0 ? linear : angularSum) += rows.direction[ri] * lambda;
        }
        SolverBody& a = bodies[static_cast<std::size_t>(rows.a)];
        SolverBody& b = bodies[static_cast<std::size_t>(rows.b)];
        a.velocity += linA * a.invMass;
        a.angularVelocity += a.invInertia * angA;
        b.velocity += linB * b.invMass;
        b.angularVelocity += b.invInertia * angB;
    }
}

} // namespace robotics::detail

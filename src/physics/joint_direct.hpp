#pragma once
// Direct solver for the bilateral joint rows (internal).
//
// Sequential impulses converge slowly on long chains and large mass ratios: an impulse needs one pass
// per link to travel along a chain, and a light link between heavy ones barely transmits it. This
// solver instead projects the velocities onto all bilateral joint rows at once by solving
//     (J M⁻¹ Jᵀ) λ = −(J v + bias)
// with a sparse block LDLᵀ factorization (blocks = joints, up to 6 rows each). The elimination order is
// greedy minimum degree on the joint graph (two joints are coupled when they share a dynamic body):
// for tree-structured robots this eliminates leaves first and produces no fill, so cost is linear in
// the number of joints, as in Baraff's "Linear-time dynamics using Lagrange multipliers" (1996).
// Closed loops are handled by the same code with some fill. A tiny diagonal regularization keeps
// redundant (over-constrained) assemblies solvable.
//
// Motors, limits and contacts (inequality / bounded rows) stay in the sequential-impulse loop; every
// pass ends with this exact projection, so the joints have the last word.

#include "joint_solver.hpp"

#include <array>
#include <vector>

namespace robotics::detail {

class DirectJointSolver {
public:
    // Symbolic analysis (ordering and fill pattern); call once per step (topology is fixed).
    void analyze(const std::vector<JointConstraint>& joints, const std::vector<SolverBody>& bodies);

    // Jacobians and numeric factorization for the current poses; call after prepareJoints().
    void factor(const std::vector<JointConstraint>& joints, const std::vector<SolverBody>& bodies);

    // Projects body velocities onto the joint rows (velocity pass: bias 0) or removes the position
    // error with pseudo-velocities (position pass). Impulses accumulate into the constraints.
    void solve(std::vector<JointConstraint>& joints, std::vector<SolverBody>& bodies, double h,
               SolvePass pass);

    // Constrained free-axis response and effective mass of every motor/limit row (after factor()).
    void computeResponses(std::vector<JointConstraint>& joints, const std::vector<SolverBody>& bodies);

    [[nodiscard]] bool empty() const { return order_.empty(); }

private:
    static constexpr int maxRows = 6;
    using Block = std::array<double, maxRows * maxRows>; // Row-major, stride maxRows.

    struct Rows {
        int count{};
        int a{};
        int b{};
        std::array<Vec3, maxRows> linA;
        std::array<Vec3, maxRows> angA;
        std::array<Vec3, maxRows> linB;
        std::array<Vec3, maxRows> angB;
        std::array<double, maxRows> error{};
        std::array<int, maxRows> kind{}; // 0: linear impulse direction, 1: angular.
        std::array<Vec3, maxRows> direction;
    };

    struct Upper {
        int position{}; // Elimination position of the coupled joint (> own position).
        int edge{};     // Index into blocks_ / factors_.
    };

    void substitute();
    void buildRows(const JointConstraint& c, const std::vector<SolverBody>& bodies, Rows& rows) const;
    Block coupling(const Rows& j, const Rows& k, const std::vector<SolverBody>& bodies) const;

    std::vector<int> order_;                 // Elimination position -> constraint index.
    std::vector<std::vector<Upper>> upper_;  // Per position: later coupled positions.
    std::vector<std::pair<int, int>> edges_; // Edge -> (position j, position k), j < k.
    std::vector<Rows> rows_;                 // Per position.
    std::vector<Block> diagonal_;            // Per position: D_j⁻¹ after factor().
    std::vector<Block> blocks_;              // Per edge: A_kj (rows of k x rows of j), updated in place.
    std::vector<Block> factors_;             // Per edge: L_kj = A_kj D_j⁻¹.
    std::vector<std::array<double, maxRows>> rhs_;
    std::vector<Vec3> dv_; // Scratch velocity changes per body (computeResponses).
    std::vector<Vec3> dw_;
};

} // namespace robotics::detail

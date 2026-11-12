#pragma once
// Sequential-impulse contact solver (internal): non-penetration and Coulomb friction (clamped to the
// friction circle) with warm starting, split-impulse penetration recovery, and a restitution pass
// after the substeps.

#include "robotics/contact.hpp"
#include "robotics/joint.hpp"
#include "solver_body.hpp"

#include <vector>

namespace robotics::detail {

struct ContactConstraint {
    int a{};
    int b{}; // Solver body index; the ground is the last solver body.
    Vec3 normal;
    Vec3 tangent1;
    Vec3 tangent2;
    Vec3 localA; // Contact anchor on A (A's frame) and on B (B's frame, world for the ground).
    Vec3 localB;
    double friction{};
    double bounce{};
    bool speculative{}; // Impact of continuous collision: rA, rB come from the impact pose.
    bool dormant{};     // Speculative and not reached yet: ignored by the solver.
    double impactTime{};

    // Per-substep geometry.
    Vec3 rA;
    Vec3 rB;
    double separation{}; // (pA - pB)·n, negative when penetrating.
    double normalMass{};
    double tangentMass1{};
    double tangentMass2{};

    // Accumulated per-substep impulses (warm started) and step totals.
    double normalImpulse{};
    double tangentImpulse1{};
    double tangentImpulse2{};
    double maxNormalImpulse{};
    double pseudoNormalImpulse{}; // Position pass, reset every substep.
    double totalNormal{};
    double totalTangent1{};
    double totalTangent2{};
};

// Builds constraints from the detected contacts. Contact::normalImpulse / tangentImpulse hold the
// warm-start impulses (already matched from the previous step) and are scaled by `warmScale`.
void initContacts(const std::vector<Contact>& contacts, const std::vector<Body>& bodies,
                  const std::vector<SolverBody>& solverBodies, std::vector<ContactConstraint>& constraints,
                  double warmScale);

void prepareContacts(std::vector<ContactConstraint>& constraints, const std::vector<SolverBody>& bodies);

void warmStartContacts(std::vector<ContactConstraint>& constraints, std::vector<SolverBody>& bodies);

// One pass. The position pass pushes penetrating contacts apart with pseudo-velocities only.
void solveContacts(std::vector<ContactConstraint>& constraints, std::vector<SolverBody>& bodies, double h,
                   SolvePass pass);

void resetPseudoImpulses(std::vector<ContactConstraint>& constraints);

void accumulateContactTotals(std::vector<ContactConstraint>& constraints);

// Runs the speculative contacts of a step in order of their impacts. A substep that reaches an impact moves
// the two bodies to it (with their current velocities) and wakes the contact; from then on they stay at that
// pose for the rest of the step, so they cannot pass anything the step did not check, and the next step's
// collision detection takes over with the new velocities. Costs a fast body at most one step of flight
// per impact. A body on a joint keeps moving with the joint solver instead: the constraint holds it, and
// holding it back would tear the joint.
class ImpactSchedule {
public:
    void init(const std::vector<ContactConstraint>& constraints, const std::vector<Joint>& joints,
              std::size_t solverBodies);

    // Call at the start of a substep that spans [time, time + h).
    void advance(std::vector<ContactConstraint>& constraints, std::vector<SolverBody>& bodies, double time,
                 double h);

    [[nodiscard]] bool stopped(std::size_t body) const { return !stopped_.empty() && stopped_[body] != 0; }

private:
    std::vector<std::size_t> order_; // Speculative constraints by impact time.
    std::size_t next_{};
    std::vector<char> jointed_;
    std::vector<char> stopped_;
    std::vector<double> stoppedAt_;
};

// Restitution after all substeps: drives approaching contacts to their bounce velocity.
void applyRestitution(std::vector<ContactConstraint>& constraints, std::vector<SolverBody>& bodies);

// Writes the step totals into the public contacts.
void storeContactImpulses(const std::vector<ContactConstraint>& constraints, std::vector<Contact>& contacts);

// Copies impulses of previous contacts with the same (a, b, feature) and a similar normal into
// `contacts` as warm-start values; unmatched contacts start from zero.
void matchContacts(std::vector<Contact>& contacts, const std::vector<Contact>& previous);

} // namespace robotics::detail

#include "contact_solver.hpp"

#include "integrator.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace robotics::detail {
namespace {

// Fraction of the penetration (beyond the slop) removed per substep by the position pass.
constexpr double contactBaumgarte = 0.2;
// Penetration (m) tolerated without correction; avoids jitter of resting contacts.
constexpr double contactSlop = 0.001;
// Upper bound (m/s) of the penetration recovery pseudo-velocity.
constexpr double maxCorrectionVelocity = 3.0;
// Previous contacts only warm start a new one if their normals agree to this cosine.
constexpr double warmStartNormalCosine = 0.9;

void tangentBasis(Vec3 n, Vec3& t1, Vec3& t2) {
    t1 = n.cross(std::abs(n.x) < 0.57 ? Vec3{1, 0, 0} : Vec3{0, 1, 0}).normalized();
    t2 = n.cross(t1);
}

double inverseOrZero(double k) {
    return k > 1e-12 ? 1 / k : 0;
}

Vec3 relativeVelocity(const ContactConstraint& c, const SolverBody& a, const SolverBody& b) {
    return a.pointVelocity(c.rA) - b.pointVelocity(c.rB);
}

void applyPair(const ContactConstraint& c, SolverBody& a, SolverBody& b, Vec3 impulse) {
    a.applyImpulse(impulse, c.rA);
    b.applyImpulse(-impulse, c.rB);
}

} // namespace

void initContacts(const std::vector<Contact>& contacts, const std::vector<Body>& bodies,
                  const std::vector<SolverBody>& solverBodies, std::vector<ContactConstraint>& constraints,
                  double warmScale) {
    constraints.clear();
    constraints.reserve(contacts.size());
    const int ground = static_cast<int>(solverBodies.size()) - 1;
    for (const auto& contact : contacts) {
        ContactConstraint c;
        c.a = contact.a;
        c.b = contact.touchesGround() ? ground : contact.b;
        const SolverBody& a = solverBodies[static_cast<std::size_t>(c.a)];
        const SolverBody& b = solverBodies[static_cast<std::size_t>(c.b)];
        c.normal = contact.normal;
        tangentBasis(c.normal, c.tangent1, c.tangent2);
        Vec3 pointA = contact.point - contact.normal * (contact.depth * 0.5);
        Vec3 pointB = contact.point + contact.normal * (contact.depth * 0.5);
        c.localA = a.orientation.conjugate().rotate(pointA - a.position);
        c.localB = b.orientation.conjugate().rotate(pointB - b.position);
        if (contact.speculative) {
            c.speculative = true;
            c.dormant = true;
            c.impactTime = contact.impactTime;
            c.rA = contact.leverA;
            c.rB = contact.leverB;
        }
        const Body& bodyA = bodies[static_cast<std::size_t>(contact.a)];
        c.friction = contact.touchesGround()
                         ? bodyA.friction
                         : std::sqrt(bodyA.friction * bodies[static_cast<std::size_t>(contact.b)].friction);
        c.bounce = contact.bounce;
        c.normalImpulse = contact.normalImpulse * warmScale;
        c.tangentImpulse1 = contact.tangentImpulse.dot(c.tangent1) * warmScale;
        c.tangentImpulse2 = contact.tangentImpulse.dot(c.tangent2) * warmScale;
        constraints.push_back(c);
    }
}

void prepareContacts(std::vector<ContactConstraint>& constraints, const std::vector<SolverBody>& bodies) {
    for (auto& c : constraints) {
        if (c.dormant) {
            continue;
        }
        const SolverBody& a = bodies[static_cast<std::size_t>(c.a)];
        const SolverBody& b = bodies[static_cast<std::size_t>(c.b)];
        if (!c.speculative) { // A woken impact stays at its point: the bodies have stopped there.
            Vec3 pointA = a.point(c.localA);
            Vec3 pointB = b.point(c.localB);
            c.separation = (pointA - pointB).dot(c.normal);
            Vec3 middle = (pointA + pointB) * 0.5;
            c.rA = middle - a.position;
            c.rB = middle - b.position;
        }
        c.normalMass = inverseOrZero(pointMass(a, b, c.rA, c.rB, c.normal));
        c.tangentMass1 = inverseOrZero(pointMass(a, b, c.rA, c.rB, c.tangent1));
        c.tangentMass2 = inverseOrZero(pointMass(a, b, c.rA, c.rB, c.tangent2));
    }
}

void warmStartContacts(std::vector<ContactConstraint>& constraints, std::vector<SolverBody>& bodies) {
    for (auto& c : constraints) {
        if (c.speculative) {
            // An impact is a one-off impulse (kept in the totals), not a load to carry into the next substep.
            c.normalImpulse = c.tangentImpulse1 = c.tangentImpulse2 = 0;
            continue;
        }
        applyPair(c, bodies[static_cast<std::size_t>(c.a)], bodies[static_cast<std::size_t>(c.b)],
                  c.normal * c.normalImpulse + c.tangent1 * c.tangentImpulse1
                      + c.tangent2 * c.tangentImpulse2);
    }
}

void solveContacts(std::vector<ContactConstraint>& constraints, std::vector<SolverBody>& bodies, double h,
                   SolvePass pass) {
    for (auto& c : constraints) {
        if (c.dormant) {
            continue;
        }
        SolverBody& a = bodies[static_cast<std::size_t>(c.a)];
        SolverBody& b = bodies[static_cast<std::size_t>(c.b)];
        if (pass == SolvePass::Position) {
            if (c.separation >= -contactSlop) {
                continue;
            }
            double bias =
                std::max(contactBaumgarte * (c.separation + contactSlop) / h, -maxCorrectionVelocity);
            double normalVelocity = relativeVelocity(c, a, b).dot(c.normal);
            double previous = c.pseudoNormalImpulse;
            c.pseudoNormalImpulse = std::max(0.0, previous - c.normalMass * (normalVelocity + bias));
            applyPair(c, a, b, c.normal * (c.pseudoNormalImpulse - previous));
            continue;
        }

        // Coulomb friction first (uses the normal impulse of the previous pass): both tangent rows,
        // then the accumulated 2D impulse is projected onto the friction circle |λt| <= μ λn.
        Vec3 velocity = relativeVelocity(c, a, b);
        double next1 = c.tangentImpulse1 - c.tangentMass1 * velocity.dot(c.tangent1);
        double next2 = c.tangentImpulse2 - c.tangentMass2 * velocity.dot(c.tangent2);
        double limit = c.friction * c.normalImpulse;
        double length = std::hypot(next1, next2);
        if (length > limit) {
            double scale = length > 0 ? limit / length : 0;
            next1 *= scale;
            next2 *= scale;
        }
        applyPair(c, a, b,
                  c.tangent1 * (next1 - c.tangentImpulse1) + c.tangent2 * (next2 - c.tangentImpulse2));
        c.tangentImpulse1 = next1;
        c.tangentImpulse2 = next2;

        // Non-penetration: accumulated normal impulse clamped to be non-negative. A contact that has
        // separated within the step is speculative: approach is allowed until the surfaces touch.
        double normalVelocity = relativeVelocity(c, a, b).dot(c.normal);
        double previous = c.normalImpulse;
        double target = c.separation > 0 ? -c.separation / h : 0;
        double impulse = c.normalMass * (target - normalVelocity);
        c.normalImpulse = std::max(0.0, previous + impulse);
        c.maxNormalImpulse = std::max(c.maxNormalImpulse, c.normalImpulse);
        applyPair(c, a, b, c.normal * (c.normalImpulse - previous));
    }
}

void ImpactSchedule::init(const std::vector<ContactConstraint>& constraints, const std::vector<Joint>& joints,
                          std::size_t solverBodies) {
    order_.clear();
    next_ = 0;
    for (std::size_t i = 0; i < constraints.size(); ++i) {
        if (constraints[i].speculative) {
            order_.push_back(i);
        }
    }
    if (order_.empty()) {
        stopped_.clear();
        return;
    }
    std::stable_sort(order_.begin(), order_.end(), [&](std::size_t l, std::size_t r) {
        return constraints[l].impactTime < constraints[r].impactTime;
    });
    stopped_.assign(solverBodies, 0);
    stoppedAt_.assign(solverBodies, 0);
    jointed_.assign(solverBodies, 0);
    for (const auto& joint : joints) {
        jointed_[static_cast<std::size_t>(joint.a)] = 1;
        jointed_[static_cast<std::size_t>(joint.b)] = 1;
    }
}

void ImpactSchedule::advance(std::vector<ContactConstraint>& constraints, std::vector<SolverBody>& bodies,
                             double time, double h) {
    for (; next_ < order_.size(); ++next_) {
        ContactConstraint& c = constraints[order_[next_]];
        if (c.impactTime >= time + h) {
            return;
        }
        // A body stopped by an earlier impact is not where this contact expected it.
        bool stale = false;
        for (int index : {c.a, c.b}) {
            auto i = static_cast<std::size_t>(index);
            stale = stale || (stopped_[i] && stoppedAt_[i] != c.impactTime);
        }
        for (int index : {c.a, c.b}) {
            auto i = static_cast<std::size_t>(index);
            SolverBody& body = bodies[i];
            if (body.dynamic() && !jointed_[i] && !stopped_[i]) {
                integratePosition(body, body.velocity, body.angularVelocity,
                                  std::max(0.0, c.impactTime - time));
                stopped_[i] = 1;
                stoppedAt_[i] = c.impactTime;
            }
        }
        c.dormant = stale;
    }
}

void resetPseudoImpulses(std::vector<ContactConstraint>& constraints) {
    for (auto& c : constraints) {
        c.pseudoNormalImpulse = 0;
    }
}

void accumulateContactTotals(std::vector<ContactConstraint>& constraints) {
    for (auto& c : constraints) {
        c.totalNormal += c.normalImpulse;
        c.totalTangent1 += c.tangentImpulse1;
        c.totalTangent2 += c.tangentImpulse2;
    }
}

void applyRestitution(std::vector<ContactConstraint>& constraints, std::vector<SolverBody>& bodies) {
    for (auto& c : constraints) {
        if (c.bounce <= 0 || c.maxNormalImpulse <= 0) {
            continue;
        }
        SolverBody& a = bodies[static_cast<std::size_t>(c.a)];
        SolverBody& b = bodies[static_cast<std::size_t>(c.b)];
        double normalVelocity = relativeVelocity(c, a, b).dot(c.normal);
        double impulse = -c.normalMass * (normalVelocity - c.bounce);
        double previous = c.normalImpulse;
        c.normalImpulse = std::max(0.0, previous + impulse);
        c.totalNormal += c.normalImpulse - previous;
        applyPair(c, a, b, c.normal * (c.normalImpulse - previous));
    }
}

void storeContactImpulses(const std::vector<ContactConstraint>& constraints, std::vector<Contact>& contacts) {
    for (std::size_t i = 0; i < contacts.size(); ++i) {
        const ContactConstraint& c = constraints[i];
        contacts[i].normalImpulse = c.totalNormal;
        contacts[i].tangentImpulse = c.tangent1 * c.totalTangent1 + c.tangent2 * c.totalTangent2;
    }
}

void matchContacts(std::vector<Contact>& contacts, const std::vector<Contact>& previous) {
    auto key = [](const Contact& c) {
        return std::make_tuple(c.a, c.b, c.feature);
    };
    std::vector<std::size_t> order(previous.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
        order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](std::size_t l, std::size_t r) {
        return std::make_tuple(key(previous[l]), l) < std::make_tuple(key(previous[r]), r);
    });
    for (auto& contact : contacts) {
        contact.normalImpulse = 0;
        contact.tangentImpulse = {};
        if (contact.speculative) {
            continue; // A fresh impact: old impulses say nothing about it.
        }
        auto found =
            std::lower_bound(order.begin(), order.end(), key(contact),
                             [&](std::size_t index, const auto& k) { return key(previous[index]) < k; });
        if (found == order.end() || key(previous[*found]) != key(contact)) {
            continue;
        }
        const Contact& old = previous[*found];
        if (old.normal.dot(contact.normal) < warmStartNormalCosine) {
            continue;
        }
        contact.normalImpulse = old.normalImpulse;
        contact.tangentImpulse = old.tangentImpulse - contact.normal * old.tangentImpulse.dot(contact.normal);
    }
}

} // namespace robotics::detail

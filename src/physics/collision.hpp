#pragma once
// Narrow-phase collision detection (internal).

#include "robotics/body.hpp"
#include "robotics/contact.hpp"
#include "robotics/joint.hpp"

#include <vector>

namespace robotics::detail {

// Lateral faces of the prism that stands for a cylinder in polytope pairs.
inline constexpr int cylinderPrismSides = 32;

// Continuous collision settings of one step; dt = 0 detects the current poses only.
struct ContinuousCollision {
    double dt{};
    Vec3 gravity{};
};

// Replaces `contacts` with all ground and body-pair contacts of the current poses, then sets the
// restitution target (Contact::bounce) of each contact. Bodies connected by a joint never collide.
// Fast or Body::ccd bodies that touch nothing yet but will within dt add speculative contacts (see
// ccd.hpp) in the same place of the order.
// Contact order is deterministic: ground contacts by body, then pairs (i, k) with i < k.
void detectContacts(const std::vector<Body>& bodies, const std::vector<Joint>& joints,
                    std::vector<Contact>& contacts, const ContinuousCollision& continuous = {});

} // namespace robotics::detail

#pragma once
// Contact point produced by collision detection and resolved by the contact solver.

#include "robotics/math.hpp"

#include <cstdint>

namespace robotics {

// Index used in Contact::b for the infinite ground plane y = 0.
inline constexpr int groundBodyIndex = -1;

// World::contacts holds the contacts of the last World::step(). They stay valid (including the
// accumulated impulses) until the next step, so tools can read them after stepping, e.g. to
// draw contact points, normals and forces. Average force over the step = impulse / dt.
//
// The next step also uses them for warm starting: a new contact with the same (a, b, feature) starts
// from the previous impulses. Clearing World::contacts therefore just resets the warm start.
// A body pair produces a manifold of up to 4 points (box faces, cylinder caps/sides); every point
// is a separate Contact with its own feature id.
//
// Speculative contacts come from continuous collision detection: the bodies are still apart at the start
// of the step, but collide within it. `point` and `normal` describe that impact. Those that did not act
// are removed at the end of the step, so World::contacts only lists real touches.
struct Contact {
    int a{};                 // Body index.
    int b{groundBodyIndex};  // Body index, or groundBodyIndex for the ground plane.
    Vec3 point{};            // World-space contact point, midway between the two surfaces.
    Vec3 normal{};           // Unit normal, points from B towards A.
    double depth{};          // Penetration depth (m), >= 0, at the start of the step (speculative: 0).
    double normalImpulse{};  // Accumulated normal impulse on A (N·s), >= 0.
    double bounce{};         // Restitution target normal velocity (solver input).
    Vec3 tangentImpulse{};   // Accumulated friction impulse on A (N·s), world space.
    std::uint32_t feature{}; // Geometric feature id (vertex/face/edge combination), unique per body pair.
    bool speculative{};      // See above. The rest is only set for such contacts:
    double impactTime{};     // Seconds after the start of the step.
    Vec3 leverA{};           // World-space offsets of the contact from the bodies' centres at the impact.
    Vec3 leverB{};

    [[nodiscard]] bool touchesGround() const { return b == groundBodyIndex; }

    // Total impulse applied to body A (body B receives the opposite).
    [[nodiscard]] Vec3 totalImpulse() const { return normal * normalImpulse + tangentImpulse; }
};

} // namespace robotics

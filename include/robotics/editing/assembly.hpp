#pragma once
// Model edits on an assembly: a part plus everything attached below it (follow joints parent -> child).
// Used by the editor's gizmos and library; UI independent so it can be tested headless.

#include "robotics/world.hpp"

#include <vector>

namespace robotics {

// Flags (one per body) of the part `index` and everything attached below it.
[[nodiscard]] std::vector<char> assemblyMembers(const World& world, int index);

// Moves an assembly; joints to the rest of the robot stay attached at the moved side (their anchor on the
// other body follows). Encoders of those joints are re-zeroed.
void moveAssembly(World& world, int index, Vec3 delta);

// Rotates an assembly rigidly by `angle` radians about the world-space line through `pivot` along `axis`.
// Joints inside the assembly are untouched (their body-local frames stay valid). Joints to the rest of the
// robot are rebuilt at the rotated pose: anchor and axis follow the rotated side and the rotated pose becomes
// the new zero of the joint, so World::maxAnchorError() is unchanged and the encoder reads 0.
void rotateAssembly(World& world, int index, Vec3 pivot, Vec3 axis, double angle);

// Lowest world-space Y of a collision shape (exact for boxes, spheres, cylinders and hulls).
[[nodiscard]] double lowestPoint(const Body& body);

// Lowest point of the assembly, and the move that makes it touch y = 0. Returns the applied shift along Y.
[[nodiscard]] double assemblyLowestPoint(const World& world, int index);
double dropAssemblyToFloor(World& world, int index);

// Nearest multiple of `step`; step <= 0 returns the value unchanged.
[[nodiscard]] double snapToStep(double value, double step);

} // namespace robotics

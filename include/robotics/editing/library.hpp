#pragma once
// Part library: ready-made assemblies (bodies + joints + sensors) and user items saved as .robot files.

#include "robotics/world.hpp"

#include <string>
#include <vector>

namespace robotics {

// Where an item may attach on a part. Positions are relative to the part's bounds (-1..1 per local axis);
// `normal` is the outward direction in the part's frame.
struct LibraryMount {
    Vec3 surface;
    Vec3 normal;
};

struct LibraryItem {
    std::string id;
    std::string title;
    std::string summary;
    std::string category;
    // Body 0 is the root. Frame: the origin touches the host's surface and +Y points away from it.
    World assembly;
    // Connection to the selected part (body A = host, B = root). With replaceRootWithHost the root is only a
    // stand-in for a host: when attached it is left out and its joints connect to the host directly.
    JointType attachType{JointType::Fixed};
    Vec3 attachAnchor; // Assembly frame.
    Vec3 attachAxis{0, 1, 0};
    bool replaceRootWithHost{};
    std::vector<LibraryMount> mounts; // Tried in order; the first free one is used.
    bool custom{};                    // Saved by the user.
};

struct LibraryInsertion {
    int root{-1};        // First inserted body.
    int bodies{};        // Number of inserted bodies.
    int attachJoint{-1}; // Joint connecting the item to the host, or -1 when placed freely.
};

// The built-in items, in display order.
[[nodiscard]] const std::vector<LibraryItem>& builtinLibrary();

// `name` itself if no body has it, else base_1, base_2, ... where base has any numeric suffix removed.
[[nodiscard]] std::string uniqueBodyName(const World& world, const std::string& name);
[[nodiscard]] std::string uniqueSensorName(const World& world, const std::string& name);

// Inserts a copy with unique names. host >= 0 attaches the item at the host's first free mount; the host
// keeps its pose unless the item would reach below the floor, then its whole robot is lifted. host < 0 places
// the item unattached with its origin at `point`, resting on the floor. Throws for invalid input; `world` is
// unchanged then.
LibraryInsertion insertLibraryItem(World& world, const LibraryItem& item, int host, Vec3 point = {});

// Standalone copy of the part `root` and everything attached below it (see assemblyMembers): root first,
// runtime state cleared. Joints to the rest of the robot are left out.
[[nodiscard]] World extractAssembly(const World& world, int root);

// Library item from an extracted assembly: origin moved under the root onto its lowest point, fixed joint
// on top of the host (or on another free side).
[[nodiscard]] LibraryItem customLibraryItem(const std::string& title, World assembly);

} // namespace robotics

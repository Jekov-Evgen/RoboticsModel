#pragma once
// URDF importer: builds a World from a robot description. Mapping and limitations: docs/IMPORT.md.
//
//   UrdfImportOptions options;
//   options.fixBase = true; // e.g. a manipulator bolted to the floor
//   UrdfImportResult result = loadUrdf("ur5.urdf", options);
//   for (const auto& warning : result.warnings) { ... }
//
// Builds only through the public World API (addBody / connect / validate).

#include "robotics/world.hpp"

#include <iosfwd>
#include <string>
#include <vector>

namespace robotics {

enum class UrdfMeshCollision { BoundingBox, ConvexHull };

struct UrdfImportOptions {
    // Root link becomes static (mass 0), together with links welded to it by fixed joints.
    bool fixBase{false};
    // URDF is Z-up; this world is Y-up. When true the robot is rotated -90 degrees about X, so URDF
    // (x, y, z) maps to world (x, z, -y): forward stays +X, up becomes +Y, left becomes -Z.
    bool zUpToYUp{true};
    // World-space offset added to the whole robot after the axis conversion (metres).
    Vec3 spawnOffset{};
    // Lifts the robot so no collision shape starts below the floor (y = 0).
    bool liftAboveGround{true};
    // Effort (N·m or N) for actuated joints without a <limit effort>: continuous joints often omit it.
    double defaultEffort{10};
    // Mass for links that have collision geometry but no or zero <mass>.
    double minimumMass{0.01};
    // Extra folders searched for package://name/... and relative mesh paths (each folder is tried as a
    // package parent: folder/name/..., and as the package itself: folder/...). ROS_PACKAGE_PATH is also used.
    std::vector<std::string> meshSearchPaths;
    // How <collision> meshes become shapes. ConvexHull: the hull of the scaled mesh (at most 64 vertices), with
    // the body at its centre of mass and on its principal axes; flat meshes fall back to boxes.
    UrdfMeshCollision meshCollision{UrdfMeshCollision::ConvexHull};
};

struct UrdfImportResult {
    World world;
    std::string robotName;
    // User-facing notes about everything that was approximated or skipped, in plain English.
    std::vector<std::string> warnings;
};

// Parses URDF text. `sourcePath` is the file the text came from: it names the input in error messages
// and is the starting point for relative and package:// mesh paths (empty: current directory).
// Throws std::runtime_error with a line number for malformed XML or an invalid robot description.
[[nodiscard]] UrdfImportResult importUrdf(std::istream& in, const UrdfImportOptions& options = {},
                                          const std::string& sourcePath = "");

[[nodiscard]] UrdfImportResult loadUrdf(const std::string& path, const UrdfImportOptions& options = {});

// Resolves a URDF mesh reference (package://pkg/file, model://pkg/file, file:///abs, absolute or relative to
// the URDF folder) to an existing file. Returns an empty string when nothing is found.
[[nodiscard]] std::string resolveUrdfMeshPath(const std::string& reference, const std::string& urdfPath,
                                              const std::vector<std::string>& searchPaths = {});

} // namespace robotics

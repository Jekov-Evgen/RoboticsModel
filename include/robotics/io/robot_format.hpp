#pragma once
// Native text scene format (.robot). Format reference: docs/ARCHITECTURE.md, "Формат .robot".
//
// Other formats (e.g. a URDF importer) live beside this header as include/robotics/io/<format>.hpp
// and build a World only through the public World API (addBody / connect / validate).

#include "robotics/world.hpp"

#include <iosfwd>
#include <string>

namespace robotics {

// Newest version: 3 adds prismatic joints and the per-joint LIMITS record; 4 adds per-body visual meshes
// (VISUALS/MESH records) and World::iterations (ITERATIONS); 5 adds World::sensors (SENSORS/SENSOR records,
// configuration only - runtime state is not stored) and World::substeps / World::warmStarting (SOLVER); 6 adds
// convex hull bodies (HULL record: body-frame vertex list + source frame), Body::ccd (CCD) and
// World::continuousCollision (CONTINUOUS). readScene() accepts every version from robotFormatOldestVersion on.
inline constexpr int robotFormatVersion = 6;
inline constexpr int robotFormatOldestVersion = 1;

// Serializes the full editable state (bodies, visual meshes, joints, controllers, encoders, limits, solver
// iterations, sensors, hulls); validates first. Writes the oldest version that stores the world losslessly: version 6
// if a body is a convex hull or has Body::ccd or continuous collision is off, else version 5 if there are sensors or substeps/warm starting differ from the defaults, else version 4 if a body has visual meshes or a hidden primitive or iterations differ
// from the default, else version 3 if a joint is prismatic or has limit values, else version 2 - so files
// stay readable by older builds.
void writeScene(const World& world, std::ostream& out);

// Parses a scene and returns a validated World. Throws std::runtime_error on malformed input.
[[nodiscard]] World readScene(std::istream& in);

// File helpers. saveScene writes atomically (temporary file + rename) and stores mesh paths that lie inside
// the scene's folder relative to it; loadScene resolves relative mesh paths against the scene's folder.
void saveScene(const World& world, const std::string& path);
[[nodiscard]] World loadScene(const std::string& path);

} // namespace robotics

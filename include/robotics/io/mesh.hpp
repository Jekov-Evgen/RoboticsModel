#pragma once
// Triangle meshes for visuals and import: STL (binary and ASCII) and Wavefront OBJ readers, plus a cache.
// Headless and dependency-free; the editor uses MeshCache to draw Body::visuals.

#include "robotics/math.hpp"

#include <cstddef>
#include <iosfwd>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotics {

struct MeshBounds {
    Vec3 min;
    Vec3 max;

    [[nodiscard]] Vec3 size() const { return max - min; }

    [[nodiscard]] Vec3 center() const { return (min + max) * 0.5; }
};

// Flat triangle list in the file's units: vertices[3k], [3k+1], [3k+2] form triangle k (counter-clockwise
// seen from outside when the file is well formed); normals holds one unit normal per vertex.
struct TriangleMesh {
    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;

    [[nodiscard]] std::size_t triangleCount() const { return vertices.size() / 3; }

    [[nodiscard]] bool empty() const { return vertices.empty(); }

    // Axis-aligned bounds of all vertices; throws for an empty mesh.
    [[nodiscard]] MeshBounds bounds() const;
};

// Readers throw std::runtime_error with a line number (text formats) on malformed input.
// STL: binary or ASCII is detected from the content; stored normals are replaced by geometric ones.
[[nodiscard]] TriangleMesh readStl(std::istream& in);
// OBJ: v, vn and f records (v, v/vt, v//vn, v/vt/vn, negative indices); polygons are triangulated by ear
// clipping. Other records (vt, o, g, s, usemtl, mtllib, l, p) are ignored.
[[nodiscard]] TriangleMesh readObj(std::istream& in);

// True for file names ending in .stl or .obj (any letter case).
[[nodiscard]] bool isSupportedMeshFile(const std::string& path);

// Loads by extension. Errors name the file.
[[nodiscard]] TriangleMesh loadMesh(const std::string& path);

// Loads every file once. Failures are remembered too, so a missing file is not re-read every frame.
class MeshCache {
public:
    // The mesh, or nullptr when it cannot be loaded; then *error (if given) receives the reason.
    std::shared_ptr<const TriangleMesh> find(const std::string& path, std::string* error = nullptr);

    void clear() { entries.clear(); }

private:
    struct Entry {
        std::shared_ptr<const TriangleMesh> mesh;
        std::string error;
    };

    std::unordered_map<std::string, Entry> entries;
};

} // namespace robotics

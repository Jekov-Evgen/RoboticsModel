// STL/OBJ readers, mesh cache, and .robot version 4 (visual meshes, solver iterations).
#include "test_harness.hpp"

#include "robotics/io/mesh.hpp"
#include "robotics/io/urdf.hpp"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>

using namespace robotics;
using namespace test;

namespace {

std::string firstLineOf(const std::string& text) {
    return text.substr(0, text.find('\n'));
}

std::string meshPath(const std::string& name) {
    return std::string(ROBOTICS_SOURCE_DIR) + "/examples/urdf/demo_meshes/meshes/" + name;
}

void appendFloat(std::string& out, float value) {
    char bytes[4];
    std::memcpy(bytes, &value, 4);
    out.append(bytes, 4); // Test hosts are little-endian, like the STL format.
}

} // namespace

TEST_CASE("stl ascii and binary") {
    std::istringstream ascii("solid t\n facet normal 0 0 0\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n"
                             "   vertex 0 1 0\n  endloop\n endfacet\nendsolid t\n");
    TriangleMesh mesh = readStl(ascii);
    check(mesh.triangleCount() == 1 && mesh.normals.size() == 3, "ASCII STL triangle");
    near(mesh.normals[0].z, 1, 1e-12, "Normal computed from the vertices");

    // Binary STL whose header starts with "solid" (common in exported files) must still be read as binary.
    std::string binary = "solid but actually binary";
    binary.resize(80, ' ');
    std::uint32_t count = 1;
    binary.append(reinterpret_cast<const char*>(&count), 4);
    for (float v : {0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 2.f, 0.f, 0.f, 0.f, 0.f, 3.f}) {
        appendFloat(binary, v);
    }
    binary.append(2, '\0');
    std::istringstream binaryIn(binary);
    mesh = readStl(binaryIn);
    check(mesh.triangleCount() == 1, "Binary STL triangle");
    MeshBounds b = mesh.bounds();
    near(b.max.x, 2, 1e-12, "Binary STL x");
    near(b.max.z, 3, 1e-12, "Binary STL z");
    // Padded binary files (trailing bytes after the last record) are accepted, even with a "solid" header.
    std::istringstream padded(binary + std::string(37, '\0'));
    check(readStl(padded).triangleCount() == 1, "Padded binary STL");

    std::istringstream truncated("solid t\n vertex 0 0 0\n vertex 1 0\n");
    rejects([&] { (void)readStl(truncated); });
    std::istringstream garbage("not a mesh at all, and too short to be binary");
    rejects([&] { (void)readStl(garbage); });

    TriangleMesh base = loadMesh(meshPath("base.stl"));
    check(base.triangleCount() == 60, "Example binary STL");
    TriangleMesh tip = loadMesh(meshPath("tip_ascii.stl"));
    check(tip.triangleCount() == 6, "Example ASCII STL");
}

TEST_CASE("obj polygons, normals and negative indices") {
    // A concave L-shaped hexagon: ear clipping must not create triangles outside it.
    std::istringstream concave(
        "# L shape\nv 0 0 0\nv 2 0 0\nv 2 1 0\nv 1 1 0\nv 1 2 0\nv 0 2 0\nf 1 2 3 4 5 6\n");
    TriangleMesh mesh = readObj(concave);
    check(mesh.triangleCount() == 4, "Hexagon -> 4 triangles");
    double area = 0;
    for (std::size_t t = 0; t < mesh.triangleCount(); ++t) {
        Vec3 a = mesh.vertices[3 * t];
        Vec3 n = (mesh.vertices[3 * t + 1] - a).cross(mesh.vertices[3 * t + 2] - a);
        check(n.z > 0, "Triangulation keeps the polygon's winding");
        area += n.length() / 2;
        Vec3 centroid = (a + mesh.vertices[3 * t + 1] + mesh.vertices[3 * t + 2]) / 3;
        check(!(centroid.x > 1 && centroid.y > 1), "Triangle outside the concave polygon");
    }
    near(area, 3, 1e-12, "Triangulated area equals the L area");

    std::istringstream withNormals("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvn 0 0 2\nvt 0 0\n"
                                   "f -4/1/1 -3/1/1 -2/1/1 -1/1/1\n");
    mesh = readObj(withNormals);
    check(mesh.triangleCount() == 2, "Quad with negative indices");
    near(mesh.normals[0].z, 1, 1e-12, "vn normals are normalized");

    std::istringstream badIndex("v 0 0 0\nv 1 0 0\nf 1 2 3\n");
    std::string error;
    try {
        (void)readObj(badIndex);
    } catch (const std::exception& e) {
        error = e.what();
    }
    check(error.find("line 3") != std::string::npos && error.find("out of range") != std::string::npos,
          "OBJ index error names the line");
    std::istringstream empty("# nothing\n");
    rejects([&] { (void)readObj(empty); });

    TriangleMesh arm = loadMesh(meshPath("arm.obj"));
    check(arm.triangleCount() == 6 * 2 + 2 * 4, "Example OBJ: 6 quads + 2 hexagons");
    rejects([] { (void)loadMesh("model.dae"); });
}

TEST_CASE("mesh cache loads once and remembers failures") {
    MeshCache cache;
    auto first = cache.find(meshPath("arm.obj"));
    auto second = cache.find(meshPath("arm.obj"));
    check(first && first == second, "Cached mesh is shared");
    std::string error;
    check(!cache.find("/nonexistent/mesh.stl", &error) && error.find("Cannot open") != std::string::npos,
          "Missing mesh reported");
}

TEST_CASE("scene format v4 stores visual meshes and solver iterations") {
    World w = makeRoverDemo();
    check(w.iterations == 32, "Rover uses 32 iterations");
    w.bodies[0].visuals.push_back({"/robots/meshes/chassis \"v2\".stl",
                                   {0.001, 0.002, 0.003},
                                   {0.1, 0.2, 0.3},
                                   Quat::axisAngle({0, 1, 0}, 0.5)});
    w.bodies[1].primitiveHidden = true;
    std::stringstream data;
    writeScene(w, data);
    check(firstLineOf(data.str()) == "ROBOTICS_SCENE 4", "Visuals need version 4");
    World loaded = readScene(data);
    check(loaded.iterations == 32, "Iterations lost");
    check(loaded.bodies[0].visuals.size() == 1
              && loaded.bodies[0].visuals[0].path == w.bodies[0].visuals[0].path
              && loaded.bodies[0].visuals[0].scale.y == 0.002
              && loaded.bodies[0].visuals[0].position.z == 0.3,
          "Visual mesh lost");
    check(loaded.bodies[1].primitiveHidden && !loaded.bodies[2].primitiveHidden,
          "Hidden primitive flag lost");
    std::stringstream again;
    writeScene(loaded, again);
    check(again.str() == data.str(), "Second v4 round trip changed the file");

    // Iterations alone are enough for version 4 (the rover used to reload with 24).
    std::stringstream rover;
    writeScene(makeRoverDemo(), rover);
    check(firstLineOf(rover.str()) == "ROBOTICS_SCENE 4", "Non-default iterations need version 4");
    check(readScene(rover).iterations == 32, "Rover iterations after reload");
    // Default worlds keep the oldest format.
    std::stringstream plain;
    writeScene(makeDemo(), plain);
    check(firstLineOf(plain.str()) == "ROBOTICS_SCENE 2", "Plain world must stay version 2");

    // Version 4 records are validated.
    std::string text = data.str();
    std::string broken = text;
    broken.replace(broken.find("ITERATIONS 32"), 13, "ITERATIONS 0");
    rejects([&] {
        std::stringstream in(broken);
        (void)readScene(in);
    });
    broken.replace(broken.find("ITERATIONS 0"), 12, "ITERATIONS 999");
    std::string error;
    try {
        std::stringstream in(broken);
        (void)readScene(in);
    } catch (const std::exception& e) {
        error = e.what();
    }
    check(error.find("iterations must be in") != std::string::npos, "Out-of-range ITERATIONS message");
    broken = text;
    broken.replace(broken.find("MESH"), 4, "MESS");
    rejects([&] {
        std::stringstream in(broken);
        (void)readScene(in);
    });
}

TEST_CASE("saved scenes store mesh paths relative to the scene folder") {
    namespace fs = std::filesystem;
    fs::path folder = fs::temp_directory_path() / "robotics_visual_format_test";
    fs::remove_all(folder);
    fs::create_directories(folder / "meshes");
    std::string inside = (folder / "meshes" / "part.stl").string();
    World w;
    Body body;
    body.visuals.push_back({inside, {1, 1, 1}, {}, {}});
    body.visuals.push_back({"/elsewhere/other.obj", {1, 1, 1}, {}, {}});
    w.addBody(body);
    std::string scene = (folder / "robot.robot").string();
    saveScene(w, scene);
    std::ifstream file(scene);
    std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    check(text.find("MESH \"meshes/part.stl\"") != std::string::npos,
          "Path inside the folder stored relative");
    check(text.find("MESH \"/elsewhere/other.obj\"") != std::string::npos, "Outside path kept absolute");
    World loaded = loadScene(scene);
    check(fs::path(loaded.bodies[0].visuals[0].path) == fs::path(inside).lexically_normal(),
          "Relative path resolved against the scene folder");
    fs::remove_all(folder);
}

TEST_CASE("saved hulls store their mesh path relative to the scene folder") {
    namespace fs = std::filesystem;
    fs::path folder = fs::temp_directory_path() / "robotics_hull_scene";
    fs::create_directories(folder);
    auto hull = std::make_shared<ConvexHull>(*buildConvexHull(
        {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}));
    hull->sourcePath = (folder / "meshes" / "rock.stl").string();
    Body body;
    setConvexHull(body, hull);
    body.position = {0, 2, 0};
    World w;
    w.addBody(body);
    std::string scene = (folder / "hull.robot").string();
    saveScene(w, scene);
    std::ifstream in(scene);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    check(text.find("HULL \"meshes/rock.stl\"") != std::string::npos, "Relative hull path");
    World loaded = loadScene(scene);
    check(fs::path(loaded.bodies[0].hull->sourcePath) == fs::path(hull->sourcePath).lexically_normal(),
          "Hull path resolved against the scene folder");
    fs::remove_all(folder);
}

TEST_CASE("imported urdf robots survive a save/load round trip") {
    UrdfImportOptions options;
    options.fixBase = true;
    World w =
        loadUrdf(std::string(ROBOTICS_SOURCE_DIR) + "/examples/urdf/demo_meshes/urdf/mesh_arm.urdf", options)
            .world;
    std::stringstream data;
    writeScene(w, data);
    World loaded = readScene(data);
    check(loaded.bodies.size() == w.bodies.size() && loaded.bodies[1].visuals.size() == 2, "Visuals saved");
    std::stringstream again;
    writeScene(loaded, again);
    check(again.str() == data.str(), "Imported robot round trip");
}

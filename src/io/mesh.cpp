#include "robotics/io/mesh.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace robotics {
namespace {

constexpr std::size_t maxTriangles = 20'000'000;
constexpr std::size_t binaryStlHeader = 84;
constexpr std::size_t binaryStlRecord = 50;

std::string readAll(std::istream& in) {
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::string lowercase(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

void addTriangle(TriangleMesh& mesh, Vec3 a, Vec3 b, Vec3 c) {
    Vec3 normal = (b - a).cross(c - a).normalized();
    for (Vec3 p : {a, b, c}) {
        mesh.vertices.push_back(p);
        mesh.normals.push_back(normal);
    }
}

std::uint32_t readUint32(const char* data) {
    std::array<unsigned char, 4> bytes{};
    std::memcpy(bytes.data(), data, 4);
    return static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8)
           | (static_cast<std::uint32_t>(bytes[2]) << 16) | (static_cast<std::uint32_t>(bytes[3]) << 24);
}

double readFloat32(const char* data) {
    return static_cast<double>(std::bit_cast<float>(readUint32(data)));
}

void checkFinite(Vec3 v, const std::string& where) {
    if (!v.finite()) {
        throw std::runtime_error(where + ": coordinates must be finite numbers");
    }
}

TriangleMesh parseBinaryStl(const std::string& data, std::uint32_t count) {
    if (count > maxTriangles) {
        throw std::runtime_error("STL has too many triangles");
    }
    TriangleMesh mesh;
    mesh.vertices.reserve(std::size_t{count} * 3);
    mesh.normals.reserve(std::size_t{count} * 3);
    for (std::uint32_t t = 0; t < count; ++t) {
        const char* record = data.data() + binaryStlHeader + std::size_t{t} * binaryStlRecord;
        std::array<Vec3, 3> corners;
        for (int k = 0; k < 3; ++k) {
            const char* p = record + 12 + 12 * k;
            corners[static_cast<std::size_t>(k)] = {readFloat32(p), readFloat32(p + 4), readFloat32(p + 8)};
            checkFinite(corners[static_cast<std::size_t>(k)], "STL triangle " + std::to_string(t + 1));
        }
        addTriangle(mesh, corners[0], corners[1], corners[2]);
    }
    return mesh;
}

TriangleMesh parseAsciiStl(const std::string& data) {
    std::istringstream in(data);
    in.imbue(std::locale::classic());
    TriangleMesh mesh;
    std::vector<Vec3> pending;
    std::string line;
    int lineNumber = 0;
    while (std::getline(in, line)) {
        ++lineNumber;
        std::istringstream words(line);
        words.imbue(std::locale::classic());
        std::string keyword;
        if (!(words >> keyword) || lowercase(keyword) != "vertex") {
            continue;
        }
        Vec3 v;
        if (!(words >> v.x >> v.y >> v.z)) {
            throw std::runtime_error("STL line " + std::to_string(lineNumber)
                                     + ": expected three numbers after 'vertex'");
        }
        checkFinite(v, "STL line " + std::to_string(lineNumber));
        pending.push_back(v);
        if (pending.size() == 3) {
            addTriangle(mesh, pending[0], pending[1], pending[2]);
            pending.clear();
            if (mesh.triangleCount() > maxTriangles) {
                throw std::runtime_error("STL has too many triangles");
            }
        }
    }
    if (!pending.empty()) {
        throw std::runtime_error("ASCII STL ends inside a facet (vertex count is not a multiple of 3)");
    }
    if (mesh.empty()) {
        throw std::runtime_error("ASCII STL contains no triangles");
    }
    return mesh;
}

// ---- OBJ ---------------------------------------------------------------------------------------

// Ear clipping in the polygon's best-fit plane; returns index triples. Falls back to a fan for
// degenerate or self-intersecting input.
std::vector<std::array<std::size_t, 3>> triangulate(const std::vector<Vec3>& points) {
    const std::size_t n = points.size();
    std::vector<std::array<std::size_t, 3>> result;
    if (n == 3) {
        result.push_back({0, 1, 2});
        return result;
    }
    Vec3 normal; // Newell's method: robust for non-planar and concave polygons.
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3& a = points[i];
        const Vec3& b = points[(i + 1) % n];
        normal += Vec3{(a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)};
    }
    auto fan = [&](const std::vector<std::size_t>& order) {
        for (std::size_t i = 1; i + 1 < order.size(); ++i) {
            result.push_back({order[0], order[i], order[i + 1]});
        }
    };
    std::vector<std::size_t> remaining(n);
    for (std::size_t i = 0; i < n; ++i) {
        remaining[i] = i;
    }
    if (normal.length() < 1e-20) {
        fan(remaining);
        return result;
    }
    normal = normal.normalized();
    Vec3 u = (std::abs(normal.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0}).cross(normal).normalized();
    Vec3 v = normal.cross(u);
    std::vector<std::array<double, 2>> flat(n);
    for (std::size_t i = 0; i < n; ++i) {
        flat[i] = {points[i].dot(u), points[i].dot(v)};
    }
    auto cross2 = [&](std::size_t a, std::size_t b, std::size_t c) {
        return (flat[b][0] - flat[a][0]) * (flat[c][1] - flat[a][1])
               - (flat[b][1] - flat[a][1]) * (flat[c][0] - flat[a][0]);
    };
    auto inside = [&](std::size_t p, std::size_t a, std::size_t b, std::size_t c) {
        return cross2(a, b, p) >= 0 && cross2(b, c, p) >= 0 && cross2(c, a, p) >= 0;
    };
    while (remaining.size() > 3) {
        bool clipped = false;
        const std::size_t count = remaining.size();
        for (std::size_t i = 0; i < count; ++i) {
            std::size_t a = remaining[(i + count - 1) % count];
            std::size_t b = remaining[i];
            std::size_t c = remaining[(i + 1) % count];
            if (cross2(a, b, c) <= 1e-30) {
                continue; // Reflex or degenerate corner.
            }
            bool ear = true;
            for (std::size_t other : remaining) {
                if (other != a && other != b && other != c && inside(other, a, b, c)) {
                    ear = false;
                    break;
                }
            }
            if (ear) {
                result.push_back({a, b, c});
                remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
                clipped = true;
                break;
            }
        }
        if (!clipped) {
            fan(remaining);
            return result;
        }
    }
    result.push_back({remaining[0], remaining[1], remaining[2]});
    return result;
}

// Resolves a 1-based (or negative, relative) OBJ index.
std::size_t objIndex(long long index, std::size_t count, int lineNumber, const char* what) {
    long long resolved = index > 0 ? index - 1 : static_cast<long long>(count) + index;
    if (index == 0 || resolved < 0 || resolved >= static_cast<long long>(count)) {
        throw std::runtime_error("OBJ line " + std::to_string(lineNumber) + ": " + what + " index "
                                 + std::to_string(index) + " is out of range (" + std::to_string(count)
                                 + " defined so far)");
    }
    return static_cast<std::size_t>(resolved);
}

long long parseIndex(std::string_view text, int lineNumber) {
    long long value = 0;
    bool negative = false;
    std::size_t i = 0;
    if (!text.empty() && (text[0] == '-' || text[0] == '+')) {
        negative = text[0] == '-';
        i = 1;
    }
    if (i == text.size()) {
        throw std::runtime_error("OBJ line " + std::to_string(lineNumber) + ": invalid face index '"
                                 + std::string(text) + "'");
    }
    for (; i < text.size(); ++i) {
        if (text[i] < '0' || text[i] > '9' || value > 1'000'000'000) {
            throw std::runtime_error("OBJ line " + std::to_string(lineNumber) + ": invalid face index '"
                                     + std::string(text) + "'");
        }
        value = value * 10 + (text[i] - '0');
    }
    return negative ? -value : value;
}

} // namespace

MeshBounds TriangleMesh::bounds() const {
    if (vertices.empty()) {
        throw std::runtime_error("Mesh has no vertices");
    }
    MeshBounds b{vertices[0], vertices[0]};
    for (const Vec3& p : vertices) {
        b.min = {std::min(b.min.x, p.x), std::min(b.min.y, p.y), std::min(b.min.z, p.z)};
        b.max = {std::max(b.max.x, p.x), std::max(b.max.y, p.y), std::max(b.max.z, p.z)};
    }
    return b;
}

TriangleMesh readStl(std::istream& in) {
    std::string data = readAll(in);
    std::uint32_t count = data.size() >= binaryStlHeader ? readUint32(data.data() + 80) : 0;
    // Exporters may pad binary files, so only require that the declared triangles fit.
    const std::size_t binarySize = binaryStlHeader + std::size_t{count} * binaryStlRecord;
    const bool binaryFits = count > 0 && data.size() >= binarySize;
    if (binaryFits && data.size() == binarySize) {
        return parseBinaryStl(data, count);
    }
    std::size_t first = data.find_first_not_of(" \t\r\n");
    if (first != std::string::npos && lowercase(data.substr(first, 5)) == "solid") {
        try {
            return parseAsciiStl(data);
        } catch (const std::exception&) {
            if (!binaryFits) {
                throw;
            }
            // A binary file whose header happens to start with "solid".
        }
    }
    if (binaryFits) {
        return parseBinaryStl(data, count);
    }
    throw std::runtime_error("Not an STL file: neither ASCII ('solid ...') nor a binary STL with "
                             + std::to_string(count) + " triangles (file too short)");
}

TriangleMesh readObj(std::istream& in) {
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    TriangleMesh mesh;
    std::string line;
    int lineNumber = 0;
    while (std::getline(in, line)) {
        ++lineNumber;
        std::istringstream words(line);
        words.imbue(std::locale::classic());
        std::string keyword;
        if (!(words >> keyword) || keyword[0] == '#') {
            continue;
        }
        if (keyword == "v" || keyword == "vn") {
            Vec3 p;
            if (!(words >> p.x >> p.y >> p.z)) {
                throw std::runtime_error("OBJ line " + std::to_string(lineNumber)
                                         + ": expected three numbers after '" + keyword + "'");
            }
            checkFinite(p, "OBJ line " + std::to_string(lineNumber));
            (keyword == "v" ? positions : normals).push_back(p);
            continue;
        }
        if (keyword != "f") {
            continue;
        }
        std::vector<Vec3> corners;
        std::vector<Vec3> cornerNormals;
        bool allNormals = true;
        std::string token;
        while (words >> token) {
            std::string_view text(token);
            std::size_t slash = text.find('/');
            corners.push_back(positions[objIndex(parseIndex(text.substr(0, slash), lineNumber),
                                                 positions.size(), lineNumber, "vertex")]);
            std::size_t second = slash == std::string_view::npos ? slash : text.find('/', slash + 1);
            if (second != std::string_view::npos && second + 1 < text.size()) {
                cornerNormals.push_back(normals[objIndex(parseIndex(text.substr(second + 1), lineNumber),
                                                         normals.size(), lineNumber, "normal")]
                                            .normalized());
            } else {
                allNormals = false;
            }
        }
        if (corners.size() < 3) {
            throw std::runtime_error("OBJ line " + std::to_string(lineNumber)
                                     + ": a face needs at least 3 vertices");
        }
        for (const auto& [a, b, c] : triangulate(corners)) {
            Vec3 flat = (corners[b] - corners[a]).cross(corners[c] - corners[a]).normalized();
            for (std::size_t k : {a, b, c}) {
                mesh.vertices.push_back(corners[k]);
                Vec3 n = allNormals ? cornerNormals[k] : flat;
                mesh.normals.push_back(n.length2() > 0 ? n : flat);
            }
        }
        if (mesh.triangleCount() > maxTriangles) {
            throw std::runtime_error("OBJ has too many triangles");
        }
    }
    if (mesh.empty()) {
        throw std::runtime_error("OBJ contains no faces");
    }
    return mesh;
}

bool isSupportedMeshFile(const std::string& path) {
    std::string lower = lowercase(path);
    auto endsWith = [&](std::string_view suffix) {
        return lower.size() >= suffix.size()
               && lower.compare(lower.size() - suffix.size(), suffix.size(), suffix) == 0;
    };
    return endsWith(".stl") || endsWith(".obj");
}

TriangleMesh loadMesh(const std::string& path) {
    if (!isSupportedMeshFile(path)) {
        throw std::runtime_error("Unsupported mesh format (use .stl or .obj): " + path);
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot open mesh file: " + path);
    }
    try {
        std::string lower = lowercase(path);
        return lower.compare(lower.size() - 4, 4, ".stl") == 0 ? readStl(in) : readObj(in);
    } catch (const std::exception& e) {
        throw std::runtime_error(path + ": " + e.what());
    }
}

std::shared_ptr<const TriangleMesh> MeshCache::find(const std::string& path, std::string* error) {
    auto found = entries.find(path);
    if (found == entries.end()) {
        Entry entry;
        try {
            entry.mesh = std::make_shared<const TriangleMesh>(loadMesh(path));
        } catch (const std::exception& e) {
            entry.error = e.what();
        }
        found = entries.emplace(path, std::move(entry)).first;
    }
    if (!found->second.mesh && error) {
        *error = found->second.error;
    }
    return found->second.mesh;
}

} // namespace robotics

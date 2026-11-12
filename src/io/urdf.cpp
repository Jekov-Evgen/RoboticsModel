// URDF importer: XML -> link/joint records (parse) -> World (build). Mapping rules: docs/IMPORT.md.

#include "robotics/io/urdf.hpp"

#include "robotics/io/mesh.hpp"
#include "xml.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace robotics {
namespace {

namespace fs = std::filesystem;
using detail::XmlElement;

constexpr double placeholderDiameter = 0.04;
constexpr double minimumShapeSize = 0.002;
constexpr std::size_t maxNamesInWarning = 8;

// ---- Small math helpers ------------------------------------------------------------------------

struct Pose {
    Vec3 position{};
    Quat orientation{};
};

Pose compose(const Pose& a, const Pose& b) {
    return {a.position + a.orientation.rotate(b.position), (a.orientation * b.orientation).normalized()};
}

Pose inverse(const Pose& pose) {
    Quat conjugate = pose.orientation.conjugate();
    return {conjugate.rotate(-pose.position), conjugate};
}

// URDF rpy: fixed-axis roll about X, then pitch about Y, then yaw about Z: R = Rz(yaw) Ry(pitch) Rx(roll).
Quat rollPitchYaw(Vec3 rpy) {
    return (Quat::axisAngle({0, 0, 1}, rpy.z) * Quat::axisAngle({0, 1, 0}, rpy.y)
            * Quat::axisAngle({1, 0, 0}, rpy.x))
        .normalized();
}

// ---- Records --------------------------------------------------------------------------------------

enum class GeometryKind { Box, Sphere, Cylinder, Mesh, Unsupported };

struct Geometry {
    GeometryKind kind{GeometryKind::Box};
    Vec3 size{};
    double radius{};
    double length{};
    std::string filename;
    Vec3 scale{1, 1, 1};
    std::string unsupportedName;
    int line{};
};

struct GeometryElement {
    Pose origin;
    Geometry geometry;
};

struct LinkRecord {
    std::string name;
    int line{};
    bool hasMass{};
    double mass{};
    std::vector<GeometryElement> visuals;
    std::vector<GeometryElement> collisions;
};

struct JointRecord {
    std::string name;
    std::string type;
    std::string parent;
    std::string child;
    int line{};
    Pose origin;
    Vec3 axis{1, 0, 0};
    bool hasLimit{};
    double lower{};
    double upper{};
    bool hasEffort{};
    double effort{};
    std::string mimic;
    bool hasDynamics{};
};

struct ParsedUrdf {
    std::string robotName;
    std::vector<LinkRecord> links;
    std::vector<JointRecord> joints;
};

// ---- Parsing ---------------------------------------------------------------------------------------

class Parser {
public:
    explicit Parser(std::string source) : source_(std::move(source)) {}

    ParsedUrdf parse(const XmlElement& root) {
        if (root.name != "robot") {
            fail(root.line, root.name == "sdf" ? "this is an SDF file; only URDF (<robot>) is supported"
                                               : "the root element is <" + root.name + ">, expected <robot>");
        }
        rejectXacro(root);
        ParsedUrdf robot;
        robot.robotName = optionalAttribute(root, "name");
        for (const auto& element : root.children) {
            if (element.name == "link") {
                robot.links.push_back(link(element));
            } else if (element.name == "joint") {
                robot.joints.push_back(joint(element));
            }
            // <material>, <gazebo>, <transmission> and others do not affect the simulation.
        }
        if (robot.links.empty()) {
            fail(root.line, "<robot> contains no <link> elements");
        }
        return robot;
    }

private:
    std::string source_;

    [[noreturn]] void fail(int line, const std::string& message) const {
        throw std::runtime_error(source_ + ", line " + std::to_string(line) + ": " + message);
    }

    void rejectXacro(const XmlElement& element) const {
        if (element.name.rfind("xacro:", 0) == 0) {
            fail(element.line, "this is a xacro template (<" + element.name
                                   + ">). Expand it first, e.g. `xacro robot.urdf.xacro > robot.urdf`");
        }
        for (const auto& [key, value] : element.attributes) {
            if (value.find("${") != std::string::npos || value.find("$(") != std::string::npos) {
                fail(element.line, "attribute " + key + "=\"" + value
                                       + "\" uses a xacro expression. Expand the file first, e.g. "
                                         "`xacro robot.urdf.xacro > robot.urdf`");
            }
        }
        for (const auto& child : element.children) {
            rejectXacro(child);
        }
    }

    static std::string optionalAttribute(const XmlElement& element, std::string_view key) {
        const std::string* value = element.attribute(key);
        return value ? *value : std::string();
    }

    std::string requiredAttribute(const XmlElement& element, std::string_view key) const {
        const std::string* value = element.attribute(key);
        if (!value || value->empty()) {
            fail(element.line, "<" + element.name + "> needs a " + std::string(key) + "=\"...\" attribute");
        }
        return *value;
    }

    std::vector<double> numbers(const XmlElement& element, std::string_view key, std::size_t count) const {
        const std::string text = requiredAttribute(element, key);
        std::istringstream in(text);
        in.imbue(std::locale::classic());
        std::vector<double> values;
        std::string word;
        while (in >> word) {
            std::istringstream number(word);
            number.imbue(std::locale::classic());
            double value{};
            char extra{};
            if (!(number >> value) || number >> extra || !std::isfinite(value)) {
                fail(element.line, "<" + element.name + " " + std::string(key) + "=\"" + text + "\">: '"
                                       + word + "' is not a valid number");
            }
            values.push_back(value);
        }
        if (values.size() != count) {
            fail(element.line, "<" + element.name + " " + std::string(key) + "=\"" + text
                                   + "\"> must contain " + std::to_string(count)
                                   + (count == 1 ? " number" : " numbers"));
        }
        return values;
    }

    double number(const XmlElement& element, std::string_view key) const {
        return numbers(element, key, 1)[0];
    }

    double optionalNumber(const XmlElement& element, std::string_view key, double fallback) const {
        return element.attribute(key) ? number(element, key) : fallback;
    }

    Vec3 vector(const XmlElement& element, std::string_view key, Vec3 fallback) const {
        if (!element.attribute(key)) {
            return fallback;
        }
        auto v = numbers(element, key, 3);
        return {v[0], v[1], v[2]};
    }

    Pose origin(const XmlElement& parent) const {
        const XmlElement* element = parent.child("origin");
        if (!element) {
            return {};
        }
        return {vector(*element, "xyz", {}), rollPitchYaw(vector(*element, "rpy", {}))};
    }

    void requirePositive(const XmlElement& element, std::string_view key, double value) const {
        if (!(value > 0)) {
            fail(element.line, "<" + element.name + "> " + std::string(key) + " must be greater than zero");
        }
    }

    Geometry geometry(const XmlElement& parent) const {
        const XmlElement* holder = parent.child("geometry");
        if (!holder) {
            fail(parent.line, "<" + parent.name + "> needs a <geometry> element");
        }
        if (holder->children.empty()) {
            fail(holder->line, "<geometry> is empty: expected <box>, <sphere>, <cylinder> or <mesh>");
        }
        const XmlElement& shape = holder->children.front();
        Geometry g;
        g.line = shape.line;
        if (shape.name == "box") {
            g.kind = GeometryKind::Box;
            g.size = vector(shape, "size", {});
            if (!shape.attribute("size")) {
                requiredAttribute(shape, "size");
            }
            if (!(g.size.x > 0 && g.size.y > 0 && g.size.z > 0)) {
                fail(shape.line, "<box size> values must be greater than zero");
            }
        } else if (shape.name == "sphere") {
            g.kind = GeometryKind::Sphere;
            g.radius = number(shape, "radius");
            requirePositive(shape, "radius", g.radius);
        } else if (shape.name == "cylinder") {
            g.kind = GeometryKind::Cylinder;
            g.radius = number(shape, "radius");
            g.length = number(shape, "length");
            requirePositive(shape, "radius", g.radius);
            requirePositive(shape, "length", g.length);
        } else if (shape.name == "mesh") {
            g.kind = GeometryKind::Mesh;
            g.filename = requiredAttribute(shape, "filename");
            g.scale = vector(shape, "scale", {1, 1, 1});
            if (std::abs(g.scale.x) < 1e-12 || std::abs(g.scale.y) < 1e-12 || std::abs(g.scale.z) < 1e-12) {
                fail(shape.line, "<mesh scale> values must not be zero");
            }
        } else {
            g.kind = GeometryKind::Unsupported;
            g.unsupportedName = shape.name;
        }
        return g;
    }

    LinkRecord link(const XmlElement& element) const {
        LinkRecord record;
        record.name = requiredAttribute(element, "name");
        record.line = element.line;
        if (const XmlElement* inertial = element.child("inertial")) {
            if (const XmlElement* mass = inertial->child("mass")) {
                record.mass = number(*mass, "value");
                if (record.mass < 0) {
                    fail(mass->line, "<mass value> must not be negative");
                }
                record.hasMass = true;
            }
        }
        for (const XmlElement* visual : element.childrenNamed("visual")) {
            record.visuals.push_back({origin(*visual), geometry(*visual)});
        }
        for (const XmlElement* collision : element.childrenNamed("collision")) {
            record.collisions.push_back({origin(*collision), geometry(*collision)});
        }
        return record;
    }

    JointRecord joint(const XmlElement& element) const {
        JointRecord record;
        record.name = requiredAttribute(element, "name");
        record.type = requiredAttribute(element, "type");
        record.line = element.line;
        const XmlElement* parent = element.child("parent");
        const XmlElement* child = element.child("child");
        if (!parent || !child) {
            fail(element.line,
                 "joint '" + record.name + "' needs both <parent link=\"...\"/> and <child link=\"...\"/>");
        }
        record.parent = requiredAttribute(*parent, "link");
        record.child = requiredAttribute(*child, "link");
        record.origin = origin(element);
        if (const XmlElement* axis = element.child("axis")) {
            record.axis = vector(*axis, "xyz", {1, 0, 0});
            if (record.axis.length() < 1e-9) {
                fail(axis->line, "joint '" + record.name + "' has a zero <axis xyz>");
            }
            record.axis = record.axis.normalized();
        }
        if (const XmlElement* limit = element.child("limit")) {
            record.hasLimit = true;
            record.lower = optionalNumber(*limit, "lower", 0);
            record.upper = optionalNumber(*limit, "upper", 0);
            record.hasEffort = limit->attribute("effort") != nullptr;
            record.effort = optionalNumber(*limit, "effort", 0);
            if (record.effort < 0) {
                fail(limit->line, "joint '" + record.name + "': <limit effort> must not be negative");
            }
            if (record.lower > record.upper && (record.type == "revolute" || record.type == "prismatic")) {
                fail(limit->line, "joint '" + record.name + "': <limit lower> is greater than upper");
            }
        }
        if (const XmlElement* mimic = element.child("mimic")) {
            record.mimic = optionalAttribute(*mimic, "joint");
        }
        record.hasDynamics = element.child("dynamics") != nullptr;
        return record;
    }
};

// ---- Warnings ----------------------------------------------------------------------------------

// Collects names per message so that e.g. twenty massless links give one readable line.
class WarningList {
public:
    void add(const std::string& message) { singles_.push_back(message); }

    void addName(const std::string& group, const std::string& name) {
        auto found =
            std::find_if(groups_.begin(), groups_.end(), [&](const auto& g) { return g.first == group; });
        if (found == groups_.end()) {
            groups_.push_back({group, {name}});
        } else if (std::find(found->second.begin(), found->second.end(), name) == found->second.end()) {
            found->second.push_back(name);
        }
    }

    std::vector<std::string> finish() const {
        std::vector<std::string> result = singles_;
        for (const auto& [group, names] : groups_) {
            std::string line = group + ": ";
            for (std::size_t i = 0; i < names.size() && i < maxNamesInWarning; ++i) {
                line += (i ? ", " : "") + names[i];
            }
            if (names.size() > maxNamesInWarning) {
                line += " and " + std::to_string(names.size() - maxNamesInWarning) + " more";
            }
            result.push_back(line + ".");
        }
        return result;
    }

private:
    std::vector<std::string> singles_;
    std::vector<std::pair<std::string, std::vector<std::string>>> groups_;
};

// ---- Building --------------------------------------------------------------------------------------

struct CollisionShape {
    Shape shape{Shape::Box};
    Vec3 size{};
    Pose pose; // World pose of the body.
    std::string meshPath;
    Vec3 meshScale{1, 1, 1};
    Pose meshPose; // World pose of the mesh frame (mesh collisions only).
    std::shared_ptr<const ConvexHull> hull; // Shape::ConvexHull only.
};

struct ResolvedVisual {
    std::string path;
    Vec3 scale{1, 1, 1};
    Pose pose; // World pose of the mesh frame.
};

struct LinkState {
    int parentJoint{-1};
    std::vector<int> childJoints;
    Pose frame;
    int owner{-1}; // Body that carries this link (own, placeholder or merged-into); -1: none.
    bool ownsBody{};
    bool isStatic{};
};

bool isMovingJoint(const std::string& type) {
    return type == "revolute" || type == "continuous" || type == "prismatic";
}

double shapeVolume(const CollisionShape& s) {
    switch (s.shape) {
    case Shape::Box:
        return s.size.x * s.size.y * s.size.z;
    case Shape::Sphere:
        return pi / 6 * s.size.x * s.size.x * s.size.x;
    case Shape::Cylinder:
        return pi / 4 * s.size.x * s.size.x * s.size.z;
    case Shape::ConvexHull:
        return s.hull->volume;
    }
    return 1;
}

// Hull of a scaled mesh, placed at `meshPose`; nullptr for meshes that have no usable volume (flat, or
// thinner than the importer's smallest shape), which then stay bounding boxes.
std::shared_ptr<const ConvexHull> meshHull(const TriangleMesh& mesh, Vec3 scale, const std::string& path) {
    std::vector<Vec3> points;
    points.reserve(mesh.vertices.size());
    for (Vec3 v : mesh.vertices) {
        points.push_back(componentProduct(v, scale));
    }
    try {
        auto built = std::make_shared<ConvexHull>(*buildConvexHull(points));
        if (built->minWidth < minimumShapeSize) {
            return nullptr;
        }
        built->sourcePath = path;
        built->sourceScale = scale;
        return built;
    } catch (const std::runtime_error&) {
        return nullptr;
    }
}

// Lowest world Y of a body's collision shape.
double lowestPoint(const Body& body) {
    if (body.shape == Shape::Sphere) {
        return body.position.y - body.size.x / 2;
    }
    if (body.shape == Shape::Cylinder) {
        Vec3 axis = body.orientation.rotate({0, 0, 1});
        double radial = std::sqrt(std::max(0.0, 1 - axis.y * axis.y));
        return body.position.y - std::abs(axis.y) * body.size.z / 2 - radial * body.size.x / 2;
    }
    double lowest = body.position.y;
    if (body.shape == Shape::ConvexHull) {
        for (Vec3 v : body.hull->vertices) {
            lowest = std::min(lowest, body.point(v).y);
        }
        return lowest;
    }
    for (const Vec3& corner : body.vertices()) {
        lowest = std::min(lowest, corner.y);
    }
    return lowest;
}

std::string fileName(const std::string& path) {
    std::string name = fs::path(path).filename().string();
    return name.empty() ? path : name;
}

class Builder {
public:
    Builder(const ParsedUrdf& robot, const UrdfImportOptions& options, std::string sourcePath,
            std::string label)
        : robot_(robot), options_(options), sourcePath_(std::move(sourcePath)), label_(std::move(label)) {}

    UrdfImportResult build() {
        indexTopology();
        std::vector<int> order = traversalOrder();
        Pose base;
        if (options_.zUpToYUp) {
            base.orientation = Quat::axisAngle({1, 0, 0}, -pi / 2);
        }
        base.position = options_.spawnOffset;
        for (int index : order) {
            placeLink(index, base);
        }
        if (options_.liftAboveGround && !result_.world.bodies.empty()) {
            liftAboveGround();
        }
        result_.world.validate();
        result_.robotName = robot_.robotName;
        result_.warnings = warnings_.finish();
        return std::move(result_);
    }

private:
    const ParsedUrdf& robot_;
    const UrdfImportOptions& options_;
    std::string sourcePath_;
    std::string label_;
    std::map<std::string, int> linkIndex_;
    std::vector<LinkState> state_;
    MeshCache meshes_;
    WarningList warnings_;
    UrdfImportResult result_;

    [[noreturn]] void fail(int line, const std::string& message) const {
        throw std::runtime_error(label_ + ", line " + std::to_string(line) + ": " + message);
    }

    void indexTopology() {
        state_.resize(robot_.links.size());
        for (std::size_t i = 0; i < robot_.links.size(); ++i) {
            const LinkRecord& link = robot_.links[i];
            if (!linkIndex_.emplace(link.name, static_cast<int>(i)).second) {
                fail(link.line, "link name '" + link.name + "' is used twice");
            }
        }
        std::map<std::string, int> jointNames;
        for (std::size_t j = 0; j < robot_.joints.size(); ++j) {
            const JointRecord& joint = robot_.joints[j];
            if (!jointNames.emplace(joint.name, static_cast<int>(j)).second) {
                fail(joint.line, "joint name '" + joint.name + "' is used twice");
            }
            if (joint.type != "revolute" && joint.type != "continuous" && joint.type != "prismatic"
                && joint.type != "fixed" && joint.type != "floating" && joint.type != "planar") {
                fail(joint.line,
                     "joint '" + joint.name + "' has unknown type '" + joint.type
                         + "' (expected revolute, continuous, prismatic, fixed, floating or planar)");
            }
            auto parent = linkIndex_.find(joint.parent);
            auto child = linkIndex_.find(joint.child);
            if (parent == linkIndex_.end()) {
                fail(joint.line, "joint '" + joint.name + "' refers to parent link '" + joint.parent
                                     + "', which is not defined");
            }
            if (child == linkIndex_.end()) {
                fail(joint.line, "joint '" + joint.name + "' refers to child link '" + joint.child
                                     + "', which is not defined");
            }
            if (parent->second == child->second) {
                fail(joint.line, "joint '" + joint.name + "' connects link '" + joint.parent + "' to itself");
            }
            LinkState& childState = state_[static_cast<std::size_t>(child->second)];
            if (childState.parentJoint >= 0) {
                const JointRecord& other = robot_.joints[static_cast<std::size_t>(childState.parentJoint)];
                fail(joint.line, "link '" + joint.child + "' has two parent joints: '" + other.name
                                     + "' (line " + std::to_string(other.line) + ") and '" + joint.name
                                     + "'. URDF describes a tree; closed loops are not supported");
            }
            childState.parentJoint = static_cast<int>(j);
            state_[static_cast<std::size_t>(parent->second)].childJoints.push_back(static_cast<int>(j));
        }
    }

    // Parents before children; every link exactly once.
    std::vector<int> traversalOrder() {
        std::vector<int> roots;
        for (std::size_t i = 0; i < state_.size(); ++i) {
            if (state_[i].parentJoint < 0) {
                roots.push_back(static_cast<int>(i));
            }
        }
        if (roots.empty()) {
            fail(robot_.links.front().line,
                 "no root link found: every link has a parent, so the joints form a loop");
        }
        if (roots.size() > 1) {
            std::string names;
            for (int r : roots) {
                names += (names.empty() ? "" : ", ") + robot_.links[static_cast<std::size_t>(r)].name;
            }
            warnings_.add("The URDF has " + std::to_string(roots.size()) + " separate root links (" + names
                          + "); they are imported as unconnected parts.");
        }
        std::vector<int> order;
        std::deque<int> queue(roots.begin(), roots.end());
        while (!queue.empty()) {
            int link = queue.front();
            queue.pop_front();
            order.push_back(link);
            for (int j : state_[static_cast<std::size_t>(link)].childJoints) {
                queue.push_back(linkIndex_.at(robot_.joints[static_cast<std::size_t>(j)].child));
            }
        }
        if (order.size() != state_.size()) {
            for (std::size_t i = 0; i < state_.size(); ++i) {
                if (std::find(order.begin(), order.end(), static_cast<int>(i)) == order.end()) {
                    fail(robot_.links[i].line,
                         "link '" + robot_.links[i].name
                             + "' is part of a joint loop and cannot be reached from the root");
                }
            }
        }
        return order;
    }

    std::string resolve(const std::string& reference) const {
        return resolveUrdfMeshPath(reference, sourcePath_, options_.meshSearchPaths);
    }

    // Loads a referenced mesh; on failure records a warning for `link` and returns nullptr.
    std::shared_ptr<const TriangleMesh> mesh(const std::string& reference, const std::string& link,
                                             const char* role, std::string& path) {
        path.clear();
        if (!isSupportedMeshFile(reference)) {
            warnings_.addName(std::string(role)
                                  + " meshes in an unsupported format (only STL and OBJ are read; convert "
                                    "e.g. COLLADA .dae files with Blender or MeshLab)",
                              link + " (" + fileName(reference) + ")");
            return nullptr;
        }
        path = resolve(reference);
        if (path.empty()) {
            warnings_.addName(std::string(role) + " meshes that were not found (check the package folders)",
                              link + " (" + reference + ")");
            return nullptr;
        }
        std::string error;
        auto loaded = meshes_.find(path, &error);
        if (!loaded) {
            warnings_.add("Link '" + link + "': the " + role + " mesh could not be read: " + error);
        }
        return loaded;
    }

    std::vector<CollisionShape> collisionShapes(const LinkRecord& link, const Pose& frame) {
        std::vector<CollisionShape> shapes;
        for (const auto& element : link.collisions) {
            const Geometry& g = element.geometry;
            CollisionShape shape;
            Pose pose = compose(frame, element.origin);
            shape.pose = pose;
            switch (g.kind) {
            case GeometryKind::Box:
                shape.shape = Shape::Box;
                shape.size = g.size;
                break;
            case GeometryKind::Sphere:
                shape.shape = Shape::Sphere;
                shape.size = Vec3{1, 1, 1} * (2 * g.radius);
                break;
            case GeometryKind::Cylinder:
                shape.shape = Shape::Cylinder; // Both URDF and Body cylinders run along local Z.
                shape.size = {2 * g.radius, 2 * g.radius, g.length};
                break;
            case GeometryKind::Mesh: {
                std::string path;
                auto loaded = mesh(g.filename, link.name, "Collision", path);
                if (!loaded) {
                    continue;
                }
                shape.meshPath = path;
                shape.meshScale = g.scale;
                shape.meshPose = pose;
                if (options_.meshCollision == UrdfMeshCollision::ConvexHull) {
                    shape.hull = meshHull(*loaded, g.scale, path);
                }
                if (shape.hull) {
                    shape.shape = Shape::ConvexHull;
                    shape.size = shape.hull->boundingSize();
                    shape.hull->placeBody(pose.position, pose.orientation, shape.pose.position,
                                          shape.pose.orientation);
                    if (shape.hull->approximationError > 0) {
                        warnings_.addName("Collision hulls were simplified to 64 vertices", link.name);
                    }
                    break;
                }
                if (options_.meshCollision == UrdfMeshCollision::ConvexHull) {
                    warnings_.addName("Flat collision meshes have no volume for a hull and became boxes",
                                      link.name);
                }
                MeshBounds bounds = loaded->bounds();
                Vec3 a = componentProduct(bounds.min, g.scale);
                Vec3 b = componentProduct(bounds.max, g.scale);
                Vec3 extent{std::abs(b.x - a.x), std::abs(b.y - a.y), std::abs(b.z - a.z)};
                shape.shape = Shape::Box;
                shape.size = extent;
                shape.pose = compose(pose, Pose{(a + b) * 0.5, {}});
                warnings_.addName("Collision meshes were approximated by their bounding boxes", link.name);
                break;
            }
            case GeometryKind::Unsupported:
                warnings_.add("Link '" + link.name + "': collision shape <" + g.unsupportedName
                              + "> is not supported and was skipped.");
                continue;
            }
            Vec3 clamped{std::max(shape.size.x, minimumShapeSize), std::max(shape.size.y, minimumShapeSize),
                         std::max(shape.size.z, minimumShapeSize)};
            if (clamped.x != shape.size.x || clamped.y != shape.size.y || clamped.z != shape.size.z) {
                warnings_.addName("Very thin collision shapes were thickened to 2 mm", link.name);
                shape.size = clamped;
            }
            shapes.push_back(shape);
        }
        return shapes;
    }

    std::vector<ResolvedVisual> visuals(const LinkRecord& link, const Pose& frame,
                                        const std::vector<CollisionShape>& shapes) {
        std::vector<ResolvedVisual> result;
        bool failed = false;
        bool primitiveOnly = false;
        for (const auto& element : link.visuals) {
            const Geometry& g = element.geometry;
            if (g.kind != GeometryKind::Mesh) {
                primitiveOnly = true;
                continue;
            }
            std::string path;
            if (!mesh(g.filename, link.name, "Visual", path)) {
                failed = true;
                continue;
            }
            result.push_back({path, g.scale, compose(frame, element.origin)});
        }
        if (result.empty() && (failed || link.visuals.empty())) {
            // Better than a bounding box: draw the collision mesh itself.
            for (const auto& shape : shapes) {
                if (!shape.meshPath.empty()) {
                    result.push_back({shape.meshPath, shape.meshScale, shape.meshPose});
                }
            }
            if (failed && !result.empty()) {
                warnings_.addName(
                    "Links drawn with their collision mesh because the visual mesh is unavailable",
                    link.name);
            }
        }
        if (result.empty() && primitiveOnly && shapes.empty()) {
            warnings_.addName(
                "Visual-only primitive shapes (box/sphere/cylinder without collision) are not drawn",
                link.name);
        }
        return result;
    }

    double linkMass(const LinkRecord& link) {
        if (link.hasMass && link.mass > 0) {
            return link.mass;
        }
        warnings_.addName("Links without mass were given " + formatNumber(options_.minimumMass) + " kg",
                          link.name);
        return options_.minimumMass;
    }

    static std::string formatNumber(double value) {
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out << value;
        return out.str();
    }

    void attachVisuals(int bodyIndex, const std::vector<ResolvedVisual>& resolved) {
        Body& body = result_.world.bodies[static_cast<std::size_t>(bodyIndex)];
        Pose inverseBody = inverse({body.position, body.orientation});
        for (const auto& visual : resolved) {
            Pose local = compose(inverseBody, visual.pose);
            body.visuals.push_back({visual.path, visual.scale, local.position, local.orientation});
        }
    }

    int addBody(const std::string& name, Shape shape, Vec3 size, const Pose& pose, double mass,
                std::shared_ptr<const ConvexHull> hull = nullptr) {
        Body body;
        body.name = name;
        body.shape = shape;
        body.size = size;
        body.hull = std::move(hull);
        body.position = pose.position;
        body.orientation = pose.orientation;
        body.mass = mass;
        return result_.world.addBody(body);
    }

    void placeLink(int index, const Pose& base) {
        const LinkRecord& link = robot_.links[static_cast<std::size_t>(index)];
        LinkState& state = state_[static_cast<std::size_t>(index)];
        const JointRecord* joint =
            state.parentJoint >= 0 ? &robot_.joints[static_cast<std::size_t>(state.parentJoint)] : nullptr;
        const LinkState* parent =
            joint ? &state_[static_cast<std::size_t>(linkIndex_.at(joint->parent))] : nullptr;

        state.frame = parent ? compose(parent->frame, joint->origin) : base;
        state.isStatic = parent ? (joint->type == "fixed" && parent->isStatic)
                                : (options_.fixBase || link.name == "world");

        std::vector<CollisionShape> shapes = collisionShapes(link, state.frame);
        std::vector<ResolvedVisual> resolved = visuals(link, state.frame, shapes);
        bool anyMovingChild = std::any_of(state.childJoints.begin(), state.childJoints.end(), [&](int j) {
            return isMovingJoint(robot_.joints[static_cast<std::size_t>(j)].type);
        });

        if (!shapes.empty()) {
            createBodies(link, state, shapes, resolved);
        } else if (joint && joint->type == "fixed" && parent->owner >= 0) {
            // Massless frames such as tool0 or a sensor mount: merge into the parent's body.
            state.owner = parent->owner;
            Body& owner = result_.world.bodies[static_cast<std::size_t>(state.owner)];
            if (!owner.isStatic() && link.hasMass && link.mass > 0) {
                owner.mass += link.mass;
            }
            attachVisuals(state.owner, resolved);
        } else if ((joint && isMovingJoint(joint->type)) || anyMovingChild || !resolved.empty()) {
            // Moving links need a body for their joint even without collision geometry.
            double mass = state.isStatic ? 0 : linkMass(link);
            state.owner =
                addBody(link.name, Shape::Sphere, Vec3{1, 1, 1} * placeholderDiameter, state.frame, mass);
            state.ownsBody = true;
            result_.world.bodies[static_cast<std::size_t>(state.owner)].primitiveHidden = true;
            attachVisuals(state.owner, resolved);
            warnings_.addName(
                "Links without collision geometry got a small hidden placeholder body (4 cm sphere)",
                link.name);
        }
        // Otherwise the link is a pure reference frame (e.g. base_footprint): no body at all.

        if (joint && state.ownsBody) {
            connect(*joint, *parent, state);
        }
    }

    void createBodies(const LinkRecord& link, LinkState& state, const std::vector<CollisionShape>& shapes,
                      const std::vector<ResolvedVisual>& resolved) {
        double total = state.isStatic ? 0 : linkMass(link);
        double volume = 0;
        for (const auto& shape : shapes) {
            volume += shapeVolume(shape);
        }
        for (std::size_t i = 0; i < shapes.size(); ++i) {
            const CollisionShape& shape = shapes[i];
            std::string name = i == 0 ? link.name : link.name + "_collision_" + std::to_string(i + 1);
            double mass = shapes.size() == 1 ? total : total * shapeVolume(shape) / volume;
            int body = addBody(name, shape.shape, shape.size, shape.pose, mass, shape.hull);
            if (i == 0) {
                state.owner = body;
                state.ownsBody = true;
                continue;
            }
            // Extra collision elements are welded to the first one; the link's meshes are drawn by the first.
            result_.world.bodies[static_cast<std::size_t>(body)].primitiveHidden = !resolved.empty();
            if (!state.isStatic) {
                result_.world.connect(state.owner, body, JointType::Fixed, shape.pose.position);
            }
        }
        if (shapes.size() > 1) {
            warnings_.addName(
                "Links with several collision shapes became several parts welded by fixed joints", link.name);
        }
        attachVisuals(state.owner, resolved);
    }

    void connect(const JointRecord& joint, const LinkState& parent, const LinkState& child) {
        World& world = result_.world;
        if (joint.type == "floating" || joint.type == "planar") {
            warnings_.add("Joint '" + joint.name + "' is " + joint.type + ", which is not supported; link '"
                          + joint.child + "' was imported as a free part.");
            return;
        }
        if (parent.owner < 0) {
            return; // Child of a pure reference frame: a free part, like the root itself.
        }
        Vec3 anchor = child.frame.position;
        if (joint.type == "fixed") {
            if (!(parent.isStatic && child.isStatic)) {
                world.connect(parent.owner, child.owner, JointType::Fixed, anchor);
            }
            return;
        }
        Vec3 axis = child.frame.orientation.rotate(joint.axis);
        JointType type = joint.type == "prismatic" ? JointType::Prismatic : JointType::Hinge;
        Joint& j = world.joints[static_cast<std::size_t>(
            world.connect(parent.owner, child.owner, type, anchor, axis))];
        if (joint.hasEffort) {
            j.maxTorque = joint.effort;
        } else {
            j.maxTorque = options_.defaultEffort;
            if (joint.type != "continuous") {
                warnings_.addName("Joints without <limit effort> use the default effort of "
                                      + formatNumber(options_.defaultEffort)
                                      + (type == JointType::Hinge ? " N·m" : " N"),
                                  joint.name);
            }
        }
        if (joint.type != "continuous") {
            if (joint.hasLimit) {
                j.setLimits(joint.lower, joint.upper);
            } else {
                warnings_.addName("Joints without <limit> move without range limits", joint.name);
            }
        }
        if (!joint.mimic.empty()) {
            warnings_.add("Joint '" + joint.name + "' mimics '" + joint.mimic
                          + "'; mimic joints are not supported, so it moves independently.");
        }
        if (joint.hasDynamics) {
            warnings_.addName("Joint <dynamics> damping/friction is not simulated", joint.name);
        }
    }

    void liftAboveGround() {
        double lowest = lowestPoint(result_.world.bodies.front());
        for (const Body& body : result_.world.bodies) {
            lowest = std::min(lowest, lowestPoint(body));
        }
        if (lowest >= 0) {
            return;
        }
        bool fixedToWorld = false;
        for (const LinkState& link : state_) {
            fixedToWorld = fixedToWorld || (link.parentJoint < 0 && link.isStatic);
        }
        if (fixedToWorld) {
            // Lifting would make the bolted-down base float; report the overlap instead.
            if (lowest < -1e-6) {
                warnings_.add(
                    "Some parts start " + formatNumber(std::round(-lowest * 1000) / 10)
                    + " cm below the floor at the zero pose; the base is fixed, so the robot was not "
                      "lifted. Move the joints before running or expect ground contacts.");
            }
            return;
        }
        // A uniform translation keeps every joint's local anchors valid.
        for (Body& body : result_.world.bodies) {
            body.position.y -= lowest;
        }
    }
};

std::vector<fs::path> packagePathEntries() {
    std::vector<fs::path> result;
    if (const char* env = std::getenv("ROS_PACKAGE_PATH")) {
        std::string list = env;
        std::size_t start = 0;
        while (start <= list.size()) {
            std::size_t end = list.find(':', start);
            std::string entry =
                list.substr(start, end == std::string::npos ? std::string::npos : end - start);
            if (!entry.empty()) {
                result.emplace_back(entry);
            }
            if (end == std::string::npos) {
                break;
            }
            start = end + 1;
        }
    }
    return result;
}

bool isFile(const fs::path& path) {
    std::error_code ignored;
    return fs::is_regular_file(path, ignored);
}

// True when `folder` is a ROS package of this name: folder name or <name> in its package.xml.
bool isPackageFolder(const fs::path& folder, const std::string& package) {
    if (folder.filename() == package) {
        return true;
    }
    fs::path manifest = folder / "package.xml";
    if (!isFile(manifest)) {
        return false;
    }
    try {
        std::ifstream in(manifest, std::ios::binary);
        std::string text(std::istreambuf_iterator<char>(in), {});
        const XmlElement manifestRoot = detail::parseXml(text, "package.xml");
        const XmlElement* name = manifestRoot.child("name");
        if (!name) {
            return false;
        }
        std::string value = name->text;
        value.erase(0, value.find_first_not_of(" \t\r\n"));
        value.erase(value.find_last_not_of(" \t\r\n") + 1);
        return value == package;
    } catch (const std::exception&) {
        return false; // A broken manifest simply does not match.
    }
}

} // namespace

std::string resolveUrdfMeshPath(const std::string& reference, const std::string& urdfPath,
                                const std::vector<std::string>& searchPaths) {
    std::error_code ignored;
    fs::path urdfDirectory =
        urdfPath.empty() ? fs::current_path(ignored) : fs::absolute(urdfPath, ignored).parent_path();
    std::vector<fs::path> folders;
    for (const auto& entry : searchPaths) {
        folders.push_back(fs::absolute(entry, ignored));
    }
    for (auto& entry : packagePathEntries()) {
        folders.push_back(fs::absolute(entry, ignored));
    }
    auto found = [&](const fs::path& path) {
        return fs::absolute(path, ignored).lexically_normal().string();
    };

    for (std::string_view scheme : {"package://", "model://"}) {
        if (reference.rfind(scheme, 0) != 0) {
            continue;
        }
        std::string rest = reference.substr(scheme.size());
        std::size_t slash = rest.find('/');
        if (slash == std::string::npos || slash == 0) {
            return {};
        }
        std::string package = rest.substr(0, slash);
        std::string inside = rest.substr(slash + 1);
        // Walk up from the URDF: a folder named like the package, or a folder that contains it.
        for (fs::path dir = urdfDirectory; !dir.empty(); dir = dir.parent_path()) {
            if (isFile(dir / inside) && isPackageFolder(dir, package)) {
                return found(dir / inside);
            }
            if (isFile(dir / package / inside)) {
                return found(dir / package / inside);
            }
            if (dir == dir.parent_path()) {
                break;
            }
        }
        for (const auto& folder : folders) {
            if (isFile(folder / package / inside)) {
                return found(folder / package / inside);
            }
            if (isFile(folder / inside)) {
                return found(folder / inside);
            }
            // Packages whose folder name differs from their package.xml <name> (e.g. git checkouts).
            std::error_code error;
            for (fs::directory_iterator it(folder, error), end; !error && it != end; it.increment(error)) {
                if (it->is_directory(error) && isFile(it->path() / inside)
                    && isPackageFolder(it->path(), package)) {
                    return found(it->path() / inside);
                }
            }
        }
        return {};
    }
    std::string plain = reference.rfind("file://", 0) == 0 ? reference.substr(7) : reference;
    fs::path path(plain);
    if (path.is_absolute()) {
        return isFile(path) ? found(path) : std::string();
    }
    if (isFile(urdfDirectory / path)) {
        return found(urdfDirectory / path);
    }
    for (const auto& folder : folders) {
        if (isFile(folder / path)) {
            return found(folder / path);
        }
    }
    return {};
}

UrdfImportResult importUrdf(std::istream& in, const UrdfImportOptions& options,
                            const std::string& sourcePath) {
    if (!(options.minimumMass > 0) || !std::isfinite(options.minimumMass) || !(options.defaultEffort >= 0)
        || !std::isfinite(options.defaultEffort) || !options.spawnOffset.finite()) {
        throw std::runtime_error("Invalid URDF import options");
    }
    std::string text(std::istreambuf_iterator<char>(in), {});
    std::string label = sourcePath.empty() ? std::string("URDF") : fileName(sourcePath);
    XmlElement root = detail::parseXml(text, label);
    ParsedUrdf robot = Parser(label).parse(root);
    return Builder(robot, options, sourcePath, label).build();
}

UrdfImportResult loadUrdf(const std::string& path, const UrdfImportOptions& options) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot open URDF file: " + path);
    }
    return importUrdf(in, options, path);
}

} // namespace robotics

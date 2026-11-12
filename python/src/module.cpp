// Python bindings for the Robotics Model core (module robotics_model._core).
//
// Design rules:
//  * Bodies, joints and sensors live in std::vectors inside World, so Python never holds references into them.
//    It holds small handles (owner + index) that re-resolve on every access and keep the World alive.
//  * Vectors cross the boundary as numpy arrays; anything array-like (list, tuple, ndarray) is accepted as input.
//  * Enums accept strings, e.g. world.connect(a, b, "hinge", ...). Matching is case-insensitive.
//  * World.step() releases the GIL, so long runs do not block other Python threads.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "robotics/demo_scenes.hpp"
#include "robotics/io/robot_format.hpp"
#include "robotics/io/urdf.hpp"
#include "robotics/sensors/raycast.hpp"
#include "robotics/sensors/sensor_system.hpp"
#include "robotics/world.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

namespace py = pybind11;
using namespace robotics;

namespace {

using Array = py::array_t<double>;
using Input = py::array_t<double, py::array::c_style | py::array::forcecast>;

// ---------- conversions ----------

Array toArray(Vec3 v) {
    Array a(3);
    auto r = a.mutable_unchecked<1>();
    r(0) = v.x;
    r(1) = v.y;
    r(2) = v.z;
    return a;
}

Array toArray(Quat q) {
    Array a(4);
    auto r = a.mutable_unchecked<1>();
    r(0) = q.w;
    r(1) = q.x;
    r(2) = q.y;
    r(3) = q.z;
    return a;
}

Array toArray(const std::vector<double>& values) {
    Array a(static_cast<py::ssize_t>(values.size()));
    std::copy(values.begin(), values.end(), a.mutable_data());
    return a;
}

Array toMatrix(const std::vector<Vec3>& rows) {
    Array a({static_cast<py::ssize_t>(rows.size()), static_cast<py::ssize_t>(3)});
    auto r = a.mutable_unchecked<2>();
    for (std::size_t i = 0; i < rows.size(); ++i) {
        r(static_cast<py::ssize_t>(i), 0) = rows[i].x;
        r(static_cast<py::ssize_t>(i), 1) = rows[i].y;
        r(static_cast<py::ssize_t>(i), 2) = rows[i].z;
    }
    return a;
}

Input asInput(const py::object& value, py::ssize_t count, const char* what) {
    Input a = Input::ensure(value);
    if (!a || a.ndim() != 1 || a.shape(0) != count) {
        throw py::value_error(std::string(what) + " must be a sequence of " + std::to_string(count) + " numbers");
    }
    return a;
}

Vec3 toVec3(const py::object& value, const char* what) {
    Input a = asInput(value, 3, what);
    Vec3 v{a.data()[0], a.data()[1], a.data()[2]};
    if (!v.finite()) {
        throw py::value_error(std::string(what) + " must be finite");
    }
    return v;
}

Quat toQuat(const py::object& value, const char* what) {
    Input a = asInput(value, 4, what);
    Quat q{a.data()[0], a.data()[1], a.data()[2], a.data()[3]};
    return q.normalized(); // Throws for zero or non-finite input.
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

// Python classes of the registered enums, for toEnum(). Non-owning: the classes live as long as the module.
std::map<std::type_index, py::handle>& enumClasses() {
    static std::map<std::type_index, py::handle> classes;
    return classes;
}

// Registers an enum whose members are also reachable from strings (case-insensitive, '-' and ' ' mean '_').
template <class E>
void defEnum(py::module_& m, const char* name, std::vector<std::pair<std::string, E>> items) {
    py::enum_<E> e(m, name);
    for (const auto& [label, value] : items) {
        std::string upper = label;
        std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        e.value(upper.c_str(), value);
    }
    std::string typeName = name;
    enumClasses()[typeid(E)] = e;
    e.def(py::init([items, typeName](const std::string& text) {
        std::string key = lower(text);
        std::replace(key.begin(), key.end(), '-', '_');
        std::replace(key.begin(), key.end(), ' ', '_');
        for (const auto& [label, value] : items) {
            if (lower(label) == key) {
                return value;
            }
        }
        std::string known;
        for (const auto& item : items) {
            known += (known.empty() ? "" : ", ") + lower(item.first);
        }
        throw py::value_error("unknown " + typeName + " '" + text + "' (expected one of: " + known + ")");
    }));
    py::implicitly_convertible<py::str, E>();
}

// An enum argument given as a member or as a string. Unknown strings raise ValueError with the list of names
// (pybind11's implicit conversion would turn that into a less helpful "incompatible arguments" TypeError).
template <class E>
E toEnum(const py::object& value) {
    if (py::isinstance<E>(value)) {
        return value.cast<E>();
    }
    if (py::isinstance<py::str>(value)) {
        return enumClasses().at(typeid(E))(value).template cast<E>();
    }
    throw py::type_error("expected an enum member or a string");
}

// ControllerConfig::validate() throws runtime_error; for Python arguments that is a ValueError.
void validated(const ControllerConfig& c) {
    try {
        c.validate();
    } catch (const std::runtime_error& e) {
        throw py::value_error(e.what());
    }
}

// ---------- handles ----------

struct BodyRef {
    py::object owner;
    World* world;
    int index;
    [[nodiscard]] Body& get() const {
        if (index < 0 || static_cast<std::size_t>(index) >= world->bodies.size()) {
            throw py::index_error("body handle is no longer valid");
        }
        return world->bodies[static_cast<std::size_t>(index)];
    }
};

struct JointRef {
    py::object owner;
    World* world;
    int index;
    [[nodiscard]] Joint& get() const {
        if (index < 0 || static_cast<std::size_t>(index) >= world->joints.size()) {
            throw py::index_error("joint handle is no longer valid");
        }
        return world->joints[static_cast<std::size_t>(index)];
    }
};

struct SensorRef {
    py::object owner;
    World* world;
    int index;
    [[nodiscard]] Sensor& get() const {
        if (index < 0 || static_cast<std::size_t>(index) >= world->sensors.size()) {
            throw py::index_error("sensor handle is no longer valid");
        }
        return world->sensors[static_cast<std::size_t>(index)];
    }
};

World& worldOf(const py::object& self) {
    return self.cast<World&>();
}

int bodyIndex(const py::object& value, const World& w) {
    int index = 0;
    if (py::isinstance<BodyRef>(value)) {
        const auto& ref = value.cast<const BodyRef&>();
        if (ref.world != &w) {
            throw py::value_error("body belongs to a different World");
        }
        index = ref.index;
    } else {
        index = value.cast<int>();
    }
    if (index < 0 || static_cast<std::size_t>(index) >= w.bodies.size()) {
        throw py::index_error("body index out of range");
    }
    return index;
}

BodyRef makeBody(const py::object& owner, int index) {
    return {owner, &worldOf(owner), index};
}
JointRef makeJoint(const py::object& owner, int index) {
    return {owner, &worldOf(owner), index};
}
SensorRef makeSensorRef(const py::object& owner, int index) {
    return {owner, &worldOf(owner), index};
}

// Speed of a joint coordinate (rad/s or m/s) from the relative motion of its two bodies.
double jointSpeed(const World& w, const Joint& j) {
    const Body& a = w.bodies[static_cast<std::size_t>(j.a)];
    const Body& b = w.bodies[static_cast<std::size_t>(j.b)];
    Vec3 axis = a.orientation.rotate(j.axisA).normalized();
    if (j.type == JointType::Hinge) {
        return (b.angularVelocity - a.angularVelocity).dot(axis);
    }
    if (j.type == JointType::Prismatic) {
        Vec3 pa = a.point(j.anchorA);
        Vec3 pb = b.point(j.anchorB);
        return (b.pointVelocity(pb) - a.pointVelocity(pa)).dot(axis);
    }
    return 0;
}

// Torque or force the actuator is applying right now (last step).
double jointEffort(const World& w, const Joint& j) {
    if (j.usesSpeedMotor()) {
        return j.motorImpulse / World::defaultTimeStep;
    }
    return j.history.empty() ? 0.0 : j.history.back().effort;
}

std::vector<int> actuatedJoints(const World& w) {
    std::vector<int> out;
    for (std::size_t i = 0; i < w.joints.size(); ++i) {
        if (w.joints[i].isActuated()) {
            out.push_back(static_cast<int>(i));
        }
    }
    return out;
}

void requireCount(const Input& values, std::size_t expected, const char* what) {
    if (values.ndim() != 1 || static_cast<std::size_t>(values.shape(0)) != expected) {
        throw py::value_error(std::string(what) + " must have one value per actuated joint (" + std::to_string(expected) + ")");
    }
}

// ---------- contacts and ray hits as plain Python objects ----------

struct ContactInfo {
    int a;
    int b;
    Vec3 point;
    Vec3 normal;
    double depth;
    double normalImpulse;
    Vec3 tangentImpulse;
    bool speculative;
};

struct UrdfRobot {
    World world;
    std::string name;
    std::vector<std::string> warnings;
};

} // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "Robotics Model core: rigid-body physics, joints, PID controllers, sensors, URDF import.";
    // Engine errors (std::runtime_error) become RoboticsError(RuntimeError). pybind11's own ValueError/IndexError/...
    // are runtime_errors too, so they are passed through untouched.
    static py::exception<std::runtime_error> roboticsError(m, "RoboticsError", PyExc_RuntimeError);
    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p) { std::rethrow_exception(p); }
        } catch (const std::runtime_error& e) {
            if (dynamic_cast<const py::builtin_exception*>(&e) != nullptr) { throw; }
            roboticsError(e.what());
        }
    });

    // ---------- enums ----------
    defEnum<Shape>(m, "Shape", {{"box", Shape::Box}, {"sphere", Shape::Sphere}, {"cylinder", Shape::Cylinder}, {"convex_hull", Shape::ConvexHull}});
    defEnum<JointType>(m, "JointType", {{"fixed", JointType::Fixed}, {"ball", JointType::Ball}, {"hinge", JointType::Hinge}, {"prismatic", JointType::Prismatic}, {"slider", JointType::Prismatic}});
    defEnum<ControllerType>(m, "ControllerType",
                            {{"speed_motor", ControllerType::SpeedMotor}, {"p", ControllerType::P}, {"pi", ControllerType::PI}, {"pd", ControllerType::PD}, {"pid", ControllerType::PID}, {"relay", ControllerType::Relay}});
    defEnum<ControlVariable>(m, "ControlVariable", {{"position", ControlVariable::Position}, {"velocity", ControlVariable::Velocity}});
    defEnum<ReferenceSignal>(m, "ReferenceSignal", {{"constant", ReferenceSignal::Constant}, {"step", ReferenceSignal::Step}, {"sine", ReferenceSignal::Sine}});
    defEnum<SensorType>(m, "SensorType",
                        {{"encoder", SensorType::JointEncoder}, {"joint_velocity", SensorType::JointVelocity}, {"imu", SensorType::Imu}, {"force_torque", SensorType::ForceTorque},
                         {"contact", SensorType::Contact}, {"rangefinder", SensorType::Rangefinder}, {"lidar2d", SensorType::Lidar2D}, {"camera", SensorType::Camera}});
    defEnum<UrdfMeshCollision>(m, "MeshCollision", {{"convex_hull", UrdfMeshCollision::ConvexHull}, {"bounding_box", UrdfMeshCollision::BoundingBox}});

    // ---------- small value classes ----------
    py::class_<ContactInfo>(m, "Contact", "A contact point of the last step (read-only).")
        .def_readonly("body_a", &ContactInfo::a)
        .def_readonly("body_b", &ContactInfo::b, "Body index, or -1 for the ground plane.")
        .def_property_readonly("point", [](const ContactInfo& c) { return toArray(c.point); })
        .def_property_readonly("normal", [](const ContactInfo& c) { return toArray(c.normal); }, "Unit normal pointing from B towards A.")
        .def_readonly("depth", &ContactInfo::depth)
        .def_readonly("normal_impulse", &ContactInfo::normalImpulse)
        .def_property_readonly("tangent_impulse", [](const ContactInfo& c) { return toArray(c.tangentImpulse); })
        .def_readonly("speculative", &ContactInfo::speculative)
        .def("__repr__", [](const ContactInfo& c) {
            std::ostringstream s;
            s << "Contact(body_a=" << c.a << ", body_b=" << c.b << ", depth=" << c.depth << ", normal_impulse=" << c.normalImpulse << ")";
            return s.str();
        });

    py::class_<UrdfRobot>(m, "UrdfRobot", "Result of load_urdf().")
        .def_property_readonly("world", [](const UrdfRobot& r) { return r.world; }, "A copy of the imported World.")
        .def_readonly("name", &UrdfRobot::name)
        .def_readonly("warnings", &UrdfRobot::warnings, "Everything that was approximated or skipped.");

    // ---------- handles ----------
    py::class_<BodyRef>(m, "Body", "A rigid body of a World (a handle: reads and writes go to the World).")
        .def_property("name", [](const BodyRef& r) { return r.get().name; }, [](const BodyRef& r, const std::string& v) { r.get().name = v; })
        .def_property_readonly("index", [](const BodyRef& r) { return r.index; })
        .def_property_readonly("shape", [](const BodyRef& r) { return r.get().shape; })
        .def_property("size", [](const BodyRef& r) { return toArray(r.get().size); },
                      [](const BodyRef& r, const py::object& v) { r.get().size = toVec3(v, "size"); },
                      "Box: edge lengths. Sphere: diameters. Cylinder: x = y = diameter, z = width.")
        .def_property("mass", [](const BodyRef& r) { return r.get().mass; }, [](const BodyRef& r, double v) {
            if (!std::isfinite(v) || v < 0) { throw py::value_error("mass must be finite and >= 0"); }
            r.get().mass = v;
        }, "Kilograms; 0 makes the body static.")
        .def_property("position", [](const BodyRef& r) { return toArray(r.get().position); },
                      [](const BodyRef& r, const py::object& v) { r.get().position = toVec3(v, "position"); })
        .def_property("orientation", [](const BodyRef& r) { return toArray(r.get().orientation); },
                      [](const BodyRef& r, const py::object& v) { r.get().orientation = toQuat(v, "orientation"); }, "Quaternion (w, x, y, z).")
        .def_property("velocity", [](const BodyRef& r) { return toArray(r.get().velocity); },
                      [](const BodyRef& r, const py::object& v) { r.get().velocity = toVec3(v, "velocity"); })
        .def_property("angular_velocity", [](const BodyRef& r) { return toArray(r.get().angularVelocity); },
                      [](const BodyRef& r, const py::object& v) { r.get().angularVelocity = toVec3(v, "angular_velocity"); })
        .def_property("friction", [](const BodyRef& r) { return r.get().friction; }, [](const BodyRef& r, double v) { r.get().friction = v; })
        .def_property("restitution", [](const BodyRef& r) { return r.get().restitution; }, [](const BodyRef& r, double v) { r.get().restitution = v; })
        .def_property("ccd", [](const BodyRef& r) { return r.get().ccd; }, [](const BodyRef& r, bool v) { r.get().ccd = v; }, "Continuous collision detection for this body.")
        .def_property_readonly("is_static", [](const BodyRef& r) { return r.get().isStatic(); })
        .def("apply_force", [](const BodyRef& r, const py::object& force, const py::object& point) {
            Body& b = r.get();
            Vec3 f = toVec3(force, "force");
            if (point.is_none()) { b.force += f; } else { b.applyForce(f, toVec3(point, "point")); }
        }, py::arg("force"), py::arg("point") = py::none(),
           "Adds a world-space force (N) for the next step only; with `point` (world space) it also adds the torque.")
        .def("apply_torque", [](const BodyRef& r, const py::object& torque) { r.get().torque += toVec3(torque, "torque"); }, py::arg("torque"),
             "Adds a world-space torque (N·m) for the next step only.")
        .def("local_to_world", [](const BodyRef& r, const py::object& p) { return toArray(r.get().point(toVec3(p, "point"))); }, py::arg("point"))
        .def("point_velocity", [](const BodyRef& r, const py::object& p) { return toArray(r.get().pointVelocity(toVec3(p, "point"))); }, py::arg("point"),
             "Velocity of a world-space point rigidly attached to the body.")
        .def("__repr__", [](const BodyRef& r) { return "<Body " + std::to_string(r.index) + " '" + r.get().name + "'>"; });

    py::class_<JointRef>(m, "Joint", "A joint of a World (a handle).")
        .def_property_readonly("index", [](const JointRef& r) { return r.index; })
        .def_property_readonly("type", [](const JointRef& r) { return r.get().type; })
        .def_property_readonly("parent", [](const JointRef& r) { return makeBody(r.owner, r.get().a); })
        .def_property_readonly("child", [](const JointRef& r) { return makeBody(r.owner, r.get().b); })
        .def_property_readonly("is_actuated", [](const JointRef& r) { return r.get().isActuated(); })
        .def_property_readonly("angle", [](const JointRef& r) { return r.get().angle; }, "Joint coordinate: rad (hinge, multi-turn) or m (prismatic), zero at connect().")
        .def_property_readonly("speed", [](const JointRef& r) { return jointSpeed(*r.world, r.get()); }, "rad/s or m/s.")
        .def_property_readonly("effort", [](const JointRef& r) { return jointEffort(*r.world, r.get()); }, "Torque (N·m) or force (N) the actuator applied in the last step.")
        .def_property_readonly("reaction_force", [](const JointRef& r) { return toArray(r.get().reactionImpulse / World::defaultTimeStep); },
                               "Constraint force (N, world space) transmitted through the joint in the last step.")
        .def_property_readonly("reaction_torque", [](const JointRef& r) { return toArray(r.get().reactionAngularImpulse / World::defaultTimeStep); })
        .def_property("max_effort", [](const JointRef& r) { return r.get().maxTorque; },
                      [](const JointRef& r, double v) {
                          if (!std::isfinite(v) || v < 0) { throw py::value_error("max_effort must be finite and >= 0"); }
                          r.get().maxTorque = v;
                      }, "Actuator limit: N·m (hinge) or N (prismatic). 0 disables a speed motor.")
        .def_property("motor_speed", [](const JointRef& r) { return r.get().motorSpeed; }, [](const JointRef& r, double v) {
            if (!std::isfinite(v)) { throw py::value_error("motor_speed must be finite"); }
            r.get().motorSpeed = v;
        }, "Target speed of the speed motor (controller kind 'speed_motor').")
        .def_property_readonly("limits", [](const JointRef& r) -> py::object {
            const Joint& j = r.get();
            if (!j.limitsEnabled) { return py::none(); }
            return py::make_tuple(j.lowerLimit, j.upperLimit);
        }, "(lower, upper) or None.")
        .def("set_limits", [](const JointRef& r, double lower, double upper) { r.get().setLimits(lower, upper); }, py::arg("lower"), py::arg("upper"),
             "Hard one-sided range limits (rad or m) that also stop motors and controllers.")
        .def("clear_limits", [](const JointRef& r) { r.get().limitsEnabled = false; })
        .def("set_controller", [](const JointRef& r, const py::object& kindArg, const py::object& kp, const py::object& ki, const py::object& kd,
                                  const py::object& variableArg, const py::object& integralLimit, const py::object& derivativeFilter) {
            Joint& j = r.get();
            if (!j.isActuated()) { throw py::value_error("only hinge and prismatic joints have controllers"); }
            ControllerConfig c = j.controller;
            c.type = toEnum<ControllerType>(kindArg);
            c.variable = toEnum<ControlVariable>(variableArg);
            if (!kp.is_none()) { c.kp = kp.cast<double>(); }
            if (!ki.is_none()) { c.ki = ki.cast<double>(); }
            if (!kd.is_none()) { c.kd = kd.cast<double>(); }
            if (!integralLimit.is_none()) { c.integralLimit = integralLimit.cast<double>(); }
            if (!derivativeFilter.is_none()) { c.derivativeFilter = derivativeFilter.cast<double>(); }
            validated(c);
            j.controller = c;
            j.controllerState = ControllerState{};
        }, py::arg("kind") = py::str("pid"), py::arg("kp") = py::none(), py::arg("ki") = py::none(), py::arg("kd") = py::none(),
           py::arg("variable") = py::str("position"), py::arg("integral_limit") = py::none(), py::arg("derivative_filter") = py::none(),
           "Selects the controller (P, PI, PD, PID, Relay or the speed motor) and sets its gains; unset gains keep their values.")
        .def_property("controller", [](const JointRef& r) { return r.get().controller.type; }, [](const JointRef& r, ControllerType v) {
            r.get().controller.type = v;
            r.get().controllerState = ControllerState{};
        })
        .def_property("kp", [](const JointRef& r) { return r.get().controller.kp; }, [](const JointRef& r, double v) { r.get().controller.kp = v; validated(r.get().controller); })
        .def_property("ki", [](const JointRef& r) { return r.get().controller.ki; }, [](const JointRef& r, double v) { r.get().controller.ki = v; validated(r.get().controller); })
        .def_property("kd", [](const JointRef& r) { return r.get().controller.kd; }, [](const JointRef& r, double v) { r.get().controller.kd = v; validated(r.get().controller); })
        .def_property("target", [](const JointRef& r) { return r.get().controller.reference(0.0); }, [](const JointRef& r, double v) {
            ControllerConfig& c = r.get().controller;
            if (!std::isfinite(v)) { throw py::value_error("target must be finite"); }
            c.signal = ReferenceSignal::Constant;
            c.target = v;
        }, "Constant setpoint (rad, m, rad/s or m/s depending on the control variable).")
        .def("set_step", [](const JointRef& r, double base, double amplitude, double start) {
            ControllerConfig& c = r.get().controller;
            c.signal = ReferenceSignal::Step; c.target = base; c.amplitude = amplitude; c.startTime = start;
            validated(c);
        }, py::arg("base"), py::arg("amplitude"), py::arg("start_time") = 1.0, "Setpoint is `base` until start_time, then base + amplitude.")
        .def("set_sine", [](const JointRef& r, double base, double amplitude, double frequency, double start) {
            ControllerConfig& c = r.get().controller;
            c.signal = ReferenceSignal::Sine; c.target = base; c.amplitude = amplitude; c.frequency = frequency; c.startTime = start;
            validated(c);
        }, py::arg("base"), py::arg("amplitude"), py::arg("frequency"), py::arg("start_time") = 0.0)
        .def("reference", [](const JointRef& r, double time) { return r.get().controller.reference(time); }, py::arg("time"))
        .def("__repr__", [](const JointRef& r) {
            const Joint& j = r.get();
            return "<Joint " + std::to_string(r.index) + " " + std::string(jointTypeName(j.type)) + " " + std::to_string(j.a) + "->" + std::to_string(j.b) + ">";
        });

    py::class_<SensorRef>(m, "Sensor", "A sensor of a World (a handle). Updated by World.step().")
        .def_property_readonly("index", [](const SensorRef& r) { return r.index; })
        .def_property("name", [](const SensorRef& r) { return r.get().name; }, [](const SensorRef& r, const std::string& v) { r.get().name = v; })
        .def_property_readonly("type", [](const SensorRef& r) { return r.get().type; })
        .def_property_readonly("body", [](const SensorRef& r) { return makeBody(r.owner, r.get().body); })
        .def_property_readonly("channels", [](const SensorRef& r) { return sensorChannelNames(*r.world, r.get()); }, "Names of the output channels.")
        .def_property_readonly("units", [](const SensorRef& r) {
            std::vector<std::string> out;
            for (std::size_t i = 0; i < r.get().channelCount(); ++i) { out.push_back(sensorChannelUnit(*r.world, r.get(), i)); }
            return out;
        })
        .def_property_readonly("value", [](const SensorRef& r) { return toArray(r.get().state.value); }, "Latest reading: noisy, quantized, delayed, held between samples.")
        .def_property_readonly("truth", [](const SensorRef& r) { return toArray(r.get().state.truth); }, "Ideal value of the latest sample (no noise or delay).")
        .def_property_readonly("valid", [](const SensorRef& r) { return r.get().state.valid; })
        .def_property_readonly("fresh", [](const SensorRef& r) { return r.get().state.fresh; }, "True when the last step produced a new sample.")
        .def_property_readonly("samples", [](const SensorRef& r) { return r.get().state.samples; })
        .def_property_readonly("dropped", [](const SensorRef& r) { return r.get().state.dropped; })
        .def_property_readonly("ray_ends", [](const SensorRef& r) { return toMatrix(r.get().state.rayEnds); }, "Range sensors: end point of every beam of the latest scan (world space).")
        .def_property_readonly("ray_hits", [](const SensorRef& r) {
            std::vector<bool> hits;
            for (char h : r.get().state.rayHits) { hits.push_back(h != 0); }
            return hits;
        })
        .def_property("rate", [](const SensorRef& r) { return r.get().rate; }, [](const SensorRef& r, double v) {
            Sensor s = r.get(); s.rate = v; validateSensor(s, r.world->bodies.size()); r.get().rate = v;
        })
        .def_property("latency", [](const SensorRef& r) { return r.get().latency; }, [](const SensorRef& r, int v) {
            Sensor s = r.get(); s.latency = v; validateSensor(s, r.world->bodies.size()); r.get().latency = v;
        })
        .def_property("noise_stddev", [](const SensorRef& r) { return r.get().noise.stddev; }, [](const SensorRef& r, double v) {
            Sensor s = r.get(); s.noise.stddev = v; validateSensor(s, r.world->bodies.size()); r.get().noise.stddev = v;
        })
        .def_property("feeds_controller", [](const SensorRef& r) { return r.get().feedsController; }, [](const SensorRef& r, bool v) { r.get().feedsController = v; },
                      "Encoder / joint velocity sensors: the joint's feedback controller reads this sensor instead of the ideal value.")
        .def("__repr__", [](const SensorRef& r) { return "<Sensor " + std::to_string(r.index) + " '" + r.get().name + "' " + std::string(sensorTypeName(r.get().type)) + ">"; });

    // ---------- World ----------
    py::class_<World>(m, "World", "Bodies, joints, sensors and the fixed-step physics pipeline (240 Hz by default).")
        .def(py::init<>())
        .def("__copy__", [](const World& w) { return w; })
        .def("__deepcopy__", [](const World& w, const py::dict&) { return w; }, py::arg("memo"))
        .def("clone", [](const World& w) { return w; }, "An independent copy of the whole state (bodies, joints, controllers, sensors, time).")
        .def("copy_state_from", [](World& w, const World& other) { w = other; }, py::arg("other"),
             "Overwrites this World with the state of `other`. Existing handles stay valid when the topology matches.")
        .def("validate", &World::validate, "Raises RoboticsError describing the first invalid setting.")
        .def_property("gravity", [](const World& w) { return toArray(w.gravity); }, [](World& w, const py::object& v) { w.gravity = toVec3(v, "gravity"); })
        .def_readwrite("iterations", &World::iterations, "Solver iterations per step.")
        .def_readwrite("substeps", &World::substeps, "Substeps per step (more = stiffer joints and contacts).")
        .def_readwrite("warm_starting", &World::warmStarting)
        .def_readwrite("continuous_collision", &World::continuousCollision, "Sweep fast bodies so they cannot tunnel through thin ones.")
        .def_readonly("time", &World::time, "Simulated seconds.")
        .def_property_readonly("num_bodies", [](const World& w) { return w.bodies.size(); })
        .def_property_readonly("num_joints", [](const World& w) { return w.joints.size(); })
        .def_property_readonly("num_sensors", [](const World& w) { return w.sensors.size(); })
        .def_property_readonly("bodies", [](py::object self) {
            py::list out;
            for (std::size_t i = 0; i < worldOf(self).bodies.size(); ++i) { out.append(makeBody(self, static_cast<int>(i))); }
            return out;
        })
        .def_property_readonly("joints", [](py::object self) {
            py::list out;
            for (std::size_t i = 0; i < worldOf(self).joints.size(); ++i) { out.append(makeJoint(self, static_cast<int>(i))); }
            return out;
        })
        .def_property_readonly("sensors", [](py::object self) {
            py::list out;
            for (std::size_t i = 0; i < worldOf(self).sensors.size(); ++i) { out.append(makeSensorRef(self, static_cast<int>(i))); }
            return out;
        })
        .def("body", [](py::object self, const py::object& key) {
            World& w = worldOf(self);
            if (py::isinstance<py::str>(key)) {
                std::string name = key.cast<std::string>();
                for (std::size_t i = 0; i < w.bodies.size(); ++i) {
                    if (w.bodies[i].name == name) { return makeBody(self, static_cast<int>(i)); }
                }
                throw py::key_error("no body named '" + name + "'");
            }
            return makeBody(self, bodyIndex(key, w));
        }, py::arg("key"), "Body by index or name.")
        .def("joint", [](py::object self, int index) {
            if (index < 0 || static_cast<std::size_t>(index) >= worldOf(self).joints.size()) { throw py::index_error("joint index out of range"); }
            return makeJoint(self, index);
        }, py::arg("index"))
        .def("sensor", [](py::object self, const py::object& key) {
            World& w = worldOf(self);
            if (py::isinstance<py::str>(key)) {
                std::string name = key.cast<std::string>();
                for (std::size_t i = 0; i < w.sensors.size(); ++i) {
                    if (w.sensors[i].name == name) { return makeSensorRef(self, static_cast<int>(i)); }
                }
                throw py::key_error("no sensor named '" + name + "'");
            }
            int index = key.cast<int>();
            if (index < 0 || static_cast<std::size_t>(index) >= w.sensors.size()) { throw py::index_error("sensor index out of range"); }
            return makeSensorRef(self, index);
        }, py::arg("key"), "Sensor by index or name.")
        .def("add_body", [](py::object self, const py::object& shapeArg, const py::object& size, double mass, const py::object& position, const py::object& orientation,
                            const std::string& name, double friction, double restitution, bool ccd) {
            World& w = worldOf(self);
            Body b;
            b.shape = toEnum<Shape>(shapeArg);
            b.size = toVec3(size, "size");
            b.mass = mass;
            b.position = toVec3(position, "position");
            b.orientation = toQuat(orientation, "orientation");
            b.name = name.empty() ? "body_" + std::to_string(w.bodies.size()) : name;
            b.friction = friction;
            b.restitution = restitution;
            b.ccd = ccd;
            return makeBody(self, w.addBody(std::move(b)));
        }, py::arg("shape") = py::str("box"), py::arg("size") = py::make_tuple(0.5, 0.5, 0.5), py::arg("mass") = 1.0,
           py::arg("position") = py::make_tuple(0.0, 0.0, 0.0), py::arg("orientation") = py::make_tuple(1.0, 0.0, 0.0, 0.0),
           py::arg("name") = "", py::arg("friction") = 0.65, py::arg("restitution") = 0.05, py::arg("ccd") = false,
           "Adds a body. `size`: box edges, sphere diameters or cylinder (diameter, diameter, width). mass = 0 makes it static.")
        .def("connect", [](py::object self, const py::object& parent, const py::object& child, const py::object& typeArg, const py::object& anchor, const py::object& axis) {
            World& w = worldOf(self);
            int index = w.connect(bodyIndex(parent, w), bodyIndex(child, w), toEnum<JointType>(typeArg), toVec3(anchor, "anchor"), toVec3(axis, "axis"));
            return makeJoint(self, index);
        }, py::arg("parent"), py::arg("child"), py::arg("type") = py::str("hinge"), py::arg("anchor") = py::make_tuple(0.0, 0.0, 0.0), py::arg("axis") = py::make_tuple(0.0, 0.0, 1.0),
           "Connects two bodies at a world-space anchor. The current pose becomes the joint's zero.")
        .def("add_sensor", [](py::object self, const py::object& typeArg, const py::object& target, const std::string& name, const py::object& rate, const py::object& latency,
                              const py::object& dropout, const py::object& seed, const py::object& noiseStddev, const py::object& noiseBias,
                              const py::object& noiseBiasWalk, const py::object& noiseResolution, const py::object& noise2Stddev,
                              const py::object& encoderCounts, bool feedsController, const py::object& minRange, const py::object& maxRange,
                              const py::object& fov, const py::object& beams, const py::object& width, const py::object& height,
                              const py::object& position, const py::object& orientation) {
            World& w = worldOf(self);
            Sensor s = makeSensor(toEnum<SensorType>(typeArg), bodyIndex(target, w));
            if (!name.empty()) { s.name = name; }
            if (!rate.is_none()) { s.rate = rate.cast<double>(); }
            if (!latency.is_none()) { s.latency = latency.cast<int>(); }
            if (!dropout.is_none()) { s.dropout = dropout.cast<double>(); }
            if (!seed.is_none()) { s.seed = seed.cast<std::uint64_t>(); }
            if (!noiseStddev.is_none()) { s.noise.stddev = noiseStddev.cast<double>(); }
            if (!noiseBias.is_none()) { s.noise.bias = noiseBias.cast<double>(); }
            if (!noiseBiasWalk.is_none()) { s.noise.biasWalk = noiseBiasWalk.cast<double>(); }
            if (!noiseResolution.is_none()) { s.noise.resolution = noiseResolution.cast<double>(); }
            if (!noise2Stddev.is_none()) { s.noise2.stddev = noise2Stddev.cast<double>(); }
            if (!encoderCounts.is_none()) { s.encoderCounts = encoderCounts.cast<double>(); }
            s.feedsController = feedsController;
            if (!minRange.is_none()) { s.minRange = minRange.cast<double>(); }
            if (!maxRange.is_none()) { s.maxRange = maxRange.cast<double>(); }
            if (!fov.is_none()) { s.fov = fov.cast<double>(); }
            if (!beams.is_none()) { s.beams = beams.cast<int>(); }
            if (!width.is_none()) { s.width = width.cast<int>(); }
            if (!height.is_none()) { s.height = height.cast<int>(); }
            if (!position.is_none()) { s.position = toVec3(position, "position"); }
            if (!orientation.is_none()) { s.orientation = toQuat(orientation, "orientation"); }
            validateSensor(s, w.bodies.size());
            w.sensors.push_back(std::move(s));
            return makeSensorRef(self, static_cast<int>(w.sensors.size()) - 1);
        }, py::arg("type"), py::arg("target"), py::arg("name") = "", py::arg("rate") = py::none(), py::arg("latency") = py::none(), py::arg("dropout") = py::none(),
           py::arg("seed") = py::none(), py::arg("noise_stddev") = py::none(), py::arg("noise_bias") = py::none(), py::arg("noise_bias_walk") = py::none(),
           py::arg("noise_resolution") = py::none(), py::arg("noise2_stddev") = py::none(), py::arg("encoder_counts") = py::none(), py::arg("feeds_controller") = false,
           py::arg("min_range") = py::none(), py::arg("max_range") = py::none(), py::arg("fov") = py::none(), py::arg("beams") = py::none(),
           py::arg("width") = py::none(), py::arg("height") = py::none(), py::arg("position") = py::none(), py::arg("orientation") = py::none(),
           "Mounts a sensor on a body (joint sensors: on the child body of the measured joint). Unset arguments keep the type's defaults, which are realistic: e.g. an encoder quantizes to 4096 counts per turn, an IMU is noisy. Pass encoder_counts=0 / noise_stddev=0 for an ideal sensor.")
        .def("step", [](World& w, int n, double dt, bool sensors) {
            if (n < 0) { throw py::value_error("n must be >= 0"); }
            py::gil_scoped_release release;
            for (int i = 0; i < n; ++i) {
                if (sensors) { stepSimulation(w, dt); } else { w.step(dt); }
            }
        }, py::arg("n") = 1, py::arg("dt") = World::defaultTimeStep, py::arg("sensors") = true,
           "Advances the simulation by n steps of dt seconds (default 1/240) and updates the sensors.")
        .def("run", [](World& w, double seconds, double dt) {
            if (!std::isfinite(seconds) || seconds < 0) { throw py::value_error("seconds must be finite and >= 0"); }
            int n = static_cast<int>(std::llround(seconds / dt));
            py::gil_scoped_release release;
            for (int i = 0; i < n; ++i) { stepSimulation(w, dt); }
        }, py::arg("seconds"), py::arg("dt") = World::defaultTimeStep, "Steps for `seconds` of simulated time.")
        .def("reset_controllers", &World::resetControllers, "Clears integrators and telemetry; keeps the encoder calibration.")
        .def("max_anchor_error", &World::maxAnchorError, "Largest distance between the two anchors of any joint (m): constraint drift.")
        // ---- bulk numpy access ----
        .def("positions", [](const World& w) {
            std::vector<Vec3> v; for (const auto& b : w.bodies) { v.push_back(b.position); } return toMatrix(v);
        }, "(N, 3) body positions.")
        .def("velocities", [](const World& w) {
            std::vector<Vec3> v; for (const auto& b : w.bodies) { v.push_back(b.velocity); } return toMatrix(v);
        })
        .def("angular_velocities", [](const World& w) {
            std::vector<Vec3> v; for (const auto& b : w.bodies) { v.push_back(b.angularVelocity); } return toMatrix(v);
        })
        .def("orientations", [](const World& w) {
            Array a({static_cast<py::ssize_t>(w.bodies.size()), static_cast<py::ssize_t>(4)});
            auto r = a.mutable_unchecked<2>();
            for (std::size_t i = 0; i < w.bodies.size(); ++i) {
                const Quat& q = w.bodies[i].orientation;
                auto row = static_cast<py::ssize_t>(i);
                r(row, 0) = q.w; r(row, 1) = q.x; r(row, 2) = q.y; r(row, 3) = q.z;
            }
            return a;
        }, "(N, 4) quaternions (w, x, y, z).")
        .def_property_readonly("actuated_joints", [](const World& w) { return actuatedJoints(w); }, "Indices of the hinge and prismatic joints, in order.")
        .def("joint_angles", [](const World& w) {
            std::vector<double> v; for (int i : actuatedJoints(w)) { v.push_back(w.joints[static_cast<std::size_t>(i)].angle); } return toArray(v);
        }, "Coordinates of the actuated joints.")
        .def("joint_speeds", [](const World& w) {
            std::vector<double> v; for (int i : actuatedJoints(w)) { v.push_back(jointSpeed(w, w.joints[static_cast<std::size_t>(i)])); } return toArray(v);
        })
        .def("joint_efforts", [](const World& w) {
            std::vector<double> v; for (int i : actuatedJoints(w)) { v.push_back(jointEffort(w, w.joints[static_cast<std::size_t>(i)])); } return toArray(v);
        })
        .def("set_joint_targets", [](World& w, const py::object& values) {
            auto ids = actuatedJoints(w);
            Input a = Input::ensure(values);
            if (!a) { throw py::value_error("targets must be an array of numbers"); }
            requireCount(a, ids.size(), "targets");
            for (std::size_t k = 0; k < ids.size(); ++k) {
                double v = a.data()[k];
                if (!std::isfinite(v)) { throw py::value_error("targets must be finite"); }
                ControllerConfig& c = w.joints[static_cast<std::size_t>(ids[k])].controller;
                c.signal = ReferenceSignal::Constant;
                c.target = v;
            }
        }, py::arg("targets"), "Sets the constant setpoint of every actuated joint's feedback controller.")
        .def("set_motor_speeds", [](World& w, const py::object& values) {
            auto ids = actuatedJoints(w);
            Input a = Input::ensure(values);
            if (!a) { throw py::value_error("speeds must be an array of numbers"); }
            requireCount(a, ids.size(), "speeds");
            for (std::size_t k = 0; k < ids.size(); ++k) {
                if (!std::isfinite(a.data()[k])) { throw py::value_error("speeds must be finite"); }
                w.joints[static_cast<std::size_t>(ids[k])].motorSpeed = a.data()[k];
            }
        }, py::arg("speeds"), "Sets the speed-motor target of every actuated joint (joints using a speed motor).")
        .def("contacts", [](const World& w) {
            std::vector<ContactInfo> out;
            for (const Contact& c : w.contacts) {
                out.push_back({c.a, c.b, c.point, c.normal, c.depth, c.normalImpulse, c.tangentImpulse, c.speculative});
            }
            return out;
        }, "Contacts of the last step.")
        .def("raycast", [](const World& w, const py::object& origin, const py::object& direction, double maxDistance, const py::object& ignore, bool includeGround) -> py::object {
            Vec3 d = toVec3(direction, "direction");
            if (d.length2() < 1e-20) { throw py::value_error("direction must be non-zero"); }
            int skip = ignore.is_none() ? noBodyIndex : bodyIndex(ignore, w);
            RayHit hit = raycast(w, toVec3(origin, "origin"), d.normalized(), maxDistance, skip, includeGround);
            if (!hit.hit) { return py::none(); }
            py::dict out;
            out["body"] = hit.body;
            out["distance"] = hit.distance;
            out["point"] = toArray(hit.point);
            out["normal"] = toArray(hit.normal);
            return out;
        }, py::arg("origin"), py::arg("direction"), py::arg("max_distance") = 100.0, py::arg("ignore") = py::none(), py::arg("include_ground") = true,
           "Nearest hit as a dict (body, distance, point, normal), or None. body = -1 means the ground plane.")
        .def("dumps", [](const World& w) { std::ostringstream s; writeScene(w, s); return s.str(); }, "The scene in the .robot text format.")
        .def_static("loads", [](const std::string& text) { std::istringstream s(text); return readScene(s); }, py::arg("text"), "Parses the .robot text format.")
        .def("__repr__", [](const World& w) {
            return "<World t=" + std::to_string(w.time) + " s, " + std::to_string(w.bodies.size()) + " bodies, " + std::to_string(w.joints.size()) + " joints, " +
                   std::to_string(w.sensors.size()) + " sensors>";
        });

    // ---------- I/O ----------
    m.def("load_scene", [](const std::string& path) { return loadScene(path); }, py::arg("path"), "Loads a .robot scene.");
    m.def("save_scene", [](const World& w, const std::string& path) { saveScene(w, path); }, py::arg("world"), py::arg("path"), "Saves a World as a .robot scene.");
    m.def("load_urdf", [](const std::string& path, bool fixBase, bool zUpToYUp, const py::object& spawnOffset, bool liftAboveGround, double defaultEffort,
                          double minimumMass, const std::vector<std::string>& meshSearchPaths, const py::object& meshCollision) {
        UrdfImportOptions o;
        o.fixBase = fixBase;
        o.zUpToYUp = zUpToYUp;
        o.spawnOffset = toVec3(spawnOffset, "spawn_offset");
        o.liftAboveGround = liftAboveGround;
        o.defaultEffort = defaultEffort;
        o.minimumMass = minimumMass;
        o.meshSearchPaths = meshSearchPaths;
        o.meshCollision = toEnum<UrdfMeshCollision>(meshCollision);
        UrdfImportResult r = loadUrdf(path, o);
        return UrdfRobot{std::move(r.world), std::move(r.robotName), std::move(r.warnings)};
    }, py::arg("path"), py::arg("fix_base") = false, py::arg("z_up_to_y_up") = true, py::arg("spawn_offset") = py::make_tuple(0.0, 0.0, 0.0),
       py::arg("lift_above_ground") = true, py::arg("default_effort") = 10.0, py::arg("minimum_mass") = 0.01,
       py::arg("mesh_search_paths") = std::vector<std::string>{}, py::arg("mesh_collision") = py::str("convex_hull"),
       "Imports a URDF robot. Returns an UrdfRobot with .world, .name and .warnings.");

    // ---------- demos ----------
    auto demos = m.def_submodule("demos", "Built-in example robots.");
    demos.def("manipulator", &makeDemo, "Two-link motorized arm with a tool and a free ball.");
    demos.def("pid_servo", &makeControlDemo, "Single hinge driven by a PID step response.");
    demos.def("linear_actuator", &makeLinearActuatorDemo, "Slider carriage with PID and limits, carrying a hinged flap.");
    demos.def("rover", &makeRoverDemo, "Four-wheel differential-drive rover.");
    demos.def("sensor_rover", &makeSensorRoverDemo, "Rover with lidar, IMU, camera and encoders.");
    demos.def("noisy_pid", &makeNoisyPidDemo, "PID bench fed by a noisy, quantized, delayed encoder.");
    demos.def("drive_rover", [](World& w, double forward, double turn) { return driveRover(w, forward, turn); }, py::arg("world"), py::arg("forward"), py::arg("turn"),
              "Sets wheel speeds (rad/s); returns the number of wheels that received the command.");

    m.attr("DEFAULT_TIME_STEP") = World::defaultTimeStep;
    m.attr("__version__") = "0.4.0";
}

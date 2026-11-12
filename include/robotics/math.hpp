#pragma once
// Small value types for 3D rigid-body math: Vec3 and unit quaternion Quat.
// Header-only and dependency-free so every module (core, I/O, UI, bindings) can use it.

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robotics {

inline constexpr double pi = 3.14159265358979323846;

struct Vec3 {
    double x{};
    double y{};
    double z{};

    [[nodiscard]] constexpr double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

    [[nodiscard]] constexpr double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }

    [[nodiscard]] constexpr Vec3 operator-() const { return {-x, -y, -z}; }

    [[nodiscard]] constexpr Vec3 operator+(Vec3 b) const { return {x + b.x, y + b.y, z + b.z}; }

    [[nodiscard]] constexpr Vec3 operator-(Vec3 b) const { return {x - b.x, y - b.y, z - b.z}; }

    [[nodiscard]] constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }

    [[nodiscard]] constexpr Vec3 operator/(double s) const { return *this * (1 / s); }

    constexpr Vec3& operator+=(Vec3 b) { return *this = *this + b; }

    constexpr Vec3& operator-=(Vec3 b) { return *this = *this - b; }

    [[nodiscard]] constexpr double dot(Vec3 b) const { return x * b.x + y * b.y + z * b.z; }

    [[nodiscard]] constexpr Vec3 cross(Vec3 b) const {
        return {y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x};
    }

    [[nodiscard]] constexpr double length2() const { return dot(*this); }

    [[nodiscard]] double length() const { return std::sqrt(length2()); }

    // Returns the zero vector for (near) zero input instead of dividing by zero.
    [[nodiscard]] Vec3 normalized() const {
        double n = length();
        return n > 1e-12 ? *this / n : Vec3{};
    }

    [[nodiscard]] bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

[[nodiscard]] constexpr Vec3 operator*(double s, Vec3 v) {
    return v * s;
}

[[nodiscard]] constexpr Vec3 componentProduct(Vec3 a, Vec3 b) {
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

// Rotation quaternion (w, x, y, z). Callers keep it normalized; World::validate() enforces it.
struct Quat {
    double w{1};
    double x{};
    double y{};
    double z{};

    [[nodiscard]] constexpr Quat conjugate() const { return {w, -x, -y, -z}; }

    [[nodiscard]] constexpr Quat operator*(Quat b) const {
        return {w * b.w - x * b.x - y * b.y - z * b.z, w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x, w * b.z + x * b.y - y * b.x + z * b.w};
    }

    // Throws for zero/non-finite quaternions: an orientation must stay meaningful.
    [[nodiscard]] Quat normalized() const {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        if (!std::isfinite(n) || n < 1e-12) {
            throw std::runtime_error("Invalid orientation quaternion");
        }
        return {w / n, x / n, y / n, z / n};
    }

    [[nodiscard]] constexpr Vec3 rotate(Vec3 v) const {
        Vec3 u{x, y, z};
        return v + 2 * w * u.cross(v) + 2 * u.cross(u.cross(v));
    }

    [[nodiscard]] static Quat axisAngle(Vec3 axis, double angle) {
        if (axis.length2() < 1e-20) {
            return {};
        }
        Vec3 u = axis.normalized() * std::sin(angle / 2);
        return {std::cos(angle / 2), u.x, u.y, u.z};
    }

    // Intrinsic Z-Y-X composition of rotations given in radians (x = roll, y = pitch, z = yaw).
    [[nodiscard]] static Quat euler(Vec3 radians) {
        return axisAngle({0, 0, 1}, radians.z) * axisAngle({0, 1, 0}, radians.y)
               * axisAngle({1, 0, 0}, radians.x);
    }

    [[nodiscard]] Vec3 eulerAngles() const {
        return {std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0)),
                std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))};
    }
};

// 3x3 matrix (row-major), used for world-space inertia tensors and small block solves.
struct Mat3 {
    Vec3 r0{1, 0, 0};
    Vec3 r1{0, 1, 0};
    Vec3 r2{0, 0, 1};

    [[nodiscard]] static constexpr Mat3 zero() { return {Vec3{}, Vec3{}, Vec3{}}; }

    [[nodiscard]] static constexpr Mat3 diagonal(Vec3 d) { return {{d.x, 0, 0}, {0, d.y, 0}, {0, 0, d.z}}; }

    // Cross-product matrix: skew(a) * b == a.cross(b).
    [[nodiscard]] static constexpr Mat3 skew(Vec3 a) {
        return {{0, -a.z, a.y}, {a.z, 0, -a.x}, {-a.y, a.x, 0}};
    }

    // Rotation matrix of a unit quaternion: rotation(q) * v == q.rotate(v).
    [[nodiscard]] static constexpr Mat3 rotation(Quat q) {
        double xx = q.x * q.x;
        double yy = q.y * q.y;
        double zz = q.z * q.z;
        double xy = q.x * q.y;
        double xz = q.x * q.z;
        double yz = q.y * q.z;
        double wx = q.w * q.x;
        double wy = q.w * q.y;
        double wz = q.w * q.z;
        return {{1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)},
                {2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)},
                {2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)}};
    }

    [[nodiscard]] constexpr Vec3 operator*(Vec3 v) const { return {r0.dot(v), r1.dot(v), r2.dot(v)}; }

    [[nodiscard]] constexpr Mat3 transposed() const {
        return {{r0.x, r1.x, r2.x}, {r0.y, r1.y, r2.y}, {r0.z, r1.z, r2.z}};
    }

    [[nodiscard]] constexpr Mat3 operator*(const Mat3& b) const {
        Mat3 t = b.transposed();
        return {{r0.dot(t.r0), r0.dot(t.r1), r0.dot(t.r2)},
                {r1.dot(t.r0), r1.dot(t.r1), r1.dot(t.r2)},
                {r2.dot(t.r0), r2.dot(t.r1), r2.dot(t.r2)}};
    }

    [[nodiscard]] constexpr Mat3 operator+(const Mat3& b) const { return {r0 + b.r0, r1 + b.r1, r2 + b.r2}; }

    [[nodiscard]] constexpr Mat3 operator-(const Mat3& b) const { return {r0 - b.r0, r1 - b.r1, r2 - b.r2}; }

    [[nodiscard]] constexpr Mat3 operator*(double s) const { return {r0 * s, r1 * s, r2 * s}; }

    [[nodiscard]] constexpr double determinant() const { return r0.dot(r1.cross(r2)); }

    // Inverse, or the zero matrix when (nearly) singular.
    [[nodiscard]] constexpr Mat3 inverse() const {
        Vec3 c0 = r1.cross(r2);
        Vec3 c1 = r2.cross(r0);
        Vec3 c2 = r0.cross(r1);
        double det = r0.dot(c0);
        if (!(std::abs(det) > 1e-300)) {
            return zero();
        }
        return Mat3{c0, c1, c2}.transposed() * (1 / det);
    }
};

// Unit quaternion of a proper rotation matrix (orthonormal, determinant +1): rotation(q) == m.
[[nodiscard]] inline Quat quatFromMatrix(const Mat3& m) {
    double trace = m.r0.x + m.r1.y + m.r2.z;
    Quat q;
    if (trace > 0) {
        double s = std::sqrt(trace + 1) * 2;
        q = {0.25 * s, (m.r2.y - m.r1.z) / s, (m.r0.z - m.r2.x) / s, (m.r1.x - m.r0.y) / s};
    } else if (m.r0.x > m.r1.y && m.r0.x > m.r2.z) {
        double s = std::sqrt(1 + m.r0.x - m.r1.y - m.r2.z) * 2;
        q = {(m.r2.y - m.r1.z) / s, 0.25 * s, (m.r0.y + m.r1.x) / s, (m.r0.z + m.r2.x) / s};
    } else if (m.r1.y > m.r2.z) {
        double s = std::sqrt(1 + m.r1.y - m.r0.x - m.r2.z) * 2;
        q = {(m.r0.z - m.r2.x) / s, (m.r0.y + m.r1.x) / s, 0.25 * s, (m.r1.z + m.r2.y) / s};
    } else {
        double s = std::sqrt(1 + m.r2.z - m.r0.x - m.r1.y) * 2;
        q = {(m.r1.x - m.r0.y) / s, (m.r0.z + m.r2.x) / s, (m.r1.z + m.r2.y) / s, 0.25 * s};
    }
    return q.normalized();
}

// Axis * angle (radians) of the shortest rotation equivalent to q.
[[nodiscard]] inline Vec3 rotationVector(Quat q) {
    q = q.normalized();
    if (q.w < 0) {
        q = {-q.w, -q.x, -q.y, -q.z};
    }
    Vec3 v{q.x, q.y, q.z};
    double s = v.length();
    return s < 1e-10 ? 2 * v : v * (2 * std::atan2(s, q.w) / s);
}

} // namespace robotics

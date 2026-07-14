// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// core/types.h
//
// Lightweight, zero-dependency math primitives.
//   - Vec3      3-vector (double)
//   - Matrix3   3x3 row-major matrix (double)
//   - Quaternion scalar-first Hamilton convention (w, x, y, z)
//
// All operations are defined inline so the types can be used without linking
// against the library. They are sufficient for the flight model and are not
// meant to replace a general-purpose linear-algebra package.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace f4flight {

// ---------------------------------------------------------------------------
// Vec3 - 3-component double vector
// ---------------------------------------------------------------------------
struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}

    // Element access: 0 -> x, 1 -> y, 2 -> z
    constexpr double&       operator[](std::size_t i)       { return (i == 0) ? x : (i == 1) ? y : z; }
    constexpr const double& operator[](std::size_t i) const { return (i == 0) ? x : (i == 1) ? y : z; }

    // Compound operators
    constexpr Vec3& operator+=(const Vec3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(double s) noexcept       { x *= s;   y *= s;   z *= s;   return *this; }
    constexpr Vec3& operator/=(double s) noexcept       { return *this *= (1.0 / s); }

    double norm() const noexcept { return std::sqrt(x * x + y * y + z * z); }
    double normSquared() const noexcept { return x * x + y * y + z * z; }

    Vec3 normalized() const noexcept {
        const double n = norm();
        return (n > 1e-12) ? Vec3{x / n, y / n, z / n} : Vec3{};
    }
};

// Free-function operators
constexpr Vec3 operator+(Vec3 a, Vec3 b) noexcept { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3 operator-(Vec3 a, Vec3 b) noexcept { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3 operator-(Vec3 a)        noexcept { return Vec3{-a.x, -a.y, -a.z}; }
constexpr Vec3 operator*(Vec3 a, double s) noexcept { return Vec3{a.x * s, a.y * s, a.z * s}; }
constexpr Vec3 operator*(double s, Vec3 a) noexcept { return a * s; }
constexpr Vec3 operator/(Vec3 a, double s) noexcept { return a * (1.0 / s); }

constexpr double dot(Vec3 a, Vec3 b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

constexpr Vec3 cross(Vec3 a, Vec3 b) noexcept {
    return Vec3{a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
}

// ---------------------------------------------------------------------------
// Matrix3 - 3x3 row-major matrix
//   m[row][col]
// ---------------------------------------------------------------------------
struct Matrix3 {
    // rows
    std::array<std::array<double, 3>, 3> m{};

    constexpr Matrix3() = default;

    constexpr static Matrix3 identity() noexcept {
        Matrix3 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
        return r;
    }

    constexpr double&       at(std::size_t r, std::size_t c)       { return m[r][c]; }
    constexpr const double& at(std::size_t r, std::size_t c) const { return m[r][c]; }

    // Matrix * Vec3
    constexpr Vec3 operator*(Vec3 v) const noexcept {
        return Vec3{m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                    m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                    m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    // Matrix * Matrix
    Matrix3 operator*(const Matrix3& o) const noexcept {
        Matrix3 r;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j) {
                double s = 0.0;
                for (std::size_t k = 0; k < 3; ++k) s += m[i][k] * o.m[k][j];
                r.m[i][j] = s;
            }
        return r;
    }

    // Transpose
    constexpr Matrix3 transposed() const noexcept {
        Matrix3 r;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                r.m[i][j] = m[j][i];
        return r;
    }
};

// ---------------------------------------------------------------------------
// Quaternion - scalar-first Hamilton convention (w, x, y, z)
//   Represents the body-to-world rotation.
//   Multiplication: Hamilton product.
// ---------------------------------------------------------------------------
struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Quaternion() = default;
    constexpr Quaternion(double w_, double x_, double y_, double z_) noexcept
        : w(w_), x(x_), y(y_), z(z_) {}

    // Build from body angular rates (p, q, r) for kinematic integration
    //   q_dot = 0.5 * q (p i + q j + r k)   (Hamilton, scalar-first)
    static constexpr Quaternion fromAngularRates(double p, double q, double r) noexcept {
        // (0, p, q, r) as a "pure" quaternion
        return Quaternion{0.0, p, q, r};
    }

    // Hamilton product
    Quaternion operator*(const Quaternion& o) const noexcept {
        return Quaternion{
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w};
    }

    Quaternion& operator*=(const Quaternion& o) noexcept {
        *this = *this * o;
        return *this;
    }

    double norm() const noexcept {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    Quaternion normalized() const noexcept {
        const double n = norm();
        return (n > 1e-12) ? Quaternion{w / n, x / n, y / n, z / n} : Quaternion{};
    }

    // Conjugate (equals inverse for unit quaternions)
    constexpr Quaternion conjugate() const noexcept {
        return Quaternion{w, -x, -y, -z};
    }

    // Rotate a vector by this quaternion
    Vec3 rotate(Vec3 v) const noexcept {
        const Quaternion qv{0.0, v.x, v.y, v.z};
        const Quaternion r = (*this) * qv * conjugate();
        return Vec3{r.x, r.y, r.z};
    }

    // Convert to a body-to-world direction cosine matrix
    Matrix3 toMatrix() const noexcept {
        const double ww = w * w, xx = x * x, yy = y * y, zz = z * z;
        const double wx = w * x, wy = w * y, wz = w * z;
        const double xy = x * y, xz = x * z, yz = y * z;
        Matrix3 r;
        r.m[0][0] = ww + xx - yy - zz; r.m[0][1] = 2.0 * (xy - wz); r.m[0][2] = 2.0 * (xz + wy);
        r.m[1][0] = 2.0 * (xy + wz);   r.m[1][1] = ww - xx + yy - zz; r.m[1][2] = 2.0 * (yz - wx);
        r.m[2][0] = 2.0 * (xz - wy);   r.m[2][1] = 2.0 * (yz + wx);   r.m[2][2] = ww - xx - yy + zz;
        return r;
    }
};

// ---------------------------------------------------------------------------
// Build a body-to-world DCM from Euler angles (psi, theta, phi) in radians.
//   Standard ZYX rotation: R = Rz(psi) * Ry(theta) * Rx(phi)
// Row-major layout: result_world = R * v_body
// ---------------------------------------------------------------------------
inline Matrix3 dcmFromEuler(double psi, double theta, double phi) noexcept {
    const double sp = std::sin(psi),   cp = std::cos(psi);
    const double st = std::sin(theta), ct = std::cos(theta);
    const double sf = std::sin(phi),   cf = std::cos(phi);
    Matrix3 r;
    r.m[0][0] = cp * ct;
    r.m[0][1] = cp * st * sf - sp * cf;
    r.m[0][2] = cp * st * cf + sp * sf;
    r.m[1][0] = sp * ct;
    r.m[1][1] = sp * st * sf + cp * cf;
    r.m[1][2] = sp * st * cf - cp * sf;
    r.m[2][0] = -st;
    r.m[2][1] = ct * sf;
    r.m[2][2] = ct * cf;
    return r;
}

// ---------------------------------------------------------------------------
// Build a quaternion from Euler angles (psi, theta, phi) in radians.
//   Matches the convention used in Falcon 4's eom.cpp where
//     e1 = scalar component, e2,e3,e4 = x,y,z vector components.
//   Here we map (e1,e2,e3,e4) -> (w,x,y,z).
// ---------------------------------------------------------------------------
inline Quaternion quatFromEuler(double psi, double theta, double phi) noexcept {
    const double cy = std::cos(psi * 0.5),   sy = std::sin(psi * 0.5);
    const double cp = std::cos(theta * 0.5), sp = std::sin(theta * 0.5);
    const double cr = std::cos(phi * 0.5),   sr = std::sin(phi * 0.5);
    return Quaternion{cr * cp * cy + sr * sp * sy,
                      sr * cp * cy - cr * sp * sy,
                      cr * sp * cy + sr * cp * sy,
                      cr * cp * sy - sr * sp * cy};
}

// Recover Euler angles (psi, theta, phi) in radians from a quaternion.
// Convention: ZYX rotation (R = Rz(psi) * Ry(theta) * Rx(phi)).
inline void eulerFromQuat(const Quaternion& q, double& psi, double& theta, double& phi) noexcept {
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    // Standard ZYX quaternion-to-euler:
    //   theta = asin(2*(w*y - x*z))
    //   psi   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    //   phi   = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    const double sin_theta = 2.0 * (w * y - x * z);
    const double sin_t_clamped = std::max(-1.0, std::min(1.0, sin_theta));
    theta = std::asin(sin_t_clamped);
    psi   = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    phi   = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
}

} // namespace f4flight

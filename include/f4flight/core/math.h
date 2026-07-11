// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// core/math.h
//
// Small math helpers used by the FCS and EOM:
//   - limit, rateLimit, deadBand, wrap
//   - first-order Tustin lag filter (FLTust)
//   - trapezoidal integrator (FITust)
//   - 2nd-order Adams-Bashforth integrator (FIAdamsBash)
//
// These are direct ports of the filter math in simlib/math.cpp so that the
// closed-loop FCS behaviour matches the original Falcon 4 feel exactly.

#pragma once

#include "f4flight/core/constants.h"

#include <algorithm>
#include <cmath>

namespace f4flight {

// ---------------------------------------------------------------------------
// Scalar utilities
// ---------------------------------------------------------------------------

// Clamp x to [lo, hi].
template <typename T>
constexpr T limit(T x, T lo, T hi) noexcept {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

// Symmetric clamp x to [-mag, +mag].
template <typename T>
constexpr T limit(T x, T mag) noexcept {
    return limit(x, -mag, mag);
}

// Dead-band: returns 0 inside [-band, +band], x otherwise (no scaling).
template <typename T>
constexpr T deadBand(T x, T band) noexcept {
    return (x > band) ? (x - band) : (x < -band) ? (x + band) : T{0};
}

// Wrap angle (radians) to [-pi, +pi]. Maps +pi to +pi (not -pi).
inline double wrapPi(double x) noexcept {
    double y = std::fmod(x + PI, TWO_PI);
    if (y < 0.0) y += TWO_PI;
    y -= PI;
    // Map the -pi boundary back to +pi so the result is in (-pi, +pi]
    if (y <= -PI + 1e-15) y = PI;
    return y;
}

// Wrap angle (radians) to [0, 2*pi).
inline double wrap2Pi(double x) noexcept {
    double y = std::fmod(x, TWO_PI);
    if (y < 0.0) y += TWO_PI;
    return y;
}

// Linear interpolation.
constexpr double lerp(double a, double b, double t) noexcept {
    return a + (b - a) * t;
}

// ---------------------------------------------------------------------------
// Discrete filters (Tustin / bilinear transform equivalents)
//
// Each filter carries a small history array. The array is owned by the caller
// and is mutated in place. The conventions mirror simlib/math.cpp so that the
// legacy flight model reproduces identical step responses.
//
//   save[0] = current output (y_n)
//   save[1] = previous output (y_{n-1})
//   save[2] = previous input  (u_{n-1})
//   save[3] = (used by Adams-Bashforth) current input
//   save[4..5] = scratch
// ---------------------------------------------------------------------------

// First-order low-pass (lag) filter:   H(s) = 1 / (tau*s + 1)
// Tustin (bilinear) equivalent:
//   y_n = (dt*(u_n + u_{n-1}) + y_{n-1}*(2*tau - dt)) / (2*tau + dt)
// DC gain = 1 (so a constant input eventually tracks exactly).
struct LagFilter {
    double y_prev{0.0};   // y_{n-1}
    double u_prev{0.0};   // u_{n-1}

    double step(double u, double tau, double dt) noexcept {
        const double denom = 2.0 * tau + dt;
        if (denom < 1e-12) { y_prev = u; u_prev = u; return u; }
        const double y = (dt * (u + u_prev) + y_prev * (2.0 * tau - dt)) / denom;
        y_prev = y;
        u_prev = u;
        return y;
    }

    void reset(double y = 0.0) noexcept { y_prev = y; u_prev = y; }
};

// Trapezoidal integrator: H(s) = 1/s
//   y_n = y_{n-1} + (dt/2) * (u_n + u_{n-1})
struct Integrator {
    double y_prev{0.0};
    double u_prev{0.0};

    double step(double u, double dt) noexcept {
        const double y = y_prev + 0.5 * dt * (u + u_prev);
        y_prev = y;
        u_prev = u;
        return y;
    }

    void reset(double y = 0.0) noexcept { y_prev = y; u_prev = y; }
};

// 2nd-order Adams-Bashforth integrator: y_{n+1} = y_n + (dt/2)*(3*u_n - u_{n-1})
// Replicates FIAdamsBash() from simlib/math.cpp.
struct AdamsBash2 {
    double y_prev{0.0};
    double u_prev{0.0};   // u_{n-1}
    double u_now{0.0};    // u_n (current input)

    double step(double u, double dt) noexcept {
        u_prev = u_now;
        u_now  = u;
        const double y = y_prev + 0.5 * dt * (3.0 * u_now - u_prev);
        y_prev = y;
        return y;
    }

    void reset(double y = 0.0) noexcept { y_prev = y; u_prev = y; u_now = y; }
};

// Washout (high-pass) filter:  H(s) = tau s / (tau s + 1)
struct WashoutFilter {
    double y_prev{0.0};
    double u_prev{0.0};

    double step(double u, double tau, double dt) noexcept {
        const double denom = 2.0 * tau + dt;
        if (denom < 1e-12) { y_prev = 0.0; u_prev = u; return 0.0; }
        const double y = (2.0 * tau * (u - u_prev) + y_prev * (2.0 * tau - dt)) / denom;
        y_prev = y;
        u_prev = u;
        return y;
    }

    void reset() noexcept { y_prev = 0.0; u_prev = 0.0; }
};

} // namespace f4flight

// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// core/lookup.h
//
// 1-D and 2-D linear interpolation with cached breakpoint indices for speed.
// Direct ports of OnedInterp() / TwodInterp() from simlib/math.cpp.
//
// All data is row-major. For 2-D tables:
//   data[ix * numY + iy]
// where ix indexes the outer (slow) axis and iy indexes the inner (fast) axis.

#pragma once

#include <vector>
#include <cstddef>
#include <stdexcept>

namespace f4flight {

// ---------------------------------------------------------------------------
// 1-D linear interpolation with a cached last breakpoint index.
// Returns the first or last data value if x is outside the breakpoints
// (i.e. clamps; this matches the legacy behaviour).
// ---------------------------------------------------------------------------
class Lookup1D {
public:
    Lookup1D() = default;
    explicit Lookup1D(std::vector<double> breakpoints, std::vector<double> values)
        : x_(std::move(breakpoints)), y_(std::move(values)) {
        if (x_.size() != y_.size() || x_.empty()) {
            throw std::invalid_argument("Lookup1D: size mismatch or empty input");
        }
        last_ = 0;
    }

    double operator()(double x) const noexcept {
        const std::size_t n = x_.size();
        if (n == 1) return y_[0];

        // Clamp at ends
        if (x <= x_[0])      return y_[0];
        if (x >= x_[n - 1])  return y_[n - 1];

        // Search forward from cached index first (typical case: x advances
        // smoothly frame-to-frame).
        std::size_t i = last_;
        if (i >= n - 1) i = 0;
        if (x < x_[i]) {
            // Need to scan backward
            while (i > 0 && x < x_[i]) --i;
        } else {
            while (i < n - 2 && x >= x_[i + 1]) ++i;
        }
        last_ = i;

        const double t = (x - x_[i]) / (x_[i + 1] - x_[i]);
        return y_[i] + t * (y_[i + 1] - y_[i]);
    }

    std::size_t size() const noexcept { return x_.size(); }
    const std::vector<double>& breakpoints() const noexcept { return x_; }
    const std::vector<double>& values() const noexcept { return y_; }

private:
    std::vector<double> x_;
    std::vector<double> y_;
    mutable std::size_t last_{0};
};

// ---------------------------------------------------------------------------
// 2-D bilinear interpolation with cached breakpoint indices.
//   data[ix * numY + iy]
// ---------------------------------------------------------------------------
class Lookup2D {
public:
    Lookup2D() = default;
    Lookup2D(std::vector<double> xBreakpoints,
             std::vector<double> yBreakpoints,
             std::vector<double> data)
        : x_(std::move(xBreakpoints)),
          y_(std::move(yBreakpoints)),
          data_(std::move(data)) {
        if (x_.empty() || y_.empty()) {
            throw std::invalid_argument("Lookup2D: empty breakpoints");
        }
        if (data_.size() != x_.size() * y_.size()) {
            throw std::invalid_argument("Lookup2D: data size mismatch");
        }
        lastX_ = 0;
        lastY_ = 0;
    }

    double operator()(double x, double y) const noexcept {
        const std::size_t nx = x_.size();
        const std::size_t ny = y_.size();
        if (nx == 1 && ny == 1) return data_[0];

        // --- X-axis bracket ---
        std::size_t ix = clampIndex(x, x_, lastX_);
        lastX_ = ix;
        double tx = (nx == 1) ? 0.0
            : (x - x_[ix]) / (x_[ix + 1] - x_[ix]);
        // Clamp tx to [0,1] so out-of-range inputs don't extrapolate.
        if (tx < 0.0) tx = 0.0;
        if (tx > 1.0) tx = 1.0;

        // --- Y-axis bracket ---
        std::size_t iy = clampIndex(y, y_, lastY_);
        lastY_ = iy;
        double ty = (ny == 1) ? 0.0
            : (y - y_[iy]) / (y_[iy + 1] - y_[iy]);
        if (ty < 0.0) ty = 0.0;
        if (ty > 1.0) ty = 1.0;

        // Four corner values
        const double v00 = data_[(ix + 0) * ny + (iy + 0)];
        const double v10 = (ix + 1 < nx) ? data_[(ix + 1) * ny + (iy + 0)] : v00;
        const double v01 = (iy + 1 < ny) ? data_[(ix + 0) * ny + (iy + 1)] : v00;
        const double v11 = (ix + 1 < nx && iy + 1 < ny) ? data_[(ix + 1) * ny + (iy + 1)] : v00;

        const double a = v00 + tx * (v10 - v00);
        const double b = v01 + tx * (v11 - v01);
        return a + ty * (b - a);
    }

    std::size_t sizeX() const noexcept { return x_.size(); }
    std::size_t sizeY() const noexcept { return y_.size(); }

private:
    static std::size_t clampIndex(double v, const std::vector<double>& bp, std::size_t hint) noexcept {
        const std::size_t n = bp.size();
        if (n == 1) return 0;
        if (v <= bp[0])     return 0;
        if (v >= bp[n - 1]) return n - 2; // index of the left edge of the final bracket

        std::size_t i = hint;
        if (i >= n - 1) i = 0;
        if (v < bp[i]) {
            while (i > 0 && v < bp[i]) --i;
        } else {
            while (i < n - 2 && v >= bp[i + 1]) ++i;
        }
        return i;
    }

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> data_;
    mutable std::size_t lastX_{0};
    mutable std::size_t lastY_{0};
};

} // namespace f4flight

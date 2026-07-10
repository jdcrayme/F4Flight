// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// aircraft_config.cpp
//
// Implementation of Limiter::limit() and a few small config helpers.

#include "f4flight/aircraft_config.h"
#include "f4flight/core/math.h"

namespace f4flight {

// Evaluate a limiter. Direct port of the legacy Limiter subclasses.
double Limiter::limit(double x) const noexcept {
    switch (type) {
        case LimiterType::Line: {
            // y = m x + b, then clamp the output range to [y1, y2] and clamp
            // the input range to [x1, x2].
            const double dx = x2 - x1;
            if (std::fabs(dx) < 1e-12) return y1;
            const double xc = f4flight::limit(x, std::min(x1, x2), std::max(x1, x2));
            const double t = (xc - x1) / dx;
            const double y = y1 + t * (y2 - y1);
            return f4flight::limit(y, std::min(y1, y2), std::max(y1, y2));
        }
        case LimiterType::Value:
            return x1;
        case LimiterType::Percent:
            return x * x1;
        case LimiterType::ThreePoint: {
            // Two segments: (x0,y0)-(x1,y1) and (x1,y1)-(x2,y2)
            if (x <= x1) {
                const double dx = x1 - x0;
                if (std::fabs(dx) < 1e-12) return y1;
                const double t = f4flight::limit((x - x0) / dx, 0.0, 1.0);
                return y0 + t * (y1 - y0);
            } else {
                const double dx = x2 - x1;
                if (std::fabs(dx) < 1e-12) return y1;
                const double t = f4flight::limit((x - x1) / dx, 0.0, 1.0);
                return y1 + t * (y2 - y1);
            }
        }
        case LimiterType::MinMax:
            return f4flight::limit(x, x1, x2);
    }
    return x;
}

} // namespace f4flight

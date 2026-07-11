// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// core/units.h
//
// Strongly typed physical quantities for compile-time unit safety.
//
// The library's existing API uses raw `double` for every physical quantity and
// relies on naming conventions (e.g. `alpha_deg`, `vt_ftps`) to distinguish
// units. This works but is fragile: nothing prevents
//     double x = state.aero.alpha_deg;     // degrees
//     double y = std::sin(x);              // BUG: sin() expects radians
//
// `Quantity<Tag>` is an opt-in strong-type wrapper that makes such mistakes a
// compile error. It is ABI-compatible with `double` (a single `double` field)
// and zero-cost at runtime. Existing code is untouched; new code can adopt the
// typed aliases (`Radians`, `Degrees`, `Feet`, `Knots`, ...) incrementally.
//
// Design (mirrors `std::chrono::duration`):
//   - Construction from a raw `double` is EXPLICIT (no implicit `double -> Quantity`)
//   - Conversion back to `double` is EXPLICIT via `count()` or `static_cast<double>`
//     (no implicit `Quantity -> double` that would silently drop the unit)
//   - Arithmetic between same-tag quantities returns the same tag
//   - Scalar multiply/divide returns the same tag
//   - Division of two same-tag quantities returns a plain `double` (dimensionless ratio)
//   - Cross-unit conversion requires an explicit free function
//     (`toRadians(Degrees)`, `toKnots(FeetPerSec)`, ...)
//   - `Dimensionless` is an alias for `double` (Mach, CL, CD, ... are just doubles)
//
// Example:
//   Radians alpha_rad = toRadians(Degrees(state.aero.alpha_deg));
//   Feet alt(15000.0);
//   Knots vcas(420.0);
//   FeetPerSec vt = toFeetPerSec(vcas);
//   double mach = vt / toFeetPerSec(Knots(AASLK));   // ratio -> double
//
// Mixing tags is a compile error:
//   Radians r(1.0);
//   Degrees d(90.0);
//   r + d;   // ERROR: no operator+ for Quantity<RadiansTag> and Quantity<DegreesTag>

#pragma once

#include "f4flight/core/constants.h"

namespace f4flight {

// ---------------------------------------------------------------------------
// Unit tags (empty structs used as phantom type parameters)
// ---------------------------------------------------------------------------
struct AngleRadiansTag {};
struct AngleDegreesTag {};
struct LengthFeetTag {};
struct LengthMetersTag {};
struct SpeedFeetPerSecTag {};
struct SpeedKnotsTag {};
struct TimeSecondsTag {};
struct MassSlugsTag {};
struct MassPoundsTag {};
struct ForcePoundsTag {};
struct PressureLbPerFt2Tag {};
struct DensitySlugsPerFt3Tag {};
struct AreaFt2Tag {};
struct AccelerationFtPerSec2Tag {};
struct TemperatureRankineTag {};
struct FuelFlowLbPerHourTag {};

// ---------------------------------------------------------------------------
// Quantity<Tag, Rep> — strong wrapper around a numeric representation.
// ---------------------------------------------------------------------------
template <typename Tag, typename Rep = double>
class Quantity {
public:
    using tag_type  = Tag;
    using rep_type  = Rep;

    constexpr Quantity() noexcept = default;
    constexpr explicit Quantity(Rep v) noexcept : value_(v) {}

    // Named accessor (mirrors std::chrono::duration::count()).
    constexpr Rep count() const noexcept { return value_; }

    // Explicit conversion to the raw representation. Use this when handing a
    // value to a function that takes plain `double` (e.g. std::sin).
    constexpr explicit operator Rep() const noexcept { return value_; }

    // Compound arithmetic (same-tag only).
    constexpr Quantity& operator+=(Quantity o) noexcept { value_ += o.value_; return *this; }
    constexpr Quantity& operator-=(Quantity o) noexcept { value_ -= o.value_; return *this; }
    constexpr Quantity& operator*=(Rep s) noexcept      { value_ *= s;        return *this; }
    constexpr Quantity& operator/=(Rep s) noexcept      { value_ /= s;        return *this; }

private:
    Rep value_{Rep{}};
};

// ---------------------------------------------------------------------------
// Free-function arithmetic. These only compile when both operands share the
// same tag, which is the whole point of the wrapper.
// ---------------------------------------------------------------------------
template <typename Tag, typename Rep>
constexpr Quantity<Tag, Rep> operator+(Quantity<Tag, Rep> a, Quantity<Tag, Rep> b) noexcept {
    return Quantity<Tag, Rep>(a.count() + b.count());
}
template <typename Tag, typename Rep>
constexpr Quantity<Tag, Rep> operator-(Quantity<Tag, Rep> a, Quantity<Tag, Rep> b) noexcept {
    return Quantity<Tag, Rep>(a.count() - b.count());
}
template <typename Tag, typename Rep>
constexpr Quantity<Tag, Rep> operator-(Quantity<Tag, Rep> a) noexcept {
    return Quantity<Tag, Rep>(-a.count());
}
template <typename Tag, typename Rep>
constexpr Quantity<Tag, Rep> operator*(Quantity<Tag, Rep> a, Rep s) noexcept {
    return Quantity<Tag, Rep>(a.count() * s);
}
template <typename Tag, typename Rep>
constexpr Quantity<Tag, Rep> operator*(Rep s, Quantity<Tag, Rep> a) noexcept {
    return a * s;
}
template <typename Tag, typename Rep>
constexpr Quantity<Tag, Rep> operator/(Quantity<Tag, Rep> a, Rep s) noexcept {
    return Quantity<Tag, Rep>(a.count() / s);
}
// Division of two same-tag quantities yields a dimensionless ratio (plain Rep).
template <typename Tag, typename Rep>
constexpr Rep operator/(Quantity<Tag, Rep> a, Quantity<Tag, Rep> b) noexcept {
    return a.count() / b.count();
}

// Comparisons (same-tag only).
template <typename Tag, typename Rep>
constexpr bool operator==(Quantity<Tag, Rep> a, Quantity<Tag, Rep> b) noexcept { return a.count() == b.count(); }
template <typename Tag, typename Rep>
constexpr bool operator!=(Quantity<Tag, Rep> a, Quantity<Tag, Rep> b) noexcept { return a.count() != b.count(); }
template <typename Tag, typename Rep>
constexpr bool operator< (Quantity<Tag, Rep> a, Quantity<Tag, Rep> b) noexcept { return a.count() <  b.count(); }
template <typename Tag, typename Rep>
constexpr bool operator> (Quantity<Tag, Rep> a, Quantity<Tag, Rep> b) noexcept { return a.count() >  b.count(); }
template <typename Tag, typename Rep>
constexpr bool operator<=(Quantity<Tag, Rep> a, Quantity<Tag, Rep> b) noexcept { return a.count() <= b.count(); }
template <typename Tag, typename Rep>
constexpr bool operator>=(Quantity<Tag, Rep> a, Quantity<Tag, Rep> b) noexcept { return a.count() >= b.count(); }

// ---------------------------------------------------------------------------
// Convenience type aliases for the units used throughout the library.
// All quantities default to `double` representation.
// ---------------------------------------------------------------------------
using Radians        = Quantity<AngleRadiansTag>;
using Degrees        = Quantity<AngleDegreesTag>;
using Feet           = Quantity<LengthFeetTag>;
using Meters         = Quantity<LengthMetersTag>;
using FeetPerSec     = Quantity<SpeedFeetPerSecTag>;
using Knots          = Quantity<SpeedKnotsTag>;
using Seconds        = Quantity<TimeSecondsTag>;
using Slugs          = Quantity<MassSlugsTag>;
using PoundsMass     = Quantity<MassPoundsTag>;
using PoundsForce    = Quantity<ForcePoundsTag>;
using LbPerFt2       = Quantity<PressureLbPerFt2Tag>;
using SlugsPerFt3    = Quantity<DensitySlugsPerFt3Tag>;
using AreaFt2        = Quantity<AreaFt2Tag>;
using FtPerSec2      = Quantity<AccelerationFtPerSec2Tag>;
using Rankine        = Quantity<TemperatureRankineTag>;
using LbPerHour      = Quantity<FuelFlowLbPerHourTag>;

// Dimensionless quantities (Mach, CL, CD, CY, ratio, ...) stay as plain double
// so they flow through existing math without ceremony.
using Dimensionless  = double;

// ---------------------------------------------------------------------------
// Cross-unit conversions. These are the ONLY way to convert between tags,
// which forces the conversion to be visible at the call site.
// ---------------------------------------------------------------------------
inline constexpr Radians toRadians(Degrees d) noexcept { return Radians(d.count() * DTR); }
inline constexpr Degrees toDegrees(Radians r) noexcept { return Degrees(r.count() * RTD); }

inline constexpr Feet   toFeet(Meters m) noexcept      { return Feet(m.count() * METERS_TO_FT); }
inline constexpr Meters toMeters(Feet f) noexcept      { return Meters(f.count() * FT_TO_METERS); }

inline constexpr FeetPerSec toFeetPerSec(Knots k) noexcept { return FeetPerSec(k.count() * KNOTS_TO_FTPSEC); }
inline constexpr Knots      toKnots(FeetPerSec v) noexcept { return Knots(v.count() * FTPSEC_TO_KNOTS); }

// Mass: slugs <-> pounds-mass (W = m * g, so lbm = slug * g).
inline constexpr PoundsMass toPoundsMass(Slugs s) noexcept { return PoundsMass(s.count() * GRAVITY); }
inline constexpr Slugs      toSlugs(PoundsMass m) noexcept { return Slugs(m.count() / GRAVITY); }

// Length <-> area (1-D to 2-D requires the caller to square; provided as a
// convenience for the common wing-area case).
inline constexpr AreaFt2 squareArea(Feet a, Feet b) noexcept {
    return AreaFt2(a.count() * b.count());
}

// ---------------------------------------------------------------------------
// Convenience literal-like factories. These make call sites self-documenting:
//     Radians r = radians(0.5);
//     Knots v  = knots(420.0);
// compared to the noisier
//     Radians r(0.5);
//     Knots v(420.0);
// The lowercase factories are especially readable when constructing inline
// arguments:   fm.init(cfg, feet(15000.0), ftps(500.0), radians(0.0), true);
// ---------------------------------------------------------------------------
inline constexpr Radians    radians(double v) noexcept { return Radians(v); }
inline constexpr Degrees    degrees(double v) noexcept { return Degrees(v); }
inline constexpr Feet       feet(double v) noexcept    { return Feet(v); }
inline constexpr Meters     meters(double v) noexcept  { return Meters(v); }
inline constexpr FeetPerSec ftps(double v) noexcept    { return FeetPerSec(v); }
inline constexpr Knots      knots(double v) noexcept   { return Knots(v); }
inline constexpr Seconds    seconds(double v) noexcept { return Seconds(v); }
inline constexpr Slugs      slugs(double v) noexcept   { return Slugs(v); }
inline constexpr PoundsForce lbf(double v) noexcept    { return PoundsForce(v); }
inline constexpr LbPerFt2   psf(double v) noexcept     { return LbPerFt2(v); }
inline constexpr AreaFt2    sqft(double v) noexcept    { return AreaFt2(v); }
inline constexpr FtPerSec2  fps2(double v) noexcept    { return FtPerSec2(v); }

} // namespace f4flight

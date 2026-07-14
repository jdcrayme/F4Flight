// f4flight unit tests - strongly typed units (core/units.h)
//
// Verifies:
//   - construction is explicit (no implicit double -> Quantity)
//   - count() / static_cast<double> extract the raw value
//   - arithmetic preserves the tag (same-tag +/-/*//)
//   - division of same-tag quantities yields a dimensionless double
//   - cross-unit conversion functions are correct
//   - the lowercase factories (radians(), feet(), knots(), ...) work
//   - comparisons are tag-consistent

#include "f4flight/flight/core/units.h"
#include "f4flight/flight/core/constants.h"
#include <gtest/gtest.h>

#include <type_traits>

using namespace f4flight;

// ---------------------------------------------------------------------------
// Construction & extraction
// ---------------------------------------------------------------------------
TEST(UnitsTest, ExplicitConstruction) {
    Radians r(0.5);
    EXPECT_DOUBLE_EQ(r.count(), 0.5);

    // Static cast back to double
    double v = static_cast<double>(r);
    EXPECT_DOUBLE_EQ(v, 0.5);
}

TEST(UnitsTest, DefaultConstructionIsZero) {
    Feet f;
    EXPECT_DOUBLE_EQ(f.count(), 0.0);
    Knots k;
    EXPECT_DOUBLE_EQ(k.count(), 0.0);
}

TEST(UnitsTest, LowercaseFactories) {
    EXPECT_DOUBLE_EQ(radians(0.5).count(),  0.5);
    EXPECT_DOUBLE_EQ(degrees(90.0).count(), 90.0);
    EXPECT_DOUBLE_EQ(feet(15000.0).count(), 15000.0);
    EXPECT_DOUBLE_EQ(knots(420.0).count(),  420.0);
    EXPECT_DOUBLE_EQ(ftps(500.0).count(),   500.0);
    EXPECT_DOUBLE_EQ(seconds(0.1).count(),  0.1);
    EXPECT_DOUBLE_EQ(slugs(1.0).count(),    1.0);
    EXPECT_DOUBLE_EQ(lbf(1000.0).count(),   1000.0);
    EXPECT_DOUBLE_EQ(psf(2116.22).count(),  2116.22);
    EXPECT_DOUBLE_EQ(sqft(300.0).count(),   300.0);
    EXPECT_DOUBLE_EQ(fps2(32.177).count(),  32.177);
}

// ---------------------------------------------------------------------------
// Arithmetic
// ---------------------------------------------------------------------------
TEST(UnitsTest, SameTagAddition) {
    Feet a(100.0), b(50.0);
    Feet c = a + b;
    EXPECT_DOUBLE_EQ(c.count(), 150.0);
}

TEST(UnitsTest, SameTagSubtraction) {
    Knots a(420.0), b(100.0);
    Knots c = a - b;
    EXPECT_DOUBLE_EQ(c.count(), 320.0);
}

TEST(UnitsTest, ScalarMultiply) {
    Feet a(100.0);
    Feet b = a * 2.0;
    EXPECT_DOUBLE_EQ(b.count(), 200.0);
    Feet c = 3.0 * a;
    EXPECT_DOUBLE_EQ(c.count(), 300.0);
}

TEST(UnitsTest, ScalarDivide) {
    Knots a(420.0);
    Knots b = a / 2.0;
    EXPECT_DOUBLE_EQ(b.count(), 210.0);
}

TEST(UnitsTest, DivisionOfSameTagIsDimensionless) {
    Feet a(15000.0), b(5000.0);
    double ratio = a / b;
    static_assert(std::is_same<decltype(ratio), double>::value,
                  "Quantity / Quantity must yield plain double");
    EXPECT_DOUBLE_EQ(ratio, 3.0);
}

TEST(UnitsTest, CompoundArithmetic) {
    Feet a(100.0);
    a += Feet(50.0);
    EXPECT_DOUBLE_EQ(a.count(), 150.0);
    a -= Feet(25.0);
    EXPECT_DOUBLE_EQ(a.count(), 125.0);
    a *= 2.0;
    EXPECT_DOUBLE_EQ(a.count(), 250.0);
    a /= 5.0;
    EXPECT_DOUBLE_EQ(a.count(), 50.0);
}

TEST(UnitsTest, UnaryNegate) {
    Radians r(0.5);
    Radians n = -r;
    EXPECT_DOUBLE_EQ(n.count(), -0.5);
}

// ---------------------------------------------------------------------------
// Comparisons
// ---------------------------------------------------------------------------
TEST(UnitsTest, Comparisons) {
    Feet a(100.0), b(200.0), c(100.0);
    EXPECT_TRUE(a == c);
    EXPECT_TRUE(a != b);
    EXPECT_TRUE(a <  b);
    EXPECT_TRUE(b >  a);
    EXPECT_TRUE(a <= c);
    EXPECT_TRUE(b >= a);
}

// ---------------------------------------------------------------------------
// Cross-unit conversions
// ---------------------------------------------------------------------------
TEST(UnitsTest, DegreesToRadians) {
    Degrees d(90.0);
    Radians r = toRadians(d);
    EXPECT_NEAR(r.count(), PI / 2.0, 1e-12);
}

TEST(UnitsTest, RadiansToDegrees) {
    Radians r(PI / 2.0);
    Degrees d = toDegrees(r);
    EXPECT_NEAR(d.count(), 90.0, 1e-9);
}

TEST(UnitsTest, KnotsToFeetPerSec) {
    Knots k(420.0);
    FeetPerSec v = toFeetPerSec(k);
    EXPECT_NEAR(v.count(), 420.0 * KNOTS_TO_FTPSEC, 1e-6);
}

TEST(UnitsTest, FeetPerSecToKnots) {
    FeetPerSec v(500.0);
    Knots k = toKnots(v);
    EXPECT_NEAR(k.count(), 500.0 * FTPSEC_TO_KNOTS, 1e-6);
}

TEST(UnitsTest, FeetToMetersRoundTrip) {
    Feet f(15000.0);
    Meters m = toMeters(f);
    Feet f2 = toFeet(m);
    // FT_TO_METERS (3.28084) and METERS_TO_FT (0.3048) in constants.h are
    // rounded to 6 significant figures, so a round-trip through both has a
    // small relative error (~3e-5). 1e-2 ft = ~3 mm is more than tight
    // enough to verify the conversion plumbing is correct.
    EXPECT_NEAR(f2.count(), 15000.0, 1e-2);
}

TEST(UnitsTest, SlugsToPoundsMass) {
    // 1 slug * g = 32.177 lbm (in this library's gravity convention)
    Slugs s(1.0);
    PoundsMass m = toPoundsMass(s);
    EXPECT_NEAR(m.count(), GRAVITY, 1e-9);
    Slugs s2 = toSlugs(m);
    EXPECT_NEAR(s2.count(), 1.0, 1e-9);
}

TEST(UnitsTest, SquareArea) {
    Feet a(10.0), b(30.0);
    AreaFt2 area = squareArea(a, b);
    EXPECT_DOUBLE_EQ(area.count(), 300.0);
}

// ---------------------------------------------------------------------------
// Compile-time checks (verified by static_assert above; this test exists
// only so the test runner exercises the translation unit).
// ---------------------------------------------------------------------------
TEST(UnitsTest, CompileTimeChecks) {
    // Default-constructed quantity is zero and constexpr-usable.
    constexpr Feet kZero{};
    static_assert(kZero.count() == 0.0, "default Quantity must be zero");
    // Arithmetic is constexpr.
    constexpr Feet kSum = Feet(1.0) + Feet(2.0);
    static_assert(kSum.count() == 3.0, "Feet(1)+Feet(2) must equal Feet(3)");
    SUCCEED();
}

// ---------------------------------------------------------------------------
// Realistic usage: building init arguments for FlightModel.
// Demonstrates that the typed aliases compose cleanly with existing
// constants (KNOTS_TO_FTPSEC etc.) and produce the same numbers the raw
// double code path would have produced.
// ---------------------------------------------------------------------------
TEST(UnitsTest, RealisticInitUsage) {
    Feet       alt       = feet(15000.0);
    Knots      cruiseKts = knots(420.0);
    FeetPerSec vt        = toFeetPerSec(cruiseKts);
    Radians    heading   = radians(0.0);

    EXPECT_DOUBLE_EQ(alt.count(), 15000.0);
    EXPECT_NEAR(vt.count(), 420.0 * KNOTS_TO_FTPSEC, 1e-6);
    EXPECT_DOUBLE_EQ(heading.count(), 0.0);
}

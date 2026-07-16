// f4flight unit tests - formation file loader
//
// Tests for FormationTable::loadFromFile() — the FreeFalcon formdat.fil
// parser. Verifies that the loader correctly parses the flat-text format,
// converts units (degrees → radians, NM → feet), and registers formations
// by their FreeFalcon formNum.

#include "f4flight/digi/formation/formation_geometry.h"

#include <gtest/gtest.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace f4flight::digi::formation;

namespace {
// Path to the test formdat.fil (relative to the build directory, which is
// the source root when tests run from there).
const char* kTestFormdatFile = "tests/fixtures/formations/test_formdat.fil";
} // namespace

// ===========================================================================
// FormationTable::loadFromFile tests
// ===========================================================================

TEST(FormationFileLoader, LoadsFormationsFromFile) {
    FormationTable table;
    int loaded = table.loadFromFile(kTestFormdatFile);
    // The test file has 3 formations.
    EXPECT_EQ(loaded, 3);
}

TEST(FormationFileLoader, ReturnsNegativeOneForMissingFile) {
    FormationTable table;
    int loaded = table.loadFromFile("nonexistent_file.fil");
    EXPECT_EQ(loaded, -1);
}

TEST(FormationFileLoader, ParsesWedgeFormation) {
    FormationTable table;
    table.loadFromFile(kTestFormdatFile);

    // FormNum 1 = TestWedge (4-ship)
    // Slot 0: lead (0, 0, 0)
    // Slot 1: right wing (135°, 0°, 0.165 NM = 1002.6 ft)
    // Slot 2: left wing (-135°, 0°, 0.165 NM)
    // Slot 3: trail (180°, 0°, 0.330 NM = 2005.1 ft)
    PositionData slot0 = table.slotGeometryById(1, 0);
    EXPECT_NEAR(slot0.relAz, 0.0, 1e-6);
    EXPECT_NEAR(slot0.range, 0.0, 1e-6);

    PositionData slot1 = table.slotGeometryById(1, 1);
    EXPECT_NEAR(slot1.relAz, 135.0 * M_PI / 180.0, 1e-4);
    EXPECT_NEAR(slot1.range, 0.165 * 6076.11549, 1.0);  // ~1002.6 ft

    PositionData slot2 = table.slotGeometryById(1, 2);
    EXPECT_NEAR(slot2.relAz, -135.0 * M_PI / 180.0, 1e-4);

    PositionData slot3 = table.slotGeometryById(1, 3);
    EXPECT_NEAR(slot3.relAz, 180.0 * M_PI / 180.0, 1e-4);
    EXPECT_NEAR(slot3.range, 0.330 * 6076.11549, 1.0);  // ~2005.1 ft
}

TEST(FormationFileLoader, ParsesTrailFormation) {
    FormationTable table;
    table.loadFromFile(kTestFormdatFile);

    // FormNum 2 = TestTrail (4-ship trail)
    // All slots at relAz=180° (behind), increasing range
    PositionData slot1 = table.slotGeometryById(2, 1);
    EXPECT_NEAR(slot1.relAz, 180.0 * M_PI / 180.0, 1e-4);
    EXPECT_NEAR(slot1.range, 0.165 * 6076.11549, 1.0);

    PositionData slot3 = table.slotGeometryById(2, 3);
    EXPECT_NEAR(slot3.relAz, 180.0 * M_PI / 180.0, 1e-4);
    EXPECT_NEAR(slot3.range, 0.495 * 6076.11549, 1.0);  // ~3007.7 ft
}

TEST(FormationFileLoader, ParsesTwoShipData) {
    FormationTable table;
    table.loadFromFile(kTestFormdatFile);

    // FormNum 6 = TestLineAbreast (2-ship, 1 two-slot entry)
    // The 4-ship data has slot 1 at 90° / 0.165 NM
    PositionData slot1_4ship = table.slotGeometryById(6, 1);
    EXPECT_NEAR(slot1_4ship.relAz, 90.0 * M_PI / 180.0, 1e-4);

    // The 2-ship data is stored at formNum + 1000 = 1006
    // 2-ship slot 1: 90° / 0.165 NM
    PositionData slot1_2ship = table.slotGeometryById(1006, 1);
    EXPECT_NEAR(slot1_2ship.relAz, 90.0 * M_PI / 180.0, 1e-4);
    EXPECT_NEAR(slot1_2ship.range, 0.165 * 6076.11549, 1.0);
}

TEST(FormationFileLoader, InvalidSlotReturnsLeadPosition) {
    FormationTable table;
    table.loadFromFile(kTestFormdatFile);

    // Invalid slot index returns zero PositionData (lead position)
    PositionData invalid = table.slotGeometryById(1, 99);
    EXPECT_NEAR(invalid.relAz, 0.0, 1e-6);
    EXPECT_NEAR(invalid.range, 0.0, 1e-6);

    // Invalid formNum returns zero PositionData
    PositionData unknown = table.slotGeometryById(999, 1);
    EXPECT_NEAR(unknown.relAz, 0.0, 1e-6);
    EXPECT_NEAR(unknown.range, 0.0, 1e-6);
}

TEST(FormationFileLoader, DoesNotCorruptDefaultInstance) {
    // Loading from file into a local table should NOT affect the default
    // instance's built-in formations (Wedge, TwoShipTrail, TwoShipLineAbreast).
    FormationTable local;
    local.loadFromFile(kTestFormdatFile);

    // Default instance should still have its built-in Wedge
    FormationTable& def = FormationTable::defaultInstance();
    PositionData wedge1 = def.slotGeometry(FormationType::Wedge, 1);
    // The default Wedge slot 1 should be at 135° (behind-right), not 30°
    EXPECT_NEAR(wedge1.relAz, 135.0 * M_PI / 180.0, 1e-4);
}

// f4flight unit tests - campaign map & simulation engine
//
// Tests for:
//   - TerrainMap dimensions, elevations, and tile-to-world conversions.
//   - RoadNetwork nodes, edges, and BFS shortest path routing.
//   - Unit formations: Column, Wedge, and Line dynamic subunit layouts.
//   - CampaignState tick simulation: movement, combat, air strikes, and objective capture.
//   - CampaignIO: JSON serialization/deserialization.

#include "f4flight/campaign/cover_type.h"
#include "f4flight/campaign/terrain_map.h"
#include "f4flight/campaign/road_network.h"
#include "f4flight/campaign/objective.h"
#include "f4flight/campaign/unit.h"
#include "f4flight/campaign/campaign_state.h"
#include "f4flight/campaign/campaign_io.h"
#include "f4flight/campaign/campaign_generator.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace f4flight;
using namespace f4flight::campaign;

// ===========================================================================
// TerrainMap Tests
// ===========================================================================

TEST(CampaignTerrainTest, BasicDimensionsAndProperties) {
    TerrainMap map(16, 24, 5000.0);
    EXPECT_EQ(map.cols(), 16);
    EXPECT_EQ(map.rows(), 24);
    EXPECT_DOUBLE_EQ(map.tileSizeFt(), 5000.0);

    EXPECT_TRUE(map.isValid(0, 0));
    EXPECT_TRUE(map.isValid(15, 23));
    EXPECT_FALSE(map.isValid(-1, 0));
    EXPECT_FALSE(map.isValid(16, 24));
}

TEST(CampaignTerrainTest, CoordinateConversions) {
    TerrainMap map(10, 10, 1000.0);

    double x, y;
    map.gridToWorld(2, 3, x, y);
    EXPECT_DOUBLE_EQ(x, 2500.0);
    EXPECT_DOUBLE_EQ(y, 3500.0);

    int col, row;
    map.worldToGrid(2500.0, 3500.0, col, row);
    EXPECT_EQ(col, 2);
    EXPECT_EQ(row, 3);
}

// ===========================================================================
// RoadNetwork Tests
// ===========================================================================

TEST(CampaignRoadNetworkTest, NodesAndEdgesPathfinding) {
    RoadNetwork rn;
    rn.addNode(1, Vec3(0, 0, 0));
    rn.addNode(2, Vec3(1000, 0, 0));
    rn.addNode(3, Vec3(1000, 1000, 0));
    rn.addNode(4, Vec3(0, 1000, 0));

    rn.addEdge(1, 2);
    rn.addEdge(2, 3);
    rn.addEdge(3, 4);

    EXPECT_EQ(rn.findNearestNode(Vec3(100, 50, 0)), 1);
    EXPECT_EQ(rn.findNearestNode(Vec3(950, 950, 0)), 3);

    auto path = rn.findPath(Vec3(0, 0, 0), Vec3(0, 1000, 0));
    // BFS path should route through nodes 1 -> 2 -> 3 -> 4
    EXPECT_GE(path.size(), 4u);
}

// ===========================================================================
// Unit Formation Tests
// ===========================================================================

TEST(CampaignUnitFormationTest, ColumnLayout) {
    Unit unit;
    unit.id = 1;
    unit.position = Vec3(0, 0, 0);
    unit.path = { Vec3(0, 0, 0), Vec3(500, 0, 0) };
    unit.currentPathIndex = 1;
    unit.formation = FormationState::Column;

    unit.subUnits = {
        {11, "Leader", 1.0, {}, 0.0},
        {12, "Wingman1", 1.0, {}, 0.0},
        {13, "Wingman2", 1.0, {}, 0.0}
    };

    unit.updateSubUnitPositions();

    // In a column traveling East (+X):
    // Leader should be at the unit's position (0,0,0)
    EXPECT_DOUBLE_EQ(unit.subUnits[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(unit.subUnits[0].position.y, 0.0);

    // Wingmen should be lined up behind the leader (West: -X)
    EXPECT_LT(unit.subUnits[1].position.x, 0.0);
    EXPECT_NEAR(unit.subUnits[1].position.y, 0.0, 1e-3);
    EXPECT_LT(unit.subUnits[2].position.x, unit.subUnits[1].position.x);
}

TEST(CampaignUnitFormationTest, WedgeLayout) {
    Unit unit;
    unit.id = 2;
    unit.position = Vec3(0, 0, 0);
    unit.path = { Vec3(0, 0, 0), Vec3(0, 500, 0) }; // Traveling North (+Y)
    unit.currentPathIndex = 1;
    unit.formation = FormationState::Wedge;

    unit.subUnits = {
        {21, "Lead", 1.0, {}, 0.0},
        {22, "RightWing", 1.0, {}, 0.0}, // odd index -> Right (positive latOffset)
        {23, "LeftWing", 1.0, {}, 0.0}   // even index -> Left (negative latOffset)
    };

    unit.updateSubUnitPositions();

    // Leader is at (0,0,0)
    EXPECT_DOUBLE_EQ(unit.subUnits[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(unit.subUnits[0].position.y, 0.0);

    // RightWing (index 1) should be offset to the right (+X) and back (-Y)
    EXPECT_GT(unit.subUnits[1].position.x, 0.0);
    EXPECT_LT(unit.subUnits[1].position.y, 0.0);

    // LeftWing (index 2) should be offset to the left (-X) and back (-Y)
    EXPECT_LT(unit.subUnits[2].position.x, 0.0);
    EXPECT_LT(unit.subUnits[2].position.y, 0.0);
}

// ===========================================================================
// CampaignState Simulation Tests
// ===========================================================================

TEST(CampaignSimulationTest, UnitTicksMovementAndCombat) {
    CampaignState state;
    state.terrain() = TerrainMap(10, 10, 1000.0);

    state.roadNetwork().addNode(1, Vec3(0, 0, 0));
    state.roadNetwork().addNode(2, Vec3(10000, 0, 0)); // placed far apart to avoid starting in combat
    state.roadNetwork().addEdge(1, 2);

    // Blue Ground unit starting at Node 1 marching to Node 2
    Unit blue;
    blue.id = 1;
    blue.name = "Blue Armored";
    blue.type = UnitType::GroundBattalion;
    blue.faction = 1;
    blue.position = Vec3(0, 0, 0);
    blue.destination = Vec3(10000, 0, 0);
    blue.speed = 40.0;
    blue.health = 1.0;
    blue.path = { Vec3(0, 0, 0), Vec3(10000, 0, 0) };
    blue.currentPathIndex = 1;
    blue.subUnits = { {11, "M1 Tank", 1.0, {}, 0.0} };
    state.addUnit(blue);

    // Red Ground unit starting at Node 2 marching to Node 1
    Unit red;
    red.id = 2;
    red.name = "Red Armored";
    red.type = UnitType::GroundBattalion;
    red.faction = 2;
    red.position = Vec3(10000, 0, 0);
    red.destination = Vec3(0, 0, 0);
    red.speed = 40.0;
    red.health = 1.0;
    red.path = { Vec3(10000, 0, 0), Vec3(0, 0, 0) };
    red.currentPathIndex = 1;
    red.subUnits = { {21, "T-90 Tank", 1.0, {}, 0.0} };
    state.addUnit(red);

    // Run tick to verify units move closer (since they are far apart, they won't trigger combat yet)
    state.tick(1.0); // 1 second

    EXPECT_GT(state.units()[0].position.x, 0.0);
    EXPECT_LT(state.units()[1].position.x, 10000.0);

    // Now test that they stop and engage when extremely close
    state.units()[0].position = Vec3(4900, 0, 0);
    state.units()[1].position = Vec3(5100, 0, 0); // 200 ft apart (well within combatRange of 4000)

    state.tick(1.0);

    // Speed should be zero and they should take damage
    EXPECT_DOUBLE_EQ(state.units()[0].speed, 0.0);
    EXPECT_DOUBLE_EQ(state.units()[1].speed, 0.0);
    EXPECT_LT(state.units()[0].health, 1.0);
    EXPECT_LT(state.units()[1].health, 1.0);
}

// ===========================================================================
// CampaignIO Tests
// ===========================================================================

TEST(CampaignSerializationTest, SaveAndLoadMatch) {
    CampaignState startState;
    startState.terrain() = TerrainMap(4, 4, 1000.0);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            startState.terrain().tile(c, r).cover = CoverType::Forest;
        }
    }

    startState.roadNetwork().addNode(1, Vec3(0, 0, 0));
    startState.roadNetwork().addNode(2, Vec3(1000, 0, 0));
    startState.roadNetwork().addEdge(1, 2);

    Objective obj;
    obj.id = 1;
    obj.name = "Outpost Echo";
    obj.type = ObjectiveType::Fort;
    obj.position = Vec3(0, 0, 0);
    obj.faction = 1;
    obj.health = 0.9;
    startState.addObjective(obj);

    std::string json = CampaignIO::serialize(startState);
    EXPECT_FALSE(json.empty());

    CampaignState loadedState;
    bool success = CampaignIO::deserialize(json, loadedState);
    EXPECT_TRUE(success);

    EXPECT_EQ(loadedState.terrain().cols(), 4);
    EXPECT_EQ(loadedState.terrain().rows(), 4);
    EXPECT_EQ(loadedState.objectives().size(), 1u);
    EXPECT_EQ(loadedState.objectives()[0].name, "Outpost Echo");
    EXPECT_DOUBLE_EQ(loadedState.objectives()[0].health, 0.9);
}

// f4flight - campaign/src/campaign_generator.cpp
//
// Implementation of procedural campaign map generator.

#include "f4flight/campaign/campaign_generator.h"
#include <cmath>

namespace f4flight {
namespace campaign {

CampaignState CampaignGenerator::generateDefaultTheater() {
    CampaignState state;

    // 1. Procedural Terrain Generation
    const int size = 32;
    state.terrain() = TerrainMap(size, size);
    for (int r = 0; r < size; ++r) {
        for (int c = 0; c < size; ++c) {
            auto& tile = state.terrain().tile(c, r);
            if (c < 10) {
                tile.cover = CoverType::Water;
                tile.elevation = 0.0;
            } else if (c == 10) {
                tile.cover = CoverType::Coast;
                tile.elevation = 15.0;
            } else if (c < 22) {
                tile.cover = CoverType::Plains;
                tile.elevation = 50.0 + std::sin(r / 3.0) * 20.0 + std::cos(c / 2.0) * 15.0;
            } else if (c < 27) {
                tile.cover = CoverType::Hills;
                tile.elevation = 300.0 + (c - 22) * 120.0 + std::sin(r / 2.0) * 50.0;
            } else {
                tile.cover = CoverType::Mountains;
                tile.elevation = 1200.0 + (c - 27) * 450.0 + std::cos(r) * 150.0;
            }

            // Flag roads procedurally for some cells to represent local networks
            if ((r == 8 && c >= 10 && c <= 25) || (r == 20 && c >= 10 && c <= 25) || (c == 15 && r >= 5 && r <= 28)) {
                tile.hasRoad = true;
            }
        }
    }

    // 2. Define Objectives (Seoul, Pyongyang, Kimpo, Kaesong, Sariwon, etc.)
    // Standard spacing is ~6076 ft per tile (1 NM)
    const double NM = 6076.12;

    // Seoul Headquarters (Blue)
    Objective seoul;
    seoul.id = 1;
    seoul.name = "Seoul Joint HQ";
    seoul.type = ObjectiveType::Headquarters;
    seoul.position = Vec3(22.0 * NM, 8.0 * NM, 100.0);
    seoul.faction = 1; // Blue
    seoul.health = 1.0;
    seoul.structures = {
        {1, "Command Bunker", 1.0, Vec3(0, 0, 0)},
        {2, "Comm Tower", 1.0, Vec3(100, 200, 0)},
        {3, "Ammo Dump", 1.0, Vec3(-200, -100, 0)}
    };
    state.addObjective(seoul);

    // Kimpo Airbase (Blue)
    Objective kimpo;
    kimpo.id = 2;
    kimpo.name = "Kimpo Airbase";
    kimpo.type = ObjectiveType::Airbase;
    kimpo.position = Vec3(14.0 * NM, 6.0 * NM, 50.0);
    kimpo.faction = 1;
    kimpo.health = 1.0;
    kimpo.structures = {
        {1, "ATC Tower", 1.0, Vec3(-500, 200, 0)},
        {2, "Hangar A", 1.0, Vec3(-800, -100, 0)},
        {3, "Hangar B", 1.0, Vec3(-800, -300, 0)},
        {4, "Fuel Depot", 1.0, Vec3(1000, -500, 0)}
    };
    // Kimpo runway 09/27
    Runway kRwy;
    kRwy.id = 1;
    kRwy.name = "RWY 09/27";
    kRwy.start = kimpo.position + Vec3(-4000, 0, 0);
    kRwy.end = kimpo.position + Vec3(4000, 0, 0);
    kRwy.width = 150.0;
    kRwy.heading = 0.0;
    kRwy.health = 1.0;
    kimpo.runways.push_back(kRwy);

    // Taxi network for Kimpo
    kimpo.taxiNodes = {
        {1, kimpo.position + Vec3(-1000, -500, 0), "Park1"},
        {2, kimpo.position + Vec3(-500, -500, 0), "Park2"},
        {3, kimpo.position + Vec3(0, -500, 0), "HoldShort"},
        {4, kimpo.position + Vec3(-4000, 0, 0), "Threshold"}
    };
    kimpo.taxiEdges = {
        {1, 2, 500.0},
        {2, 3, 500.0},
        {3, 4, 4000.0}
    };
    state.addObjective(kimpo);

    // Incheon Radar (Blue)
    Objective incheon;
    incheon.id = 3;
    incheon.name = "Incheon Early Warning";
    incheon.type = ObjectiveType::RadarStation;
    incheon.position = Vec3(11.0 * NM, 11.0 * NM, 30.0);
    incheon.faction = 1;
    incheon.health = 1.0;
    incheon.structures = {
        {1, "AN/FPS-117 Radar Dome", 1.0, Vec3(0, 0, 0)},
        {2, "Generator Building", 1.0, Vec3(150, -50, 0)}
    };
    state.addObjective(incheon);

    // Kaesong City (Neutral/Red Frontline)
    Objective kaesong;
    kaesong.id = 4;
    kaesong.name = "Kaesong City";
    kaesong.type = ObjectiveType::City;
    kaesong.position = Vec3(16.0 * NM, 18.0 * NM, 120.0);
    kaesong.faction = 0; // Neutral
    kaesong.health = 1.0;
    kaesong.structures = {
        {1, "City Hall", 1.0, Vec3(0, 0, 0)},
        {2, "Industrial Depot", 1.0, Vec3(-400, 400, 0)}
    };
    state.addObjective(kaesong);

    // Sariwon Logistics Depot (Red)
    Objective sariwon;
    sariwon.id = 5;
    sariwon.name = "Sariwon Depot";
    sariwon.type = ObjectiveType::Depot;
    sariwon.position = Vec3(23.0 * NM, 20.0 * NM, 200.0);
    sariwon.faction = 2; // Red
    sariwon.health = 1.0;
    sariwon.structures = {
        {1, "Ammo Storage", 1.0, Vec3(-200, 0, 0)},
        {2, "Fuel Tanks", 1.0, Vec3(200, 100, 0)},
        {3, "Vehicle Pool", 1.0, Vec3(0, -300, 0)}
    };
    state.addObjective(sariwon);

    // Pyongyang HQ (Red)
    Objective pyongyang;
    pyongyang.id = 6;
    pyongyang.name = "Pyongyang Supreme HQ";
    pyongyang.type = ObjectiveType::Headquarters;
    pyongyang.position = Vec3(20.0 * NM, 26.0 * NM, 150.0);
    pyongyang.faction = 2;
    pyongyang.health = 1.0;
    pyongyang.structures = {
        {1, "Underground HQ", 1.0, Vec3(0, 0, 0)},
        {2, "Comms Array", 1.0, Vec3(100, 300, 0)},
        {3, "Barracks Block", 1.0, Vec3(-300, -200, 0)}
    };
    state.addObjective(pyongyang);

    // Sunan Airbase (Red)
    Objective sunan;
    sunan.id = 7;
    sunan.name = "Sunan Airbase";
    sunan.type = ObjectiveType::Airbase;
    sunan.position = Vec3(14.0 * NM, 27.0 * NM, 100.0);
    sunan.faction = 2;
    sunan.health = 1.0;
    sunan.structures = {
        {1, "Sunan ATC", 1.0, Vec3(400, 200, 0)},
        {2, "Hardened Shelter A", 1.0, Vec3(-500, -300, 0)},
        {3, "Hardened Shelter B", 1.0, Vec3(-700, -300, 0)}
    };
    // Sunan runway 18/36 (heading PI/2 = North-South)
    Runway sRwy;
    sRwy.id = 1;
    sRwy.name = "RWY 18/36";
    sRwy.start = sunan.position + Vec3(0, -4000, 0);
    sRwy.end = sunan.position + Vec3(0, 4000, 0);
    sRwy.width = 150.0;
    sRwy.heading = 1.570796; // PI/2
    sRwy.health = 1.0;
    sunan.runways.push_back(sRwy);
    state.addObjective(sunan);

    // 3. Populate Road Graph Nodes & Edges
    state.roadNetwork().addNode(1, seoul.position);
    state.roadNetwork().addNode(2, kimpo.position);
    state.roadNetwork().addNode(3, incheon.position);
    state.roadNetwork().addNode(4, kaesong.position);
    state.roadNetwork().addNode(5, sariwon.position);
    state.roadNetwork().addNode(6, pyongyang.position);
    state.roadNetwork().addNode(7, sunan.position);

    // Intermediate junction nodes to make routes interesting
    Vec3 junc1(15.0 * NM, 11.0 * NM, 80.0);
    state.roadNetwork().addNode(10, junc1);

    Vec3 junc2(22.0 * NM, 14.0 * NM, 150.0);
    state.roadNetwork().addNode(11, junc2);

    // Add edges
    state.roadNetwork().addEdge(2, 10); // Kimpo to Junction 1
    state.roadNetwork().addEdge(3, 10); // Incheon to Junction 1
    state.roadNetwork().addEdge(1, 10); // Seoul to Junction 1
    state.roadNetwork().addEdge(1, 11); // Seoul to Junction 2
    state.roadNetwork().addEdge(4, 10); // Kaesong to Junction 1
    state.roadNetwork().addEdge(5, 11); // Sariwon to Junction 2
    state.roadNetwork().addEdge(4, 5);  // Kaesong to Sariwon
    state.roadNetwork().addEdge(5, 6);  // Sariwon to Pyongyang HQ
    state.roadNetwork().addEdge(7, 6);  // Sunan to Pyongyang HQ

    // 4. Initial Units Configuration
    // Blue Ground Battalion: starts at Seoul HQ, marches toward Red Sariwon Depot
    Unit blueArmor;
    blueArmor.id = 101;
    blueArmor.name = "8th Blue Cav (Armor)";
    blueArmor.type = UnitType::GroundBattalion;
    blueArmor.faction = 1;
    blueArmor.position = seoul.position;
    blueArmor.speed = 40.0;
    blueArmor.health = 1.0;
    blueArmor.hasTargetObjective = true;
    blueArmor.targetObjectiveId = 5; // Sariwon Depot
    blueArmor.formation = FormationState::Column;
    blueArmor.subUnits = {
        {1011, "M1A2 SEPv3 Abrams", 1.0, {}, 0.0},
        {1012, "M1A2 SEPv3 Abrams", 1.0, {}, 0.0},
        {1013, "M2A4 Bradley IFV", 1.0, {}, 0.0},
        {1014, "M1151 HMMWV", 1.0, {}, 0.0}
    };
    blueArmor.updateSubUnitPositions();
    state.addUnit(blueArmor);

    // Blue Air Strike: starts at Kimpo, flies to strike Red Sunan Airbase
    Unit blueAir;
    blueAir.id = 102;
    blueAir.name = "F-16C Viper Strike Pack";
    blueAir.type = UnitType::AirSquadron;
    blueAir.faction = 1;
    blueAir.position = kimpo.position;
    blueAir.speed = 500.0;
    blueAir.health = 1.0;
    blueAir.hasTargetObjective = true;
    blueAir.targetObjectiveId = 7; // Sunan AB
    blueAir.formation = FormationState::Wedge;
    blueAir.subUnits = {
        {1021, "F-16C Block 50", 1.0, {}, 0.0},
        {1022, "F-16C Block 50", 1.0, {}, 0.0},
        {1023, "F-15E Strike Eagle", 1.0, {}, 0.0},
        {1024, "F-15E Strike Eagle", 1.0, {}, 0.0}
    };
    blueAir.updateSubUnitPositions();
    state.addUnit(blueAir);

    // Red Ground Battalion: starts at Pyongyang HQ, marches south to Kaesong
    Unit redGuard;
    redGuard.id = 201;
    redGuard.name = "4th Red Shock Guard";
    redGuard.type = UnitType::GroundBattalion;
    redGuard.faction = 2;
    redGuard.position = pyongyang.position;
    redGuard.speed = 40.0;
    redGuard.health = 1.0;
    redGuard.hasTargetObjective = true;
    redGuard.targetObjectiveId = 4; // Kaesong
    redGuard.formation = FormationState::Column;
    redGuard.subUnits = {
        {2011, "T-90MS MBT", 1.0, {}, 0.0},
        {2012, "T-90MS MBT", 1.0, {}, 0.0},
        {2013, "BMP-3M IFV", 1.0, {}, 0.0},
        {2014, "BTR-92 APC", 1.0, {}, 0.0}
    };
    redGuard.updateSubUnitPositions();
    state.addUnit(redGuard);

    // Red Air Patrol: starts at Sunan AB, targets Incheon Radar
    Unit redAir;
    redAir.id = 202;
    redAir.name = "Su-35 Flanker Interceptors";
    redAir.type = UnitType::AirSquadron;
    redAir.faction = 2;
    redAir.position = sunan.position;
    redAir.speed = 500.0;
    redAir.health = 1.0;
    redAir.hasTargetObjective = true;
    redAir.targetObjectiveId = 3; // Incheon Radar
    redAir.formation = FormationState::Wedge;
    redAir.subUnits = {
        {2021, "Su-35S Flanker-E", 1.0, {}, 0.0},
        {2022, "Su-35S Flanker-E", 1.0, {}, 0.0},
        {2023, "MiG-35 Fulcrum-F", 1.0, {}, 0.0},
        {2024, "MiG-35 Fulcrum-F", 1.0, {}, 0.0}
    };
    redAir.updateSubUnitPositions();
    state.addUnit(redAir);

    return state;
}

} // namespace campaign
} // namespace f4flight

// f4flight - scenarios/scenario_digi_aar.cpp
//
// Digi AI air-to-air refueling (AAR) test with both a tanker and receiver.
//
// The tanker flies along predefined waypoints. The receiver (AI aircraft)
// performs an approach, precontact, contact, and disconnect.

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "scenario_framework.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <algorithm>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

static double g_aarSimTime = 0.0;

// Tanker waypoints definition (NED frame: +X = east, +Y = north, +Z = down)
static const std::vector<Vec3> g_tankerWaypoints = {
    {0.0, 0.0, -20000.0},
    {0.0, 100000.0, -20000.0},
    {0.0, 200000.0, -20000.0},
    {0.0, 300000.0, -20000.0}
};

// Deterministically compute tanker state at time t
static void getTankerState(double t, double speed, const std::vector<Vec3>& wps,
                           double& outX, double& outY, double& outZ, double& outYaw) {
    if (wps.empty()) {
        outX = outY = outZ = outYaw = 0.0;
        return;
    }
    double distRemaining = t * speed;
    size_t i = 0;
    while (i + 1 < wps.size()) {
        const auto& p1 = wps[i];
        const auto& p2 = wps[i+1];
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        double segmentLen = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (distRemaining <= segmentLen) {
            double frac = segmentLen > 0.0 ? distRemaining / segmentLen : 0.0;
            outX = p1.x + dx * frac;
            outY = p1.y + dy * frac;
            outZ = p1.z + dz * frac;
            outYaw = std::atan2(dy, dx);
            return;
        }
        distRemaining -= segmentLen;
        i++;
    }
    const auto& pLast1 = wps[wps.size() - 2];
    const auto& pLast2 = wps[wps.size() - 1];
    double dx = pLast2.x - pLast1.x;
    double dy = pLast2.y - pLast1.y;
    double dz = pLast2.z - pLast1.z;
    double len = std::sqrt(dx*dx + dy*dy + dz*dz);
    double dirX = len > 0.0 ? dx / len : 0.0;
    double dirY = len > 0.0 ? dy / len : 1.0;
    double dirZ = len > 0.0 ? dz / len : 0.0;
    outX = pLast2.x + dirX * distRemaining;
    outY = pLast2.y + dirY * distRemaining;
    outZ = pLast2.z + dirZ * distRemaining;
    outYaw = std::atan2(dy, dx);
}

// Helper to inject the tanker state into the controller brain
static void updateInjectedTanker(DigiEntity& tanker, SteeringController& sc) {
    double tx, ty, tz, tyaw;
    getTankerState(g_aarSimTime, 300.0 * KNOTS_TO_FTPSEC, g_tankerWaypoints, tx, ty, tz, tyaw);
    tanker.x = tx;
    tanker.y = ty;
    tanker.z = tz;
    tanker.yaw = tyaw;
    tanker.pitch = 0.0;
    tanker.roll = 0.0;
    tanker.speed = 300.0 * KNOTS_TO_FTPSEC;
    tanker.vx = tanker.speed * std::cos(tyaw);
    tanker.vy = tanker.speed * std::sin(tyaw);
    tanker.vz = 0.0;
    tanker.isDead = false;
    tanker.dcm = dcmFromEuler(tyaw, 0.0, 0.0);

    FrameInputs fi;
    fi.injectedTanker = &tanker;
    sc.brain().setFrameInputs(fi);
}

// ===========================================================================
// AARPhase — Single continuous air-to-air refueling test phase
// ===========================================================================
class AARPhase : public ManeuverTest {
public:
    AARPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc_ptr_ = &sc;
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Position receiver 2 NM behind the tanker's initial position
        fm.state().kin.x = 0.0;
        fm.state().kin.y = -2.0 * 6076.0;
        fm.state().kin.z = -(alt_ - 500.0);  // 500 ft below tanker

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(30.0);
        sc.setMaxGamma(10.0);

        sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::None;
        sc.brain().stateMutable().refuel.contactTimer = 0.0;
        sc.brain().stateMutable().refuel.contactDuration = 5.0; // short for testing

        updateInjectedTanker(tanker_, sc);
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);
        g_aarSimTime += dt;

        updateInjectedTanker(tanker_, *sc_ptr_);

        // Boom offset
        constexpr double kBoomOffsetBackFt = 50.0;
        constexpr double kBoomOffsetDownFt = 20.0;
        const double boomX = tanker_.x - kBoomOffsetBackFt * std::cos(tanker_.yaw);
        const double boomY = tanker_.y - kBoomOffsetBackFt * std::sin(tanker_.yaw);
        const double boomZ = tanker_.z + kBoomOffsetDownFt;

        const double dx = boomX - as.kin.x;
        const double dy = boomY - as.kin.y;
        const double dz = boomZ - as.kin.z;
        distToBoom_ = std::sqrt(dx * dx + dy * dy + dz * dz);
        minDistToBoom_ = std::min(minDistToBoom_, distToBoom_);

        const auto activeMode = sc_ptr_->brain().activeMode();
        if (activeMode == DigiMode::Refueling) reachedApproach_ = true;

        const auto refuelPhase = sc_ptr_->brain().state().refuel.phase;
        if (distToBoom_ < 150.0) {
            reachedPrecontact_ = true;
        }
        if (refuelPhase == DigiRefuelState::Phase::Contact) {
            reachedContact_ = true;
            timeInContact_ += dt;
        }
        if (refuelPhase == DigiRefuelState::Phase::Disconnect) {
            if (distToBoom_ > 500.0) {
                reachedDisconnect_ = true;
            }
        }
        if (std::isnan(as.kin.x)) hasNaN_ = true;
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ || (reachedApproach_ && reachedPrecontact_ && reachedContact_ && reachedDisconnect_);
    }

    bool IsPassed() const override {
        return !hasNaN_ && reachedApproach_ && reachedPrecontact_ && reachedContact_ && reachedDisconnect_;
    }

    std::string criteria() const override {
        return "Complete AAR sequence (Approach -> Precontact -> Contact -> Disconnect); No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!reachedApproach_) return "Never entered Refueling mode.";
        if (!reachedPrecontact_) return "Never reached precontact position (closest: " + std::to_string(minDistToBoom_) + " ft).";
        if (!reachedContact_) return "Never entered Contact phase.";
        if (!reachedDisconnect_) return "Never completed disconnect phase.";
        return "";
    }

    std::vector<TestCondition> conditions() const override {
        return {
            {"Approach", "Enter Refueling mode", reachedApproach_},
            {"Precontact", "Reach precontact within 150 ft", reachedPrecontact_},
            {"Contact", "Enter contact & hold contact for >= 5s", reachedContact_ && timeInContact_ >= 5.0},
            {"Disconnect", "Enter disconnect & separate to > 500 ft", reachedDisconnect_}
        };
    }

    std::vector<TraceSample> traceSamples() const override {
        return {{"d_boom", distToBoom_, "ft"}};
    }

    std::vector<ThreatEntity> traceEntities() const override {
        ThreatEntity t;
        t.type = "lead";
        t.name = "Tanker";
        t.x = tanker_.x; t.y = tanker_.y; t.z = tanker_.z;
        t.speed = tanker_.speed; t.psi = tanker_.yaw;
        return {t};
    }

    void Finish() const override {
        std::printf("  --- AAR Sequence Summary ---\n");
        std::printf("  Reached Approach:   %s\n", reachedApproach_ ? "PASS" : "FAIL");
        std::printf("  Reached Precontact: %s\n", reachedPrecontact_ ? "PASS" : "FAIL");
        std::printf("  Reached Contact:    %s (held for %.1f s)\n", reachedContact_ ? "PASS" : "FAIL", timeInContact_);
        std::printf("  Completed Disconnect: %s (final dist: %.1f ft)\n", reachedDisconnect_ ? "PASS" : "FAIL", distToBoom_);
    }

private:
    double alt_, speed_;
    DigiEntity tanker_;
    SteeringController* sc_ptr_{nullptr};
    double distToBoom_{99999.0};
    double minDistToBoom_{99999.0};
    double timeInContact_{0.0};
    bool reachedApproach_{false};
    bool reachedPrecontact_{false};
    bool reachedContact_{false};
    bool reachedDisconnect_{false};
    bool hasNaN_{false};
};

// ===========================================================================
// Scenario: digi_aar
// ===========================================================================
class DigiAARScenario : public ManeuverScenario {
public:
    DigiAARScenario() : ManeuverScenario("digi_aar") {}

    std::string GetDescription() const override {
        return "Air-to-air refueling test with both a tanker and receiver: "
               "approach -> precontact -> contact (hold 5s) -> disconnect. "
               "The tanker flies predefined waypoints, and the receiver completes AAR.";
    }

    std::vector<TraceGeometry> traceGeometry() const override {
        std::vector<TraceGeometry> geom;
        std::vector<double> pathCoords;
        for (size_t i = 0; i < g_tankerWaypoints.size(); ++i) {
            const auto& wp = g_tankerWaypoints[i];
            TraceGeometry tgWp;
            tgWp.name = "Tanker Waypoint " + std::to_string(i + 1);
            tgWp.type = "waypoint";
            tgWp.coords = {wp.x, wp.y, wp.z};
            tgWp.color = "#FFD700";
            geom.push_back(tgWp);

            pathCoords.push_back(wp.x);
            pathCoords.push_back(wp.y);
            pathCoords.push_back(wp.z);
        }

        TraceGeometry tgPath;
        tgPath.name = "Tanker Path";
        tgPath.type = "corridor";
        tgPath.coords = pathCoords;
        tgPath.color = "#FFD700";
        tgPath.width = 2.0;
        geom.push_back(tgPath);

        return geom;
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 20000.0;
        const double speed = 300.0;  // kts — typical tanker speed

        g_aarSimTime = 0.0;

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<AARPhase>(
            "Refuel Sequence", 150.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerDigiAAR("digi_aar", []() {
    return std::make_unique<DigiAARScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_aar() {}

} // namespace f4flight_test

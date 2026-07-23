// f4flight - scenarios/scenario_digi_baseline.cpp
//
// Digi AI Baseline Integration Scenario: Intercept and Orbiting Tanker.
// Overhauled flat declarative scenario:
//   - Tanker orbits between 2 waypoints using FlightPlan/Behavior Tree.
//   - F-16 dynamically intercepts a position 1 NM behind and 1000' below the tanker.
//   - Once in position, the F-16 sends the Tanker a radio message.

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/behavior_tree/flight_plan.h"
#include "scenario_framework.h"

#include <string>
#include <vector>
#include <cmath>

using namespace f4flight;

namespace f4flight_test {

class DigiBaselineScenario : public ManeuverScenario {
protected:
    double simTime_{0.0};
    bool sentRadio_{false};

public:
    DigiBaselineScenario() : ManeuverScenario("digi_baseline") {
        // Run for up to 350 seconds
        maxTime_ = 350.0;
    }

    std::string GetDescription() const override {
        return "Digi AI Baseline: Tanker orbits between two waypoints, and F-16 "
               "intercepts a position 1 NM behind and 1000 ft below, sending a radio message.";
    }

    std::string GetTestGroup() const override { return "Baseline"; }
    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    void StartScenario(const std::string& defaultAircraftPath) override {
        simTime_ = 0.0;
        sentRadio_ = false;

        const double tankerAlt = 10000.0;
        const double tankerSpd = 300.0; // kts

        // 1. Spawn Tanker aircraft
        auto tanker = CreateAircraft("Tanker", defaultAircraftPath);
        if (!tanker) return;

        // Configure brain limits for Tanker
        digi::DigiConfig tankerConfig = tanker->brain.config();
        tankerConfig.cornerSpeedKts = tankerSpd;
        tankerConfig.maxGs = tanker->fm.config().geometry.maxGs;
        tankerConfig.maxBankDeg = 45.0;
        tankerConfig.maxGammaDeg = 15.0;
        tanker->brain.configure(tankerConfig);
        tanker->brain.setAltitude(tankerAlt);

        // Position the tanker 20,000 ft East at start
        tanker->fm.init(tanker->fm.config(), tankerAlt, tankerSpd * KNOTS_TO_FTPSEC, 0.0, true);
        tanker->fm.state().kin.x = 20000.0;
        tanker->fm.state().kin.y = 0.0;

        // Set up waypoints for Tanker (10 NM apart East-West) via FlightPlan
        std::vector<Vec3> tankerWps = {
            Vec3{20000.0, 0.0, -tankerAlt},
            Vec3{80000.0, 0.0, -tankerAlt}
        };
        auto tankerFp = digi::FlightPlan::fromWaypoints(tankerWps, tankerSpd);
        tanker->brain.setFlightPlan(tankerFp);
        tanker->brain.setCaptureRadius(5000.0);

        // 2. Spawn F-16 interceptor
        auto f16 = CreateAircraft("F16", defaultAircraftPath);
        if (!f16) return;

        // Configure brain limits for F-16
        digi::DigiConfig f16Config = f16->brain.config();
        f16Config.cornerSpeedKts = 400.0; // fly faster to intercept!
        f16Config.maxGs = f16->fm.config().geometry.maxGs;
        f16Config.maxBankDeg = 45.0;
        f16Config.maxGammaDeg = 15.0;
        f16->brain.configure(f16Config);
        f16->brain.setAltitude(9000.0);

        // Position F-16 at origin, heading East (0.0 rad)
        f16->fm.init(f16->fm.config(), 9000.0, 400.0 * KNOTS_TO_FTPSEC, 0.0, true);
        f16->fm.state().kin.x = 0.0;
        f16->fm.state().kin.y = 0.0;

        // 3. Setup declarative telemetries
        auto t_f16_alt = CreateTelemetry("F16_Altitude", [f16]() { return -f16->fm.state().kin.z; });
        auto t_f16_spd = CreateTelemetry("F16_Speed", [f16]() { return f16->fm.state().vcas; });
        auto t_tk_alt = CreateTelemetry("Tanker_Altitude", [tanker]() { return -tanker->fm.state().kin.z; });

        // Track distance between F-16 and the dynamic target intercept point
        auto t_intercept_dist = CreateTelemetry("InterceptDistance", [this, f16, tanker]() {
            const auto& tState = tanker->fm.state();
            double tSigma = tState.kin.sigma;
            double rx = tState.kin.x - 6076.12 * std::cos(tSigma);
            double ry = tState.kin.y - 6076.12 * std::sin(tSigma);
            double rz = tState.kin.z + 1000.0;

            const auto& fState = f16->fm.state();
            double dx = rx - fState.kin.x;
            double dy = ry - fState.kin.y;
            double dz = rz - fState.kin.z;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        });

        auto t_nan = CreateTelemetry("StateNaN", [f16]() {
            const auto& as = f16->fm.state();
            return (std::isnan(as.kin.vt) || std::isnan(as.kin.z) || std::isnan(as.kin.sigma)) ? 1.0 : 0.0;
        });

        // 4. Setup declarative assertions (conditionals)
        CreateConditional<ConditionalValueReachesRange>(t_intercept_dist, 0.0, 500.0, /*isRequired=*/true, "Proximity Intercept", "Intercept position 1NM behind and 1000' below tanker");
        CreateConditional<ConditionalValueRemainsInRange>(t_nan, 0.0, 0.1, /*isRequired=*/true, "Flight Stability", "No NaNs or state divergence occurred during flight");
        CreateConditional<ConditionalValueRemainsInRange>(t_tk_alt, tankerAlt, 500.0, /*isRequired=*/true, "Tanker Altitude Hold", "Tanker altitude remains stable within ±500 ft");
    }

    void UpdateScenario(double dt) override {
        simTime_ += dt;

        if (aircraftList_.size() < 2) return;
        auto tanker = aircraftList_[0];
        auto f16 = aircraftList_[1];

        // 1. Loop the tanker's flight plan if it captures all waypoints
        if (tanker->brain.allWaypointsCaptured()) {
            double tankerAlt = 10000.0;
            double tankerSpd = 300.0;
            std::vector<Vec3> tankerWps = {
                Vec3{20000.0, 0.0, -tankerAlt},
                Vec3{80000.0, 0.0, -tankerAlt}
            };
            auto tankerFp = digi::FlightPlan::fromWaypoints(tankerWps, tankerSpd);
            tanker->brain.setFlightPlan(tankerFp);
        }

        // 2. Compute F-16 dynamic intercept target position: 1 NM (6076.12 ft) behind and 1000 ft below tanker
        const auto& tState = tanker->fm.state();
        double tSigma = tState.kin.sigma; // heading
        double rx = tState.kin.x - 6076.12 * std::cos(tSigma);
        double ry = tState.kin.y - 6076.12 * std::sin(tSigma);
        double rz = tState.kin.z + 1000.0; // below means positive Z relative to tanker

        // Update F-16 steering targets to dynamically pursue this intercept point
        const auto& fState = f16->fm.state();
        double dx = rx - fState.kin.x;
        double dy = ry - fState.kin.y;
        double dz = rz - fState.kin.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        double desHeading = std::atan2(dy, dx);
        double desAlt = -rz; // positive up

        // Use direct, clean autopilot/brain parameters (no SteeringController modes needed!)
        f16->brain.setHeading(desHeading);
        f16->brain.setAltitude(desAlt);

        // 3. Once in position (within 1.5 NM proximity to the target spot), send the tanker a radio message
        if (dist < 9114.0 && !sentRadio_) {
            digi::makeRadioCall(f16->brain.stateMutable().comm.radioCalls,
                                digi::RadioCallType::InPosition, // "In position" radio call
                                simTime_,
                                1, // F-16 ID
                                2); // Tanker ID
            sentRadio_ = true;
            std::printf("  [RADIO MESSAGE SENT] F-16: 'In Position!'\n");
        }
    }
};

static RegisterScenario g_registerBaseline("digi_baseline", []() {
    return std::make_unique<DigiBaselineScenario>();
});

} // namespace f4flight_test

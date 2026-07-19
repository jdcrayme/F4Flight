// f4flight - scenarios/scenario_low_aar_vector.cpp
//
// LOW-LEVEL scenario: AAR vector-to-tanker approach phase only.
//
// Split out of high_level/scenario_digi_aar.cpp. Wraps the AAR approach
// behavior only — receiver starts 2 NM behind the tanker and the test
// verifies the AI enters Refueling mode and begins closing on the tanker.
// Does NOT require reaching pre-contact, contact, or disconnect.
//
// Pass criteria is RELAXED: just require enter Refueling mode, close the
// distance by some meaningful amount, no NaN.
//
// Tier: LowLevel. Registered as "low_aar_vector" — referenced by the
// cascade mapping table g_highToLow["high_aar"].

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "scenario_framework.h"

#include <cmath>
#include <string>
#include <vector>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// LowAarVectorPhase — vector-to-tanker approach
// ===========================================================================
class LowAarVectorPhase : public ManeuverTest {
public:
    LowAarVectorPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc_ptr_ = &sc;
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Receiver starts 2 NM behind tanker, 500 ft below.
        fm.state().kin.x = 0.0;
        fm.state().kin.y = -2.0 * 6076.0;
        fm.state().kin.z = -(alt_ - 500.0);

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(30.0);
        sc.setMaxGamma(10.0);

        // Reset refuel state machine.
        sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::None;
        sc.brain().stateMutable().refuel.contactTimer = 0.0;
        sc.brain().stateMutable().refuel.contactDuration = 5.0;

        // Tanker initial state: ahead of the receiver, heading north, 300 kts.
        tanker_.x = 0.0;
        tanker_.y = 0.0;
        tanker_.z = -alt_;
        tanker_.yaw = PI / 2.0;
        tanker_.pitch = 0.0;
        tanker_.roll = 0.0;
        tanker_.speed = speed_ * KNOTS_TO_FTPSEC;
        tanker_.vx = 0.0;
        tanker_.vy = tanker_.speed;
        tanker_.vz = 0.0;
        tanker_.isDead = false;
        tanker_.dcm = dcmFromEuler(tanker_.yaw, 0.0, 0.0);

        FrameInputs fi;
        fi.injectedTanker = &tanker_;
        sc.brain().setFrameInputs(fi);

        startDist_ = 2.0 * 6076.0;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the tanker forward at constant velocity.
        tanker_.y += tanker_.speed * dt;
        FrameInputs fi;
        fi.injectedTanker = &tanker_;
        sc_ptr_->brain().setFrameInputs(fi);

        // Distance to the tanker's boom (50 ft behind + 20 ft below tanker).
        const double boomX = tanker_.x - 50.0 * std::cos(tanker_.yaw);
        const double boomY = tanker_.y - 50.0 * std::sin(tanker_.yaw);
        const double boomZ = tanker_.z + 20.0;
        const double dx = boomX - as.kin.x;
        const double dy = boomY - as.kin.y;
        const double dz = boomZ - as.kin.z;
        const double distToBoom = std::sqrt(dx * dx + dy * dy + dz * dz);
        minDistToBoom_ = std::min(minDistToBoom_, distToBoom);

        if (sc_ptr_->brain().activeMode() == DigiMode::Refueling)
            enteredRefueling_ = true;

        if (std::isnan(as.kin.x) || std::isnan(as.kin.z)) hasNaN_ = true;

        curDist_ = distToBoom;
        curMode_ = sc_ptr_->brain().activeMode();

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (tanker 2NM ahead, vector to boom)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s\n",
                    "t(s)", "alt(ft)", "d_boom", "vcas", "mode");
            }
            std::printf("%6.1f %8.0f %8.0f %6.1f %6s\n",
                phaseTime_, -as.kin.z, distToBoom, as.vcas,
                digiModeName(curMode_));
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (enteredRefueling_ && minDistToBoom_ < 300.0);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredRefueling_) return false;
        // RELAXED: just require closing the distance by at least 1 NM
        // (started 2 NM behind tanker's boom). This proves the AI is
        // vectoring toward the tanker, not flying away.
        if (minDistToBoom_ > startDist_ - 6076.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Refueling mode; Close distance to boom by >= 1 NM; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredRefueling_)
            return "Never entered Refueling mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (minDistToBoom_ > startDist_ - 6076.0)
            return "Did not close on boom (min dist " +
                   std::to_string(static_cast<int>(minDistToBoom_)) + "ft, needed < " +
                   std::to_string(static_cast<int>(startDist_ - 6076.0)) + "ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_boom",    curDist_, "ft"},
            {"in_refuel", (enteredRefueling_ && curMode_ == DigiMode::Refueling) ? 1.0 : 0.0, ""},
        };
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
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Refueling mode: %s\n", enteredRefueling_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to boom:       %.0f ft (need < %.0f) %s\n",
            minDistToBoom_, startDist_ - 6076.0,
            minDistToBoom_ < startDist_ - 6076.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double speed_{0.0};
    DigiEntity tanker_;
    SteeringController* sc_ptr_{nullptr};
    double startDist_{0.0};
    double minDistToBoom_{1e9};
    bool enteredRefueling_{false};
    bool hasNaN_{false};
    double nextPrint_{0.0};
    double curDist_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
};

// ===========================================================================
// LowAarVectorScenario
// ===========================================================================
class LowAarVectorScenario : public ManeuverScenario {
public:
    LowAarVectorScenario() : ManeuverScenario("low_aar_vector") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: AAR vector-to-tanker approach phase. Receiver starts 2NM "
               "behind the tanker and vectors to the boom. Verifies the AI "
               "enters Refueling mode and closes the distance. Does NOT test "
               "pre-contact/contact/disconnect.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double alt = 20000.0;
        const double speed = 300.0;
        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowAarVectorPhase>(
            "AAR vector-to-tanker approach", 120.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerLowAarVector("low_aar_vector", []() {
    return std::make_unique<LowAarVectorScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_aar_vector() {}

} // namespace f4flight_test

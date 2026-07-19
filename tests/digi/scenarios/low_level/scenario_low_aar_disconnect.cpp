// f4flight - scenarios/scenario_low_aar_disconnect.cpp
//
// LOW-LEVEL scenario: AAR disconnect phase (depart the tanker).
//
// Split out of high_level/scenario_digi_aar.cpp. Receiver starts AT the
// boom with a very short contactDuration (0.5s), so the brain enters
// Contact then immediately transitions to Disconnect. The test verifies
// the AI enters the Disconnect phase and increases distance from the boom.
//
// Pass criteria is RELAXED: parent requires > 500 ft of separation; we
// require > 250 ft ( Disconnect only needs to start moving away).
//
// Tier: LowLevel. Registered as "low_aar_disconnect" — referenced by the
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
// LowAarDisconnectPhase — short contact → enter Disconnect → separate
// ===========================================================================
class LowAarDisconnectPhase : public ManeuverTest {
public:
    LowAarDisconnectPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc_ptr_ = &sc;
        const double initialHeading = PI / 2.0;
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Receiver starts AT the boom.
        const double tankerY = 0.0;
        const double boomY = tankerY - 50.0;
        const double boomZ = -alt_ + 20.0;

        fm.state().kin.x = 0.0;
        fm.state().kin.y = boomY;
        fm.state().kin.z = boomZ;

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(30.0);
        sc.setMaxGamma(10.0);

        // Short contact duration → quick transition to Disconnect.
        sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::None;
        sc.brain().stateMutable().refuel.contactTimer = 0.0;
        sc.brain().stateMutable().refuel.contactDuration = 0.5;

        tanker_.x = 0.0;
        tanker_.y = tankerY;
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
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move tanker forward.
        tanker_.y += tanker_.speed * dt;
        FrameInputs fi;
        fi.injectedTanker = &tanker_;
        sc_ptr_->brain().setFrameInputs(fi);

        const double boomX = tanker_.x - 50.0 * std::cos(tanker_.yaw);
        const double boomY = tanker_.y - 50.0 * std::sin(tanker_.yaw);
        const double boomZ = tanker_.z + 20.0;
        const double dx = boomX - as.kin.x;
        const double dy = boomY - as.kin.y;
        const double dz = boomZ - as.kin.z;
        const double distToBoom = std::sqrt(dx * dx + dy * dy + dz * dz);
        maxDistToBoom_ = std::max(maxDistToBoom_, distToBoom);

        if (sc_ptr_->brain().activeMode() == DigiMode::Refueling)
            enteredRefueling_ = true;
        if (sc_ptr_->brain().state().refuel.phase == DigiRefuelState::Phase::Disconnect) {
            enteredDisconnect_ = true;
        }

        if (std::isnan(as.kin.x) || std::isnan(as.kin.z)) hasNaN_ = true;

        curDist_ = distToBoom;
        curMode_ = sc_ptr_->brain().activeMode();
        curPhase_ = sc_ptr_->brain().state().refuel.phase;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (short contact then disconnect)\n", testName_.c_str());
                std::printf("%6s %8s %8s %6s %6s\n",
                    "t(s)", "alt(ft)", "d_boom", "vcas", "phase");
            }
            std::printf("%6.1f %8.0f %8.0f %6.1f %6d\n",
                phaseTime_, -as.kin.z, distToBoom, as.vcas,
                static_cast<int>(curPhase_));
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        // End early once we've separated from the boom after Disconnect.
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (enteredDisconnect_ && maxDistToBoom_ > 250.0);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredRefueling_) return false;
        if (!enteredDisconnect_) return false;
        // RELAXED: parent requires > 500ft of separation; we require > 250ft.
        if (maxDistToBoom_ < 250.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Refueling mode; Enter Disconnect phase; Separate to > 250ft; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredRefueling_)
            return "Never entered Refueling mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!enteredDisconnect_)
            return "Never entered Disconnect phase (refuel phase id: " +
                   std::to_string(static_cast<int>(curPhase_)) + ").";
        if (maxDistToBoom_ < 250.0)
            return "Did not separate from boom (max dist " +
                   std::to_string(static_cast<int>(maxDistToBoom_)) +
                   "ft, needed > 250ft).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_boom",        curDist_, "ft"},
            {"in_disconnect", (enteredDisconnect_ &&
                               curPhase_ == DigiRefuelState::Phase::Disconnect) ? 1.0 : 0.0, ""},
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
        std::printf("  Entered Refueling:   %s\n", enteredRefueling_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Disconnect:  %s\n", enteredDisconnect_ ? "[PASS]" : "[FAIL]");
        std::printf("  Max dist from boom:  %.0f ft (need > 250) %s\n",
            maxDistToBoom_, maxDistToBoom_ > 250.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double speed_{0.0};
    DigiEntity tanker_;
    SteeringController* sc_ptr_{nullptr};
    double maxDistToBoom_{0.0};
    bool enteredRefueling_{false};
    bool enteredDisconnect_{false};
    bool hasNaN_{false};
    double nextPrint_{0.0};
    double curDist_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    DigiRefuelState::Phase curPhase_{DigiRefuelState::Phase::None};
};

// ===========================================================================
// LowAarDisconnectScenario
// ===========================================================================
class LowAarDisconnectScenario : public ManeuverScenario {
public:
    LowAarDisconnectScenario() : ManeuverScenario("low_aar_disconnect") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: AAR disconnect phase. Receiver starts AT the boom with a "
               "0.5s contactDuration, enters Contact, transitions to Disconnect, "
               "and separates from the tanker. Verifies the AI enters Disconnect "
               "and increases distance from the boom.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double alt = 20000.0;
        const double speed = 300.0;
        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowAarDisconnectPhase>(
            "AAR disconnect", 60.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerLowAarDisconnect("low_aar_disconnect", []() {
    return std::make_unique<LowAarDisconnectScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_aar_disconnect() {}

} // namespace f4flight_test

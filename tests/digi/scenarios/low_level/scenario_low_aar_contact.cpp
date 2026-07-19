// f4flight - scenarios/scenario_low_aar_contact.cpp
//
// LOW-LEVEL scenario: AAR contact phase (hold at the boom, take fuel).
//
// Split out of high_level/scenario_digi_aar.cpp. Receiver starts very close
// to the boom (50 ft behind, 20 ft below). The test verifies the AI enters
// Refueling mode AND enters the Contact sub-phase AND holds contact for
// a few seconds.
//
// Pass criteria is RELAXED: parent requires 5s of contact; we require 2s.
//
// Tier: LowLevel. Registered as "low_aar_contact" — referenced by the
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
// LowAarContactPhase — enter Contact + hold for >= 2s
// ===========================================================================
class LowAarContactPhase : public ManeuverTest {
public:
    LowAarContactPhase(const char* name, double duration, double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        sc_ptr_ = &sc;
        const double initialHeading = PI / 2.0;
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Receiver starts AT the boom position (50 ft behind tanker, 20 ft
        // below). The brain should immediately enter Contact phase.
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

        sc.brain().stateMutable().refuel.phase = DigiRefuelState::Phase::None;
        sc.brain().stateMutable().refuel.contactTimer = 0.0;
        // contactDuration large so we don't transition to Disconnect during
        // this test.
        sc.brain().stateMutable().refuel.contactDuration = 600.0;

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
        minDistToBoom_ = std::min(minDistToBoom_, distToBoom);

        if (sc_ptr_->brain().activeMode() == DigiMode::Refueling)
            enteredRefueling_ = true;
        if (sc_ptr_->brain().state().refuel.phase == DigiRefuelState::Phase::Contact) {
            enteredContact_ = true;
            timeInContact_ += dt;
        }

        if (std::isnan(as.kin.x) || std::isnan(as.kin.z)) hasNaN_ = true;

        curDist_ = distToBoom;
        curMode_ = sc_ptr_->brain().activeMode();
        curPhase_ = sc_ptr_->brain().state().refuel.phase;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (start at boom, hold contact)\n", testName_.c_str());
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
        return phaseTime_ >= maxTime_ || hasNaN_ ||
               (enteredContact_ && timeInContact_ >= 2.0);
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        if (!enteredRefueling_) return false;
        if (!enteredContact_) return false;
        // RELAXED: parent requires 5s of contact; we require 2s.
        if (timeInContact_ < 2.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Refueling mode; Enter Contact phase; Hold contact >= 2s; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredRefueling_)
            return "Never entered Refueling mode (final: " +
                   std::string(digiModeName(curMode_)) + ").";
        if (!enteredContact_)
            return "Never entered Contact phase (refuel phase id: " +
                   std::to_string(static_cast<int>(curPhase_)) + ").";
        if (timeInContact_ < 2.0)
            return "Held contact for only " + std::to_string(timeInContact_) +
                   "s (needed >= 2.0s).";
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_boom",      curDist_, "ft"},
            {"in_contact",  (enteredContact_ &&
                             curPhase_ == DigiRefuelState::Phase::Contact) ? 1.0 : 0.0, ""},
            {"contact_t",   timeInContact_, "s"},
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
        std::printf("  Entered Refueling:  %s\n", enteredRefueling_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Contact:    %s\n", enteredContact_ ? "[PASS]" : "[FAIL]");
        std::printf("  Time in contact:    %.1fs (need >= 2.0) %s\n",
            timeInContact_, timeInContact_ >= 2.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double alt_{0.0};
    double speed_{0.0};
    DigiEntity tanker_;
    SteeringController* sc_ptr_{nullptr};
    double minDistToBoom_{1e9};
    double timeInContact_{0.0};
    bool enteredRefueling_{false};
    bool enteredContact_{false};
    bool hasNaN_{false};
    double nextPrint_{0.0};
    double curDist_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    DigiRefuelState::Phase curPhase_{DigiRefuelState::Phase::None};
};

// ===========================================================================
// LowAarContactScenario
// ===========================================================================
class LowAarContactScenario : public ManeuverScenario {
public:
    LowAarContactScenario() : ManeuverScenario("low_aar_contact") {}

    TestTier GetTestTier() const override { return TestTier::LowLevel; }

    std::string GetDescription() const override {
        return "LOW: AAR contact phase. Receiver starts AT the boom and "
               "verifies the AI enters Contact phase and holds for >= 2s.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {
        const double alt = 20000.0;
        const double speed = 300.0;
        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);
        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<LowAarContactPhase>(
            "AAR contact hold", 60.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerLowAarContact("low_aar_contact", []() {
    return std::make_unique<LowAarContactScenario>();
});

extern "C" void f4flight_forceLink_scenario_low_aar_contact() {}

} // namespace f4flight_test

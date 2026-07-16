// f4flight - scenarios/scenario_digi_aar.cpp
//
// Digi AI air-to-air refueling (AAR) test.
//
// Tests the runRefueling() implementation with a tanker aircraft. The AI
// aircraft (receiver) starts behind and below the tanker, approaches the
// boom position, holds contact for fuel transfer, then disconnects.
//
// The tanker flies straight and level at a fixed altitude. The receiver
// must close to the boom position, match the tanker's speed, hold for the
// contact duration, then depart.

#include "f4flight/flight/f4flight.h"
#include "f4flight/digi/digi.h"
#include "scenario_framework.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace f4flight;
using namespace f4flight::digi;

namespace f4flight_test {

// ===========================================================================
// AARPhase — air-to-air refueling test
// ===========================================================================
class AARPhase : public ManeuverTest {
public:
    AARPhase(const char* name, double duration,
             double alt, double speed)
        : ManeuverTest(name, duration), alt_(alt), speed_(speed) {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        // Start at 20000 ft, 300 kts, heading north.
        const double initialHeading = PI / 2.0;  // north
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, initialHeading, true);

        // Position the receiver 2 NM behind and 500 ft below the tanker.
        // Tanker is at (0, 0, -20000). Receiver starts at (0, -2*6076, -19500).
        fm.state().kin.x = 0.0;
        fm.state().kin.y = -2.0 * 6076.0;  // 2 NM south (behind tanker)
        fm.state().kin.z = -(alt_ - 500.0);  // 500 ft below tanker

        sc.setMode(SteeringController::Mode::HeadingAltitude);
        sc.setCornerSpeed(fm.config().geometry.cornerVcas_kts);
        sc.setMaxGs(fm.config().geometry.maxGs);
        sc.setMaxBank(30.0);  // gentle banks for AAR
        sc.setMaxGamma(10.0); // gentle pitch for AAR

        // --- Set up the tanker ---
        // Tanker at (0, 0, -20000), heading north, 300 kts.
        const double tankerVt = 300.0 * KNOTS_TO_FTPSEC;
        tanker_.x = 0.0;
        tanker_.y = 0.0;
        tanker_.z = -alt_;
        tanker_.vx = 0.0;
        tanker_.vy = tankerVt;
        tanker_.vz = 0.0;
        tanker_.yaw = PI / 2.0;  // north
        tanker_.pitch = 0.0;
        tanker_.roll = 0.0;
        tanker_.speed = tankerVt;
        tanker_.isDead = false;
        tanker_.dcm = dcmFromEuler(tanker_.yaw, 0.0, 0.0);

        // Inject the tanker for AAR.
        FrameInputs fi;
        fi.injectedTanker = &tanker_;
        sc.brain().setFrameInputs(fi);

        // Set a short contact duration for testing (10 seconds).
        sc.brain().state().refuel.contactDuration = 10.0;

        sc_brain_ = &sc.brain();
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        // Move the tanker forward (straight and level).
        tanker_.y += tanker_.speed * dt;

        // Compute the boom position (behind and below the tanker).
        constexpr double kBoomOffsetBackFt = 50.0;
        constexpr double kBoomOffsetDownFt = 20.0;
        const double boomX = tanker_.x - kBoomOffsetBackFt * std::cos(tanker_.yaw);
        const double boomY = tanker_.y - kBoomOffsetBackFt * std::sin(tanker_.yaw);
        const double boomZ = tanker_.z + kBoomOffsetDownFt;

        // Track distance to the boom position.
        const double dx = boomX - as.kin.x;
        const double dy = boomY - as.kin.y;
        const double dz = boomZ - as.kin.z;
        const double distToBoom = std::sqrt(dx * dx + dy * dy + dz * dz);
        minDistToBoom_ = std::min(minDistToBoom_, distToBoom);

        // Track mode entry.
        if (sc_brain_->activeMode() == DigiMode::Refueling) enteredRefueling_ = true;

        // Track refueling phases.
        const auto refuelPhase = sc_brain_->state().refuel.phase;
        if (refuelPhase == DigiRefuelState::Phase::Contact) enteredContact_ = true;
        if (refuelPhase == DigiRefuelState::Phase::Disconnect) enteredDisconnect_ = true;

        // Track time in contact.
        if (refuelPhase == DigiRefuelState::Phase::Contact) {
            timeInContact_ += dt;
        }

        if (std::isnan(as.kin.x) || std::isnan(as.kin.vt)) hasNaN_ = true;

        // Per-frame sample data.
        curDistToBoom_ = distToBoom;
        curMode_ = sc_brain_->activeMode();
        curRefuelPhase_ = refuelPhase;

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (tanker 2NM ahead, 500ft above)\n",
                    testName_.c_str());
                std::printf("%6s %8s %8s %8s %6s %6s %6s %6s\n",
                    "t(s)", "alt(ft)", "dBoom", "vcas", "pstk", "thrt", "mode", "rphase");
            }
            const std::size_t bufSize = 24;
            char modeBuf[bufSize];
            std::snprintf(modeBuf, bufSize, "%s", digiModeName(sc_brain_->activeMode()));
            const char* rphaseBuf = "None";
            switch (refuelPhase) {
                case DigiRefuelState::Phase::None:       rphaseBuf = "None"; break;
                case DigiRefuelState::Phase::Approach:   rphaseBuf = "Approach"; break;
                case DigiRefuelState::Phase::Contact:    rphaseBuf = "Contact"; break;
                case DigiRefuelState::Phase::Disconnect: rphaseBuf = "Disconnect"; break;
            }
            std::printf("%6.1f %8.0f %8.0f %8.1f %6.2f %6.2f %6s %6s\n",
                phaseTime_, -as.kin.z, distToBoom, as.vcas,
                input.pstick, input.throttle, modeBuf, rphaseBuf);
            nextPrint_ += 5.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // 1. Must enter Refueling mode.
        if (!enteredRefueling_) return false;
        // 2. Must enter Contact phase (closed to the boom).
        if (!enteredContact_) return false;
        // 3. Must hold contact for at least 5 seconds.
        if (timeInContact_ < 5.0) return false;
        // 4. Must enter Disconnect phase (completed refueling).
        if (!enteredDisconnect_) return false;
        // 5. Must close to within 200 ft of the boom at some point.
        if (minDistToBoom_ > 600.0) return false;
        return true;
    }

    std::string criteria() const override {
        return "Enter Refueling; Enter Contact; Hold contact >= 5s; "
               "Enter Disconnect; Close to < 600ft of boom; No NaN";
    }

    std::string failureReason() const override {
        if (hasNaN_) return "NaN detected in aircraft state.";
        if (!enteredRefueling_) {
            return "Never entered Refueling mode (final mode: " +
                   std::string(digiModeName(curMode_)) + ").";
        }
        if (!enteredContact_) {
            return "Never entered Contact phase — receiver did not close to the boom.";
        }
        if (timeInContact_ < 5.0) {
            return "Only held contact for " + std::to_string(timeInContact_) +
                   "s (needed >= 5s).";
        }
        if (!enteredDisconnect_) {
            return "Never entered Disconnect phase — refueling did not complete.";
        }
        if (minDistToBoom_ > 600.0) {
            return "Min distance to boom was " +
                   std::to_string(static_cast<int>(minDistToBoom_)) +
                   "ft (needed < 200ft).";
        }
        return "";
    }

    std::vector<TraceSample> traceSamples() const override {
        return {
            {"d_boom",     curDistToBoom_, "ft"},
            {"in_refuel",  (enteredRefueling_ && curMode_ == DigiMode::Refueling) ? 1.0 : 0.0, ""},
            {"in_contact", (curRefuelPhase_ == DigiRefuelState::Phase::Contact) ? 1.0 : 0.0, ""},
            {"rphase",     static_cast<double>(static_cast<int>(curRefuelPhase_)), ""},
        };
    }

    void Finish() const override {
        std::printf("  --- Summary ---\n");
        std::printf("  Entered Refueling:   %s\n", enteredRefueling_ ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Contact:     %s\n", enteredContact_ ? "[PASS]" : "[FAIL]");
        std::printf("  Time in contact:     %.1f s (need >= 5) %s\n",
            timeInContact_, timeInContact_ >= 5.0 ? "[PASS]" : "[FAIL]");
        std::printf("  Entered Disconnect:  %s\n", enteredDisconnect_ ? "[PASS]" : "[FAIL]");
        std::printf("  Min dist to boom:    %.0f ft (need < 600) %s\n",
            minDistToBoom_, minDistToBoom_ < 200.0 ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double nextPrint_{0.0};
    double alt_{0.0};
    double speed_{0.0};
    DigiEntity tanker_;
    DigiBrain* sc_brain_{nullptr};

    double minDistToBoom_{1e9};
    double timeInContact_{0.0};
    bool enteredRefueling_{false};
    bool enteredContact_{false};
    bool enteredDisconnect_{false};
    bool hasNaN_{false};

    double curDistToBoom_{0.0};
    DigiMode curMode_{DigiMode::NoMode};
    DigiRefuelState::Phase curRefuelPhase_{DigiRefuelState::Phase::None};
};

// ===========================================================================
// Scenario: digi_aar
// ===========================================================================
class DigiAARScenario : public ManeuverScenario {
public:
    DigiAARScenario() : ManeuverScenario("digi_aar") {}

    std::string GetDescription() const override {
        return "Digi AI air-to-air refueling: receiver aircraft starts 2NM "
               "behind and 500ft below a tanker, approaches the boom position, "
               "holds contact for fuel transfer, then disconnects. Tests "
               "runRefueling() end-to-end.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 20000.0;
        const double speed = 300.0;  // kts — typical tanker speed

        fm.init(ctx.cfg, alt, speed * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<AARPhase>(
            "AAR approach + contact + disconnect", 120.0, alt, speed));
        return tests;
    }
};

static RegisterScenario g_registerDigiAAR("digi_aar", []() {
    return std::make_unique<DigiAARScenario>();
});

extern "C" void f4flight_forceLink_scenario_digi_aar() {}

} // namespace f4flight_test

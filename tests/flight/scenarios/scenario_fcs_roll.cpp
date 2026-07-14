// f4flight - scenarios/scenario_fcs_roll.cpp
//
// FCS roll-axis authority test. Directly commands rstick via inputOverride()
// — BYPASSES the AI steering controller entirely.
//
// Tests the FCS roll lag filter + RollIt limiting + rollCmd table, NOT the
// digi AI. The old name "roll" was ambiguous; renamed to fcs_roll to make
// clear this is a raw FCS test.

#include "f4flight/flight/f4flight.h"
#include "scenario_framework.h"

#include <cmath>
#include <limits>
#include <string>

using namespace f4flight;

namespace f4flight_test {

class RollStepPhase : public ManeuverTest {
public:
    RollStepPhase(const char* name, double rstick, double duration,
                  double alt, double speed, bool heavy)
        : ManeuverTest(name, duration + 3.0)
        , rstick_(rstick)
        , alt_(alt)
        , speed_(speed)
        , isHeavy_(heavy)
    {}

    void Init(SteeringController& sc, FlightModel& fm) override {
        fm.init(fm.config(), alt_, speed_ * KNOTS_TO_FTPSEC, 0.0, true);
        (void)sc;
    }

    bool inputOverride(PilotInput& out, const AircraftState& state) const override {
        out.pstick = 0.0;
        out.rstick = rstick_;
        out.ypedal = 0.0;
        const double speedErr = speed_ - state.vcas;
        out.throttle = limit(0.5 + speedErr * 0.02, 0.0, 1.0);
        out.refueling = false;
        return true;
    }

    void Evaluate(const AircraftState& as, const PilotInput& input, double dt) override {
        ManeuverTest::Evaluate(as, input, dt);

        const double rollRate_degps = as.kin.p * RTD;
        const double bank = as.kin.phi * RTD;

        if (phaseTime_ >= 2.0) {
            if (rollRate_degps > maxPosRollRate_) maxPosRollRate_ = rollRate_degps;
            if (rollRate_degps < maxNegRollRate_) maxNegRollRate_ = rollRate_degps;
            maxRollRate_ = std::max(maxRollRate_, std::fabs(rollRate_degps));
            rollRateSum_ += std::fabs(rollRate_degps);
            rollRateCount_++;
        }

        maxBank_ = std::max(maxBank_, std::fabs(bank));

        if (std::isnan(rollRate_degps) || std::isnan(as.kin.vt)) {
            hasNaN_ = true;
        }

        if (phaseTime_ >= nextPrint_) {
            if (nextPrint_ == 0.0) {
                std::printf("\n%s (rstick: %.1f, %d kts)%s\n", testName_.c_str(),
                    rstick_, (int)speed_, isHeavy_ ? " [HEAVY]" : "");
                std::printf("%6s %8s %10s %8s %8s %6s %8s\n",
                    "t(s)", "bank(d)", "roll(d/s)", "G", "alpha", "throt", "vt(kts)");
            }
            std::printf("%6.1f %8.1f %10.1f %8.2f %8.1f %6.2f %8.1f\n",
                phaseTime_, bank, rollRate_degps,
                as.loads.nzcgs, as.aero.alpha_deg,
                input.throttle, as.kin.vt * FTPSEC_TO_KNOTS);
            nextPrint_ += 2.0;
        }
    }

    bool IsFinished() const override {
        return phaseTime_ >= maxTime_ || hasNaN_;
    }

    bool IsPassed() const override {
        if (hasNaN_) return false;
        // Roll-rate threshold scales with aircraft class. Fighters with
        // maxRoll ~190° and high aileron authority reach 60+ deg/s. Heavy
        // aircraft (B-52 185 ft span, 60° max bank, no ailerons — spoilers
        // only) reach ~15 deg/s at 350 kts. C-130 similar.
        //   Fighter : 60 deg/s peak, 30 deg/s directional
        //   Heavy   : 12 deg/s peak,  6 deg/s directional
        const double peakThr = isHeavy_ ? 12.0 : 60.0;
        const double dirThr  = isHeavy_ ?  6.0 : 30.0;
        if (maxRollRate_ < peakThr) return false;
        if (rstick_ > 0.5 && maxPosRollRate_ < dirThr) return false;
        if (rstick_ < -0.5 && maxNegRollRate_ > -dirThr) return false;
        // Steady-state check: avg roll rate must be at least 40% of peak.
        // The old test only checked peak — a single-frame spike would pass.
        // rollRateSum_ / rollRateCount_ is the avg of |rate| over frames 2+.
        const double avgRoll = (rollRateCount_ > 0) ? (rollRateSum_ / rollRateCount_) : 0.0;
        if (avgRoll < 0.4 * peakThr) return false;
        // Max bank: with full stick for the full duration, the aircraft
        // should reach a significant bank. The old test tracked maxBank_
        // but never asserted it. At 60+ deg/s for 8 s, the aircraft should
        // easily exceed 60° (or hit the FCS bank limiter).
        const double bankThr = isHeavy_ ? 30.0 : 60.0;
        if (maxBank_ < bankThr) return false;
        return true;
    }

    void Finish() const override {
        const double peakThr = isHeavy_ ? 12.0 : 60.0;
        const double dirThr  = isHeavy_ ?  6.0 : 30.0;
        const double bankThr = isHeavy_ ? 30.0 : 60.0;
        const double avgRoll = (rollRateCount_ > 0) ? (rollRateSum_ / rollRateCount_) : 0.0;
        std::printf("  --- Summary ---\n");
        std::printf("  Max roll rate: %.1f deg/s (need >= %.0f) %s\n",
            maxRollRate_, peakThr, (maxRollRate_ >= peakThr) ? "[PASS]" : "[FAIL]");
        std::printf("  Direction check: max +%.1f / min %.1f deg/s (rstick %+.1f, need +/-%.0f) %s\n",
            maxPosRollRate_, maxNegRollRate_, rstick_, dirThr,
            ((rstick_ > 0.5 && maxPosRollRate_ >= dirThr) ||
             (rstick_ < -0.5 && maxNegRollRate_ <= -dirThr)) ? "[PASS]" : "[FAIL]");
        std::printf("  Avg roll rate:  %.1f deg/s (need >= %.1f = 40%% peak) %s\n",
            avgRoll, 0.4 * peakThr, avgRoll >= 0.4 * peakThr ? "[PASS]" : "[FAIL]");
        std::printf("  Max bank:       %.1f deg (need >= %.0f) %s\n",
            maxBank_, bankThr, maxBank_ >= bankThr ? "[PASS]" : "[FAIL]");
        if (hasNaN_) std::printf("  NaN detected!  [FAIL]\n");
    }

private:
    double rstick_;
    double alt_;
    double speed_;
    bool   isHeavy_;
    double maxRollRate_{0.0};
    double maxPosRollRate_{std::numeric_limits<double>::lowest()};
    double maxNegRollRate_{std::numeric_limits<double>::max()};
    double maxBank_{0.0};
    double rollRateSum_{0.0};
    int rollRateCount_{0};
    double nextPrint_{0.0};
    bool hasNaN_{false};
};

class FcsRollScenario : public ManeuverScenario {
public:
    FcsRollScenario() : ManeuverScenario("fcs_roll") {}

    std::string GetDescription() const override {
        return "FCS roll authority: full stick L/R at 250/350/450 kts. "
               "Bypasses AI; tests roll FCS + rollCmd table.";
    }

    std::vector<std::unique_ptr<ManeuverTest>>
        StartScenario(FlightModel& fm, const ScenarioContext& ctx) override {

        const double alt = 15000.0;
        const bool heavy = isHeavy(ctx.cfg);
        fm.init(ctx.cfg, alt, 350.0 * KNOTS_TO_FTPSEC, 0.0, true);

        std::vector<std::unique_ptr<ManeuverTest>> tests;
        tests.push_back(std::make_unique<RollStepPhase>("Roll right 350kts", 1.0, 8.0, alt, 350.0, heavy));
        tests.push_back(std::make_unique<RollStepPhase>("Roll left 350kts", -1.0, 8.0, alt, 350.0, heavy));
        tests.push_back(std::make_unique<RollStepPhase>("Roll right 250kts", 1.0, 8.0, alt, 250.0, heavy));
        tests.push_back(std::make_unique<RollStepPhase>("Roll right 450kts", 1.0, 8.0, alt, 450.0, heavy));
        return tests;
    }
};

static RegisterScenario g_registerFcsRoll("fcs_roll", []() {
    return std::make_unique<FcsRollScenario>();
});

extern "C" void f4flight_forceLink_scenario_fcs_roll() {}

} // namespace f4flight_test

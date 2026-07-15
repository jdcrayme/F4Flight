// f4flight - digi/wingman/wingman_state.h
//
// WingmanState — persistent state for the wingman / flight-lead system.
//
// Port of FreeFalcon's DigitalBrain wingman fields (digi.h:908-1031).
// This header is the prerequisite for any Wingy/FollowOrders mode work.
// Round-3 structural addition (DIGI_AUDIT_ROUND3.md §3.2).

#pragma once

#include "f4flight/digi/comms/message.h"
#include "f4flight/digi/formation/formation_geometry.h"

namespace f4flight {
namespace digi {

enum class WingmanAction : int {
    RTB = 0, Landing = 1, FollowFormation = 2, EngageTarget = 3,
    ExecuteManeuver = 4, UseComplex = 5, COUNT = 6,
};

enum class WingmanSearch : int {
    SearchForTarget = 0, MonitorTarget = 1, FixateOnTarget = 2, COUNT = 3,
};

enum class DesignatedType : int { None = 0, Target = 1, Group = 2 };

enum class WeaponsAction : int { Hold = 0, Free = 1 };

enum class WingmanManeuver : int {
    None = 0, BreakLeft = 1, BreakRight = 2, Posthole = 3,
    Chainsaw = 4, Pince = 5, Flex = 6, SSOffset = 7,
    ClearSix = 8, CheckSix = 9,
};

struct WingmanState {
    int actionFlags[static_cast<int>(WingmanAction::COUNT)] = {0,0,0,0,0,0};
    int searchFlags[static_cast<int>(WingmanSearch::COUNT)] = {0,0,0};
    EntityId designatedTargetId{kInvalidEntityId};
    DesignatedType designatedType{DesignatedType::None};
    formation::FormationType currentFormation{formation::FormationType::Wedge};
    WingmanManeuver currentManeuver{WingmanManeuver::None};
    WeaponsAction weaponsAction{WeaponsAction::Hold};
    WeaponsAction savedWeaponsAction{WeaponsAction::Hold};
    double headingOrdered{0.0};
    double altitudeOrdered{0.0};
    double speedOrdered{0.0};
    double formRelativeAltitude{0.0};
    int    formSide{0};
    double formLateralSpaceFactor{1.0};
    bool inPosition{false};

    void reset() noexcept;
};

bool receiveOrders(WingmanState& ws, const Message& msg);

} // namespace digi
} // namespace f4flight

// f4flight - digi/wingman/wingman_state.cpp
//
// Implementation of WingmanState::reset() and receiveOrders().

#include "f4flight/digi/wingman/wingman_state.h"

#include <algorithm>

namespace f4flight {
namespace digi {

void WingmanState::reset() noexcept {
    for (int i = 0; i < static_cast<int>(WingmanAction::COUNT); ++i)
        actionFlags[i] = 0;
    for (int i = 0; i < static_cast<int>(WingmanSearch::COUNT); ++i)
        searchFlags[i] = 0;
    designatedTargetId = kInvalidEntityId;
    designatedType = DesignatedType::None;
    currentFormation = formation::FormationType::Wedge;
    currentManeuver = WingmanManeuver::None;
    weaponsAction = WeaponsAction::Hold;
    savedWeaponsAction = WeaponsAction::Hold;
    headingOrdered = 0.0;
    altitudeOrdered = 0.0;
    speedOrdered = 0.0;
    formRelativeAltitude = 0.0;
    formSide = 0;
    formLateralSpaceFactor = 1.0;
    inPosition = false;
}

// ===========================================================================
// receiveOrders — map Flight* messages to WingmanState changes.
// Port of FreeFalcon DigitalBrain::ReceiveOrders (wingman.cpp:34-570),
// simplified to the ~25 most-used commands.
//
// Returns true if the message was a wingman command (consumed);
// false otherwise (ATC, threat call, etc.).
// ===========================================================================
bool receiveOrders(WingmanState& ws, const Message& msg) {
    switch (msg.type) {
        // --- Action state changes ---
        case MessageType::FlightCmdRTB:
            ws.actionFlags[static_cast<int>(WingmanAction::RTB)] = 1;
            return true;
        case MessageType::FlightCmdEngage:
        case MessageType::FlightCmdEngageMyTarget:
            ws.actionFlags[static_cast<int>(WingmanAction::EngageTarget)] = 1;
            if (msg.payload.entityId != kInvalidEntityId) {
                ws.designatedTargetId = msg.payload.entityId;
                ws.designatedType = DesignatedType::Target;
            }
            ws.weaponsAction = WeaponsAction::Free;
            return true;
        case MessageType::FlightCmdRejoin:
            ws.actionFlags[static_cast<int>(WingmanAction::FollowFormation)] = 1;
            ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 0;
            ws.currentManeuver = WingmanManeuver::None;
            return true;
        case MessageType::FlightCmdBreak:
            ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;
            ws.currentManeuver = (msg.payload.heading < 0.0)
                ? WingmanManeuver::BreakLeft
                : WingmanManeuver::BreakRight;
            // Store the ordered heading for AiExecBreakRL. payload.heading
            // is the break direction (positive = right, negative = left).
            // The actual target heading is set by the brain when it arms
            // mnverTime (it adds the break angle to the current heading).
            ws.headingOrdered = msg.payload.heading;
            return true;
        // --- Tactical maneuvers (Round-5 additions) ---
        // Each sets currentManeuver + arms ExecuteManeuver. The brain's
        // resolveMode queues FollowOrders mode when currentManeuver != None.
        // AiPerformManeuver then dispatches to the appropriate AiExec*.
        case MessageType::FlightCmdClearSix:
            ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;
            ws.currentManeuver = WingmanManeuver::ClearSix;
            // ClearSix = 180° turn. The brain sets headingOrdered to
            // (self.yaw + PI) when it arms the maneuver.
            return true;
        case MessageType::FlightCmdPosthole:
            ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;
            ws.currentManeuver = WingmanManeuver::Posthole;
            // Posthole = descend to ordered altitude + engage. The host
            // sets altitudeOrdered + speedOrdered before sending the command
            // (or the brain defaults them).
            return true;
        case MessageType::FlightCmdChainsaw:
            ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;
            ws.currentManeuver = WingmanManeuver::Chainsaw;
            return true;
        case MessageType::FlightCmdSSOffset:
            ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;
            ws.currentManeuver = WingmanManeuver::SSOffset;
            return true;
        case MessageType::FlightCmdFlex:
            ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;
            ws.currentManeuver = WingmanManeuver::Flex;
            return true;
        case MessageType::FlightCmdPince:
            ws.actionFlags[static_cast<int>(WingmanAction::ExecuteManeuver)] = 1;
            ws.currentManeuver = WingmanManeuver::Pince;
            return true;

        // --- Formation spacing commands (Round-5 additions) ---
        // These adjust the formation geometry without entering a maneuver.
        // They're consumed immediately by AiFollowLead on the next frame.
        case MessageType::FlightCmdKickout:
            // Double the lateral spacing (e.g. 1000 ft → 2000 ft).
            ws.formLateralSpaceFactor = std::min(4.0, ws.formLateralSpaceFactor * 2.0);
            return true;
        case MessageType::FlightCmdCloseup:
            // Halve the lateral spacing (e.g. 1000 ft → 500 ft).
            ws.formLateralSpaceFactor = std::max(0.25, ws.formLateralSpaceFactor * 0.5);
            return true;
        case MessageType::FlightCmdToggleSide:
            // Mirror the formation side (+1 → -1, or -1 → +1, 0 → +1).
            ws.formSide = (ws.formSide >= 0) ? -1 : 1;
            return true;
        case MessageType::FlightCmdIncreaseRelAlt:
            // +1000 ft relative altitude (wingman climbs relative to lead).
            ws.formRelativeAltitude += 1000.0;
            return true;
        case MessageType::FlightCmdDecreaseRelAlt:
            // -1000 ft relative altitude (wingman descends relative to lead).
            ws.formRelativeAltitude -= 1000.0;
            return true;
        case MessageType::FlightCmdWeaponsHold:
            ws.weaponsAction = WeaponsAction::Hold;
            return true;
        case MessageType::FlightCmdWeaponsFree:
            ws.weaponsAction = WeaponsAction::Free;
            return true;
        case MessageType::FlightCmdJettison:
            // The host handles the actual jettison; the brain just notes it.
            return true;
        case MessageType::FlightCmdPromote:
            // The host handles lead promotion; the brain reads flightLeadId.
            return true;

        // --- Formation changes ---
        case MessageType::FlightCmdWedge:
            ws.currentFormation = formation::FormationType::Wedge;
            ws.actionFlags[static_cast<int>(WingmanAction::FollowFormation)] = 1;
            return true;
        case MessageType::FlightCmdTrail:
            ws.currentFormation = formation::FormationType::Trail;
            ws.actionFlags[static_cast<int>(WingmanAction::FollowFormation)] = 1;
            return true;
        case MessageType::FlightCmdSpread:
            ws.currentFormation = formation::FormationType::Spread;
            ws.actionFlags[static_cast<int>(WingmanAction::FollowFormation)] = 1;
            return true;
        case MessageType::FlightCmdEchelon:
            ws.currentFormation = formation::FormationType::Echelon;
            ws.actionFlags[static_cast<int>(WingmanAction::FollowFormation)] = 1;
            return true;
        case MessageType::FlightCmdFingerFour:
            ws.currentFormation = formation::FormationType::Fingertip;
            ws.actionFlags[static_cast<int>(WingmanAction::FollowFormation)] = 1;
            return true;

        // --- System commands ---
        case MessageType::FlightCmdECMOn:
        case MessageType::FlightCmdECMOff:
        case MessageType::FlightCmdRadarOn:
        case MessageType::FlightCmdRadarOff:
            // The host handles the actual system toggle; the brain doesn't
            // track these in WingmanState (they're host-side effects).
            return true;

        // --- Not wingman commands ---
        default:
            return false;
    }
}

} // namespace digi
} // namespace f4flight

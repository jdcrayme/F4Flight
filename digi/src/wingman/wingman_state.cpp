// f4flight - digi/wingman/wingman_state.cpp
//
// Implementation of WingmanState::reset() and receiveOrders().

#include "f4flight/digi/wingman/wingman_state.h"

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

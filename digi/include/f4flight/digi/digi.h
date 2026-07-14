// f4flight - digi/digi.h
//
// Digi AI umbrella header. Include this to get the entire digi AI public API.
//
// The digi AI library depends on the flight model library (one-way):
//   digi reads AircraftState, writes PilotInput → FlightModel.update()
//
// Include "f4flight/flight/f4flight.h" separately if you also need the
// flight model headers.

#pragma once

// Core AI types
#include "f4flight/digi/digi_brain.h"
#include "f4flight/digi/digi_state.h"
#include "f4flight/digi/digi_mode.h"
#include "f4flight/digi/digi_skill.h"
#include "f4flight/digi/digi_entity.h"

// Compat shim (preserves the old SteeringController API)
#include "f4flight/digi/steering.h"

// Maneuver primitives (used by all AI modes)
#include "f4flight/digi/maneuvers/maneuver_primitives.h"

// Subsystems
#include "f4flight/digi/sensors/sensor_fusion.h"
#include "f4flight/digi/sensors/sensor_picture.h"
#include "f4flight/digi/sensors/sensor.h"
#include "f4flight/digi/sensors/radar_sensor.h"
#include "f4flight/digi/sensors/rwr_sensor.h"
#include "f4flight/digi/sensors/visual_sensor.h"

#include "f4flight/digi/weapons/weapon_spec.h"
#include "f4flight/digi/weapons/weapon_types.h"
#include "f4flight/digi/weapons/sms.h"
#include "f4flight/digi/weapons/fire_control.h"

#include "f4flight/digi/comms/message.h"
#include "f4flight/digi/comms/mailbox.h"
#include "f4flight/digi/comms/message_bus.h"

#include "f4flight/digi/atc/atc_messages.h"
#include "f4flight/digi/atc/atc_controller.h"
#include "f4flight/digi/atc/taxi_graph.h"

#include "f4flight/digi/ground/ground_avoid.h"
#include "f4flight/digi/ground/ground_ops.h"
#include "f4flight/digi/ground/ag_doctrine.h"

#include "f4flight/digi/formation/formation_geometry.h"

#include "f4flight/digi/defensive/missile_defeat.h"
#include "f4flight/digi/defensive/guns_jink.h"
#include "f4flight/digi/defensive/collision_avoid.h"

#include "f4flight/digi/offensive/roll_and_pull.h"
#include "f4flight/digi/offensive/guns_engage.h"
#include "f4flight/digi/offensive/missile_engage.h"
#include "f4flight/digi/offensive/bvr_engage.h"
#include "f4flight/digi/offensive/merge.h"

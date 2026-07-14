// f4flight - Modern C++17 flight model library extracted from Falcon 4 / FreeFalcon
// f4flight.h
//
// Flight Model umbrella header. Include this to get the entire FM public API.
//
// Note: this header includes ONLY flight-model headers. The digi AI subsystem
// (including the SteeringController compat shim) is in the separate `digi`
// library — include "f4flight/digi/digi_brain.h" or
// "f4flight/digi/steering.h" directly if you need AI.

#pragma once

#include "f4flight/flight/core/constants.h"
#include "f4flight/flight/core/types.h"
#include "f4flight/flight/core/math.h"
#include "f4flight/flight/core/lookup.h"
#include "f4flight/flight/core/units.h"
#include "f4flight/flight/core/trig.h"

#include "f4flight/flight/atmosphere.h"
#include "f4flight/flight/aircraft_config.h"
#include "f4flight/flight/aircraft_state.h"
#include "f4flight/flight/aerodynamics.h"
#include "f4flight/flight/engine.h"
#include "f4flight/flight/gear.h"
#include "f4flight/flight/fcs.h"
#include "f4flight/flight/eom.h"
#include "f4flight/flight/flight_model.h"
#include "f4flight/flight/dat_loader.h"
#include "f4flight/flight/json_io.h"
#include "f4flight/flight/config/f16c_config.h"

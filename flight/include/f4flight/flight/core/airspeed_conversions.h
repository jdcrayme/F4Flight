// f4flight - flight/core/airspeed_conversions.h
//
// CAS ↔ TAS conversion function definitions.
//
// These functions are declared in units.h but defined here because they need
// the full AircraftState definition (to read vcas and kin.vt for the
// CAS/TAS ratio). units.h only forward-declares AircraftState to avoid a
// circular include.
//
// Include this header in any .cpp that calls tasFromCas / casFromTas /
// casFromTasFps.

#pragma once

#include "f4flight/flight/core/units.h"
#include "f4flight/flight/aircraft_state.h"

namespace f4flight {

// CAS → TAS. Uses the aircraft's current CAS/TAS ratio (from vcas and vt).
// This is accurate for small deviations from the current flight condition.
// At sea level the ratio is ~1.0; at 40k ft it can be ~2.0.
inline TasKnots tasFromCas(CasKnots cas, const AircraftState& as) {
    const double wingTasKts = as.kin.vt * FTPSEC_TO_KNOTS;
    if (wingTasKts > 1.0) {
        const double ratio = wingTasKts / as.vcas;  // TAS/CAS > 1
        return TasKnots(cas.count() * ratio);
    }
    return TasKnots(cas.count());  // fallback: assume CAS ≈ TAS
}

// TAS → CAS. Inverse of tasFromCas.
inline CasKnots casFromTas(TasKnots tas, const AircraftState& as) {
    const double wingTasKts = as.kin.vt * FTPSEC_TO_KNOTS;
    if (wingTasKts > 1.0) {
        const double ratio = as.vcas / wingTasKts;  // CAS/TAS < 1
        return CasKnots(tas.count() * ratio);
    }
    return CasKnots(tas.count());
}

// Convenience: TAS ft/s → CAS kts (common when reading kin.vt).
// Converts ft/s → kts, then TAS → CAS.
inline CasKnots casFromTasFps(TasFtPerSec tas, const AircraftState& as) {
    return casFromTas(toTasKnots(tas), as);
}

} // namespace f4flight

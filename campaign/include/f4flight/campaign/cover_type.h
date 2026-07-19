// f4flight - campaign/include/f4flight/campaign/cover_type.h
//
// CoverType represents the primary surface tile type on the campaign map,
// defining its visual rendering, movement speed penalty, and defensive modifiers.

#pragma once

#include <string>

namespace f4flight {
namespace campaign {

enum class CoverType {
    Plains,
    Forest,
    Mountains,
    Water,
    Swamp,
    City,
    Coast,
    Desert,
    Hills
};

inline std::string coverTypeToString(CoverType cover) {
    switch (cover) {
        case CoverType::Plains:     return "Plains";
        case CoverType::Forest:     return "Forest";
        case CoverType::Mountains:  return "Mountains";
        case CoverType::Water:      return "Water";
        case CoverType::Swamp:      return "Swamp";
        case CoverType::City:       return "City";
        case CoverType::Coast:      return "Coast";
        case CoverType::Desert:     return "Desert";
        case CoverType::Hills:      return "Hills";
    }
    return "Unknown";
}

inline double coverTypeSpeedMultiplier(CoverType cover) {
    switch (cover) {
        case CoverType::Plains:     return 1.0;
        case CoverType::Forest:     return 0.6;
        case CoverType::Mountains:  return 0.3;
        case CoverType::Water:      return 0.0; // impassable for ground
        case CoverType::Swamp:      return 0.4;
        case CoverType::City:       return 0.7;
        case CoverType::Coast:      return 0.8;
        case CoverType::Desert:     return 0.9;
        case CoverType::Hills:      return 0.5;
    }
    return 1.0;
}

inline double coverTypeDefenseBonus(CoverType cover) {
    switch (cover) {
        case CoverType::Plains:     return 1.0;  // baseline
        case CoverType::Forest:     return 1.4;  // cover
        case CoverType::Mountains:  return 1.8;  // height + cover
        case CoverType::Water:      return 1.0;
        case CoverType::Swamp:      return 1.2;
        case CoverType::City:       return 2.0;  // extreme urban defense
        case CoverType::Coast:      return 1.1;
        case CoverType::Desert:     return 0.8;  // exposed
        case CoverType::Hills:      return 1.3;
    }
    return 1.0;
}

} // namespace campaign
} // namespace f4flight

// f4flight - campaign/include/f4flight/campaign/terrain_map.h
//
// TerrainMap represents the 2D grid of Wang tiles, altitudes, and terrain features.

#pragma once

#include "cover_type.h"
#include <vector>

namespace f4flight {
namespace campaign {

struct TerrainTile {
    CoverType cover{CoverType::Plains};
    double elevation{0.0}; // ft, above sea level (Z)
    bool hasRoad{false};
    bool hasRail{false};
    int wangId{0}; // Wang tile representation ID for blended textures
};

class TerrainMap {
public:
    TerrainMap() = default;
    TerrainMap(int cols, int rows, double tileSizeFt = 6076.12) // 1 NM by default
        : cols_(cols), rows_(rows), tileSizeFt_(tileSizeFt) {
        tiles_.resize(cols * rows);
    }

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    double tileSizeFt() const { return tileSizeFt_; }

    const TerrainTile& tile(int col, int row) const {
        return tiles_[row * cols_ + col];
    }

    TerrainTile& tile(int col, int row) {
        return tiles_[row * cols_ + col];
    }

    bool isValid(int col, int row) const {
        return col >= 0 && col < cols_ && row >= 0 && row < rows_;
    }

    // Convert from Grid coordinates (col, row) to world coordinates (x, y) in feet (ENU: East-North-Up)
    // Centered around the tile.
    void gridToWorld(int col, int row, double& x, double& y) const {
        x = (col + 0.5) * tileSizeFt_;
        y = (row + 0.5) * tileSizeFt_;
    }

    // Convert from world coordinates (x, y) in feet to grid (col, row)
    void worldToGrid(double x, double y, int& col, int& row) const {
        col = static_cast<int>(x / tileSizeFt_);
        row = static_cast<int>(y / tileSizeFt_);
        if (col < 0) col = 0;
        if (col >= cols_) col = cols_ - 1;
        if (row < 0) row = 0;
        if (row >= rows_) row = rows_ - 1;
    }

    double getElevationAt(double x, double y) const {
        int col, row;
        worldToGrid(x, y, col, row);
        return tile(col, row).elevation;
    }

private:
    int cols_{0};
    int rows_{0};
    double tileSizeFt_{6076.12};
    std::vector<TerrainTile> tiles_;
};

} // namespace campaign
} // namespace f4flight

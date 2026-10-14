#pragma once

#include <cmath>

#include "types.hpp"

namespace sim2d {

// True if a circle of radius r centred at (x, y) overlaps any wall cell
// or leaves the map.
inline bool circleHitsWall(const MapGrid &map, float x, float y, float r)
{
    int32_t x0 = (int32_t)std::floor(x - r - map.originX);
    int32_t x1 = (int32_t)std::floor(x + r - map.originX);
    int32_t y0 = (int32_t)std::floor(y - r - map.originY);
    int32_t y1 = (int32_t)std::floor(y + r - map.originY);

    if (x0 < 0 || y0 < 0 || x1 >= map.width || y1 >= map.height) {
        return true;
    }

    for (int32_t cy = y0; cy <= y1; cy++) {
        for (int32_t cx = x0; cx <= x1; cx++) {
            if (!map.cells[cy * map.width + cx]) continue;

            float minx = map.originX + cx, maxx = minx + 1.f;
            float miny = map.originY + cy, maxy = miny + 1.f;
            float qx = std::fmin(std::fmax(x, minx), maxx);
            float qy = std::fmin(std::fmax(y, miny), maxy);
            float dx = x - qx, dy = y - qy;
            if (dx * dx + dy * dy < r * r) return true;
        }
    }
    return false;
}

}

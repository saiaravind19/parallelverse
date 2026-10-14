#include "world_spec.hpp"

#include <algorithm>
#include <cmath>

#include "geometry.hpp"

namespace sim2d {

namespace {

struct Cell {
    int32_t x, y;
    float theta;
};

float dirTheta(uint8_t d)
{
    const float pi = 3.14159265358979f;
    if (d == 1) return pi / 2.f;   // '^'
    if (d == 2) return -pi / 2.f;  // 'v'
    if (d == 4) return 0.f;        // '>'
    return pi;                     // '<'
}

}

int countMovableCells(const WorldSpec &spec)
{
    const MapGrid &map = spec.map;
    bool has_rules = false;
    for (uint8_t d : spec.dirs) {
        if (d) { has_rules = true; break; }
    }
    int n = 0;
    for (int32_t y = 0; y < map.height; y++) {
        for (int32_t x = 0; x < map.width; x++) {
            if (map.cells[y * map.width + x]) continue;
            uint8_t d = spec.dirs.empty() ? 0 : spec.dirs[y * map.width + x];
            if (!has_rules || d) n++;
        }
    }
    return n;
}

std::vector<Sim::RobotSpawn> placeRobots(const WorldSpec &spec,
                                         const std::vector<Occupant> &occupied,
                                         int count,
                                         const RobotDefaults &def)
{
    const MapGrid &map = spec.map;
    auto free_cell = [&](int32_t x, int32_t y) {
        return x >= 0 && y >= 0 && x < map.width && y < map.height &&
               !map.cells[y * map.width + x];
    };

    bool has_rules = false;
    for (uint8_t d : spec.dirs) {
        if (d) { has_rules = true; break; }
    }

    // Eligible cells, top row first like the layout file.
    std::vector<Cell> clear_cells, other_cells;  // other_cells: multi-direction cells with rules
    for (int32_t y = map.height - 1; y >= 0; y--) {
        for (int32_t x = 0; x < map.width; x++) {
            if (!free_cell(x, y)) continue;
            uint8_t d = spec.dirs.empty() ? 0 : spec.dirs[y * map.width + x];
            float cx = x + 0.5f, cy = y + 0.5f;

            if (circleHitsWall(map, cx, cy, def.radius)) continue;
            bool blocked = false;
            for (const Occupant &o : occupied) {
                float dx = cx - o.x, dy = cy - o.y;
                float rr = def.radius + o.r + 0.1f;
                if (dx * dx + dy * dy < rr * rr) { blocked = true; break; }
            }
            if (blocked) continue;

            if (has_rules) {
                if (!d) continue;  // no arrow: not a movable cell
                if (d == 1 || d == 2 || d == 4 || d == 8) {
                    clear_cells.push_back({x, y, dirTheta(d)});  // lane cell
                } else {
                    uint8_t first = d & (uint8_t)(-d);           // first arrow in the cell
                    other_cells.push_back({x, y, dirTheta(first)});
                }
            } else {
                bool clear = true;
                for (int dy = -1; dy <= 1 && clear; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                        if (!free_cell(x + dx, y + dy)) { clear = false; break; }
                (clear ? clear_cells : other_cells).push_back({x, y, 0.f});
            }
        }
    }
    std::vector<Cell> cells = clear_cells;
    cells.insert(cells.end(), other_cells.begin(), other_cells.end());

    // Keep chosen cells apart when possible, then take evenly spaced ones.
    std::vector<Cell> picks;
    for (int gap : {3, 2, 1}) {
        std::vector<Cell> spaced;
        for (const Cell &c : cells) {
            bool ok = true;
            for (const Cell &q : spaced) {
                if (std::max(std::abs(c.x - q.x), std::abs(c.y - q.y)) < gap) {
                    ok = false;
                    break;
                }
            }
            if (ok) spaced.push_back(c);
        }
        if ((int)spaced.size() >= count) {
            for (int i = 0; i < count; i++) {
                picks.push_back(spaced[(size_t)i * spaced.size() / (size_t)count]);
            }
            break;
        }
        if (gap == 1) picks = spaced;  // not enough room: take what fits
    }

    std::vector<Sim::RobotSpawn> out;
    for (const Cell &c : picks) {
        Sim::RobotSpawn s {};
        s.x = c.x + 0.5f;
        s.y = c.y + 0.5f;
        s.theta = c.theta;
        s.radius = def.radius;
        s.vMax = def.vMax;
        s.aMax = def.aMax;
        s.omegaMax = def.omegaMax;
        s.active = 1;
        out.push_back(s);
    }
    return out;
}

}

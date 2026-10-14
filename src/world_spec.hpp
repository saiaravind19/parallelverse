#pragma once

#include <vector>

#include "sim.hpp"

namespace sim2d {

// Host-side description of one world: its map, direction rules, and the robots
// to start with. The simulator copies what it needs from this.
struct WorldSpec {
    MapGrid map;
    // Direction rule per cell (row-major, width * height): bit 0 '^' (+y),
    // bit 1 'v' (-y), bit 2 '>' (+x), bit 3 '<' (-x). 0 = no rule.
    std::vector<uint8_t> dirs;
    std::vector<Sim::RobotSpawn> spawns;
};

// Size and limits used for new robots.
struct RobotDefaults {
    float radius = 0.3f;
    float vMax = 2.f;
    float aMax = 1.f;
    float omegaMax = 1.f;
};

// A circle that is already in the world.
struct Occupant {
    float x, y, r;
};

// Number of cells a robot can occupy: with direction rules, the cells that have
// at least one arrow; otherwise every free cell. No world can hold more robots
// than this.
int countMovableCells(const WorldSpec &spec);

// Choose up to `count` start poses for new robots, spread over the layout.
// Layouts with direction rules: lane cells (exactly one allowed direction)
// first, facing it, then cells with several directions once those run out.
// Other layouts: on free cells, preferring cells whose
// neighbours are all free. Cells that overlap `occupied` are skipped.
// Returns fewer than `count` if the layout does not have room.
std::vector<Sim::RobotSpawn> placeRobots(const WorldSpec &spec,
                                         const std::vector<Occupant> &occupied,
                                         int count,
                                         const RobotDefaults &def);

}

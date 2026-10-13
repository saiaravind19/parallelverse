#pragma once
#include <cmath>
#include <vector>
#include "map.hpp"
#include "profile.hpp"

namespace bs2d {

struct Vec2 { float x, y; };

struct Robot {
    Vec2 pos{0, 0};
    float theta = 0.f;        // heading (rad)
    float v = 0.f;            // linear speed (m/s), in [0, vMax]
    float radius = 0.3f;      // circular footprint (m)
    float vMax = 2.f;
    float aMax = 1.f;         // m/s^2
    float omegaMax = 1.f;     // rad/s
    std::vector<Vec2> waypoints;
    size_t next = 0;          // index of current target waypoint
    bool stopped = false;     // set on collision
};

struct World {
    Map map;
    std::vector<Robot> robots;
    float dt = 0.04f;

    // Advance one step: profile, heading, integrate, then collision check.
    void step();

private:
    bool collidesWall(Vec2 p, float r) const;
};

} // namespace bs2d

#include "world.hpp"
#include <algorithm>

namespace bs2d {

static float wrapAngle(float a)
{
    while (a > (float)M_PI) a -= 2.f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.f * (float)M_PI;
    return a;
}

// True if a circle of radius r at p overlaps any wall cell.
bool World::collidesWall(Vec2 p, float r) const
{
    int32_t x0 = (int32_t)std::floor(p.x - r - map.originX);
    int32_t x1 = (int32_t)std::floor(p.x + r - map.originX);
    int32_t y0 = (int32_t)std::floor(p.y - r - map.originY);
    int32_t y1 = (int32_t)std::floor(p.y + r - map.originY);
    for (int32_t cy = y0; cy <= y1; cy++) {
        for (int32_t cx = x0; cx <= x1; cx++) {
            if (!map.isOccupied(cx, cy)) continue;
            // Closest point on the cell rectangle to the circle centre.
            float minx = map.originX + cx, maxx = minx + 1.f;
            float miny = map.originY + cy, maxy = miny + 1.f;
            float qx = std::clamp(p.x, minx, maxx);
            float qy = std::clamp(p.y, miny, maxy);
            float dx = p.x - qx, dy = p.y - qy;
            if (dx * dx + dy * dy < r * r) return true;
        }
    }
    return false;
}

void World::step()
{
    for (auto &r : robots) {
        if (r.stopped || r.next >= r.waypoints.size()) {
            r.v = 0.f;
            continue;
        }
        Vec2 goal = r.waypoints[r.next];
        float dx = goal.x - r.pos.x, dy = goal.y - r.pos.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // Reached this waypoint: advance to the next one.
        if (dist < 0.05f) {
            r.next++;
            r.v = 0.f;
            continue;
        }

        // Turn toward the goal at up to omegaMax, then drive.
        float desired = std::atan2(dy, dx);
        float err = wrapAngle(desired - r.theta);
        float maxTurn = r.omegaMax * dt;
        float turn = std::clamp(err, -maxTurn, maxTurn);
        r.theta = wrapAngle(r.theta + turn);

        // Speed from the trapezoidal profile; only drive when roughly aligned.
        float speed = trapezoidSpeed(dist, r.v, r.vMax, r.aMax, dt);
        if (std::fabs(err) > 0.5f) speed = 0.f;
        r.v = speed;

        Vec2 next{ r.pos.x + std::cos(r.theta) * speed * dt,
                   r.pos.y + std::sin(r.theta) * speed * dt };
        if (collidesWall(next, r.radius)) {
            r.stopped = true;
            r.v = 0.f;
            continue;
        }
        r.pos = next;
    }

    // Robot-robot: stop on overlap (both robots stop).
    for (size_t i = 0; i < robots.size(); i++) {
        for (size_t j = i + 1; j < robots.size(); j++) {
            auto &a = robots[i];
            auto &b = robots[j];
            float dx = a.pos.x - b.pos.x, dy = a.pos.y - b.pos.y;
            float rr = a.radius + b.radius;
            if (dx * dx + dy * dy < rr * rr) {
                a.stopped = true; b.stopped = true;
                a.v = 0.f; b.v = 0.f;
            }
        }
    }
}

} // namespace bs2d

#pragma once
#include <algorithm>
#include <cmath>

namespace bs2d {

// Trapezoidal velocity profile along a straight segment.
// Starts at 0, accelerates at aMax to vMax, cruises, then decelerates
// so the robot arrives at the goal with speed 0. If the segment is too
// short to reach vMax the profile is triangular.
//
// s: remaining distance to goal (m), v: current speed (m/s).
// Returns the commanded speed for the next step.
inline float trapezoidSpeed(float s, float v, float vMax, float aMax, float dt)
{
    if (s <= 1e-4f) {
        return 0.f;
    }
    // Speed from which we can still stop exactly at the goal.
    float v_stop = std::sqrt(2.f * aMax * s);
    float target = std::min(vMax, v_stop);
    // Accelerate or decelerate toward target, limited by aMax per step.
    float dv = std::clamp(target - v, -aMax * dt, aMax * dt);
    return std::clamp(v + dv, 0.f, vMax);
}

} // namespace bs2d

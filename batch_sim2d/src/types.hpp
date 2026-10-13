#pragma once

#include <madrona/components.hpp>
#include <madrona/types.hpp>

#include <cstdint>

namespace sim2d {

using madrona::Entity;
using madrona::CountT;

// Limits fixed at compile time so that every world has the same table shape
// (required by Madrona's exported buffers).
namespace limits {
#ifndef SIM2D_MAX_ROBOTS
#error "Build settings are missing: configure with CMake (see batch_sim2d/CMakeLists.txt)"
#endif
inline constexpr CountT maxRobotsPerWorld = SIM2D_MAX_ROBOTS;
inline constexpr CountT maxWaypoints = SIM2D_MAX_WAYPOINTS;
inline constexpr int32_t maxMapDim = SIM2D_MAX_MAP_DIM; // cells per side
}

struct Vec2 {
    float x;
    float y;
};

// ---- Per-world singleton: occupancy grid, 1 m per cell ----
struct MapGrid {
    int32_t width;
    int32_t height;
    float originX;
    float originY;
    uint8_t cells[limits::maxMapDim * limits::maxMapDim]; // 1 = wall
};

// ---- Per-robot components ----

// Robot pose in world frame (metres, radians).
struct RobotPose {
    float x;
    float y;
    float theta;
};

// Physical / control parameters, set from the config.
struct RobotParams {
    float radius;    // circular footprint (m)
    float vMax;      // m/s, set per robot in the scene (must be > 0)
    float aMax;      // m/s^2, trapezoid acceleration limit
    float omegaMax;  // rad/s, turn-rate limit
};

// Motion bookkeeping owned by the sim.
struct RobotMotion {
    float v;         // current linear speed (m/s)
    float prevX;     // pose before this step (for collision revert)
    float prevY;
    int32_t active;  // 0 = unused slot in this world
    int32_t stopped; // 1 = halted (collision or wall); cleared by new command
    int32_t collided;// set by collision detection this step
    int32_t hitRobot;// 1 = stopped by contact with another robot; cleared by new command
    int32_t cmdSeq;  // last command sequence applied
};

// Path currently being followed (copied from WaypointCmd).
struct Path {
    Vec2 pts[limits::maxWaypoints];
    int32_t count;
    int32_t next;    // index of current target
};

// Exported: host writes a new waypoint list here. The sim applies it when
// seq changes. Layout: [numWorlds * maxRobotsPerWorld] entries.
struct WaypointCmd {
    int32_t seq;
    int32_t count;
    Vec2 pts[limits::maxWaypoints];
};

// Exported: state the host reads each step for publishing and rendering.
struct RobotState {
    float x;
    float y;
    float theta;
    float v;
    int32_t next;
    int32_t count;
    int32_t stopped;
    int32_t collided;
    int32_t hitRobot;
    int32_t active;
};

// Entities of the other robots in the same world (for collision checks).
struct OtherRobots {
    Entity e[limits::maxRobotsPerWorld - 1];
};

// Exported ids for the Madrona export slots.
enum class ExportID : uint32_t {
    WaypointCmd,
    RobotState,
    NumExports,
};

struct Robot : public madrona::Archetype<
    RobotPose,
    RobotParams,
    RobotMotion,
    Path,
    WaypointCmd,
    RobotState,
    OtherRobots
> {};

}

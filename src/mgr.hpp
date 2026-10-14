#pragma once

#include <madrona/heap_array.hpp>
#include <madrona/mw_cpu.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "sim.hpp"
#include "types.hpp"
#include "world_spec.hpp"

namespace sim2d {

// Host-side interface to the batch simulation. Owns the Madrona CPU task
// graph executor and exposes exported buffers for waypoint commands and
// robot state.
class Manager {
public:
    struct Config {
        float dt;
        // Robot slots per world. 0 = choose from the starting robots
        // (twice as many, with room for at least 64 more).
        int32_t maxRobots = 0;
    };

    Manager(const Config &cfg, const std::vector<WorldSpec> &worlds);
    ~Manager();

    uint32_t numWorlds() const { return numWorlds_; }

    // Robot slots per world (set at launch) and robots in a world now.
    int32_t maxRobots() const { return maxRobots_; }
    int32_t robotCount(uint32_t world) const { return count_[world]; }
    // Most robots a world can hold: its movable cells (see countMovableCells).
    int32_t movableCells(uint32_t world) const { return movable_[world]; }

    // Capacity to use when none is given: room to grow beyond `initial`.
    static int32_t defaultCapacity(int32_t initial);

    // Advance all worlds by one step.
    void step();

    // Send a new waypoint list to one robot. Takes effect on the next step.
    // Returns false (with a reason in err) if the robot does not exist, the
    // path is too long, or the robot is in velocity mode.
    bool setWaypoints(uint32_t world, uint32_t robot,
                      const std::vector<Vec2> &pts, std::string &err);
    bool setWaypoints(uint32_t world, uint32_t robot,
                      const std::vector<Vec2> &pts);

    // Drive a robot in velocity mode: linear m/s, angular rad/s. Takes effect
    // on the next step and stays in effect until the next command.
    bool setVelocity(uint32_t world, uint32_t robot, float linear,
                     float angular, std::string &err);

    // How a robot takes commands: kModeWaypoint or kModeVelocity (-1 if the
    // robot does not exist). Switching drops the waypoint path and stops it.
    int32_t controlMode(uint32_t world, uint32_t robot) const;
    bool setControlMode(uint32_t world, uint32_t robot, int32_t mode,
                        std::string &err);

    // Add a robot at run time. Returns its id, or -1 with a reason in err
    // (world full, inside a wall, or overlapping another robot).
    // The robot appears on the next step.
    int32_t addRobot(uint32_t world, const Sim::RobotSpawn &spawn,
                     std::string &err);

    // Pick start poses for up to `count` new robots (see placeRobots).
    std::vector<Sim::RobotSpawn> autoPlace(uint32_t world, int count,
                                           const RobotDefaults &def) const;

    // Read robot state after step(). Index: world * maxRobots() + robot.
    const RobotState *robotStates() const;
    RobotState robotState(uint32_t world, uint32_t robot) const;

    // Radius of a robot (known on the host from its spawn).
    float robotRadius(uint32_t world, uint32_t robot) const;

private:
    using TaskGraphT = madrona::TaskGraphExecutor<Engine, Sim,
        Sim::Config, Sim::WorldInit>;

    uint32_t numWorlds_;
    int32_t maxRobots_;
    uint64_t steps_ = 0;
    std::vector<WorldSpec> specs_;       // kept for placement and checks
    std::vector<int32_t> count_;         // robots per world
    std::vector<int32_t> movable_;       // movable cells per world
    std::vector<int32_t> cmdSeq_;        // per slot: waypoint command number
    std::vector<int32_t> spawnSeq_;      // per slot: spawn command number
    std::vector<int32_t> ctrlSeq_;       // per slot: control command number
    std::vector<int32_t> mode_;          // per slot: ControlModeId
    std::vector<Occupant> slotPose_;     // per slot: pose at spawn
    std::vector<uint64_t> slotStep_;     // per slot: step count at spawn
    std::unique_ptr<TaskGraphT> exec_;
    WaypointCmd *cmdBuf_;
    SpawnCmd *spawnBuf_;
    ControlCmd *ctrlBuf_;
    const RobotState *stateBuf_;

    // Where robot `r` of `world` is now (spawn pose until it has stepped).
    Occupant currentPose(uint32_t world, uint32_t r) const;
    void occupants(uint32_t world, std::vector<Occupant> &out) const;
};

}

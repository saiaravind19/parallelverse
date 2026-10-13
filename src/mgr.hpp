#pragma once

#include <madrona/heap_array.hpp>
#include <madrona/mw_cpu.hpp>

#include <cstdint>
#include <memory>
#include <vector>

#include "sim.hpp"
#include "types.hpp"

namespace sim2d {

// Host-side interface to the batch simulation. Owns the Madrona CPU task
// graph executor and exposes exported buffers for waypoint commands and
// robot state.
class Manager {
public:
    struct Config {
        float dt;
    };

    Manager(const Config &cfg,
            const std::vector<Sim::WorldInit> &worlds);
    ~Manager();

    uint32_t numWorlds() const { return numWorlds_; }

    // Advance all worlds by one step.
    void step();

    // Send a new waypoint list to one robot. Takes effect on the next step.
    void setWaypoints(uint32_t world, uint32_t robot,
                      const std::vector<Vec2> &pts);

    // Read robot state after step(). Index: world * maxRobotsPerWorld + robot.
    const RobotState *robotStates() const;
    RobotState robotState(uint32_t world, uint32_t robot) const;

private:
    using TaskGraphT = madrona::TaskGraphExecutor<Engine, Sim,
        Sim::Config, Sim::WorldInit>;

    uint32_t numWorlds_;
    std::vector<int32_t> cmdSeq_; // per robot slot, host side
    std::unique_ptr<TaskGraphT> exec_;
    WaypointCmd *cmdBuf_;
    const RobotState *stateBuf_;
};

}

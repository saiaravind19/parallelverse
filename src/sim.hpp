#pragma once

#include <madrona/taskgraph_builder.hpp>
#include <madrona/custom_context.hpp>

#include "types.hpp"

namespace sim2d {

class Engine;

enum class TaskGraphID : uint32_t {
    Step,
    NumTaskGraphs,
};

// Per-world sim state. Accessible from systems via ctx.data().
struct Sim : public madrona::WorldBase {
    struct Config {
        float dt;           // seconds per step
        int32_t maxRobots;  // robot slots per world (chosen at launch)
    };

    struct RobotSpawn {
        float x, y, theta;
        float radius, vMax, aMax, omegaMax;
        int32_t active;
    };

    // Per-world initial data, built on the host. The spawns array is owned
    // by the host and must stay valid while the manager is being created.
    struct WorldInit {
        MapGrid map;
        const RobotSpawn *spawns;
        int32_t numSpawns;
    };

    static void registerTypes(madrona::ECSRegistry &registry,
                              const Config &cfg);
    static void setupTasks(madrona::TaskGraphManager &mgr,
                           const Config &cfg);

    Sim(Engine &ctx, const Config &cfg, const WorldInit &init);

    float dt;
    madrona::Query<RobotPose, RobotParams, RobotMotion> robotQuery;
};

class Engine : public ::madrona::CustomContext<Engine, Sim> {
public:
    using CustomContext::CustomContext;
};

}

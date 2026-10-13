#include "mgr.hpp"

#include <cstring>

using namespace madrona;

namespace sim2d {

Manager::Manager(const Config &cfg, const std::vector<Sim::WorldInit> &worlds)
    : numWorlds_((uint32_t)worlds.size()),
      cmdSeq_(worlds.size() * limits::maxRobotsPerWorld, 0)
{
    Sim::Config sim_cfg { cfg.dt };

    HeapArray<Sim::WorldInit> inits(numWorlds_);
    for (uint32_t i = 0; i < numWorlds_; i++) {
        inits[i] = worlds[i];
    }

    exec_.reset(new TaskGraphT(
        ThreadPoolExecutor::Config {
            .numWorlds = numWorlds_,
            .numExportedBuffers = (uint32_t)ExportID::NumExports,
        },
        sim_cfg,
        inits.data(),
        (uint32_t)TaskGraphID::NumTaskGraphs));

    cmdBuf_ = (WaypointCmd *)exec_->getExported(
        (uint32_t)ExportID::WaypointCmd);
    stateBuf_ = (const RobotState *)exec_->getExported(
        (uint32_t)ExportID::RobotState);
}

Manager::~Manager() = default;

void Manager::step()
{
    exec_->run();
}

void Manager::setWaypoints(uint32_t world, uint32_t robot,
                           const std::vector<Vec2> &pts)
{
    if (world >= numWorlds_ || robot >= limits::maxRobotsPerWorld) return;
    if (pts.size() > (size_t)limits::maxWaypoints) return;

    uint32_t slot = world * limits::maxRobotsPerWorld + robot;
    int32_t seq = ++cmdSeq_[slot];

    WaypointCmd &cmd = cmdBuf_[slot];
    cmd.count = (int32_t)pts.size();
    for (size_t i = 0; i < pts.size(); i++) {
        cmd.pts[i] = pts[i];
    }
    cmd.seq = seq;
}

const RobotState *Manager::robotStates() const
{
    return stateBuf_;
}

RobotState Manager::robotState(uint32_t world, uint32_t robot) const
{
    return stateBuf_[world * limits::maxRobotsPerWorld + robot];
}

}

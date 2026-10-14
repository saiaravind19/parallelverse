#include "mgr.hpp"

#include <algorithm>
#include <cstring>

#include "geometry.hpp"

using namespace madrona;

namespace sim2d {

int32_t Manager::defaultCapacity(int32_t initial)
{
    return std::max(initial * 2, initial + 64);
}

Manager::Manager(const Config &cfg, const std::vector<WorldSpec> &worlds)
    : numWorlds_((uint32_t)worlds.size()),
      maxRobots_(cfg.maxRobots),
      specs_(worlds),
      count_(worlds.size(), 0)
{
    int32_t initial = 0, most_cells = 0;
    movable_.assign(worlds.size(), 0);
    for (size_t i = 0; i < specs_.size(); i++) {
        initial = std::max(initial, (int32_t)specs_[i].spawns.size());
        movable_[i] = countMovableCells(specs_[i]);
        most_cells = std::max(most_cells, movable_[i]);
    }
    // Default: room for every movable cell, so adding never hits a limit.
    if (maxRobots_ <= 0) maxRobots_ = most_cells;
    // No world has room for more robots than it has movable cells.
    if (maxRobots_ > most_cells) maxRobots_ = most_cells;
    if (maxRobots_ < initial) maxRobots_ = initial;

    size_t slots = (size_t)numWorlds_ * maxRobots_;
    cmdSeq_.assign(slots, 0);
    spawnSeq_.assign(slots, 0);
    ctrlSeq_.assign(slots, 0);
    mode_.assign(slots, kModeWaypoint);
    slotPose_.assign(slots, Occupant { 0.f, 0.f, 0.f });
    slotStep_.assign(slots, 0);

    HeapArray<Sim::WorldInit> inits(numWorlds_);
    for (uint32_t i = 0; i < numWorlds_; i++) {
        const WorldSpec &w = specs_[i];
        inits[i].map = w.map;
        inits[i].spawns = w.spawns.data();
        inits[i].numSpawns = (int32_t)w.spawns.size();

        count_[i] = (int32_t)w.spawns.size();
        for (size_t r = 0; r < w.spawns.size(); r++) {
            const Sim::RobotSpawn &s = w.spawns[r];
            slotPose_[(size_t)i * maxRobots_ + r] = Occupant { s.x, s.y, s.radius };
        }
    }

    Sim::Config sim_cfg { cfg.dt, maxRobots_ };
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
    spawnBuf_ = (SpawnCmd *)exec_->getExported(
        (uint32_t)ExportID::SpawnCmd);
    ctrlBuf_ = (ControlCmd *)exec_->getExported(
        (uint32_t)ExportID::ControlCmd);
    stateBuf_ = (const RobotState *)exec_->getExported(
        (uint32_t)ExportID::RobotState);
}

Manager::~Manager() = default;

void Manager::step()
{
    exec_->run();
    steps_++;
}

bool Manager::setWaypoints(uint32_t world, uint32_t robot,
                           const std::vector<Vec2> &pts, std::string &err)
{
    if (world >= numWorlds_ || robot >= (uint32_t)count_[world]) {
        err = "no such robot";
        return false;
    }
    if (pts.size() > (size_t)limits::maxWaypoints) {
        err = "path has " + std::to_string(pts.size()) + " waypoints; limit is " +
              std::to_string((int)limits::maxWaypoints);
        return false;
    }

    size_t slot = (size_t)world * maxRobots_ + robot;
    if (mode_[slot] != kModeWaypoint) {
        err = "robot is in velocity mode; switch it to waypoint mode first";
        return false;
    }
    int32_t seq = ++cmdSeq_[slot];

    WaypointCmd &cmd = cmdBuf_[slot];
    cmd.count = (int32_t)pts.size();
    for (size_t i = 0; i < pts.size(); i++) {
        cmd.pts[i] = pts[i];
    }
    cmd.seq = seq;
    return true;
}

bool Manager::setWaypoints(uint32_t world, uint32_t robot,
                           const std::vector<Vec2> &pts)
{
    std::string err;
    return setWaypoints(world, robot, pts, err);
}

bool Manager::setVelocity(uint32_t world, uint32_t robot, float linear,
                          float angular, std::string &err)
{
    if (world >= numWorlds_ || robot >= (uint32_t)count_[world]) {
        err = "no such robot";
        return false;
    }
    size_t slot = (size_t)world * maxRobots_ + robot;
    if (mode_[slot] != kModeVelocity) {
        err = "robot is in waypoint mode; switch it to velocity mode first";
        return false;
    }
    ControlCmd &cmd = ctrlBuf_[slot];
    cmd.mode = kModeVelocity;
    cmd.linear = linear;
    cmd.angular = angular;
    cmd.seq = ++ctrlSeq_[slot];
    return true;
}

int32_t Manager::controlMode(uint32_t world, uint32_t robot) const
{
    if (world >= numWorlds_ || robot >= (uint32_t)count_[world]) return -1;
    return mode_[(size_t)world * maxRobots_ + robot];
}

bool Manager::setControlMode(uint32_t world, uint32_t robot, int32_t mode,
                             std::string &err)
{
    if (world >= numWorlds_ || robot >= (uint32_t)count_[world]) {
        err = "no such robot";
        return false;
    }
    if (mode != kModeWaypoint && mode != kModeVelocity) {
        err = "unknown control mode";
        return false;
    }
    size_t slot = (size_t)world * maxRobots_ + robot;
    if (mode_[slot] == mode) return true;  // already there

    mode_[slot] = mode;
    ControlCmd &cmd = ctrlBuf_[slot];
    cmd.mode = mode;
    cmd.linear = 0.f;
    cmd.angular = 0.f;
    cmd.seq = ++ctrlSeq_[slot];
    return true;
}

Occupant Manager::currentPose(uint32_t world, uint32_t r) const
{
    size_t slot = (size_t)world * maxRobots_ + r;
    // Robots that have stepped since they were added are read from the sim.
    if (steps_ > slotStep_[slot]) {
        const RobotState &s = stateBuf_[slot];
        return Occupant { s.x, s.y, slotPose_[slot].r };
    }
    return slotPose_[slot];
}

void Manager::occupants(uint32_t world, std::vector<Occupant> &out) const
{
    out.clear();
    for (int32_t r = 0; r < count_[world]; r++) {
        out.push_back(currentPose(world, (uint32_t)r));
    }
}

int32_t Manager::addRobot(uint32_t world, const Sim::RobotSpawn &s,
                          std::string &err)
{
    if (world >= numWorlds_) {
        err = "no such world";
        return -1;
    }
    if (count_[world] >= movable_[world]) {
        err = "no free cell: this layout has " + std::to_string(movable_[world]) +
              " movable cells and every one holds a robot";
        return -1;
    }
    if (count_[world] >= maxRobots_) {
        err = "world is full (" + std::to_string(maxRobots_) +
              " robots); relaunch with a larger --max-robots";
        return -1;
    }
    if (!(s.radius > 0.f) || !(s.vMax > 0.f) || !(s.aMax > 0.f) ||
        !(s.omegaMax > 0.f)) {
        err = "radius, vmax, amax and omegamax must be > 0";
        return -1;
    }
    if (circleHitsWall(specs_[world].map, s.x, s.y, s.radius)) {
        err = "position is inside a wall or outside the map";
        return -1;
    }
    for (int32_t r = 0; r < count_[world]; r++) {
        Occupant o = currentPose(world, (uint32_t)r);
        float dx = s.x - o.x, dy = s.y - o.y;
        float rr = s.radius + o.r;
        if (dx * dx + dy * dy < rr * rr) {
            err = "position overlaps robot " + std::to_string(r);
            return -1;
        }
    }

    int32_t id = count_[world]++;
    size_t slot = (size_t)world * maxRobots_ + id;
    slotPose_[slot] = Occupant { s.x, s.y, s.radius };
    slotStep_[slot] = steps_;
    mode_[slot] = kModeWaypoint;

    SpawnCmd &cmd = spawnBuf_[slot];
    cmd.x = s.x;
    cmd.y = s.y;
    cmd.theta = s.theta;
    cmd.radius = s.radius;
    cmd.vMax = s.vMax;
    cmd.aMax = s.aMax;
    cmd.omegaMax = s.omegaMax;
    cmd.seq = ++spawnSeq_[slot];
    return id;
}

std::vector<Sim::RobotSpawn> Manager::autoPlace(uint32_t world, int count,
                                                const RobotDefaults &def) const
{
    if (world >= numWorlds_) return {};
    int room = std::min(maxRobots_, movable_[world]) - count_[world];
    if (count > room) count = room;
    if (count <= 0) return {};
    std::vector<Occupant> occ;
    occupants(world, occ);
    return placeRobots(specs_[world], occ, count, def);
}

const RobotState *Manager::robotStates() const
{
    return stateBuf_;
}

RobotState Manager::robotState(uint32_t world, uint32_t robot) const
{
    return stateBuf_[(size_t)world * maxRobots_ + robot];
}

float Manager::robotRadius(uint32_t world, uint32_t robot) const
{
    return slotPose_[(size_t)world * maxRobots_ + robot].r;
}

}

#include "sim.hpp"
#include "profile.hpp"

#include <madrona/components.hpp>
#include <cmath>

namespace sim2d {

using namespace madrona;

// ---------- helpers ----------

static float wrapAngle(float a)
{
    const float pi = 3.14159265358979f;
    while (a > pi) a -= 2.f * pi;
    while (a < -pi) a += 2.f * pi;
    return a;
}

// True if a circle of radius r centred at (x, y) overlaps any wall cell
// or leaves the map.
static bool circleHitsWall(const MapGrid &map, float x, float y, float r)
{
    int32_t x0 = (int32_t)std::floor(x - r - map.originX);
    int32_t x1 = (int32_t)std::floor(x + r - map.originX);
    int32_t y0 = (int32_t)std::floor(y - r - map.originY);
    int32_t y1 = (int32_t)std::floor(y + r - map.originY);

    if (x0 < 0 || y0 < 0 || x1 >= map.width || y1 >= map.height) {
        return true;
    }

    for (int32_t cy = y0; cy <= y1; cy++) {
        for (int32_t cx = x0; cx <= x1; cx++) {
            if (!map.cells[cy * map.width + cx]) continue;

            float minx = map.originX + cx, maxx = minx + 1.f;
            float miny = map.originY + cy, maxy = miny + 1.f;
            float qx = std::fmin(std::fmax(x, minx), maxx);
            float qy = std::fmin(std::fmax(y, miny), maxy);
            float dx = x - qx, dy = y - qy;
            if (dx * dx + dy * dy < r * r) return true;
        }
    }
    return false;
}

// ---------- systems ----------

// Copy a new waypoint list into the robot's path when its sequence number
// changes. A new command also clears the stopped / collided flags.
inline void applyCommandSystem(Engine &,
                               WaypointCmd &cmd,
                               Path &path,
                               RobotMotion &motion)
{
    if (!motion.active || cmd.seq == motion.cmdSeq) return;

    motion.cmdSeq = cmd.seq;
    motion.stopped = 0;
    motion.collided = 0;
    motion.hitRobot = 0;

    path.count = cmd.count;
    path.next = 0;
    for (CountT i = 0; i < cmd.count; i++) {
        path.pts[i] = cmd.pts[i];
    }
}

// Trapezoidal speed profile plus differential-drive heading control.
inline void driveSystem(Engine &ctx,
                        RobotPose &pose,
                        RobotParams &params,
                        RobotMotion &motion,
                        Path &path)
{
    if (!motion.active) return;

    motion.prevX = pose.x;
    motion.prevY = pose.y;

    if (motion.stopped || path.next >= path.count) {
        motion.v = 0.f;
        return;
    }

    Vec2 goal = path.pts[path.next];
    float dx = goal.x - pose.x;
    float dy = goal.y - pose.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 0.05f) {
        path.next++;
        motion.v = 0.f;
        return;
    }

    float dt = ctx.data().dt;

    // Turn toward the goal, limited by the turn-rate cap.
    float desired = std::atan2(dy, dx);
    float err = wrapAngle(desired - pose.theta);
    float maxTurn = params.omegaMax * dt;
    float turn = std::fmin(std::fmax(err, -maxTurn), maxTurn);
    pose.theta = wrapAngle(pose.theta + turn);

    // Drive along the trapezoid, but only once roughly facing the goal.
    float speed = bs2d::trapezoidSpeed(dist, motion.v, params.vMax,
                                       params.aMax, dt);
    if (std::fabs(err) > 0.5f) speed = 0.f;
    motion.v = speed;

    float nx = pose.x + std::cos(pose.theta) * speed * dt;
    float ny = pose.y + std::sin(pose.theta) * speed * dt;

    const MapGrid &map = ctx.singleton<MapGrid>();
    if (circleHitsWall(map, nx, ny, params.radius)) {
        motion.stopped = 1;
        motion.v = 0.f;
        return;
    }

    pose.x = nx;
    pose.y = ny;
}

// Flag this robot if its circle overlaps any other active robot's circle.
// Only writes this robot's own flag, so it is race-free across robots.
inline void detectCollisionSystem(Engine &ctx,
                                  RobotPose &pose,
                                  RobotParams &params,
                                  RobotMotion &motion,
                                  OtherRobots &others)
{
    motion.collided = 0;
    if (!motion.active) return;

    for (CountT i = 0; i < limits::maxRobotsPerWorld - 1; i++) {
        Entity o = others.e[i];
        if (!ctx.get<RobotMotion>(o).active) continue;

        RobotPose op = ctx.get<RobotPose>(o);
        RobotParams oparams = ctx.get<RobotParams>(o);
        float dx = pose.x - op.x;
        float dy = pose.y - op.y;
        float rr = params.radius + oparams.radius;
        if (dx * dx + dy * dy < rr * rr) {
            motion.collided = 1;
            return;
        }
    }
}

// Undo this step's move for any robot in contact, and stop it. Both robots
// in a pair are flagged by detection, so both revert.
inline void resolveCollisionSystem(Engine &,
                                   RobotPose &pose,
                                   RobotMotion &motion)
{
    if (!motion.active || !motion.collided) return;

    pose.x = motion.prevX;
    pose.y = motion.prevY;
    motion.stopped = 1;
    motion.hitRobot = 1;
    motion.v = 0.f;
}

// Publish state for the host to read after each step.
inline void statusSystem(Engine &,
                         RobotPose &pose,
                         RobotMotion &motion,
                         Path &path,
                         RobotState &state)
{
    state.x = pose.x;
    state.y = pose.y;
    state.theta = pose.theta;
    state.v = motion.v;
    state.next = path.next;
    state.count = path.count;
    state.stopped = motion.stopped;
    state.collided = motion.collided;
    state.hitRobot = motion.hitRobot;
    state.active = motion.active;
}

// ---------- registration and task graph ----------

void Sim::registerTypes(ECSRegistry &registry, const Config &)
{
    base::registerTypes(registry);

    registry.registerComponent<RobotPose>();
    registry.registerComponent<RobotParams>();
    registry.registerComponent<RobotMotion>();
    registry.registerComponent<Path>();
    registry.registerComponent<WaypointCmd>();
    registry.registerComponent<RobotState>();
    registry.registerComponent<OtherRobots>();

    registry.registerSingleton<MapGrid>();

    registry.registerArchetype<Robot>();

    registry.exportColumn<Robot, WaypointCmd>(
        (uint32_t)ExportID::WaypointCmd);
    registry.exportColumn<Robot, RobotState>(
        (uint32_t)ExportID::RobotState);
}

void Sim::setupTasks(TaskGraphManager &taskgraph_mgr, const Config &)
{
    TaskGraphBuilder &builder = taskgraph_mgr.init(TaskGraphID::Step);

    auto apply = builder.addToGraph<ParallelForNode<Engine,
        applyCommandSystem,
            WaypointCmd, Path, RobotMotion
        >>({});

    auto drive = builder.addToGraph<ParallelForNode<Engine,
        driveSystem,
            RobotPose, RobotParams, RobotMotion, Path
        >>({apply});

    auto detect = builder.addToGraph<ParallelForNode<Engine,
        detectCollisionSystem,
            RobotPose, RobotParams, RobotMotion, OtherRobots
        >>({drive});

    auto resolve = builder.addToGraph<ParallelForNode<Engine,
        resolveCollisionSystem,
            RobotPose, RobotMotion
        >>({detect});

    builder.addToGraph<ParallelForNode<Engine,
        statusSystem,
            RobotPose, RobotMotion, Path, RobotState
        >>({resolve});
}

// ---------- construction ----------

Sim::Sim(Engine &ctx, const Config &cfg, const WorldInit &init)
    : WorldBase(ctx)
{
    dt = cfg.dt;

    ctx.singleton<MapGrid>() = init.map;

    // Create a fixed number of slots per world so exported buffers have a
    // stable shape. Unused slots have active = 0.
    Entity robots[limits::maxRobotsPerWorld];
    for (CountT i = 0; i < limits::maxRobotsPerWorld; i++) {
        robots[i] = ctx.makeEntity<Robot>();
    }

    for (CountT i = 0; i < limits::maxRobotsPerWorld; i++) {
        Entity e = robots[i];
        const Sim::RobotSpawn &s = init.spawns[i];

        ctx.get<RobotPose>(e) = RobotPose { s.x, s.y, s.theta };
        ctx.get<RobotParams>(e) = RobotParams {
            s.radius, s.vMax, s.aMax, s.omegaMax };

        RobotMotion motion {};
        motion.v = 0.f;
        motion.prevX = s.x;
        motion.prevY = s.y;
        motion.active = s.active;
        ctx.get<RobotMotion>(e) = motion;

        ctx.get<Path>(e) = Path {};
        ctx.get<WaypointCmd>(e) = WaypointCmd {};
        ctx.get<RobotState>(e) = RobotState {};

        OtherRobots others {};
        CountT k = 0;
        for (CountT j = 0; j < limits::maxRobotsPerWorld; j++) {
            if (j != i) others.e[k++] = robots[j];
        }
        ctx.get<OtherRobots>(e) = others;
    }
}

}

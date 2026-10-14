#include "sim.hpp"
#include "profile.hpp"
#include "geometry.hpp"

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

// ---------- systems ----------

// Put a robot into its slot when the host writes a new spawn command.
inline void applySpawnSystem(Engine &,
                             SpawnCmd &cmd,
                             RobotPose &pose,
                             RobotParams &params,
                             RobotMotion &motion,
                             Path &path)
{
    if (cmd.seq == motion.spawnSeq) return;

    motion.spawnSeq = cmd.seq;
    pose = RobotPose { cmd.x, cmd.y, cmd.theta };
    params = RobotParams { cmd.radius, cmd.vMax, cmd.aMax, cmd.omegaMax };
    motion.v = 0.f;
    motion.prevX = cmd.x;
    motion.prevY = cmd.y;
    motion.active = 1;
    motion.stopped = 0;
    motion.collided = 0;
    motion.hitRobot = 0;
    motion.mode = kModeWaypoint;
    motion.cmdLin = 0.f;
    motion.cmdAng = 0.f;
    path.count = 0;
    path.next = 0;
}

// Apply a new control mode or velocity setpoint from the host. Switching mode
// drops the waypoint path and stops the robot. Any new command clears the
// stopped / collided flags.
inline void applyControlSystem(Engine &,
                               ControlCmd &cmd,
                               Path &path,
                               RobotMotion &motion)
{
    if (!motion.active || cmd.seq == motion.ctrlSeq) return;

    motion.ctrlSeq = cmd.seq;
    if (cmd.mode != motion.mode) {
        motion.mode = cmd.mode;
        motion.v = 0.f;
        path.count = 0;
        path.next = 0;
    }
    motion.cmdLin = cmd.linear;
    motion.cmdAng = cmd.angular;
    motion.stopped = 0;
    motion.collided = 0;
    motion.hitRobot = 0;
}

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

    float dt = ctx.data().dt;

    if (motion.mode == kModeVelocity) {
        if (motion.stopped) {
            motion.v = 0.f;
            return;
        }
        // Ramp the speed toward the setpoint, then move.
        float target = std::fmin(std::fmax(motion.cmdLin, -params.vMax), params.vMax);
        float dv = params.aMax * dt;
        motion.v += std::fmin(std::fmax(target - motion.v, -dv), dv);
        float w = std::fmin(std::fmax(motion.cmdAng, -params.omegaMax), params.omegaMax);
        pose.theta = wrapAngle(pose.theta + w * dt);

        float vx = pose.x + std::cos(pose.theta) * motion.v * dt;
        float vy = pose.y + std::sin(pose.theta) * motion.v * dt;
        const MapGrid &vmap = ctx.singleton<MapGrid>();
        if (circleHitsWall(vmap, vx, vy, params.radius)) {
            motion.stopped = 1;
            motion.v = 0.f;
            return;
        }
        pose.x = vx;
        pose.y = vy;
        return;
    }

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
                                  RobotMotion &motion)
{
    motion.collided = 0;
    if (!motion.active) return;

    bool hit = false;
    ctx.iterateQuery(ctx.data().robotQuery,
        [&](RobotPose &op, RobotParams &oparams, RobotMotion &omotion) {
            if (hit || &op == &pose || !omotion.active) return;
            float dx = pose.x - op.x;
            float dy = pose.y - op.y;
            float rr = params.radius + oparams.radius;
            if (dx * dx + dy * dy < rr * rr) hit = true;
        });
    motion.collided = hit ? 1 : 0;
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
    state.mode = motion.mode;
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
    registry.registerComponent<SpawnCmd>();
    registry.registerComponent<ControlCmd>();

    registry.registerSingleton<MapGrid>();

    registry.registerArchetype<Robot>();

    registry.exportColumn<Robot, WaypointCmd>(
        (uint32_t)ExportID::WaypointCmd);
    registry.exportColumn<Robot, RobotState>(
        (uint32_t)ExportID::RobotState);
    registry.exportColumn<Robot, SpawnCmd>(
        (uint32_t)ExportID::SpawnCmd);
    registry.exportColumn<Robot, ControlCmd>(
        (uint32_t)ExportID::ControlCmd);
}

void Sim::setupTasks(TaskGraphManager &taskgraph_mgr, const Config &)
{
    TaskGraphBuilder &builder = taskgraph_mgr.init(TaskGraphID::Step);

    auto spawn = builder.addToGraph<ParallelForNode<Engine,
        applySpawnSystem,
            SpawnCmd, RobotPose, RobotParams, RobotMotion, Path
        >>({});

    auto control = builder.addToGraph<ParallelForNode<Engine,
        applyControlSystem,
            ControlCmd, Path, RobotMotion
        >>({spawn});

    auto apply = builder.addToGraph<ParallelForNode<Engine,
        applyCommandSystem,
            WaypointCmd, Path, RobotMotion
        >>({control});

    auto drive = builder.addToGraph<ParallelForNode<Engine,
        driveSystem,
            RobotPose, RobotParams, RobotMotion, Path
        >>({apply});

    auto detect = builder.addToGraph<ParallelForNode<Engine,
        detectCollisionSystem,
            RobotPose, RobotParams, RobotMotion
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
    robotQuery = ctx.query<RobotPose, RobotParams, RobotMotion>();

    ctx.singleton<MapGrid>() = init.map;

    // Create a fixed number of slots per world so exported buffers have a
    // stable shape. Slots without a spawn start inactive and are filled at
    // run time through SpawnCmd.
    for (CountT i = 0; i < (CountT)cfg.maxRobots; i++) {
        Entity e = ctx.makeEntity<Robot>();

        RobotSpawn s {};
        if (i < (CountT)init.numSpawns) s = init.spawns[i];

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
        ctx.get<SpawnCmd>(e) = SpawnCmd {};
        ctx.get<ControlCmd>(e) = ControlCmd {};
    }
}

}

// Collision tests run through the Madrona ECS manager. Each scenario is its
// own world, so all three run in one batch.
#include <cmath>
#include <cstdio>
#include <vector>

#include "map.hpp"
#include "mgr.hpp"

using namespace sim2d;

static WorldSpec makeWorld(bool wallColumn)
{
    WorldSpec w {};
    w.map.width = 10;
    w.map.height = 10;
    w.map.originX = 0.f;
    w.map.originY = 0.f;
    for (int32_t y = 0; y < 10; y++) {
        for (int32_t x = 0; x < 10; x++) {
            bool border = x == 0 || y == 0 || x == 9 || y == 9;
            bool column = wallColumn && x == 5 && y >= 1 && y <= 8;
            w.map.cells[y * 10 + x] = (border || column) ? 1 : 0;
        }
    }
    return w;
}

static Sim::RobotSpawn spawn(float x, float y, float theta)
{
    Sim::RobotSpawn s {};
    s.x = x; s.y = y; s.theta = theta;
    s.radius = 0.3f;
    s.vMax = 2.f;
    s.aMax = 1.f;
    s.omegaMax = 1.f;
    s.active = 1;
    return s;
}

static int failures = 0;

static void check(bool ok, const char *what)
{
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

int main()
{
    std::vector<WorldSpec> worlds;

    // World 0: robot drives into a wall column and must stop before it.
    {
        WorldSpec w = makeWorld(true);
        w.spawns.push_back(spawn(1.5f, 4.5f, 0.f));
        worlds.push_back(w);
    }
    // World 1: robot drives into a stationary robot in its path.
    {
        WorldSpec w = makeWorld(false);
        w.spawns.push_back(spawn(1.5f, 4.5f, 0.f));
        w.spawns.push_back(spawn(6.5f, 4.5f, 0.f));
        worlds.push_back(w);
    }
    // World 2: two robots drive head-on at each other.
    {
        WorldSpec w = makeWorld(false);
        w.spawns.push_back(spawn(1.5f, 4.5f, 0.f));
        w.spawns.push_back(spawn(8.5f, 4.5f, 3.14159f));
        worlds.push_back(w);
    }

    // World 3: real layout from CSV. Robot at row 13 (y = 22.5), left of the
    // empty gap at columns 2-3, must stop before the gap's wall.
    {
        bs2d::Map m;
        std::string err;
        if (!bs2d::loadMapCSV("../maps/layout_sample.csv", m, err)) {
            printf("layout load failed: %s\n", err.c_str());
            return 1;
        }
        WorldSpec w {};
        w.map.width = m.width;
        w.map.height = m.height;
        w.map.originX = m.originX;
        w.map.originY = m.originY;
        for (int32_t i = 0; i < m.width * m.height; i++) {
            w.map.cells[i] = m.occupied[i];
        }
        w.spawns.push_back(spawn(0.5f, 22.5f, 0.f));
        worlds.push_back(w);
    }

    // World 4: robots added while the sim runs.
    {
        WorldSpec w = makeWorld(false);
        w.spawns.push_back(spawn(1.5f, 4.5f, 0.f));
        worlds.push_back(w);
    }

    // World 5: control modes (waypoint / velocity).
    {
        WorldSpec w = makeWorld(false);
        w.spawns.push_back(spawn(1.5f, 4.5f, 0.f));
        worlds.push_back(w);
    }

    // World 6: a corridor with only 3 movable cells.
    {
        WorldSpec w {};
        w.map.width = 5;
        w.map.height = 3;
        for (int32_t y = 0; y < 3; y++)
            for (int32_t x = 0; x < 5; x++)
                w.map.cells[y * 5 + x] = !(y == 1 && x >= 1 && x <= 3);
        w.spawns.push_back(spawn(1.5f, 1.5f, 0.f));
        worlds.push_back(w);
    }

    Manager mgr({ .dt = 0.04f, .maxRobots = 4 }, worlds);
    mgr.setWaypoints(3, 0, {{5.5f, 22.5f}});

    mgr.setWaypoints(0, 0, {{8.5f, 4.5f}});
    mgr.setWaypoints(1, 0, {{8.5f, 4.5f}});
    // Head-on: each robot drives toward the other.
    mgr.setWaypoints(2, 0, {{8.5f, 4.5f}});
    mgr.setWaypoints(2, 1, {{1.5f, 4.5f}});

    for (int i = 0; i < 400; i++) {
        mgr.step();
    }

    RobotState w0 = mgr.robotState(0, 0);
    printf("World 0 (wall): x=%.3f stopped=%d\n", w0.x, w0.stopped);
    check(w0.stopped == 1, "robot stops on wall contact");
    check(w0.x < 5.0f - 0.3f + 1e-3f, "robot stays left of the wall face");

    RobotState s1 = mgr.robotState(1, 1);
    RobotState a1 = mgr.robotState(1, 0);
    float d1 = std::hypot(a1.x - s1.x, a1.y - s1.y);
    printf("World 1 (stationary robot): mover x=%.3f, target x=%.3f, dist=%.3f\n",
           a1.x, s1.x, d1);
    check(a1.stopped == 1, "moving robot stops on contact");
    check(d1 >= 0.6f - 1e-3f, "robots never overlap");

    RobotState a2 = mgr.robotState(2, 0);
    RobotState b2 = mgr.robotState(2, 1);
    float d2 = std::hypot(a2.x - b2.x, a2.y - b2.y);
    printf("World 2 (head-on): A x=%.3f, B x=%.3f, dist=%.3f\n", a2.x, b2.x, d2);
    check(a2.stopped == 1 && b2.stopped == 1, "both robots stop on head-on contact");
    check(d2 >= 0.6f - 1e-3f, "head-on robots never overlap");

    RobotState l3 = mgr.robotState(3, 0);
    printf("World 3 (layout gap): x=%.3f stopped=%d\n", l3.x, l3.stopped);
    check(l3.stopped == 1, "robot stops at layout gap wall");
    check(l3.x < 2.0f - 0.3f + 1e-3f, "robot stays left of the gap wall");

    // ---- robots added at run time (world 4, capacity 4) ----
    std::string why;
    int id1 = mgr.addRobot(4, spawn(6.5f, 4.5f, 0.f), why);
    check(id1 == 1, "run-time robot gets the next id");
    check(mgr.addRobot(4, spawn(6.7f, 4.5f, 0.f), why) < 0, "overlapping spawn is refused");
    check(mgr.addRobot(4, spawn(0.5f, 0.5f, 0.f), why) < 0, "spawn inside a wall is refused");
    check(mgr.robotCount(4) == 2, "refused spawns do not add robots");
    mgr.setWaypoints(4, 1, {{8.5f, 4.5f}});
    for (int i = 0; i < 300; i++) mgr.step();
    RobotState n1 = mgr.robotState(4, 1);
    printf("World 4 (run-time robot): x=%.3f next=%d/%d\n", n1.x, n1.next, n1.count);
    check(n1.active == 1, "run-time robot is active");
    check(n1.x > 8.3f && n1.next >= n1.count, "run-time robot follows its path");
    check(mgr.addRobot(4, spawn(3.5f, 7.5f, 0.f), why) == 2, "third robot fits");
    check(mgr.addRobot(4, spawn(3.5f, 2.5f, 0.f), why) == 3, "fourth robot fits");
    check(mgr.addRobot(4, spawn(5.5f, 7.5f, 0.f), why) < 0 &&
          why.find("full") != std::string::npos, "full world refuses more robots");
    check(mgr.autoPlace(4, 3, RobotDefaults {}).size() == 0,
          "auto placement gives nothing for a full world");
    check(mgr.autoPlace(0, 9, RobotDefaults {}).size() == 3,
          "auto placement stops at the free slots left");
    // A new robot driven into an existing one stops on contact.
    mgr.step();
    mgr.setWaypoints(4, 2, {{8.5f, 4.5f}});
    mgr.setWaypoints(4, 3, {{8.5f, 4.5f}});
    for (int i = 0; i < 300; i++) mgr.step();
    RobotState n2 = mgr.robotState(4, 2);
    check(n2.stopped == 1 || n2.next >= n2.count, "run-time robots obey collisions");

    // ---- control modes (world 5) ----
    std::string e5;
    check(mgr.controlMode(5, 0) == kModeWaypoint, "robot starts in waypoint mode");
    check(!mgr.setVelocity(5, 0, 1.f, 0.f, e5), "velocity command refused in waypoint mode");
    check(mgr.setControlMode(5, 0, kModeVelocity, e5), "switch to velocity mode");
    check(mgr.controlMode(5, 0) == kModeVelocity, "mode reads back as velocity");
    check(!mgr.setWaypoints(5, 0, {{3.5f, 4.5f}}, e5), "path refused in velocity mode");
    check(mgr.setVelocity(5, 0, 1.f, 0.f, e5), "velocity command accepted");
    for (int i = 0; i < 100; i++) mgr.step();
    RobotState v5 = mgr.robotState(5, 0);
    printf("World 5 (velocity 1 m/s for 4 s): x=%.3f v=%.3f mode=%d\n", v5.x, v5.v, v5.mode);
    check(v5.x > 4.8f && v5.x < 5.2f, "velocity mode drives with acceleration ramp");
    check(v5.mode == kModeVelocity, "state reports velocity mode");
    for (int i = 0; i < 400; i++) mgr.step();
    v5 = mgr.robotState(5, 0);
    check(v5.stopped == 1 && v5.x < 9.0f - 0.3f + 1e-3f, "velocity mode stops at a wall");
    mgr.setVelocity(5, 0, 0.f, 1.f, e5);
    for (int i = 0; i < 25; i++) mgr.step();
    RobotState t5 = mgr.robotState(5, 0);
    check(t5.stopped == 0 && std::fabs(t5.theta - v5.theta - 1.0f) < 0.05f,
          "new velocity command clears the stop and turns at the set rate");
    check(mgr.setControlMode(5, 0, kModeWaypoint, e5), "switch back to waypoint mode");
    check(mgr.setWaypoints(5, 0, {{5.5f, 4.5f}}, e5), "path accepted again");
    for (int i = 0; i < 600; i++) mgr.step();
    RobotState b5 = mgr.robotState(5, 0);
    printf("World 5 (back to waypoints): x=%.3f next=%d/%d\n", b5.x, b5.next, b5.count);
    check(b5.mode == kModeWaypoint && b5.next >= b5.count && std::fabs(b5.x - 5.5f) < 0.1f,
          "waypoint mode works after switching back");

    // ---- no space left (world 6: 3 movable cells) ----
    std::string e6;
    check(mgr.movableCells(6) == 3, "corridor has 3 movable cells");
    check(mgr.autoPlace(6, 10, RobotDefaults {}).size() == 2, "auto placement offers only the free cells");
    check(mgr.addRobot(6, spawn(2.5f, 1.5f, 0.f), e6) == 1, "second robot fits");
    check(mgr.addRobot(6, spawn(3.5f, 1.5f, 0.f), e6) == 2, "third robot fits");
    check(mgr.addRobot(6, spawn(2.5f, 1.5f, 0.f), e6) < 0 &&
          e6.find("movable") != std::string::npos, "no robot beyond the movable cells");
    check(mgr.autoPlace(6, 1, RobotDefaults {}).size() == 0, "nothing left to place");

    printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}

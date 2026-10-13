// Collision tests run through the Madrona ECS manager. Each scenario is its
// own world, so all three run in one batch.
#include <cmath>
#include <cstdio>
#include <vector>

#include "map.hpp"
#include "mgr.hpp"

using namespace sim2d;

static Sim::WorldInit makeWorld(bool wallColumn)
{
    Sim::WorldInit w {};
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
    std::vector<Sim::WorldInit> worlds;

    // World 0: robot drives into a wall column and must stop before it.
    {
        Sim::WorldInit w = makeWorld(true);
        w.spawns[0] = spawn(1.5f, 4.5f, 0.f);
        worlds.push_back(w);
    }
    // World 1: robot drives into a stationary robot in its path.
    {
        Sim::WorldInit w = makeWorld(false);
        w.spawns[0] = spawn(1.5f, 4.5f, 0.f);
        w.spawns[1] = spawn(6.5f, 4.5f, 0.f);
        worlds.push_back(w);
    }
    // World 2: two robots drive head-on at each other.
    {
        Sim::WorldInit w = makeWorld(false);
        w.spawns[0] = spawn(1.5f, 4.5f, 0.f);
        w.spawns[1] = spawn(8.5f, 4.5f, 3.14159f);
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
        Sim::WorldInit w {};
        w.map.width = m.width;
        w.map.height = m.height;
        w.map.originX = m.originX;
        w.map.originY = m.originY;
        for (int32_t i = 0; i < m.width * m.height; i++) {
            w.map.cells[i] = m.occupied[i];
        }
        w.spawns[0] = spawn(0.5f, 22.5f, 0.f);
        worlds.push_back(w);
    }

    Manager mgr({ .dt = 0.04f }, worlds);
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

    printf("%s (%d failures)\n", failures ? "FAILED" : "ALL PASSED", failures);
    return failures ? 1 : 0;
}

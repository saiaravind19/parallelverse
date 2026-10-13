// Headless speed test: bench <worlds> <robots_per_world> <steps>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "mgr.hpp"

using namespace sim2d;

int main(int argc, char **argv)
{
    int nw = argc > 1 ? atoi(argv[1]) : 1;
    int nr = argc > 2 ? atoi(argv[2]) : 100;
    int steps = argc > 3 ? atoi(argv[3]) : 500;
    if (nr > limits::maxRobotsPerWorld) {
        printf("robots per world exceeds build capacity %d\n",
               (int)limits::maxRobotsPerWorld);
        return 1;
    }

    // Open 100 x 100 map with a wall border; robots on a 2 m lattice.
    Sim::WorldInit w {};
    w.map.width = w.map.height = 100;
    for (int y = 0; y < 100; y++)
        for (int x = 0; x < 100; x++)
            w.map.cells[y * 100 + x] = (x == 0 || y == 0 || x == 99 || y == 99);
    for (int i = 0; i < nr; i++) {
        Sim::RobotSpawn s {};
        s.x = 2.5f + 2.f * (i % 45);
        s.y = 2.5f + 2.f * (i / 45);
        s.radius = 0.3f; s.vMax = 2.f; s.aMax = 1.f; s.omegaMax = 1.f;
        s.active = 1;
        w.spawns[i] = s;
    }
    std::vector<Sim::WorldInit> worlds(nw, w);

    auto t0 = std::chrono::steady_clock::now();
    Manager mgr(Manager::Config { 0.04f }, worlds);
    auto t1 = std::chrono::steady_clock::now();

    for (int wi = 0; wi < nw; wi++)
        for (int r = 0; r < nr; r++) {
            float x = 2.5f + 2.f * (r % 45), y = 2.5f + 2.f * (r / 45);
            mgr.setWaypoints(wi, r, { {x + 1.f, y}, {x + 1.f, y + 1.f}, {x, y + 1.f}, {x, y} });
        }

    auto t2 = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; i++) mgr.step();
    auto t3 = std::chrono::steady_clock::now();

    double setup = std::chrono::duration<double>(t1 - t0).count();
    double run = std::chrono::duration<double>(t3 - t2).count();
    printf("worlds=%d robots/world=%d cap=%d | setup %.2fs | %.1f steps/s "
           "(%.2f ms/step) | %.2fM robot-steps/s | sim speed %.1fx real time\n",
           nw, nr, (int)limits::maxRobotsPerWorld, setup, steps / run,
           1000.0 * run / steps, (double)nw * nr * steps / run / 1e6,
           steps / run * 0.04);
    return 0;
}

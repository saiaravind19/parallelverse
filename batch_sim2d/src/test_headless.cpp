#include <cstdio>
#include "world.hpp"

int main(int argc, char **argv)
{
    using namespace bs2d;
    World w;
    std::string err;
    if (!loadMapCSV(argc > 1 ? argv[1] : "../../batch_sim2d/maps/open_10x10.csv", w.map, err)) {
        fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }
    Robot r;
    r.pos = {1.5f, 1.5f};
    r.waypoints = {{8.5f, 1.5f}, {8.5f, 8.5f}};
    w.robots.push_back(r);

    for (int i = 0; i < 1000; i++) {
        w.step();
        if (w.robots[0].next >= w.robots[0].waypoints.size()) {
            printf("finished at step %d, pos=(%.2f, %.2f)\n", i,
                   w.robots[0].pos.x, w.robots[0].pos.y);
            return 0;
        }
    }
    printf("did not finish: pos=(%.2f, %.2f) stopped=%d\n",
           w.robots[0].pos.x, w.robots[0].pos.y, (int)w.robots[0].stopped);
    return 1;
}

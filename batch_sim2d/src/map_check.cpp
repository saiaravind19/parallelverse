// Prints a loaded layout as ASCII: '#' = wall, '.' = drivable.
#include <cstdio>
#include "map.hpp"

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: sim2d_map_check <map.csv>\n");
        return 1;
    }
    bs2d::Map m;
    std::string err;
    if (!bs2d::loadMapCSV(argv[1], m, err)) {
        fprintf(stderr, "%s\n", err.c_str());
        return 1;
    }
    printf("%d x %d cells (1 m each), y up\n", m.width, m.height);
    for (int32_t y = m.height - 1; y >= 0; y--) {
        for (int32_t x = 0; x < m.width; x++) {
            putchar(m.isOccupied(x, y) ? '#' : '.');
        }
        putchar('\n');
    }
    return 0;
}

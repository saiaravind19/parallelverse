#include "map.hpp"

#include <fstream>
#include <sstream>

namespace bs2d {

bool loadMapCSV(const std::string &path, Map &out, std::string &err)
{
    std::ifstream f(path);
    if (!f) {
        err = "cannot open " + path;
        return false;
    }

    std::vector<std::vector<uint8_t>> rows;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::vector<uint8_t> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stoi(cell) != 0 ? 1 : 0);
        }
        rows.push_back(row);
    }
    if (rows.empty()) {
        err = "empty map";
        return false;
    }

    size_t w = rows[0].size();
    for (auto &r : rows) {
        if (r.size() != w) {
            err = "ragged CSV rows";
            return false;
        }
    }

    out.width = (int32_t)w;
    out.height = (int32_t)rows.size();
    out.occupied.assign(out.width * out.height, 0);
    // File row 0 is the top of the map, so flip it.
    for (int32_t y = 0; y < out.height; y++) {
        for (int32_t x = 0; x < out.width; x++) {
            out.occupied[(out.height - 1 - y) * out.width + x] = rows[y][x];
        }
    }
    return true;
}

} // namespace bs2d

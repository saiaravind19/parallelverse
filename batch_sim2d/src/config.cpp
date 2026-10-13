#include "config.hpp"
#include "map.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>

namespace sim2d {

using json = nlohmann::json;
namespace fs = std::filesystem;

bool loadScene(const std::string &path, float &dt,
               std::vector<Sim::WorldInit> &worlds,
               std::vector<std::string> &names, std::string &err)
{
    std::ifstream f(path);
    if (!f) {
        err = "cannot open scene file " + path;
        return false;
    }

    // Exceptions are disabled in this build, so parse without throwing.
    json root = json::parse(f, nullptr, false);
    if (root.is_discarded()) {
        err = "scene file is not valid JSON: " + path;
        return false;
    }

    dt = root.value("dt", 0.04f);
    if (!(dt > 0.f)) {
        err = "dt must be > 0";
        return false;
    }

    fs::path base = fs::path(path).parent_path();
    worlds.clear();
    names.clear();

    if (!root.contains("worlds") || !root["worlds"].is_array()) {
        err = "scene file needs a \"worlds\" array";
        return false;
    }
    for (const auto &wj : root["worlds"]) {
        if (!wj.contains("map") || !wj.contains("robots")) {
            err = "each world needs \"map\" and \"robots\"";
            return false;
        }
        bs2d::Map map;
        std::string map_err;
        fs::path map_path = base / wj["map"].get<std::string>();
        if (!bs2d::loadMapCSV(map_path.string(), map, map_err)) {
            err = "world map: " + map_err;
            return false;
        }
        if (map.width > limits::maxMapDim || map.height > limits::maxMapDim) {
            err = "map larger than " + std::to_string(limits::maxMapDim) +
                  " cells per side: " + map_path.string();
            return false;
        }

        Sim::WorldInit init {};
        init.map.width = map.width;
        init.map.height = map.height;
        init.map.originX = map.originX;
        init.map.originY = map.originY;
        for (int32_t i = 0; i < map.width * map.height; i++) {
            init.map.cells[i] = map.occupied[i];
        }

        const auto &robots = wj.at("robots");
        if ((CountT)robots.size() > limits::maxRobotsPerWorld) {
            err = "scene has " + std::to_string(robots.size()) +
                  " robots in one world; this build holds up to " +
                  std::to_string(limits::maxRobotsPerWorld) +
                  ". Rebuild with -DSIM2D_MAX_ROBOTS=<count>";
            return false;
        }

        for (CountT i = 0; i < (CountT)robots.size(); i++) {
            const auto &rj = robots[i];
            if (!rj.contains("x") || !rj.contains("y")) {
                err = "each robot needs \"x\" and \"y\"";
                return false;
            }
            Sim::RobotSpawn &s = init.spawns[i];
            s.x = rj["x"].get<float>();
            s.y = rj["y"].get<float>();
            s.theta = rj.value("theta", 0.f);
            s.radius = rj.value("radius", 0.3f);
            s.vMax = rj.value("vmax", 2.f);
            s.aMax = rj.value("amax", 1.f);
            s.omegaMax = rj.value("omegamax", 1.f);
            s.active = 1;

            if (!(s.vMax > 0.f)) {
                err = "vmax must be greater than 0";
                return false;
            }
            if (!(s.radius > 0.f) || !(s.aMax > 0.f) || !(s.omegaMax > 0.f)) {
                err = "radius, amax and omegamax must be > 0";
                return false;
            }
        }

        worlds.push_back(init);
        names.push_back(wj.value("name", map_path.stem().string()));
    }

    if (worlds.empty()) {
        err = "scene has no worlds";
        return false;
    }
    return true;
}

}

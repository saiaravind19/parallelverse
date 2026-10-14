#include "config.hpp"
#include "map.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>

namespace sim2d {

using json = nlohmann::json;
namespace fs = std::filesystem;

static bool readDefaults(const json &j, RobotDefaults &d, std::string &err)
{
    d.radius = j.value("radius", d.radius);
    d.vMax = j.value("vmax", d.vMax);
    d.aMax = j.value("amax", d.aMax);
    d.omegaMax = j.value("omegamax", d.omegaMax);
    if (!(d.vMax > 0.f)) {
        err = "vmax must be greater than 0";
        return false;
    }
    if (!(d.radius > 0.f) || !(d.aMax > 0.f) || !(d.omegaMax > 0.f)) {
        err = "radius, amax and omegamax must be > 0";
        return false;
    }
    return true;
}

bool loadScene(const std::string &path, float &dt,
               std::vector<WorldSpec> &worlds,
               std::vector<std::string> &names, std::string &err,
               int robotsOverride, int *maxRobotsOut)
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
    if (maxRobotsOut) *maxRobotsOut = root.value("max_robots", 0);
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
        if (!wj.contains("map") || (!wj.contains("robots") && robotsOverride < 0)) {
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

        WorldSpec init {};
        init.map.width = map.width;
        init.map.height = map.height;
        init.map.originX = map.originX;
        init.map.originY = map.originY;
        for (int32_t i = 0; i < map.width * map.height; i++) {
            init.map.cells[i] = map.occupied[i];
        }
        init.dirs = map.dirs;

        const json none = json::array();
        const json &robots = wj.contains("robots") ? wj["robots"] : none;
        RobotDefaults def;
        int auto_count = -1;  // >= 0: place this many automatically
        if (robots.is_number_integer()) {
            auto_count = robots.get<int>();
        } else if (robots.is_object()) {
            if (!readDefaults(robots, def, err)) return false;
            auto_count = robots.value("count", 0);
        } else if (!robots.is_array()) {
            err = "\"robots\" must be a list, a number, or an object with \"count\"";
            return false;
        }
        if (robotsOverride >= 0) auto_count = robotsOverride;

        if (auto_count >= 0) {
            init.spawns = placeRobots(init, {}, auto_count, def);
            if ((int)init.spawns.size() < auto_count) {
                err = "map " + map_path.string() + " has " +
                      std::to_string(countMovableCells(init)) +
                      " movable cells, so at most that many robots fit; asked for " +
                      std::to_string(auto_count);
                return false;
            }
        } else {
            for (const auto &rj : robots) {
                if (!rj.contains("x") || !rj.contains("y")) {
                    err = "each robot needs \"x\" and \"y\"";
                    return false;
                }
                RobotDefaults d;
                if (!readDefaults(rj, d, err)) return false;
                Sim::RobotSpawn s {};
                s.x = rj["x"].get<float>();
                s.y = rj["y"].get<float>();
                s.theta = rj.value("theta", 0.f);
                s.radius = d.radius;
                s.vMax = d.vMax;
                s.aMax = d.aMax;
                s.omegaMax = d.omegaMax;
                s.active = 1;
                init.spawns.push_back(s);
            }
        }

        int cells = countMovableCells(init);
        if ((int)init.spawns.size() > cells) {
            err = "map " + map_path.string() + " has only " + std::to_string(cells) +
                  " movable cells but the scene starts " +
                  std::to_string(init.spawns.size()) + " robots";
            return false;
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

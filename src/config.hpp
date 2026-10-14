#pragma once

#include <string>
#include <vector>

#include "world_spec.hpp"

namespace sim2d {

// Load a scene file. Format (JSON):
// {
//   "dt": 0.04,
//   "max_robots": 300,             // optional robot slots per world (default: movable cells)
//   "worlds": [
//     {
//       "name": "open room",           // optional, default = map file name
//       "map": "maps/open_10x10.csv",
//       "robots": [                    // a list of robots ...
//         {"x": 1.5, "y": 1.5, "theta": 0.0,
//          "radius": 0.3, "vmax": 2.0, "amax": 1.0, "omegamax": 1.0}
//       ]
//       // ... or just a number: "robots": 40   (placed automatically)
//       // ... or {"count": 40, "radius": 0.3, "vmax": 2.0, ...}
//     }
//   ]
// }
// Map paths are relative to the scene file's directory.
// If robotsOverride >= 0, every world starts with that many automatically
// placed robots instead of what the scene lists.
bool loadScene(const std::string &path, float &dt,
               std::vector<WorldSpec> &worlds,
               std::vector<std::string> &names, std::string &err,
               int robotsOverride = -1, int *maxRobotsOut = nullptr);

}

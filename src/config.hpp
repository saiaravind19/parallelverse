#pragma once

#include <string>
#include <vector>

#include "sim.hpp"

namespace sim2d {

// Load a scene file. Format (JSON):
// {
//   "dt": 0.04,
//   "worlds": [
//     {
//       "name": "open room",           // optional, default = map file name
//       "map": "maps/open_10x10.csv",
//       "robots": [
//         {"x": 1.5, "y": 1.5, "theta": 0.0,
//          "radius": 0.3, "vmax": 2.0, "amax": 1.0, "omegamax": 1.0}
//       ]
//     }
//   ]
// }
// Map paths are relative to the scene file's directory.
bool loadScene(const std::string &path, float &dt,
               std::vector<Sim::WorldInit> &worlds,
               std::vector<std::string> &names, std::string &err);

}

# multi_world_sim

The simulator app is called `parallelverse`.

A 2D batch simulator for mobile-robot fleets, built on the
[Madrona](https://github.com/shacklettbp/madrona) engine's ECS. Many worlds run
in one process. Each world is loaded from a CSV layout, holds up to hundreds of
differential-drive circular robots, and is driven over ZeroMQ.

- Layouts are CSV files, 1 m per cell ([docs/layout_rules.md](docs/layout_rules.md)).
- Robots follow waypoints with a trapezoidal speed profile and stop on contact
  with a wall or another robot.
- A top-down GUI shows one tab per world, with robot ids, and a Record button
  that writes an MP4.
- Robot state is published and waypoints are accepted over ZeroMQ with
  protobuf messages ([docs/communication.md](docs/communication.md)).
- Python tools generate scenes and publish waypoints.

## Setup

### 1. Install the system packages

Ubuntu / Debian (x86_64):

```bash
sudo apt update
sudo apt install git cmake python3 ffmpeg \
    libglfw3-dev libgl1-mesa-dev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    libzmq3-dev nlohmann-json3-dev
```

macOS (Homebrew):

```bash
brew install cmake glfw zeromq nlohmann-json ffmpeg
```

You do not need a C++ compiler or protobuf. Madrona downloads its own clang
toolchain the first time you run `cmake`, so the machine needs internet access
to GitHub at that point. The message code uses nanopb, which is bundled in
`third_party/nanopb`.

For the Python tools, install [uv](https://docs.astral.sh/uv/).

Linux ARM (aarch64) is not tested. The toolchain download is chosen by
`uname -m`, and Madrona's architecture check may not recognise `aarch64`. If
`cmake` fails there, please open an issue with the log.

### 2. Get the code (with the Madrona submodule)

```bash
git clone --recursive https://github.com/saiaravind19/multi_world_sim.git
cd multi_world_sim
# if you cloned without --recursive:
git submodule update --init --recursive
```

### 3. Build

```bash
cmake -S . -B build
cmake --build build -j --target parallelverse
```

Other targets: `sim2d_collision_test` (collision tests), `sim2d_bench`
(headless speed test), `sim2d_map_check` (checks a layout).

### 4. Capacity limits (build settings)

Two table sizes are fixed when you configure:

```bash
cmake -S . -B build -DSIM2D_MAX_WAYPOINTS=64 -DSIM2D_MAX_MAP_DIM=128
```

`SIM2D_MAX_WAYPOINTS` is the longest path and `SIM2D_MAX_MAP_DIM` the largest
map side in cells. The number of robots is not a build setting: you give it when
you launch (below), and can add more while it runs.

## Run

```bash
cd build
./parallelverse ../scenes/sample.json
```

Options:

```bash
./parallelverse ../scenes/sample.json --robots 300      # 300 robots in every world, placed for you
./parallelverse ../scenes/sample.json --max-robots 300  # optional: cap the slots for more speed (default: every movable cell)
```

Slots per world come from `--max-robots`, else every movable cell; they never exceed the movable cells.

A scene can also say `"robots": 40` instead of listing each robot. A world never
holds more robots than its movable cells (cells with an arrow, or all free cells
if the layout has no arrows). Add robots while it runs with the "Add robots"
button, `uv run spawn_robots.py <world> --count 20`, or a `/cmd_spawn` message.

In the map view, scroll to zoom around the cursor, drag to pan, and use Fit to
reset.

Robots take commands as waypoints (default) or velocities. Switch with
`uv run set_control_mode.py <world> <robot> velocity`; see
[docs/communication.md](docs/communication.md).

On Linux with no display, run it under a virtual display:
`xvfb-run -a ./parallelverse ../scenes/sample.json`.

### Make a scene from a layout

```bash
cd tools
uv run make_scene.py          # window: add several layouts, set robots per layout
uv run make_scene.py --map a.csv:20 --map b.csv:50 --out fleet.json   # no window
```

See [docs/make_scene_guide.md](docs/make_scene_guide.md). For layouts with
direction arrows, robots start only on lane cells, facing the lane direction.

### Send waypoints

```bash
cd tools
uv run publish_waypoints.py 0 0 "2.5,9.5 5.5,9.5"   # world 0, robot 0
uv run publish_velocity.py 0 0 1.0 0.3 --hold 5      # velocity mode only
```

### Test

```bash
cd build
./sim2d_collision_test
./sim2d_bench 1 300 100      # worlds, robots per world, steps [robot slots]
```

## Layout

```
src/          simulator, manager, GUI app, tests, bench
proto/        message definitions (nanopb options)
gen/          generated nanopb code
third_party/  nanopb runtime
tools/        Python tools (uv)
maps/ scenes/ example layouts and scenes
docs/         layout rules, scene guide, communication
external/     Madrona (git submodule)
```

## Credits

Built on [Madrona](https://github.com/shacklettbp/madrona). The repository
started from Madrona's `madrona_escape_room` example for its build setup; see
[LICENSE](LICENSE).

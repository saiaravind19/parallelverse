# Parallelverse

A 2D batch simulator for mobile-robot fleets: many independent worlds, each with
up to hundreds of differential-drive robots, stepped together in one process.

## Demo

[![Parallelverse demo](https://img.youtube.com/vi/JtwCn2WjRnU/maxresdefault.jpg)](https://youtu.be/JtwCn2WjRnU)

*Click the image to watch the demo on YouTube.*

## How it uses Madrona

Parallelverse is built on the [Madrona](https://github.com/shacklettbp/madrona)
engine, which runs thousands of environments in one batch using an
Entity Component System (ECS).

- **Worlds** — each CSV layout becomes one Madrona world. All worlds are stepped
  together by Madrona's executor (CPU thread pool today).
- **Entities and components** — every robot is a `Robot` entity built from
  components such as `RobotPose`, `RobotMotion`, `Path` and `WaypointCmd`.
- **Systems in a task graph** — each step runs a fixed graph of systems:
  spawn → control mode → waypoint commands → drive (trapezoidal profile) →
  collision detect → collision resolve → status.
- **Exported buffers** — component columns are exported as flat arrays, so the
  manager reads robot state and writes commands for every world at once, and
  the ZeroMQ bridge publishes and accepts them as protobuf messages.

## How to use

### Install

Ubuntu / Debian:

```bash
sudo apt install git cmake python3 libglfw3-dev libgl1-mesa-dev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    libzmq3-dev nlohmann-json3-dev
```

macOS:

```bash
brew install cmake glfw zeromq nlohmann-json
```

Madrona downloads its own clang toolchain on the first `cmake` run (needs
internet). Install [uv](https://docs.astral.sh/uv/) for the Python tools.

### Build

```bash
git clone --recursive https://github.com/saiaravind19/parallelverse.git
cd parallelverse
cmake -S . -B build
cmake --build build -j --target parallelverse
```

### Run

```bash
cd build
./parallelverse ../scenes/sample.json                 # robots as listed in the scene
./parallelverse ../scenes/sample.json --robots 300    # 300 robots per world
```

### Drive the robots

```bash
cd tools
uv run make_scene.py                                  # build a scene from layouts
uv run publish_waypoints.py 0 0 "2.5,9.5 5.5,9.5"     # world 0, robot 0
uv run spawn_robots.py 0 --count 20                   # add robots while running
```

More detail: [layout rules](docs/layout_rules.md),
[scene guide](docs/make_scene_guide.md),
[communication](docs/communication.md).

## Credits

Built on [Madrona](https://github.com/shacklettbp/madrona), starting from its
`madrona_escape_room` example. See [LICENSE](LICENSE).

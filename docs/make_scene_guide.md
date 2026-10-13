# make_scene.py: how to use it

`make_scene.py` builds a scene file for the simulator from one layout CSV.
You give it the CSV and the number of robots. It writes a `*_scene.json`
file next to the CSV, which `sim2d_app` can open.

## What you need

- Python 3 (any recent version). No extra packages.
- A layout CSV that follows `docs/layout_rules.md`.

## Steps

1. Put `make_scene.py` in any folder, for example next to your CSV.
2. Run it. A window opens:

   ```
   python3 make_scene.py
   ```

   Click "Choose layout CSV...", pick the robot count, then click "Generate scene".
   The window shows the scene path and the command to run.

   To use terminal prompts instead, add `--cli`:

   ```
   python3 make_scene.py --cli
   ```

   The window needs PyQt5 or PyQt6. With uv, from the `tools` folder: `uv run make_scene.py`.

3. In `--cli` mode it asks two questions:

   ```
   Layout CSV path: /Users/you/projects/multi_world_sim/maps/layout_sample.csv
   Number of robots: 3
   ```

   - You can drag the CSV file into the terminal to paste its path.
   - The robot count must be between 1 and 16.

4. It prints what it made:

   ```
   Wrote /Users/you/.../maps/layout_sample_scene.json
   Map 8 x 35 cells, 178 drivable, 3 robot(s).
   Run: ./sim2d_app "/Users/you/.../maps/layout_sample_scene.json"
   ```

5. Start the simulator with that scene file, from the build folder:

   ```
   ./sim2d_app "/Users/you/.../maps/layout_sample_scene.json"
   ```

## What the scene file contains

- **name:** the file name without `.csv`. The GUI uses it as the world tab name.
- **map:** the CSV file name. The scene file sits in the same folder, so the simulator finds the CSV.
- **robots:** one entry per robot, with:
  - `x`, `y`: start position in meters. Each cell is 1 m, and y goes up from the bottom row.
  - `theta`: start heading in radians. Every robot starts facing right (0).
  - `radius`: 0.3 m.
  - `vmax`: 2.0 m/s, the top speed.
  - `amax`: 1.0 m/s², how fast the robot speeds up and slows down.
  - `omegamax`: 1.0 rad/s, how fast the robot can turn.
- **dt:** 0.04 s, the simulation time step.

## How robot start positions are chosen

- Robots start only on drivable cells.
- The script prefers cells whose 8 neighbors are also drivable, so robots start clear of walls.
- Robots are kept at least 3 cells apart when the layout allows it.
- The choice is the same every time you run it with the same CSV and count.

Every robot starts facing right. To change a start position or heading, edit
the scene JSON by hand, or change the choice rules in the script.

## Changing the defaults

Open `make_scene.py`. The values are at the top:

```python
DT = 0.04
RADIUS = 0.3
VMAX = 2.0
AMAX = 1.0
OMEGAMAX = 1.0
MAX_ROBOTS = 16
```

## How the grid size is read

The script reads the CSV the same way the simulator does, so the grid
matches what `sim2d_map_check` prints:

- Drivable cells are `(a,b,c,d)` tuples or `0`.
- Wall cells are `1`.
- Empty cells are walls, but they do not make the grid larger. This is how the border works.
- Other text, such as labels, is ignored.

To check a layout by itself:

```
./sim2d_map_check "/path/to/layout.csv"
```

## Troubleshooting

| Message | What to do |
| --- | --- |
| `File not found: ...` | Check the path. Drag the file into the terminal to paste it correctly. |
| `Number of robots must be a whole number.` | Type a number like `3`. |
| `Number of robots must be between 1 and 16.` | Pick a smaller count. |
| `layout has only N drivable cells, need M` | Use fewer robots, or add drivable cells to the CSV. |
| `no drivable or wall cells found in the layout` | The CSV is empty or not in the expected format. |

If `sim2d_app` says `scene error`, the scene file path is probably wrong.
Use the full path it printed.

## Limits

- One world per scene file. To put several layouts in one scene, run the script once per layout and combine the `worlds` entries by hand.
- Robots all start facing right.
- Robot names are not set. The GUI names robots by their index.

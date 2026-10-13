#!/usr/bin/env python3
"""Generate a batch_sim2d scene JSON for one layout CSV.

Run it, type the layout path and the number of robots, and it writes
<layout_name>_scene.json next to the CSV file.

Cell rules (same as the simulator):
  "(a,b,c,d)" or "0"  -> drivable
  "" or "1"           -> wall
  other text          -> ignored (legend, labels)
"""
import csv
import json
import os
import sys

DT = 0.04
RADIUS = 0.3
VMAX = 2.0
AMAX = 1.0
OMEGAMAX = 1.0
MAX_ROBOTS = 16


def classify(text):
    t = text.strip()
    if t == "":
        return "wall"
    if t.startswith("(") and t.endswith(")"):
        return "free"
    if t == "0":
        return "free"
    if t == "1":
        return "wall"
    return None  # text outside the grid


def load_layout(path):
    """Return (width, height, free) with free[(x, y)] = True, y up from the bottom."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    # Grid extent = bounding block of cells that are drivable or wall.
    width = height = 0
    kinds = {}
    for r, row in enumerate(rows):
        for c, field in enumerate(row):
            k = classify(field)
            if k is None:
                continue
            kinds[(r, c)] = k
            if k == "wall" and field.strip() == "":
                continue  # empty cells are the border; they do not set the grid size
            width = max(width, c + 1)
            height = max(height, r + 1)
    if width == 0 or height == 0:
        raise ValueError("no drivable or wall cells found in the layout")

    free = set()
    for (r, c), k in kinds.items():
        if c >= width or r >= height:
            continue  # outside the grid block
        if k == "free":
            # File row 0 is the top of the layout; y grows upward.
            free.add((c, height - 1 - r))
    return width, height, free


def pick_spawns(width, height, free, count):
    """Pick robot start cells, preferring cells whose neighbours are all drivable."""
    def clear(x, y):
        return all((x + dx, y + dy) in free
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1))

    ordered = sorted(free, key=lambda p: (p[1], p[0]))
    preferred = [p for p in ordered if clear(*p)]
    candidates = preferred + [p for p in ordered if p not in preferred]
    if len(candidates) < count:
        raise ValueError(f"layout has only {len(candidates)} drivable cells, "
                         f"need {count}")

    # Spread robots out: keep chosen cells at least 3 cells apart when possible.
    chosen = []
    for p in candidates:
        if all(max(abs(p[0] - q[0]), abs(p[1] - q[1])) >= 3 for q in chosen):
            chosen.append(p)
            if len(chosen) == count:
                return chosen
    for p in candidates:  # fallback: fill remaining slots anywhere free
        if p not in chosen:
            chosen.append(p)
            if len(chosen) == count:
                break
    return chosen


def main():
    layout = input("Layout CSV path: ").strip().strip('"').strip("'")
    layout = os.path.expanduser(layout)
    if not os.path.isfile(layout):
        sys.exit(f"File not found: {layout}")

    try:
        count = int(input("Number of robots: ").strip())
    except ValueError:
        sys.exit("Number of robots must be a whole number.")
    if not 1 <= count <= MAX_ROBOTS:
        sys.exit(f"Number of robots must be between 1 and {MAX_ROBOTS}.")

    width, height, free = load_layout(layout)
    spawns = pick_spawns(width, height, free, count)

    out_dir = os.path.dirname(os.path.abspath(layout))
    stem = os.path.splitext(os.path.basename(layout))[0]
    map_rel = os.path.basename(layout)  # same folder as the scene file

    robots = [{
        "x": x + 0.5, "y": y + 0.5, "theta": 0.0,
        "radius": RADIUS, "vmax": VMAX, "amax": AMAX, "omegamax": OMEGAMAX,
    } for (x, y) in spawns]

    scene = {
        "dt": DT,
        "worlds": [{"name": stem, "map": map_rel, "robots": robots}],
    }

    out_path = os.path.join(out_dir, f"{stem}_scene.json")
    with open(out_path, "w") as f:
        json.dump(scene, f, indent=2)
        f.write("\n")

    print(f"Wrote {out_path}")
    print(f"Map {width} x {height} cells, {len(free)} drivable, {count} robot(s).")
    print(f"Run: ./sim2d_app \"{out_path}\"")


if __name__ == "__main__":
    main()

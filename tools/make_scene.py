#!/usr/bin/env python3
"""Generate a scene JSON from one or more layout CSVs (one world per layout).

    python3 make_scene.py          opens a window (needs PyQt5 or PyQt6); add
                                   several layouts and set robots for each
    python3 make_scene.py --cli    asks for layout paths and robot counts
    python3 make_scene.py --map a.csv:20 --map b.csv:50 --out fleet.json

The scene goes in scenes/ beside the maps/ folder (or next to the first CSV).
With one layout it is <layout_name>_scene.json; with several,
multi_<n>_maps_scene.json unless you name it. World ids follow the order of the
layouts.

Cell rules (same as the simulator):
  "(a,b,c,d)" or "0"  -> drivable
  "" or "1"           -> wall
  other text          -> ignored (legend, labels)
"""
import argparse
import csv
import json
import math
import os
import sys

DT = 0.04
RADIUS = 0.3
VMAX = 2.0
AMAX = 1.0
OMEGAMAX = 1.0


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


# Facing for each direction symbol in a "(a,b,c,d)" cell (file up = +y).
DIR_THETA = {"^": math.pi / 2, "v": -math.pi / 2, ">": 0.0, "<": math.pi}


def load_layout(path):
    """Return (width, height, free).

    free maps (x, y) -> string of direction symbols allowed in that cell
    ("" for plain "0" cells), with y up from the bottom.
    """
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

    free = {}
    for (r, c), k in kinds.items():
        if c >= width or r >= height:
            continue  # outside the grid block
        if k == "free":
            # File row 0 is the top of the layout; y grows upward.
            text = rows[r][c].strip() if r < len(rows) and c < len(rows[r]) else ""
            dirs = "".join(ch for ch in text if ch in DIR_THETA)
            free[(c, height - 1 - r)] = dirs
    return width, height, free


def pick_lane_spawns(free, count):
    """Spawns for a layout with direction rules.

    Robots start only on lane cells (exactly one allowed direction), facing
    that direction, spread evenly over the whole layout.
    Returns [(x, y, theta)] or None if there are not enough lane cells.
    """
    lanes = sorted((p for p, d in free.items() if len(d) == 1),
                   key=lambda p: (-p[1], p[0]))  # top row first, like the file
    for gap in (3, 2, 1):
        spaced = []
        for p in lanes:
            if all(max(abs(p[0] - q[0]), abs(p[1] - q[1])) >= gap for q in spaced):
                spaced.append(p)
        if len(spaced) >= count:
            picks = [spaced[i * len(spaced) // count] for i in range(count)]
            return [(x, y, DIR_THETA[free[(x, y)]]) for (x, y) in picks]
    return None


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


def default_scene_dir(layout):
    """scenes/ beside the maps/ folder when the CSV is in maps/, else next to the CSV."""
    csv_dir = os.path.dirname(os.path.abspath(layout))
    if os.path.basename(csv_dir) == "maps":
        scene_dir = os.path.join(os.path.dirname(csv_dir), "scenes")
        os.makedirs(scene_dir, exist_ok=True)
        return scene_dir
    return csv_dir


def build_world(layout, count, scene_dir, name):
    """One world entry for the scene. Returns (world, width, height, n_free)."""
    width, height, free = load_layout(layout)
    if count < 1:
        raise ValueError(f"{os.path.basename(layout)}: number of robots must be at least 1")
    if count > len(free):
        raise ValueError(f"{os.path.basename(layout)}: layout has only {len(free)} "
                         f"movable cells, cannot place {count} robots")
    spawns = None
    if any(free.values()):  # layout has direction rules: stay on its lanes
        spawns = pick_lane_spawns(free, count)
        if spawns is None:
            raise ValueError(f"{os.path.basename(layout)}: fewer than {count} "
                             f"lane cells to start on")
    if spawns is None:
        spawns = [(x, y, 0.0) for (x, y) in pick_spawns(width, height, free, count)]

    map_rel = os.path.relpath(os.path.abspath(layout), scene_dir).replace(os.sep, "/")
    robots = [{
        "x": x + 0.5, "y": y + 0.5, "theta": round(theta, 4),
        "radius": RADIUS, "vmax": VMAX, "amax": AMAX, "omegamax": OMEGAMAX,
    } for (x, y, theta) in spawns]
    return {"name": name, "map": map_rel, "robots": robots}, width, height, len(free)


def write_scene(layouts, out_path=None):
    """Write one scene with a world per layout.

    layouts is a list of (csv_path, robot_count). Returns (out_path, info) where
    info is a list of (name, width, height, n_free, count), one per world.
    The scene goes in a scenes/ folder beside maps/ (or next to the first CSV)
    unless out_path is given. Map paths in the scene are relative to the scene.
    """
    if not layouts:
        raise ValueError("choose at least one layout")
    if out_path:
        out_path = os.path.abspath(out_path)
        scene_dir = os.path.dirname(out_path)
        os.makedirs(scene_dir, exist_ok=True)
    else:
        scene_dir = default_scene_dir(layouts[0][0])
        if len(layouts) == 1:
            stem = os.path.splitext(os.path.basename(layouts[0][0]))[0]
            out_path = os.path.join(scene_dir, f"{stem}_scene.json")
        else:
            out_path = os.path.join(scene_dir, f"multi_{len(layouts)}_maps_scene.json")

    worlds, info, used = [], [], {}
    for path, count in layouts:
        stem = os.path.splitext(os.path.basename(path))[0]
        n = used.get(stem, 0)
        used[stem] = n + 1
        name = stem if n == 0 else f"{stem}_{n + 1}"  # world names must be unique
        world, w, h, n_free = build_world(path, count, scene_dir, name)
        worlds.append(world)
        info.append((name, w, h, n_free, count))

    with open(out_path, "w") as f:
        json.dump({"dt": DT, "worlds": worlds}, f, indent=2)
        f.write("\n")
    return out_path, info


def describe(out_path, info):
    lines = [f"Wrote {out_path}"]
    for i, (name, w, h, n_free, count) in enumerate(info):
        lines.append(f"  world {i}: {name}  {w} x {h} cells, {n_free} movable, {count} robot(s)")
    lines.append(f"Run: ./parallelverse \"{out_path}\"")
    return "\n".join(lines)


def parse_map_arg(text):
    """'path.csv' or 'path.csv:20' -> (path, count or None)."""
    path, sep, tail = text.rpartition(":")
    if sep and tail.isdigit() and path:
        return os.path.expanduser(path), int(tail)
    return os.path.expanduser(text), None


def main_cli(map_args=None, out_path=None, default_count=None):
    layouts = []
    if map_args:
        for text in map_args:
            path, count = parse_map_arg(text)
            if not os.path.isfile(path):
                sys.exit(f"File not found: {path}")
            if count is None:
                count = default_count
            if count is None:
                sys.exit(f"No robot count for {path}: use PATH:COUNT or --robots N")
            layouts.append((path, count))
    else:
        print("Enter one layout CSV path per line. Empty line when done.")
        while True:
            path = input(f"Layout CSV path #{len(layouts) + 1}: ").strip().strip('"').strip("'")
            if not path:
                break
            path = os.path.expanduser(path)
            if not os.path.isfile(path):
                print(f"File not found: {path}")
                continue
            try:
                count = int(input("  Number of robots: ").strip())
            except ValueError:
                print("  Must be a whole number; layout skipped.")
                continue
            layouts.append((path, count))
    try:
        out, info = write_scene(layouts, out_path)
    except ValueError as e:
        sys.exit(str(e))
    print(describe(out, info))


try:
    try:
        from PyQt5.QtCore import Qt
        from PyQt5.QtWidgets import (QApplication, QFileDialog, QHBoxLayout,
                                     QHeaderView, QLabel, QLineEdit, QMainWindow,
                                     QPushButton, QSpinBox, QTableWidget,
                                     QTableWidgetItem, QVBoxLayout, QWidget)
        _QT_EXEC = "exec_"
    except ImportError:
        from PyQt6.QtCore import Qt
        from PyQt6.QtWidgets import (QApplication, QFileDialog, QHBoxLayout,
                                     QHeaderView, QLabel, QLineEdit, QMainWindow,
                                     QPushButton, QSpinBox, QTableWidget,
                                     QTableWidgetItem, QVBoxLayout, QWidget)
        _QT_EXEC = "exec"
    try:  # PyQt5 spelling
        _STRETCH, _EDITABLE = QHeaderView.Stretch, Qt.ItemIsEditable
    except AttributeError:  # PyQt6 spelling
        _STRETCH = QHeaderView.ResizeMode.Stretch
        _EDITABLE = Qt.ItemFlag.ItemIsEditable
    HAVE_QT = True
except ImportError:
    HAVE_QT = False


if HAVE_QT:
    class MakeSceneWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("make_scene")
            self.resize(720, 460)

            add = QPushButton("Add layouts...")
            add.clicked.connect(self.add_layouts)
            remove = QPushButton("Remove selected")
            remove.clicked.connect(self.remove_selected)
            self.default_robots = QSpinBox()
            self.default_robots.setRange(1, 100000)
            self.default_robots.setValue(3)
            top = QHBoxLayout()
            top.addWidget(add)
            top.addWidget(remove)
            top.addStretch(1)
            top.addWidget(QLabel("Robots for new layouts:"))
            top.addWidget(self.default_robots)

            self.table = QTableWidget(0, 2)
            self.table.setHorizontalHeaderLabels(["Layout CSV (one world each)", "Robots"])
            self.table.horizontalHeader().setStretchLastSection(False)
            self.table.horizontalHeader().setSectionResizeMode(0, _STRETCH)
            self.table.setColumnWidth(1, 110)

            self.out_name = QLineEdit()
            self.out_name.setPlaceholderText("scene file name (optional, e.g. fleet.json)")

            generate = QPushButton("Generate scene")
            generate.clicked.connect(self.generate)

            self.status = QLabel("")
            self.status.setWordWrap(True)

            root = QVBoxLayout()
            root.addLayout(top)
            root.addWidget(self.table, 1)
            root.addWidget(self.out_name)
            root.addWidget(generate)
            root.addWidget(self.status)

            central = QWidget()
            central.setLayout(root)
            self.setCentralWidget(central)

        def add_layouts(self):
            paths, _ = QFileDialog.getOpenFileNames(
                self, "Choose layout CSVs (select several)", "",
                "CSV files (*.csv);;All files (*)")
            for path in paths:
                row = self.table.rowCount()
                self.table.insertRow(row)
                item = QTableWidgetItem(path)
                item.setFlags(item.flags() & ~_EDITABLE)
                self.table.setItem(row, 0, item)
                spin = QSpinBox()
                spin.setRange(1, 100000)  # checked against the layout on generate
                spin.setValue(self.default_robots.value())
                self.table.setCellWidget(row, 1, spin)
            if paths:
                self.status.setText("")

        def remove_selected(self):
            rows = sorted({i.row() for i in self.table.selectedIndexes()}, reverse=True)
            for r in rows:
                self.table.removeRow(r)

        def generate(self):
            layouts = []
            for r in range(self.table.rowCount()):
                layouts.append((self.table.item(r, 0).text(),
                                self.table.cellWidget(r, 1).value()))
            if not layouts:
                self.status.setText("Add at least one layout CSV first.")
                return
            out_path = None
            name = self.out_name.text().strip()
            if name:
                if not name.endswith(".json"):
                    name += ".json"
                out_path = os.path.join(default_scene_dir(layouts[0][0]), name)
            try:
                out, info = write_scene(layouts, out_path)
            except ValueError as e:
                self.status.setText(f"Could not make the scene: {e}")
                return
            self.status.setText(describe(out, info))


def main():
    parser = argparse.ArgumentParser(
        description="Make a sim2d scene (one world per layout CSV).")
    parser.add_argument("--cli", action="store_true",
                        help="use terminal prompts instead of the window")
    parser.add_argument("--map", action="append", metavar="PATH[:COUNT]",
                        help="a layout CSV with its robot count; repeat for several "
                             "maps (no prompts)")
    parser.add_argument("--robots", type=int, metavar="N",
                        help="robot count for --map entries without :COUNT")
    parser.add_argument("--out", metavar="FILE", help="scene file to write")
    args = parser.parse_args()
    if args.map or args.cli or not HAVE_QT:
        if not (args.map or args.cli):
            print("PyQt5/PyQt6 not found; using terminal prompts. "
                  "Install PyQt5 for the window.")
        main_cli(args.map, args.out, args.robots)
        return
    app = QApplication(sys.argv)
    win = MakeSceneWindow()
    win.show()
    sys.exit(getattr(app, _QT_EXEC)())


if __name__ == "__main__":
    main()

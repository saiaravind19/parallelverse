#!/usr/bin/env python3
"""Generate a sim2d scene JSON for one layout CSV.

    python3 make_scene.py          opens a window (needs PyQt5 or PyQt6)
    python3 make_scene.py --cli    asks for the layout path and robot count

Either way it writes <layout_name>_scene.json next to the CSV file.

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


def write_scene(layout, count):
    """Write the scene JSON and return (out_path, width, height, n_free).

    The scene goes in a scenes/ folder beside the maps/ folder when the CSV is
    in maps/ (the layout used by the sample scenes). Otherwise it goes next to
    the CSV. The map path in the scene is relative to the scene file.
    """
    width, height, free = load_layout(layout)
    if count < 1:
        raise ValueError("number of robots must be at least 1")
    if count > len(free):
        raise ValueError(f"layout has only {len(free)} movable cells, "
                         f"cannot place {count} robots")
    spawns = None
    if any(free.values()):  # layout has direction rules: stay on its lanes
        spawns = pick_lane_spawns(free, count)
        if spawns is None:
            raise ValueError(f"layout has fewer than {count} lane cells to start on")
    if spawns is None:
        spawns = [(x, y, 0.0) for (x, y) in pick_spawns(width, height, free, count)]

    csv_abs = os.path.abspath(layout)
    csv_dir = os.path.dirname(csv_abs)
    stem = os.path.splitext(os.path.basename(csv_abs))[0]
    if os.path.basename(csv_dir) == "maps":
        scene_dir = os.path.join(os.path.dirname(csv_dir), "scenes")
        os.makedirs(scene_dir, exist_ok=True)
    else:
        scene_dir = csv_dir
    map_rel = os.path.relpath(csv_abs, scene_dir).replace(os.sep, "/")

    robots = [{
        "x": x + 0.5, "y": y + 0.5, "theta": round(theta, 4),
        "radius": RADIUS, "vmax": VMAX, "amax": AMAX, "omegamax": OMEGAMAX,
    } for (x, y, theta) in spawns]

    scene = {
        "dt": DT,
        "worlds": [{"name": stem, "map": map_rel, "robots": robots}],
    }

    out_path = os.path.join(scene_dir, f"{stem}_scene.json")
    with open(out_path, "w") as f:
        json.dump(scene, f, indent=2)
        f.write("\n")
    return out_path, width, height, len(free)


def main_cli():
    layout = input("Layout CSV path: ").strip().strip('"').strip("'")
    layout = os.path.expanduser(layout)
    if not os.path.isfile(layout):
        sys.exit(f"File not found: {layout}")
    try:
        count = int(input("Number of robots: ").strip())
    except ValueError:
        sys.exit("Number of robots must be a whole number.")

    try:
        out_path, width, height, n_free = write_scene(layout, count)
    except ValueError as e:
        sys.exit(str(e))
    print(f"Wrote {out_path}")
    print(f"Map {width} x {height} cells, {n_free} drivable, {count} robot(s).")
    print(f"Run: ./sim2d_app \"{out_path}\"")


try:
    try:
        from PyQt5.QtWidgets import (QApplication, QFileDialog, QFormLayout,
                                     QLabel, QMainWindow, QPushButton,
                                     QSpinBox, QVBoxLayout, QWidget)
        _QT_EXEC = "exec_"
    except ImportError:
        from PyQt6.QtWidgets import (QApplication, QFileDialog, QFormLayout,
                                     QLabel, QMainWindow, QPushButton,
                                     QSpinBox, QVBoxLayout, QWidget)
        _QT_EXEC = "exec"
    HAVE_QT = True
except ImportError:
    HAVE_QT = False


if HAVE_QT:
    class MakeSceneWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("make_scene")
            self.resize(560, 220)
            self.layout_path = ""

            self.path_label = QLabel("No layout chosen")
            self.path_label.setWordWrap(True)
            browse = QPushButton("Choose layout CSV...")
            browse.clicked.connect(self.choose_layout)

            self.robots = QSpinBox()
            self.robots.setRange(1, 100000)  # checked against the layout on Create
            self.robots.setValue(3)

            generate = QPushButton("Generate scene")
            generate.clicked.connect(self.generate)

            self.status = QLabel("")
            self.status.setWordWrap(True)

            form = QFormLayout()
            form.addRow("Layout:", browse)
            form.addRow("", self.path_label)
            form.addRow("Robots:", self.robots)

            root = QVBoxLayout()
            root.addLayout(form)
            root.addWidget(generate)
            root.addWidget(self.status)
            root.addStretch(1)

            central = QWidget()
            central.setLayout(root)
            self.setCentralWidget(central)

        def choose_layout(self):
            path, _ = QFileDialog.getOpenFileName(
                self, "Choose layout CSV", "", "CSV files (*.csv);;All files (*)")
            if path:
                self.layout_path = path
                self.path_label.setText(path)
                self.status.setText("")

        def generate(self):
            if not self.layout_path or not os.path.isfile(self.layout_path):
                self.status.setText("Choose a layout CSV first.")
                return
            try:
                out, w, h, n_free = write_scene(self.layout_path, self.robots.value())
            except ValueError as e:
                self.status.setText(f"Could not make the scene: {e}")
                return
            self.status.setText(
                f"Wrote {out}\nMap {w} x {h} cells, {n_free} drivable, "
                f"{self.robots.value()} robot(s).\n"
                f"Run: ./sim2d_app \"{out}\"")


def main():
    parser = argparse.ArgumentParser(description="Make a sim2d scene from a layout CSV.")
    parser.add_argument("--cli", action="store_true",
                        help="use terminal prompts instead of the window")
    args = parser.parse_args()
    if args.cli or not HAVE_QT:
        if not args.cli:
            print("PyQt5/PyQt6 not found; using terminal prompts. "
                  "Install PyQt5 for the window.")
        main_cli()
        return
    app = QApplication(sys.argv)
    win = MakeSceneWindow()
    win.show()
    sys.exit(getattr(app, _QT_EXEC)())


if __name__ == "__main__":
    main()

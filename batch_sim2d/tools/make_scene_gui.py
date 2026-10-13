#!/usr/bin/env python3
"""PyQt window for make_scene: pick a layout CSV and a robot count, then
write <layout>_scene.json next to the CSV.

Needs PyQt5 or PyQt6 (pip install PyQt5) and make_scene.py in the same folder.
"""
import os
import sys

from make_scene import MAX_ROBOTS, write_scene

try:
    from PyQt5.QtWidgets import (QApplication, QFileDialog, QHBoxLayout,
                                 QLabel, QMainWindow, QPushButton, QSpinBox,
                                 QVBoxLayout, QWidget, QFormLayout)
    from PyQt5.QtCore import Qt
except ImportError:  # PyQt6 fallback
    from PyQt6.QtWidgets import (QApplication, QFileDialog, QHBoxLayout,
                                 QLabel, QMainWindow, QPushButton, QSpinBox,
                                 QVBoxLayout, QWidget, QFormLayout)
    from PyQt6.QtCore import Qt


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
        self.robots.setRange(1, MAX_ROBOTS)
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
    app = QApplication(sys.argv)
    win = MakeSceneWindow()
    win.show()
    sys.exit(app.exec() if hasattr(app, "exec") else app.exec_())


if __name__ == "__main__":
    main()

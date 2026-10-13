#!/usr/bin/env python3
"""Publish waypoints to a running sim2d_app over ZeroMQ.

The app's command socket is a SUB bound to tcp://*:5556 that listens for
topic "cmd". This script is a PUB that connects to it and sends one message:
    frame 1: b"cmd"
    frame 2: {"world": w, "robot": r, "waypoints": [[x, y], ...]}

Usage:
    pip install pyzmq
    python3 publish_waypoints.py                  # prompts for everything
    python3 publish_waypoints.py 0 1 "2.5,3.5 8.5,3.5 8.5,8.5"
    python3 publish_waypoints.py --host 192.168.1.20 0 1 "2.5,3.5"

Waypoints are "x,y" pairs in meters, separated by spaces. An empty list
clears the robot's waypoints.
"""
import argparse
import json
import sys
import time

import zmq


def parse_waypoints(text):
    pts = []
    for token in text.replace(";", " ").split():
        parts = token.split(",")
        if len(parts) != 2:
            raise ValueError(f"'{token}' is not an x,y pair; put a comma between x and y, "
                             "and a space between points")
        pts.append([float(parts[0]), float(parts[1])])
    if len(pts) > 64:  # default SIM2D_MAX_WAYPOINTS; the app warns if it is lower
        raise ValueError("at most 64 waypoints per robot (default build)")
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", nargs="?", type=int)
    ap.add_argument("robot", nargs="?", type=int)
    ap.add_argument("waypoints", nargs="?")
    ap.add_argument("--host", default="localhost", help="machine running sim2d_app")
    ap.add_argument("--port", type=int, default=5556)
    args = ap.parse_args()

    world = args.world if args.world is not None else int(input("World index (0 = first tab): "))
    robot = args.robot if args.robot is not None else int(input("Robot index (0 = first robot): "))
    text = args.waypoints if args.waypoints is not None else input(
        "Waypoints as x,y pairs separated by spaces (blank clears): ")

    try:
        waypoints = parse_waypoints(text)
    except ValueError as e:
        sys.exit(f"Could not read waypoints ({e}). Use the form 2.5,3.5 8.5,3.5")

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.connect(f"tcp://{args.host}:{args.port}")
    time.sleep(0.5)  # give the connection time to set up before sending

    msg = {"world": world, "robot": robot, "waypoints": waypoints}
    pub.send_multipart([b"cmd", json.dumps(msg).encode()])
    pub.close(linger=500)
    ctx.term()
    print(f"Sent to world {world}, robot {robot}: {waypoints}")


if __name__ == "__main__":
    main()

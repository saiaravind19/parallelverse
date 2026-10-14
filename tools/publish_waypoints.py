#!/usr/bin/env python3
"""Send a path to one robot in a running parallelverse (protobuf over ZeroMQ).

The app listens on /cmd_path for sim2d_msgs.PathCommand messages
(proto/sim2d_msgs.proto). This script connects a PUB socket and sends one.

Usage:
    uv run publish_waypoints.py                  # asks for everything
    uv run publish_waypoints.py 0 1 "2.5,3.5 8.5,3.5"
    uv run publish_waypoints.py --host 192.168.1.20 0 1 "2.5,3.5"

World and robot are 0-based indexes. Waypoints are "x,y" pairs in metres,
separated by spaces. An empty list clears the robot's path.
"""
import argparse
import sys
import time

import zmq

from sim2d_msgs_pb2 import PathCommand

MAX_WAYPOINTS = 64  # default SIM2D_MAX_WAYPOINTS; the app warns if it is lower


def parse_points(text):
    pts = []
    for token in text.replace(";", " ").split():
        parts = token.split(",")
        if len(parts) != 2:
            raise ValueError(f"'{token}' is not an x,y pair; put a comma between "
                             "x and y, and a space between points")
        pts.append((float(parts[0]), float(parts[1])))
    if len(pts) > MAX_WAYPOINTS:
        raise ValueError(f"at most {MAX_WAYPOINTS} waypoints per robot (default build)")
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", nargs="?", type=int)
    ap.add_argument("robot", nargs="?", type=int)
    ap.add_argument("waypoints", nargs="?")
    ap.add_argument("--host", default="localhost", help="machine running parallelverse")
    ap.add_argument("--port", type=int, default=5556)
    args = ap.parse_args()

    world = args.world if args.world is not None else int(input("World index (0 = first tab): "))
    robot = args.robot if args.robot is not None else int(input("Robot index (0 = first robot): "))
    text = args.waypoints if args.waypoints is not None else input(
        "Waypoints as x,y pairs separated by spaces (blank clears): ")
    try:
        points = parse_points(text)
    except ValueError as e:
        sys.exit(f"Could not read waypoints: {e}")

    cmd = PathCommand()
    cmd.header.frame_id = "map"
    cmd.world_id = world
    cmd.robot_id = robot
    cmd.path.header.frame_id = "map"
    for x, y in points:
        p = cmd.path.poses.add()
        p.x, p.y, p.theta = x, y, 0.0

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.connect(f"tcp://{args.host}:{args.port}")
    time.sleep(0.5)  # let the connection set up before sending
    pub.send_multipart([b"/cmd_path", cmd.SerializeToString()])
    pub.close(linger=500)
    ctx.term()
    print(f"Sent to world {world}, robot {robot}: {points}")


if __name__ == "__main__":
    main()

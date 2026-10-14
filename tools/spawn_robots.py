#!/usr/bin/env python3
"""Add robots to a running parallelverse (protobuf over ZeroMQ, topic /cmd_spawn).

Usage:
    uv run spawn_robots.py <world> --count 20            # placed automatically
    uv run spawn_robots.py <world> --at 4.5,10.5 --theta 1.57   # one robot at a spot

<world> is the 0-based world index. New robots get the next free ids and show up
in the world's state messages. The app refuses a spot that is inside a wall or
overlaps a robot, and a world that is already full (see --max-robots).
"""
import argparse
import time

import zmq

from sim2d_msgs_pb2 import SpawnCommand


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", type=int)
    ap.add_argument("--count", type=int, default=0, help="place this many automatically")
    ap.add_argument("--at", help="x,y of one robot (metres)")
    ap.add_argument("--theta", type=float, default=0.0)
    ap.add_argument("--radius", type=float, default=0.0, help="0 = default")
    ap.add_argument("--vmax", type=float, default=0.0, help="0 = default")
    ap.add_argument("--amax", type=float, default=0.0, help="0 = default")
    ap.add_argument("--omegamax", type=float, default=0.0, help="0 = default")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=5556)
    args = ap.parse_args()
    if not args.at and args.count < 1:
        ap.error("give --count N or --at x,y")

    cmd = SpawnCommand()
    cmd.header.frame_id = "map"
    cmd.world_id = args.world
    cmd.radius, cmd.vmax, cmd.amax, cmd.omegamax = args.radius, args.vmax, args.amax, args.omegamax
    if args.at:
        x, y = (float(v) for v in args.at.split(","))
        cmd.pose.x, cmd.pose.y, cmd.pose.theta = x, y, args.theta
    else:
        cmd.count = args.count

    pub = zmq.Context().socket(zmq.PUB)
    pub.connect(f"tcp://{args.host}:{args.port}")
    time.sleep(0.3)
    pub.send_multipart([b"/cmd_spawn", cmd.SerializeToString()])
    time.sleep(0.2)
    print("Sent. The sim's console shows whether the robots were added.")


if __name__ == "__main__":
    main()

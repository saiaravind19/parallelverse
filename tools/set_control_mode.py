#!/usr/bin/env python3
"""Read or change how a robot takes commands (ZeroMQ request/reply service).

The app answers sim2d_msgs.ControlModeRequest on tcp://<host>:5557 with a
ControlModeResponse that holds the robot's mode after the request, so the reply
confirms the change.

Usage:
    uv run set_control_mode.py <world> <robot> get
    uv run set_control_mode.py <world> <robot> waypoint
    uv run set_control_mode.py <world> <robot> velocity

<world> is the world's name (the tab title) or its index. A robot starts in
waypoint mode.
"""
import argparse
import sys

import zmq

from sim2d_msgs_pb2 import (CONTROL_MODE_UNSPECIFIED, CONTROL_MODE_VELOCITY,
                            CONTROL_MODE_WAYPOINT, ControlModeRequest,
                            ControlModeResponse)

MODES = {"get": CONTROL_MODE_UNSPECIFIED,
         "waypoint": CONTROL_MODE_WAYPOINT,
         "velocity": CONTROL_MODE_VELOCITY}
NAMES = {v: k for k, v in MODES.items()}
NAMES[CONTROL_MODE_UNSPECIFIED] = "unknown"


def request_mode(world, robot, mode, host="localhost", port=5557, timeout_ms=3000):
    """Send one request and return the ControlModeResponse (or raise on timeout)."""
    req = ControlModeRequest()
    if isinstance(world, str) and not world.isdigit():
        req.world_name = world
    else:
        req.world_id = int(world)
    req.robot_id = robot
    req.mode = mode

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
    sock.connect(f"tcp://{host}:{port}")
    try:
        sock.send(req.SerializeToString())
        resp = ControlModeResponse()
        resp.ParseFromString(sock.recv())
        return resp
    finally:
        sock.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", help="world name or index")
    ap.add_argument("robot", type=int)
    ap.add_argument("mode", choices=sorted(MODES))
    ap.add_argument("--host", default="localhost", help="machine running sim2d_app")
    ap.add_argument("--port", type=int, default=5557)
    args = ap.parse_args()

    try:
        resp = request_mode(args.world, args.robot, MODES[args.mode], args.host, args.port)
    except zmq.Again:
        sys.exit("No reply from the sim. Is sim2d_app running (and is the host/port right)?")

    where = f"world '{resp.world_name}' ({resp.world_id}) robot {resp.robot_id}" \
        if resp.world_name else f"world {args.world} robot {args.robot}"
    status = "ok" if resp.success else "FAILED"
    print(f"{status}: {where}: mode = {NAMES.get(resp.mode, resp.mode)} ({resp.message})")
    sys.exit(0 if resp.success else 1)


if __name__ == "__main__":
    main()

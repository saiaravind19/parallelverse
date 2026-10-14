#!/usr/bin/env python3
"""Drive one robot with a velocity command (protobuf over ZeroMQ).

The app listens on /cmd_vel for sim2d_msgs.VelocityCommand. It is only used
while the robot is in velocity mode (see set_control_mode.py).

Usage:
    uv run publish_velocity.py <world> <robot> <linear m/s> <angular rad/s>
    uv run publish_velocity.py 0 3 1.0 0.0 --hold 5     # repeat for 5 seconds, then stop
    uv run publish_velocity.py 0 3 0 0                  # stop

<world> and <robot> are 0-based indexes. Without --hold the command is sent once
and stays in effect until the next one.
"""
import argparse
import time

import zmq

from sim2d_msgs_pb2 import VelocityCommand


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", type=int)
    ap.add_argument("robot", type=int)
    ap.add_argument("linear", type=float)
    ap.add_argument("angular", type=float)
    ap.add_argument("--hold", type=float, default=0.0,
                    help="keep sending for this many seconds, then send a stop")
    ap.add_argument("--rate", type=float, default=10.0, help="messages per second with --hold")
    ap.add_argument("--host", default="localhost", help="machine running sim2d_app")
    ap.add_argument("--port", type=int, default=5556)
    args = ap.parse_args()

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.connect(f"tcp://{args.host}:{args.port}")
    time.sleep(0.3)  # let the connection come up so the first message is not dropped

    def send(linear, angular):
        cmd = VelocityCommand()
        cmd.header.frame_id = "base_link"
        cmd.world_id = args.world
        cmd.robot_id = args.robot
        cmd.linear = linear
        cmd.angular = angular
        pub.send_multipart([b"/cmd_vel", cmd.SerializeToString()])

    send(args.linear, args.angular)
    print(f"Sent to world {args.world}, robot {args.robot}: "
          f"linear={args.linear} m/s, angular={args.angular} rad/s")
    if args.hold > 0:
        end = time.time() + args.hold
        while time.time() < end:
            time.sleep(1.0 / args.rate)
            send(args.linear, args.angular)
        send(0.0, 0.0)
        print("Held, then sent a stop.")
    time.sleep(0.2)


if __name__ == "__main__":
    main()

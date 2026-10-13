#!/usr/bin/env bash
# Regenerate the message code after editing proto/sim2d_msgs.proto.
#   C code for the simulator (nanopb)  -> gen/sim2d_msgs.pb.c/.h
#   Python module for the tools        -> tools/sim2d_msgs_pb2.py
# Needs: protoc, and nanopb's generator (https://github.com/nanopb/nanopb, run as
# NANOPB_GEN=/path/to/nanopb/generator/nanopb_generator.py). The vendored runtime
# in third_party/nanopb is nanopb 0.4.8; use the same version for the generator.
set -euo pipefail
cd "$(dirname "$0")/.."
: "${NANOPB_GEN:?set NANOPB_GEN to nanopb_generator.py (nanopb 0.4.8)}"
python3 "$NANOPB_GEN" -I proto -D gen proto/sim2d_msgs.proto
protoc --proto_path=proto --python_out=tools proto/sim2d_msgs.proto

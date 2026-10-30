#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Protocol corruption fixture. Never imports a device SDK."""

import argparse
import json
import os
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--sdk-root")
parser.add_argument("--channel")
arguments = parser.parse_args()
mode = arguments.channel.split("_")[-1]
os.set_blocking(0, True)
request = json.loads(sys.stdin.buffer.readline())
response = {"v": 1, "session": request["session"], "id": request["id"], "ok": True, "data": {}}
if mode == "id":
    response["id"] += 1
elif mode == "ver":
    response["v"] = 2
elif mode == "time":
    time.sleep(10)
elif mode == "eof":
    sys.exit(0)
elif mode == "large":
    sys.stdout.write("x" * 33000)
    sys.stdout.flush()
    sys.exit(0)
elif mode == "half":
    sys.stdout.write('{"v":')
    sys.stdout.flush()
    sys.exit(0)
sys.stdout.write(json.dumps(response) + "\n")
sys.stdout.flush()

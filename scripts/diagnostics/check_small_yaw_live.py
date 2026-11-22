#!/usr/bin/env python3
"""Check the browser status, optionally performing one bounded small-yaw jog."""

import argparse
import json
import math
import time
from urllib.request import Request, urlopen
from uuid import uuid4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8088")
    parser.add_argument("--jog", action="store_true", help="Request at most about 0.17 degrees at 20%% speed")
    parser.add_argument("--direction", choices=("j", "l"), default="j")
    args = parser.parse_args()

    def api(path, payload=None):
        data = None if payload is None else json.dumps(payload).encode()
        request = Request(args.url.rstrip("/") + path, data=data,
                          headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=0.15) as response:
            result = json.load(response)
        if not result.get("ok"):
            raise RuntimeError(str(result))
        return result["state"]

    state = api("/api/state")
    if not args.jog:
        print(json.dumps(state))
        return 0
    if state["active_client"] is not None or state["armed"] or state["small_yaw_armed"]:
        raise RuntimeError("Another page owns control; refusing to take it over")
    if state["small_yaw_online"] is not True:
        raise RuntimeError("Small yaw does not have fresh online feedback")
    if args.direction == "j" and state.get("small_yaw_left_limit") is not False:
        raise RuntimeError("Leftward jog is blocked by the left limit or unknown limit feedback")
    if args.direction == "l" and state.get("small_yaw_right_limit") is not False:
        raise RuntimeError("Rightward jog is blocked by the right limit or unknown limit feedback")
    jog_rate = float(state["small_yaw_rate"]) * 0.2
    if not math.isfinite(jog_rate) or not 0.0 < jog_rate <= 0.12:
        raise RuntimeError("Unexpected small-yaw speed; refusing the bounded jog")
    jog_duration = min(0.1, math.radians(0.17) / jog_rate)

    client = "small-yaw-check-" + uuid4().hex
    sequence = 0
    claimed = False

    def control(keys):
        nonlocal sequence
        sequence += 1
        result = api("/api/control", {
            "client": client, "sequence": sequence, "armed": False,
            "small_yaw_armed": True, "spin_enabled": False,
            "keys": keys, "speed_scale": 0.2,
        })
        if result["armed"] or result["spin_enabled"]:
            raise RuntimeError("Unexpected chassis/spin enable")
        if not result["small_yaw_armed"] or result["small_yaw_online"] is not True:
            raise RuntimeError("Small-yaw control or feedback was lost")
        return result

    try:
        api("/api/claim", {"client": client})
        claimed = True
        started = time.monotonic()
        deadline = started + jog_duration
        for _ in range(2):
            if time.monotonic() >= deadline:
                break
            control([args.direction])
            time.sleep(min(jog_duration * 0.45, max(0.0, deadline - time.monotonic())))
        control([])
        jog_elapsed = time.monotonic() - started
    finally:
        if claimed:
            api("/api/release", {"client": client})

    final = api("/api/state")
    if final["armed"] or final["small_yaw_armed"] or final["spin_enabled"]:
        raise RuntimeError("Control did not disarm after release")
    print(json.dumps({"direction": args.direction,
                      "requested_rate_rad_s": jog_rate if args.direction == "j" else -jog_rate,
                      "jog_elapsed_s": jog_elapsed, "released": True,
                      "chassis_enabled": False, "lower_spin_enabled": False,
                      "small_yaw_online": final["small_yaw_online"],
                      "small_yaw_left_limit": final.get("small_yaw_left_limit"),
                      "small_yaw_right_limit": final.get("small_yaw_right_limit")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

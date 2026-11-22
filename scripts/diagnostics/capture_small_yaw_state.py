#!/usr/bin/env python3
"""Record read-only browser state alongside the higher-rate RTT motion trace.

Only GET /api/state is used. HTTP snapshots expose key/enable/limit/output
history, but their sampling rate cannot resolve a 20 Hz mechanical vibration;
use the 100 Hz RTT angle trace for that analysis. Missing values are written
as ``null`` so they remain distinct from false, zero and an empty key set.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time
from urllib.parse import urlparse
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[2]
STATE_FIELDS = (
    "small_yaw_mcu_time_ms", "active_client", "keys", "armed",
    "small_yaw_armed", "small_yaw_controller_active", "small_yaw_online",
    "small_yaw_left_limit", "small_yaw_right_limit", "small_yaw_ecd",
    "small_yaw_actual_deg", "small_yaw_target_deg", "small_yaw_error_deg",
    "small_yaw_speed_deg_s", "small_yaw_output", "small_yaw_feedback_age_s",
    "small_yaw_diagnostic_age_s", "small_yaw_limit_feedback_age_s",
    "small_yaw_left_limit_feedback_age_s", "heartbeat_age_s", "speed_scale",
    "small_yaw_rate", "chassis_ready", "chassis_control_enabled",
    "chassis_block_reason",
)
CSV_FIELDS = (
    "host_utc", "host_monotonic_s", "host_elapsed_s",
    "host_request_monotonic_s", "request_duration_ms", "sample_ok",
    "error", "skipped_schedule_slots",
) + STATE_FIELDS


def read_state(url):
    request = Request(url, method="GET", headers={"Cache-Control": "no-cache"})
    with urlopen(request, timeout=0.15) as response:
        payload = json.load(response)
    if not isinstance(payload, dict) or payload.get("ok") is not True:
        raise ValueError("API did not return an ok state response")
    state = payload.get("state")
    if not isinstance(state, dict):
        raise ValueError("API state is not an object")
    values = {name: state.get(name) for name in STATE_FIELDS}
    for name, value in values.items():
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f"Non-finite API field: {name}")
    keys = values["keys"]
    if keys is not None:
        if not isinstance(keys, list) or any(not isinstance(key, str) for key in keys):
            raise ValueError("API keys field is not a string array")
        values["keys"] = "|".join(keys)
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8088")
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--rate", type=float, default=20.0, help="HTTP samples/s (1..50)")
    parser.add_argument("--output", type=Path, help="New CSV path; defaults to timestamped monitor file")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0.05 <= args.seconds <= 3600.0:
        parser.error("--seconds must be between 0.05 and 3600")
    if not math.isfinite(args.rate) or not 1.0 <= args.rate <= 50.0:
        parser.error("--rate must be between 1 and 50")
    parsed = urlparse(args.url)
    if parsed.scheme not in ("http", "https") or not parsed.netloc or parsed.query or parsed.fragment:
        parser.error("--url must be an HTTP(S) base URL without a query or fragment")
    url = args.url.rstrip("/") + "/api/state"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output = args.output or Path("monitor") / f"small_yaw_state_{stamp}.csv"
    if not output.is_absolute():
        output = ROOT / output
    output.parent.mkdir(parents=True, exist_ok=True)

    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    previous_handler = signal.signal(signal.SIGINT, request_stop)
    records = successful = errors = skipped = 0
    started = time.monotonic()
    deadline = started + args.seconds
    next_sample = started
    period = 1.0 / args.rate
    try:
        # Exclusive creation avoids replacing an earlier diagnostic capture.
        with output.open("x", encoding="utf-8", newline="", buffering=1) as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            print(json.dumps({"event": "capture_started", "path": str(output),
                              "url": url, "rate_hz": args.rate, "seconds": args.seconds,
                              "note": "GET only; use 100 Hz RTT angles to analyse vibration."}), flush=True)
            while not stop_requested and time.monotonic() < deadline:
                wait = min(next_sample, deadline) - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                if stop_requested or time.monotonic() >= deadline:
                    break
                requested = time.monotonic()
                row = {name: None for name in STATE_FIELDS}
                error = ""
                try:
                    row.update(read_state(url))
                    successful += 1
                except (OSError, ValueError) as exc:
                    error = f"{type(exc).__name__}: {exc}".replace("\n", " ")
                    errors += 1
                received = time.monotonic()
                next_sample += period
                if next_sample < received:
                    missed = math.floor((received - next_sample) / period) + 1
                    skipped += missed
                    next_sample += missed * period
                row.update({
                    "host_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                    "host_monotonic_s": received,
                    "host_elapsed_s": received - started,
                    "host_request_monotonic_s": requested,
                    "request_duration_ms": (received - requested) * 1000.0,
                    "sample_ok": not error, "error": error,
                    "skipped_schedule_slots": skipped,
                })
                writer.writerow({name: "null" if value is None else value
                                 for name, value in row.items()})
                records += 1
    finally:
        signal.signal(signal.SIGINT, previous_handler)
    print(json.dumps({"event": "capture_finished", "path": str(output),
                      "records": records, "successful": successful, "errors": errors,
                      "skipped_schedule_slots": skipped,
                      "elapsed_s": time.monotonic() - started,
                      "interrupted": stop_requested}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

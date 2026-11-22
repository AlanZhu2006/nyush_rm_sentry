#!/usr/bin/env python3
"""Read recent RTT monitor records without opening a probe or commanding motors."""

import argparse
from collections import Counter, deque
import csv
import json
import math
from pathlib import Path
import statistics
import time


def summarize_small_yaw(rows):
    """Summarize mapped yaw feedback without inferring a control-loop cause."""
    online_count = 0
    samples = []
    for index, row in enumerate(rows):
        if not int(row.get("gimbal_motor_online_bitmap", "0")) & 0x01:
            continue
        online_count += 1
        actual = float(row["gimbal_yaw_actual_deg"])
        target = float(row["gimbal_yaw_target_deg"])
        speed = float(row["gimbal_yaw_actual_deg_s"])
        if all(math.isfinite(value) for value in (actual, target, speed)):
            samples.append((index, int(row["timestamp_ms"]), actual, target, speed))

    result = {"online_frames": online_count, "valid_feedback_frames": len(samples)}
    if not online_count:
        return {
            **result,
            "motion_observed": None,
            "assessment": "Small yaw is not online or this older export lacks standalone-yaw mapping; cannot assess prior J/L motion.",
        }
    if not samples:
        return {
            **result,
            "motion_observed": None,
            "assessment": "Mapped online flags are present without finite yaw measurements; cannot assess motion.",
        }

    actual = [sample[2] for sample in samples]
    target = [sample[3] for sample in samples]
    speed = [sample[4] for sample in samples]
    target_moving_frames = 0
    valid_intervals = 0
    for previous, current in zip(samples, samples[1:]):
        elapsed_ms = (current[1] - previous[1]) & 0xFFFFFFFF
        # Exclude resets, long gaps, and transitions across offline records.
        if current[0] != previous[0] + 1 or not 0 < elapsed_ms <= 250:
            continue
        valid_intervals += 1
        if abs(current[3] - previous[3]) * 1000 / elapsed_ms > 0.5:
            target_moving_frames += 1
    motion_observed = len(samples) > 10 and max(actual) - min(actual) > 0.1
    if len(samples) <= 10:
        assessment = "Too few mapped online samples to assess motion."
    elif not motion_observed:
        assessment = "Mapped small-yaw feedback is present, but this window is static; cannot assess motion pauses."
    else:
        assessment = "Motion is present; summary statistics alone do not establish pause causes or whether PID needs adjustment."
    return {
        **result,
        "actual_angle_deg_min_max": [min(actual), max(actual)],
        "target_angle_deg_min_max": [min(target), max(target)],
        "max_abs_angle_error_deg": max(abs(sample[3] - sample[2]) for sample in samples),
        "actual_speed_deg_s_min_max": [min(speed), max(speed)],
        "valid_target_intervals": valid_intervals,
        "moving_target_frames": target_moving_frames,
        "motion_observed": motion_observed,
        "assessment": assessment,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--frames", type=int, default=1000)
    args = parser.parse_args()
    if not 2 <= args.frames <= 10000:
        parser.error("--frames must be between 2 and 10000")
    with args.path.open(encoding="utf-8") as stream:
        rows = deque((row for row in csv.DictReader(stream, delimiter="\t")
                      if row and None not in row and all(value is not None for value in row.values())),
                     maxlen=args.frames)
    if len(rows) < 2:
        raise RuntimeError("Not enough complete monitor records")

    intervals = [(int(current["timestamp_ms"]) - int(previous["timestamp_ms"])) & 0xFFFFFFFF
                 for previous, current in zip(rows, list(rows)[1:])]
    sequences = [(int(current["seq"]) - int(previous["seq"])) & 0xFFFFFFFF
                 for previous, current in zip(rows, list(rows)[1:])]
    last = rows[-1]
    columns = ("swerve_steer_online_bitmap", "swerve_drive_online_bitmap", "can_link_bitmap")
    print(json.dumps({
        "path": str(args.path.resolve()), "frames": len(rows),
        "last_host_age_s": round(time.time() - int(last["host_rx_ms"]) / 1000, 3),
        "frame_rate_hz": round(1000 / statistics.mean(intervals), 2) if sum(intervals) else None,
        "interval_ms_min_max": [min(intervals), max(intervals)],
        "sequence_missing": sum(delta - 1 for delta in sequences if 1 < delta < 100000),
        "sequence_duplicates": sequences.count(0),
        "sequence_resets_or_large_gaps": sum(delta >= 100000 for delta in sequences),
        "online_bitmap_histogram": {
            name: dict(Counter(hex(int(row[name])) for row in rows)) for name in columns
        },
        "last_steer_ecd": {wheel: int(last[f"swerve_steer_actual_ecd_{wheel}"])
                           for wheel in ("lf", "rf", "lr", "rr")},
        "small_yaw": summarize_small_yaw(list(rows)),
        "note": "Only parsed RTT monitor records are checked; rejected CRC frames are not included in the export.",
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

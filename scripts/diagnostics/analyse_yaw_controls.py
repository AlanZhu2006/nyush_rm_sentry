#!/usr/bin/env python3
"""Summarize recorded browser yaw inputs and controller state without I/O to hardware."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import statistics


def truth(value):
    return value == "True"


def number(row, field):
    value = row.get(field)
    return None if value in (None, "", "null") else float(value)


def summary(rows):
    fields = ("small_yaw_ecd", "small_yaw_actual_deg", "small_yaw_target_deg",
              "small_yaw_error_deg", "small_yaw_speed_deg_s", "small_yaw_output",
              "small_yaw_feedback_age_s", "heartbeat_age_s")
    result = {"start_ms": int(rows[0]["small_yaw_mcu_time_ms"]),
              "end_ms": int(rows[-1]["small_yaw_mcu_time_ms"]),
              "duration_s": float(rows[-1]["host_monotonic_s"]) - float(rows[0]["host_monotonic_s"]),
              "rows": len(rows)}
    for field in fields:
        values = [value for row in rows if (value := number(row, field)) is not None]
        if values:
            result[field] = {"first": values[0], "last": values[-1],
                             "min": min(values), "max": max(values), "mean": statistics.mean(values)}
    for field in ("keys", "small_yaw_armed", "small_yaw_controller_active",
                  "small_yaw_online", "small_yaw_left_limit", "small_yaw_right_limit"):
        result[field] = dict(Counter(row[field] for row in rows))
    result["target_ecd_first_last"] = [
        number(row, "small_yaw_ecd") +
        (number(row, "small_yaw_target_deg") - number(row, "small_yaw_actual_deg")) * 8192 / 360
        for row in (rows[0], rows[-1])
    ]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--start-ms", type=int, default=0)
    parser.add_argument("--last", type=int, default=10)
    args = parser.parse_args()
    with args.path.open(encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream)
                if row.get("sample_ok") == "True" and
                row.get("small_yaw_mcu_time_ms") not in (None, "", "null") and
                int(row["small_yaw_mcu_time_ms"]) >= args.start_ms and
                all(row.get(field) not in (None, "", "null") for field in
                    ("small_yaw_ecd", "small_yaw_target_deg", "small_yaw_actual_deg"))]
    segments = []
    current = []
    previous = None
    for row in rows:
        l_held = "l" in row["keys"].split("|") and truth(row["small_yaw_armed"])
        contiguous = previous is None or float(row["host_monotonic_s"]) - float(previous["host_monotonic_s"]) <= .20
        if not l_held or not contiguous:
            if current:
                segments.append(current)
                current = []
        if l_held:
            current.append(row)
        previous = row
    if current:
        segments.append(current)
    print(json.dumps({"path": str(args.path.resolve()), "rows": len(rows),
                      "l_held_segments": [summary(part) for part in segments[-args.last:]],
                      "last_state": rows[-1] if rows else None}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

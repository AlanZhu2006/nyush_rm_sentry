#!/usr/bin/env python3
"""Analyse recorded standalone yaw tracking without accessing robot hardware."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys


def describe(values):
    values = list(values)
    return {"min": min(values), "mean": statistics.mean(values),
            "max": max(values), "std": statistics.pstdev(values)} if values else None


def analyse(path, start_ms, end_ms, target_rad_s):
    samples = []
    fields = ("gimbal_yaw_target_deg", "gimbal_yaw_actual_deg",
              "gimbal_yaw_target_deg_s", "gimbal_yaw_actual_deg_s")
    with path.open(encoding="utf-8") as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        for row in reader:
            if None in row or any(row.get(name) is None for name in fields):
                continue
            timestamp = int(row["timestamp_ms"])
            if timestamp < start_ms or timestamp > end_ms:
                continue
            values = [float(row[name]) for name in fields]
            if int(row["gimbal_motor_online_bitmap"]) & 1 and all(map(math.isfinite, values)):
                samples.append({"ms": timestamp, "seq": int(row["seq"]),
                                "target": values[0], "actual": values[1],
                                "speed_ref": values[2], "speed": values[3]})
    for index, sample in enumerate(samples):
        sample["rate"] = None
        sample["derived_speed"] = None
        if index and sample["ms"] - samples[index - 1]["ms"] == 10:
            sample["rate"] = (sample["target"] - samples[index - 1]["target"]) * 100
        if index >= 10 and sample["ms"] - samples[index - 10]["ms"] == 100:
            sample["derived_speed"] = (sample["actual"] - samples[index - 10]["actual"]) * 10

    # Reject target resets on disarm, where the reference jumps back to the
    # measured angle. Select constant-rate runs around the requested speed.
    expected_rate = math.degrees(target_rad_s)
    segments = []
    current = []
    direction = 0
    for sample in samples:
        rate = sample["rate"]
        sign = (1 if rate > 0 else -1) if rate is not None and 0.8 * expected_rate <= abs(rate) <= 1.2 * expected_rate else 0
        if not sign or sign != direction:
            if len(current) >= 50:
                segments.append(current)
            current = []
        if sign:
            current.append(sample)
        direction = sign
    if len(current) >= 50:
        segments.append(current)

    reports = []
    for segment in segments:
        sign = 1 if segment[0]["rate"] > 0 else -1
        # Keep the full ramp for start-up evidence and omit its first 0.3s
        # when estimating steady speed. Short segments have no steady subset.
        steady = [sample for sample in segment if sample["ms"] >= segment[0]["ms"] + 300]
        derived = [sample["derived_speed"] for sample in steady if sample["derived_speed"] is not None]
        lag = [sign * (sample["target"] - sample["actual"]) for sample in steady]
        report = {
            "start_ms": segment[0]["ms"], "end_ms": segment[-1]["ms"],
            "duration_s": (segment[-1]["ms"] - segment[0]["ms"]) / 1000,
            "direction": sign, "target_rate_deg_s": describe(sample["rate"] for sample in segment),
            "steady_lag_deg": describe(lag),
            "steady_raw_speed_deg_s": describe(sign * sample["speed"] for sample in steady),
            "steady_derived_100ms_speed_deg_s": describe(sign * value for value in derived),
            "fraction_below_2deg_s": sum(sign * value < 2 for value in derived) / len(derived) if derived else None,
            "fraction_above_30deg_s": sum(sign * value > 30 for value in derived) / len(derived) if derived else None,
            "max_abs_tracking_error_deg": max(abs(sample["target"] - sample["actual"]) for sample in segment),
            "angle_speed_ref_deg_s": describe(sample["speed_ref"] for sample in segment),
        }
        settled = [sample for sample in segment if sample["ms"] >= segment[0]["ms"] + 1000]
        report["after_first_second"] = {
            "lag_deg": describe(sign * (sample["target"] - sample["actual"]) for sample in settled),
            "derived_speed_deg_s": describe(sign * sample["derived_speed"] for sample in settled
                                             if sample["derived_speed"] is not None),
            "reported_speed_deg_s": describe(sign * sample["speed"] for sample in settled),
        }
        following = [sample for sample in samples if segment[-1]["ms"] < sample["ms"] <= segment[-1]["ms"] + 1000]
        report["post_ramp_first_second_actual_range_deg"] = describe(sample["actual"] for sample in following)
        reports.append(report)
    missing = sum(max(0, b["seq"] - a["seq"] - 1) for a, b in zip(samples, samples[1:]))
    result = {"path": str(path.resolve()), "requested_window_ms": [start_ms, end_ms],
              "requested_target_rad_s": target_rad_s,
              "samples": len(samples), "sequence_missing": missing, "ramp_segments": reports,
              "limits": "No input-command, speed PI integral, or motor-output history is present in v8. Target resets and near-limit control cannot be uniquely distinguished from this export alone."}
    return result, samples, segments


def plot_trace(samples, segments, output, target_rad_s):
    # ROS hosts commonly provide NumPy/Matplotlib through apt rather than the
    # tooling venv. Use installed packages without changing the environment.
    system = "/usr/lib/python3/dist-packages"
    if system not in sys.path:
        sys.path.append(system)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    if not segments:
        return None
    start = segments[0][0]["ms"] - 500
    stop = segments[-1][-1]["ms"] + 1000
    shown = [sample for sample in samples if start <= sample["ms"] <= stop]
    origin = segments[0][0]["ms"]
    x = [(sample["ms"] - origin) / 1000 for sample in shown]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(x, [sample["target"] for sample in shown], label="Target angle", linewidth=1.5)
    axes[0].plot(x, [sample["actual"] for sample in shown], label="Actual angle", linewidth=1.2)
    axes[0].set_ylabel("Angle (deg)")
    axes[0].set_title(f"Small yaw: recorded {target_rad_s:.2f} rad/s target tracking")
    axes[1].plot(x, [sample["target"] - sample["actual"] for sample in shown], label="Target - actual")
    axes[1].axhline(0, color="black", linewidth=0.5)
    axes[1].set_ylabel("Tracking error (deg)")
    axes[2].plot(x, [sample["speed"] for sample in shown], label="CAN RPM feedback", color="gray", alpha=0.55, linewidth=0.7)
    axes[2].plot(x, [sample["speed_ref"] for sample in shown], label="Speed PI reference", linewidth=1.1)
    axes[2].plot(x, [sample["derived_speed"] for sample in shown], label="Angle derivative (100 ms)", linewidth=1.4)
    axes[2].plot(x, [sample["rate"] if sample["rate"] is not None and abs(sample["rate"]) < 50 else math.nan
                   for sample in shown], label="Target ramp rate", color="black", linewidth=1, linestyle="--")
    axes[2].set_ylabel("Angular speed (deg/s)")
    axes[2].set_xlabel("Seconds since first target ramp")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output / "yaw_tracking.png", dpi=160)
    fig.savefig(output / "yaw_tracking.svg")
    plt.close(fig)
    return str(output / "yaw_tracking.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--start-ms", type=int, default=3200000)
    parser.add_argument("--end-ms", type=int, default=3320000)
    parser.add_argument("--target-rate-rad-s", type=float, default=0.30)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.target_rate_rad_s) or args.target_rate_rad_s <= 0:
        parser.error("--target-rate-rad-s must be finite and positive")
    result, samples, segments = analyse(args.path, args.start_ms, args.end_ms, args.target_rate_rad_s)
    print(json.dumps(result, indent=2))
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "analysis.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        with (args.output / "trace.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=("ms", "seq", "target", "actual", "speed_ref", "speed", "rate", "derived_speed"))
            writer.writeheader()
            writer.writerows(samples)
        image = plot_trace(samples, segments, args.output, args.target_rate_rad_s)
        if image:
            print(json.dumps({"plot": image}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Inspect existing RTT yaw samples; never open a probe or control a motor."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys


def stats(values):
    values = list(values)
    return {"min": min(values), "max": max(values), "mean": statistics.mean(values),
            "std": statistics.pstdev(values)} if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--start-ms", type=int, default=700000)
    parser.add_argument("--end-ms", type=int, default=947000)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    samples = []
    with args.path.open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream, delimiter="\t"):
            if None in row or None in row.values():
                continue
            timestamp = int(row["timestamp_ms"])
            if not args.start_ms <= timestamp <= args.end_ms:
                continue
            if not int(row["gimbal_motor_online_bitmap"]) & 1:
                continue
            sample = {"ms": timestamp,
                      "target": float(row["gimbal_yaw_target_deg"]),
                      "actual": float(row["gimbal_yaw_actual_deg"]),
                      "speed": float(row["gimbal_yaw_actual_deg_s"]),
                      "speed_ref": float(row["gimbal_yaw_target_deg_s"])}
            if not all(math.isfinite(value) for value in sample.values()):
                continue
            samples.append(sample)
    for index, sample in enumerate(samples):
        sample["error"] = sample["target"] - sample["actual"]
        sample["rate"] = sample["derived_10ms"] = sample["derived_100ms"] = None
        if index and sample["ms"] - samples[index - 1]["ms"] == 10:
            sample["rate"] = (sample["target"] - samples[index - 1]["target"]) * 100
            sample["derived_10ms"] = (sample["actual"] - samples[index - 1]["actual"]) * 100
        if index >= 10 and sample["ms"] - samples[index - 10]["ms"] == 100:
            sample["derived_100ms"] = (sample["actual"] - samples[index - 10]["actual"]) * 10
    if not samples:
        raise RuntimeError("No online yaw samples in selected window")
    minimum_target = min(range(len(samples)), key=lambda index: samples[index]["target"])
    minimum_actual = min(range(len(samples)), key=lambda index: samples[index]["actual"])
    equivalents = lambda angle: (7309 + (angle + 38.80535) * 8192 / 360) % 8192
    output = {
        "samples": len(samples),
        "minimum_target": {**samples[minimum_target], "equivalent_ecd": equivalents(samples[minimum_target]["target"])},
        "minimum_actual": {**samples[minimum_actual], "equivalent_ecd": equivalents(samples[minimum_actual]["actual"])},
        "leftmost_context": [sample for sample in samples if samples[minimum_target]["ms"] - 300 <= sample["ms"] <= samples[minimum_target]["ms"] + 500][::5],
        "motion_windows": [],
    }
    interior = [sample for sample in samples
                if sample["rate"] is not None and 0.5 < abs(sample["rate"]) <= 40 and
                abs(sample["speed_ref"]) > 1 and
                4593 + 3 * 8192 / 360 < equivalents(sample["actual"]) < 7285 - 3 * 8192 / 360 and
                4593 + 3 * 8192 / 360 < equivalents(sample["target"]) < 7285 - 3 * 8192 / 360]
    forced = [sample for sample in interior if abs(50 * (sample["speed_ref"] - sample["speed"])) > 9000]
    positive = sum(sample["speed_ref"] > sample["speed"] for sample in forced)
    forced_flips = [current["ms"] - previous["ms"] for previous, current in zip(forced, forced[1:])
                    if (current["speed_ref"] - current["speed"]) *
                    (previous["speed_ref"] - previous["speed"]) < 0 and current["ms"] - previous["ms"] <= 100]
    output["inferred_pi_rail_lower_bound"] = {
        "eligible_interior_moving_samples": len(interior),
        "forced_rail_samples": len(forced),
        "forced_positive": positive, "forced_negative": len(forced) - positive,
        "fraction": len(forced) / len(interior) if interior else None,
        "consecutive_forced_sign_flip_intervals_ms": stats(forced_flips),
        "inference": "For each recorded input snapshot: P=50*(speed_ref-speed), D=0, |I|<=3000. |P|>9000 would force pre-hook PI to +/-6000. These are inferred snapshots, not measured motor-output duty; CAN feedback and PID timing may differ within a task tick. Samples exclude reset jumps and positions within 3deg of limit entry.",
    }
    # Analyse selected uninterrupted full-rate ramps, including the noisy run.
    sys.path.append("/usr/lib/python3/dist-packages")
    import numpy as np
    for start, end in ((851774, 852424), (853224, 854324), (855374, 855874),
                       (856674, 857424), (857974, 860194), (866824, 867924)):
        window = [sample for sample in samples if start <= sample["ms"] <= end]
        if len(window) < 30:
            continue
        x = np.arange(len(window)) * 0.01
        actual = np.array([sample["actual"] for sample in window])
        residual = actual - np.polyval(np.polyfit(x, actual, 1), x)
        spectrum = np.abs(np.fft.rfft(residual * np.hanning(len(window))))
        frequencies = np.fft.rfftfreq(len(window), 0.01)
        high = (frequencies >= 8) & (frequencies <= 49)
        high_peak = int(np.where(high)[0][np.argmax(spectrum[high])])
        # Five-sample mean separates gross motion from sampled 8-49 Hz motion.
        high_residual = residual[2:-2] - np.convolve(residual, np.ones(5) / 5, mode="valid")
        output["motion_windows"].append({
            "start_ms": start, "end_ms": end,
            "target_rate": stats(sample["rate"] for sample in window if sample["rate"] is not None),
            "derived_10ms_speed": stats(sample["derived_10ms"] for sample in window if sample["derived_10ms"] is not None),
            "derived_100ms_speed": stats(sample["derived_100ms"] for sample in window if sample["derived_100ms"] is not None),
            "reported_speed": stats(sample["speed"] for sample in window),
            "sampled_8_49hz_peak_hz": float(frequencies[high_peak]),
            "high_frequency_angle_deg": stats(high_residual),
            "ten_ms_reverse_step_count": sum(
                sample["derived_10ms"] * sample["rate"] < -10
                for sample in window if sample["derived_10ms"] is not None and sample["rate"] is not None),
            "inferred_forced_pi_rail_samples": sum(abs(50 * (sample["speed_ref"] - sample["speed"])) > 9000 for sample in window),
        })
    output["limits"] = "Spectral peaks are from 100 Hz samples and may alias faster motion. No input keys, limit flags, Iout or final motor output are recorded; exact stall cause cannot be inferred from target resets alone."
    rendered = json.dumps(output, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Inspect or reset the USB connection of the single 1366:0105 J-Link probe."""

import argparse
import fcntl
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reset", action="store_true", help="Reset this probe's USB connection")
    args = parser.parse_args()
    probes = []
    for entry in Path("/sys/bus/usb/devices").iterdir():
        try:
            if ((entry / "idVendor").read_text().strip() == "1366" and
                    (entry / "idProduct").read_text().strip() == "0105"):
                probes.append(entry)
        except FileNotFoundError:
            continue
    if len(probes) != 1:
        raise RuntimeError(f"Expected exactly one 1366:0105 J-Link; found {len(probes)}")
    entry = probes[0]
    bus = int((entry / "busnum").read_text())
    device = int((entry / "devnum").read_text())
    path = f"/dev/bus/usb/{bus:03d}/{device:03d}"
    if args.reset:
        descriptor = os.open(path, os.O_RDWR)
        try:
            # Linux USBDEVFS_RESET = _IO('U', 20). This addresses the USB
            # probe only; it does not issue any SWD, flash or motor command.
            fcntl.ioctl(descriptor, 0x5514, 0)
        finally:
            os.close(descriptor)
    print(json.dumps({"device": path, "vid_pid": "1366:0105",
                      "usb_reset_requested": args.reset}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

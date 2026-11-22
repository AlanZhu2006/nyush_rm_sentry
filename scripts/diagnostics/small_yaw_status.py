#!/usr/bin/env python3
"""Inspect small-yaw feedback without sending motor commands or changing flash."""

import argparse
import io
import json
from pathlib import Path
import struct
import time


def _resolve(die):
    while die.tag in {"DW_TAG_typedef", "DW_TAG_const_type", "DW_TAG_volatile_type"}:
        die = die.get_DIE_from_attribute("DW_AT_type")
    return die


def _member(die, name):
    for child in _resolve(die).iter_children():
        if child.tag == "DW_TAG_member" and child.attributes["DW_AT_name"].value.decode() == name:
            location = child.attributes["DW_AT_data_member_location"].value
            if not isinstance(location, int):
                raise RuntimeError(f"Unsupported DWARF member location for {name}")
            return location, _resolve(child.get_DIE_from_attribute("DW_AT_type"))
    raise RuntimeError(f"DWARF member not found: {name}")


def _read_u32(link, address):
    return struct.unpack("<I", bytes(link.memory_read8(address, 4)))[0]


def _open_probe(args):
    import pylink

    link = pylink.JLink()
    emulators = link.connected_emulators()
    if args.serial:
        serial = int(args.serial)
    elif len(emulators) == 1:
        serial = int(emulators[0].SerialNumber)
    else:
        raise RuntimeError("Select one J-Link with --serial")
    link.open(serial_no=serial)
    try:
        link.set_tif(pylink.enums.JLinkInterfaces.SWD)
        link.connect(args.device, speed=args.speed, verbose=False)
        return link
    except Exception:
        link.close()
        raise


def backup_flash(args):
    if args.device != "STM32H723VG":
        raise RuntimeError("Flash backup size is specified for STM32H723VG only")
    destination = Path(args.backup_flash)
    link = _open_probe(args)
    try:
        with destination.open("xb") as output:
            for offset in range(0, 0x100000, 8192):
                output.write(bytes(link.memory_read8(0x08000000 + offset, 8192)))
        print(json.dumps({"flash_backup": str(destination.resolve()),
                          "bytes": destination.stat().st_size,
                          "message": "Read-only backup; no flash write or motor command."}))
        return 0
    finally:
        link.close()


def can_snapshot(args):
    if args.device != "STM32H723VG":
        raise RuntimeError("Fixed register/message-RAM inspection requires STM32H723VG")
    link = _open_probe(args)
    try:
        # STM32H723 CMSIS FDCAN_GlobalTypeDef offsets and SRAMCAN_BASE;
        # RXF0C determines each live FIFO layout, not the local ELF.
        buses = {}
        for bus, base in ((1, 0x4000A000), (2, 0x4000A400), (3, 0x4000D400)):
            registers = bytes(link.memory_read8(base, 0xC0))
            config = struct.unpack_from("<I", registers, 0xA0)[0]
            size = struct.unpack_from("<I", registers, 0xBC)[0] & 7
            count = (config >> 16) & 0x7F
            stride = 8 + (8, 12, 16, 20, 24, 32, 48, 64)[size]
            address = 0x4000AC00 + (config & 0xFFFC)
            if count > 64 or address + count * stride > 0x4000D400:
                raise RuntimeError(f"Invalid CAN{bus} receive FIFO register layout")
            buses[bus] = {"base": base, "address": address, "count": count,
                          "stride": stride, "put_indices": set(), "frames": {},
                          "cccr": hex(struct.unpack_from("<I", registers, 0x18)[0])}
        deadline = time.monotonic() + args.seconds
        while True:
            for bus, state in buses.items():
                status = _read_u32(link, state["base"] + 0xA4)
                state["put_indices"].add((status >> 16) & 0x3F)
                for index in range(state["count"]):
                    address = state["address"] + index * state["stride"]
                    words = link.memory_read32(address, state["stride"] // 4)
                    if words != link.memory_read32(address, state["stride"] // 4):
                        continue
                    memory = struct.pack(f"<{len(words)}I", *words)
                    word0, word1 = words[:2]
                    if word0 == word1 == 0:
                        continue
                    extended = bool(word0 & (1 << 30))
                    identifier = word0 & 0x1FFFFFFF if extended else (word0 >> 18) & 0x7FF
                    dlc = (word1 >> 16) & 0xF
                    if dlc > 8 or extended or word0 & (1 << 29) or word0 & 0x3FFFF:
                        continue
                    data = memory[8:8 + dlc]
                    record = state["frames"].setdefault(identifier, {"payloads": set(), "timestamps": set()})
                    record["payloads"].add(data.hex(" "))
                    record["timestamps"].add(word1 & 0xFFFF)
            if time.monotonic() >= deadline:
                break
            time.sleep(min(0.073, max(0.0, deadline - time.monotonic())))
        output = []
        for bus, state in buses.items():
            frames = [{"can_id": hex(identifier), "distinct_payloads": len(record["payloads"]),
                       "payload_examples": sorted(record["payloads"])[:3],
                       "distinct_rx_timestamps": len(record["timestamps"])}
                      for identifier, record in sorted(state["frames"].items())]
            output.append({"can_bus": bus, "cccr": state["cccr"],
                           "fifo_address": hex(state["address"]),
                           "fifo_slots": state["count"], "put_indices": sorted(state["put_indices"]),
                           "bus_activity_observed": len(state["put_indices"]) > 1,
                           "stored_standard_frames": frames})
        # The AXI SRAM region is fixed for this MCU. This scan only reads; it
        # never advances an RTT read pointer or acknowledges a CAN FIFO entry.
        ram = b"".join(bytes(link.memory_read8(0x24000000 + offset, min(8192, 0x50000 - offset)))
                       for offset in range(0, 0x50000, 8192))
        rtt_offset = ram.find(b"SEGGER RTT\x00")
        rtt = {"address": None}
        if rtt_offset >= 0 and rtt_offset + 48 <= len(ram):
            rtt["address"] = hex(0x24000000 + rtt_offset)
            up_count = struct.unpack_from("<I", ram, rtt_offset + 16)[0]
            rtt["up_channels"] = up_count
            if 0 < up_count <= 16:
                _, address, size, write_offset, read_offset, _ = struct.unpack_from("<6I", ram, rtt_offset + 24)
                if 0x20000000 <= address < 0x30000000 and 0 < size <= 65536 and write_offset < size and read_offset < size:
                    data = bytes(link.memory_read8(address, size))
                    unread = data[read_offset:write_offset] if write_offset >= read_offset else data[read_offset:] + data[:write_offset]
                    rtt["log_unread_snapshot"] = unread[-8000:].decode("utf-8", errors="replace")
        print(json.dumps({"can_snapshots": output, "rtt": rtt,
                          "message": "Stored FIFO frames can be historical. Put-index changes show bus activity; changing payloads/timestamps support freshness for that ID. Read-only; no FIFO acknowledgement, RTT read-pointer update or motor command."}))
        return 0
    finally:
        link.close()


def probe_status(args):
    from elftools.elf.elffile import ELFFile

    # Freeze the ELF before opening the probe so a concurrent build cannot
    # change the symbol table after its flash contents have been compared.
    elf = ELFFile(io.BytesIO(Path(args.elf).read_bytes()))
    symtab = elf.get_section_by_name(".symtab")
    symbols = {symbol.name: int(symbol["st_value"]) for symbol in symtab.iter_symbols()}
    dwarf = elf.get_dwarf_info()
    motor_type = None
    for unit in dwarf.iter_CUs():
        for die in unit.iter_DIEs():
            name = die.attributes.get("DW_AT_name")
            if name and name.value == b"DJIMotorInstance":
                motor_type = _resolve(die)
                break
        if motor_type is not None:
            break
    if motor_type is None:
        raise RuntimeError("ELF has no DJIMotorInstance DWARF type")

    link = _open_probe(args)
    try:
        compared = 0
        for segment in elf.iter_segments():
            address = int(segment["p_paddr"])
            if segment["p_type"] != "PT_LOAD" or not 0x08000000 <= address < 0x08200000:
                continue
            # ELF flash loaders write allocated sections, not linker padding
            # between them. Erased 0xff padding may be zero-filled in PT_LOAD.
            for section in elf.iter_sections():
                if (not int(section["sh_flags"]) & 2 or
                        section["sh_type"] == "SHT_NOBITS" or
                        not segment.section_in_segment(section)):
                    continue
                section_address = address + int(section["sh_addr"]) - int(segment["p_vaddr"])
                expected = section.data()
                for offset in range(0, len(expected), 8192):
                    chunk = expected[offset:offset + 8192]
                    actual = bytes(link.memory_read8(section_address + offset, len(chunk)))
                    if actual != chunk:
                        first_difference = next(
                            (index for index, values in enumerate(zip(chunk, actual))
                             if values[0] != values[1]), min(len(chunk), len(actual))
                        )
                        print(json.dumps({
                            "elf_matches_flash": False,
                            "elf": str(Path(args.elf).resolve()),
                            "section": section.name,
                            "mismatch_block": hex(section_address + offset),
                            "first_difference": hex(section_address + offset + first_difference),
                            "expected_bytes": chunk[first_difference:first_difference + 16].hex(" "),
                            "read_bytes": actual[first_difference:first_difference + 16].hex(" "),
                            "message": "Symbols are untrusted; no runtime pointers were dereferenced.",
                        }))
                        return 3
                    compared += len(chunk)
        if compared == 0:
            raise RuntimeError("ELF has no loadable internal-flash segments")
        if "aim_yaw_motor" not in symbols:
            print(json.dumps({"elf_matches_flash": True, "registered_small_yaw": False,
                              "message": "This firmware has no aim_yaw_motor symbol."}))
            return 4

        motor = _read_u32(link, symbols["aim_yaw_motor"])
        if not 0x20000000 <= motor < 0x30000000:
            print(json.dumps({"elf_matches_flash": True, "registered_small_yaw": False,
                              "motor_pointer": hex(motor)}))
            return 4

        can_offset, can_pointer_type = _member(motor_type, "motor_can_instance")
        can_type = _resolve(can_pointer_type.get_DIE_from_attribute("DW_AT_type"))
        can = _read_u32(link, motor + can_offset)
        if not 0x20000000 <= can < 0x30000000:
            raise RuntimeError("Registered motor has no valid CAN instance pointer")
        handle = _read_u32(link, can + _member(can_type, "can_handle")[0])
        bus = next((number for number in (1, 2, 3)
                    if symbols.get(f"hfdcan{number}") == handle), None)
        rx_id = _read_u32(link, can + _member(can_type, "rx_id")[0])
        measure_offset, measure_type = _member(motor_type, "measure")
        controller_offset, controller_type = _member(motor_type, "motor_controller")
        controller_config = {}
        for loop, fields in (("angle_PID", ("Kp", "DeadBand", "MaxOut")),
                             ("speed_PID", ("Kp", "Ki", "IntegralLimit", "MaxOut"))):
            pid_offset, pid_type = _member(controller_type, loop)
            controller_config[loop] = {
                field: struct.unpack("<f", bytes(link.memory_read8(
                    motor + controller_offset + pid_offset + _member(pid_type, field)[0], 4)))[0]
                for field in fields
            }
        feed_offset = _member(motor_type, "feed_cnt")[0]
        stop_offset, stop_type = _member(motor_type, "stop_flag")
        stop_size = int(stop_type.attributes["DW_AT_byte_size"].value)
        daemon_offset, daemon_pointer_type = _member(motor_type, "daemon")
        daemon_type = _resolve(daemon_pointer_type.get_DIE_from_attribute("DW_AT_type"))
        daemon = _read_u32(link, motor + daemon_offset)
        samples = []
        deadline = time.monotonic() + args.seconds
        while True:
            measure = bytes(link.memory_read8(motor + measure_offset,
                                             int(measure_type.attributes["DW_AT_byte_size"].value)))
            daemon_count = None
            if 0x20000000 <= daemon < 0x30000000:
                daemon_count = struct.unpack("<H", bytes(link.memory_read8(
                    daemon + _member(daemon_type, "temp_count")[0], 2)))[0]
            samples.append({
                "feedback_timestamp_ticks": _read_u32(link, motor + feed_offset),
                "encoder": struct.unpack_from("<H", measure, _member(measure_type, "ecd")[0])[0],
                "speed_deg_s": struct.unpack_from("<f", measure, _member(measure_type, "speed_aps")[0])[0],
                "current_raw": struct.unpack_from("<h", measure, _member(measure_type, "real_current")[0])[0],
                "stop_flag": int.from_bytes(bytes(link.memory_read8(motor + stop_offset, stop_size)), "little"),
                "daemon_count": daemon_count,
                "aim_reference_deg": {
                    name: struct.unpack("<f", bytes(link.memory_read8(symbols[name], 4)))[0]
                    for name in ("aim_yaw_origin_deg", "aim_yaw_target_deg",
                                 "aim_yaw_profile_rate_deg_s", "aim_yaw_speed_feedforward_deg_s")
                    if name in symbols
                },
                "usb_last_payload": (
                    bytes(link.memory_read8(symbols["UserTxBufferFS"], 29)).hex(" ")
                    if "UserTxBufferFS" in symbols else None
                ),
                "usb_timing": {
                    name: _read_u32(link, symbols[name])
                    for name in ("uwTick", "last_telemetry_ms", "last_command_ms",
                                 "telemetry_tx_started_ms", "telemetry_recovery_count")
                    if name in symbols
                },
                "usb_registers": {
                    name: hex(_read_u32(link, 0x40040000 + offset))
                    for name, offset in (("PCGCCTL", 0xE00), ("DSTS", 0x808),
                                         ("GINTSTS", 0x14), ("GINTMSK", 0x18),
                                         ("DIEPCTL1", 0x920), ("DIEPTSIZ1", 0x930),
                                         ("DTXFSTS1", 0x938), ("DIEPINT1", 0x928),
                                         ("DAINT", 0x818), ("DAINTMSK", 0x81C))
                },
            })
            if time.monotonic() >= deadline:
                break
            time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
        # DJI's feed_cnt is the last DWT cycle timestamp, not a frame counter.
        delta = (samples[-1]["feedback_timestamp_ticks"] -
                 samples[0]["feedback_timestamp_ticks"]) & 0xFFFFFFFF
        print(json.dumps({
            "elf_matches_flash": True, "compared_flash_bytes": compared,
            "can_bus": bus, "feedback_can_id": hex(rx_id),
            "gm6020_id": rx_id - 0x204 if 0x205 <= rx_id <= 0x20B else None,
            "controller_config": controller_config,
            "feedback_timestamp_delta_ticks": delta,
            "feedback_active": delta > 0,
            "first": samples[0], "last": samples[-1],
            "message": "Read-only probe inspection; no flash, reset, halt, memory write or motor command.",
        }))
        return 0
    finally:
        link.close()


def usb_status(args):
    import serial
    from serial.tools import list_ports

    port = args.port
    if not port:
        ports = [entry.device for entry in list_ports.comports()
                 if entry.vid == 0x0483 and entry.pid == 0x00CA]
        if len(ports) != 1:
            raise RuntimeError("Select one NYUSH USB CDC controller with --port")
        port = ports[0]

    def crc16(data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        return crc

    body = struct.Struct("<2sBBHI4H4hB")
    buffer = bytearray()
    statuses = {}
    count = 0
    received = 0
    with serial.Serial(port, 115200, timeout=0.1, exclusive=True) as connection:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            data = connection.read(max(1, connection.in_waiting))
            received += len(data)
            buffer.extend(data)
            while len(buffer) >= body.size + 2:
                if buffer[:4] != b"\xa5\x5a\x01\x81":
                    del buffer[0]
                    continue
                frame = bytes(buffer[:body.size + 2])
                if crc16(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                    del buffer[0]
                    continue
                del buffer[:len(frame)]
                fields = body.unpack(frame[:-2])
                count += 1
                status = fields[-1]
                statuses[hex(status)] = statuses.get(hex(status), 0) + 1
    print(json.dumps({"port": port, "received_bytes": received, "valid_frames": count,
                      "status_histogram": statuses,
                      "aim_yaw_online_bit_observed": any(int(value, 16) & 4 for value in statuses),
                      "message": "No USB writes. An absent online bit is inconclusive on older firmware."}))
    return 0 if count else 4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usb", action="store_true", help="Read USB telemetry instead of probe memory")
    parser.add_argument("--can-snapshot", action="store_true", help="Read H723 CAN FIFO snapshots and locate RTT without trusting ELF symbols")
    parser.add_argument("--backup-flash", metavar="PATH", help="Read the 1 MiB H723VG flash into a new local file")
    parser.add_argument("--port", default="", help="USB CDC port (auto-detect by VID/PID by default)")
    parser.add_argument("--elf", default="build/nyush_rm_control_h723.elf")
    parser.add_argument("--serial", help="J-Link serial number")
    parser.add_argument("--device", default="STM32H723VG")
    parser.add_argument("--speed", type=int, default=4000, help="SWD clock in kHz")
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()
    if not 0.1 <= args.seconds <= 10.0:
        parser.error("--seconds must be between 0.1 and 10")
    try:
        if args.backup_flash:
            return backup_flash(args)
        if args.usb:
            return usb_status(args)
        return can_snapshot(args) if args.can_snapshot else probe_status(args)
    except Exception as error:
        print(json.dumps({"error": str(error)}))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

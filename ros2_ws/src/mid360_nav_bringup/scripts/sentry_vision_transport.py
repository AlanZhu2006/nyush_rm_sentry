#!/usr/bin/env python3
"""Bounded shared SP/nav USB transport; deliberately independent of ROS/serial.

SP29: <2sB6fH (yaw,yaw rate,yaw accel,pitch,pitch rate,pitch accel).
SP43: <2sB9fHH (wxyz,yaw,yaw rate,pitch,pitch rate,bullet speed,count,crc).
SC12: <2sBBBBHHH (version,caps,flags,last mode,accepted,rejected,crc).
SP angles retain legacy right/up signs; this bridge performs no conversion.
"""
import errno
import math
import os
import struct
import time
import termios
import tty
from collections import deque


def crc16_sp(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8): crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def crc16_nav(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8): crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def seal(body): return body + struct.pack('<H', crc16_sp(body))


RELEASE = seal(struct.pack('<2sB6f', b'SP', 0, *([0.] * 6)))


def validate_sp(frame, command=False):
    if len(frame) < 4 or crc16_sp(frame[:-2]) != int.from_bytes(frame[-2:], 'little'): return False
    if frame[:2] == b'SC':
        return (not command and len(frame) == 12 and frame[2] == 1
                and not (frame[3] & ~63) and not (frame[4] & ~63) and frame[5] <= 2)
    if frame[:2] != b'SP' or frame[2] > (2 if command else 3): return False
    if command:
        if len(frame) != 29: return False
        values = struct.unpack_from('<6f', frame, 3)
        return all(math.isfinite(v) and abs(v) <= limit
                   for v, limit in zip(values, (3.15, 100., 1000., 3.15, 100., 1000.)))
    if len(frame) != 43: return False
    values = struct.unpack_from('<9f', frame, 3)
    if not all(math.isfinite(v) for v in values): return False
    norm = sum(v*v for v in values[:4])
    if norm:
        w, x, y, z = (value/math.sqrt(norm) for value in values[:4])
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        pitch = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        if any(abs(math.remainder(actual-encoded, 2*math.pi)) > .05
               for actual, encoded in ((yaw, values[4]), (pitch, values[6]))): return False
    return ((.95 <= norm <= 1.05 or (frame[2] == 0 and norm == 0.))
            and all(abs(values[i]) <= 3.15 for i in (4, 6))
            and all(abs(values[i]) <= 100. for i in (5, 7))
            and 0. <= values[8] <= 100.)


class MixedParser:
    """Resynchronize bytewise after invalid CRC/semantics, preserving all families."""
    def __init__(self, commands=False):
        self.buffer = bytearray(); self.commands = commands
        self.crc_errors = 0; self.discarded = 0
        self.invalid_vision_count = 0
        self.events = []
        self._byte_times = deque()

    @property
    def partial_since(self):
        return self._byte_times[0] if self._byte_times else None

    def clear(self):
        self.buffer.clear()
        self._byte_times.clear()

    def _consume(self, count):
        del self.buffer[:count]
        for _ in range(count): self._byte_times.popleft()

    def feed(self, data, now=None):
        frames = []
        self.events = []
        stamp = time.monotonic() if now is None else now
        # Small chunks bound storage even for a maliciously large read.
        for offset in range(0, len(data), 256):
            chunk = data[offset:offset+256]
            self.buffer.extend(chunk)
            self._byte_times.extend([stamp] * len(chunk))
            while len(self.buffer) >= 2:
                header = bytes(self.buffer[:2]); size = 0; kind = ''
                if header == b'SP': size, kind = (29 if self.commands else 43), 'sp'
                elif header == b'SC' and not self.commands: size, kind = 12, 'status'
                elif header == b'\xa5\x5a' and not self.commands:
                    if len(self.buffer) < 4: break
                    packet_type = self.buffer[3]
                    if self.buffer[2] == 1 and packet_type in (0x81, 0x82):
                        size, kind = 29, ('nav' if packet_type == 0x81 else 'aim')
                if not size:
                    self._consume(1); self.discarded += 1; continue
                if len(self.buffer) < size: break
                frame = bytes(self.buffer[:size])
                valid = (validate_sp(frame, self.commands) if kind in ('sp','status') else
                         crc16_nav(frame[:-2]) == int.from_bytes(frame[-2:], 'little'))
                if valid:
                    self._consume(size); frames.append((kind, frame))
                    self.events.append((kind, frame))
                else:
                    if kind in ('sp', 'status'):
                        self.invalid_vision_count += 1
                        self.events.append(('invalid_vision', frame))
                    self._consume(1); self.crc_errors += 1; self.discarded += 1
        return frames


class TxQueue:
    """One writer, bounded complete frames; retain partial tails without interleaving.

    Expired partial frames require transport reset: dropping their tail would corrupt
    the receiver. Entire unsent expired frames can safely be dropped.
    """
    def __init__(self): self.pending = deque()
    def clear(self): self.pending.clear()
    def cancel_unsent(self): self.pending = deque(p for p in self.pending if p[2])
    def put(self, frame, now, key):
        self.pending = deque(p for p in self.pending if p[3] != key or p[2])
        if len(self.pending) >= 16: raise BufferError('transmit queue full')
        self.pending.append([bytes(frame), now, 0, key])
    def flush(self, writer, now):
        while self.pending:
            frame, stamp, offset, _ = self.pending[0]
            if now - stamp > .25:
                if offset: raise TimeoutError('partly transmitted frame expired')
                self.pending.popleft(); continue
            try: count = writer(frame[offset:])
            except BlockingIOError: return
            if count is None or count == 0: return
            if count < 0 or count > len(frame)-offset: raise OSError('invalid write count')
            self.pending[0][2] += count
            if self.pending[0][2] == len(frame): self.pending.popleft()
            return


class SharedVisionTransport:
    """Own a raw PTY and revoke authority across stale links and reconnects.

    Never replace a pre-existing path. Cleanup checks the inode we created, so a
    user replacement at that path is preserved. Reconnect creates a fresh slave.
    """
    def __init__(self, path='/tmp/nyush-rm-sentry-vision'):
        self.path = path; self.master = None; self.inode = None
        self.tx = TxQueue(); self.output = TxQueue()
        self.parser = MixedParser(); self.commands = MixedParser(True)
        self.status = None; self.status_time = None; self.telemetry_time = None
        self.work_mode = None
        self.active = False; self.client_seen = False
        self.command_time = self.command_fragment_time = self.usb_fragment_time = None
        if self.path is not None: self._create()

    def _create(self):
        master, slave = os.openpty()
        try:
            tty.setraw(slave); os.set_blocking(master, False)
            os.symlink(os.ttyname(slave), self.path)
            self.inode = os.lstat(self.path).st_ino; self.master = master
        except BaseException:
            os.close(master); raise
        finally: os.close(slave)

    def close(self):
        if self.master is not None: os.close(self.master); self.master = None
        if self.inode is not None:
            try:
                if os.lstat(self.path).st_ino == self.inode: os.unlink(self.path)
            except FileNotFoundError: pass
            self.inode = None

    def reset(self, now=None):
        self.close(); self.tx.clear(); self.output.clear()
        self.parser.clear(); self.commands.clear()
        self.status = None; self.status_time = None; self.telemetry_time = None
        self.work_mode = None
        self.active = False; self.client_seen = False
        self.command_time = self.command_fragment_time = self.usb_fragment_time = None
        if self.path is not None: self._create()
        self.tx.put(RELEASE, time.monotonic() if now is None else now, 'vision')

    def release(self, now):
        if self.active: self.tx.put(RELEASE, now, 'vision')
        self.active = False

    def fresh(self, now):
        # TIMEOUT/REJECTED describe the previous command, not current sensing;
        # fresh sensing must allow a new command to recover authority.
        return (self.status_time is not None and self.telemetry_time is not None
                and now-self.status_time <= .25 and now-self.telemetry_time <= .25
                and self.status[3] & 8 and self.status[4] & 32 and self.status[4] & 1
                and not self.status[4] & 16)

    def allows(self, frame, now):
        if frame[2] == 0: return True
        if not self.fresh(now) or self.work_mode in (None, 0): return False
        caps = self.status[3]
        values = struct.unpack_from('<6f', frame, 3)
        return ((caps & 1 or all(value == 0. for value in values[:3]))
                and (caps & 2 or all(value == 0. for value in values[3:]))
                and (frame[2] != 2 or bool(caps & 4)))

    def feed_usb(self, data, now):
        nav = []
        if self.usb_fragment_time is not None and now-self.usb_fragment_time > .25:
            self.parser.clear()
            self.usb_fragment_time = None
        self.parser.feed(data, now)
        self.usb_fragment_time = self.parser.partial_since
        for kind, frame in self.parser.events:
            if kind == 'invalid_vision':
                self.work_mode = None
                self.status_time = self.telemetry_time = None
                self.output.clear()
                self.tx.put(RELEASE, now, 'vision')
                self.active = False
                continue
            if kind == 'status': self.status, self.status_time = frame, now
            elif kind == 'sp':
                self.work_mode = frame[2]
                quaternion = struct.unpack_from('<4f', frame, 3)
                self.telemetry_time = now if .95 <= sum(v*v for v in quaternion) <= 1.05 else None
            else: nav.append((kind, frame)); continue
            self.output.put(frame, now, kind)
        if not self.fresh(now) or self.work_mode in (None, 0): self.release(now)
        return nav

    def poll(self, now):
        if self.master is None: return
        if (not self.fresh(now) or (self.command_time is not None and now-self.command_time > .25)):
            self.release(now)
        if self.command_fragment_time is not None and now-self.command_fragment_time > .25:
            self.commands.clear()
            self.command_fragment_time = None
        for _ in range(16):
            try: data = os.read(self.master, 256)
            except BlockingIOError: break
            except OSError as error:
                if error.errno != errno.EIO: raise
                if self.client_seen:
                    self.release(now); self.output.clear(); self.commands.clear()
                    self.status_time = self.telemetry_time = None; self.client_seen = False
                    self.work_mode = None
                break
            if not data: break
            self.client_seen = True
            before = self.commands.discarded
            frames = self.commands.feed(data, now)
            self.command_fragment_time = self.commands.partial_since
            if self.commands.discarded != before: self.release(now)
            for _, frame in frames:
                if self.allows(frame, now):
                    self.active = frame[2] != 0
                    self.command_time = now
                    self.tx.put(frame, now, 'vision')
                else: self.release(now)
        else:
            # A producer outrunning the bounded poll budget cannot retain old
            # commands in the kernel PTY queue and have them applied later.
            termios.tcflush(self.master, termios.TCIFLUSH)
            self.commands.clear()
            self.release(now)
        try: self.output.flush(lambda data: os.write(self.master, data), now)
        except OSError as error:
            if error.errno != errno.EIO: raise
            self.output.clear(); self.release(now)

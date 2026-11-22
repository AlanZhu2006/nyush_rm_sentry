#!/usr/bin/env python3
"""Offline tests for the production shared USB/PTY transport (no ROS or hardware)."""
import errno
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import types
import importlib.util
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/mid360_nav_bringup/scripts'))
from sentry_vision_transport import MixedParser, TxQueue, SharedVisionTransport, crc16_sp, seal, validate_sp


def command(mode=1, values=(0.,)*6):
    return seal(struct.pack('<2sB6f', b'SP', mode, *values))


def status(caps=8, flags=33):
    return seal(struct.pack('<2sBBBBHH', b'SC', 1, caps, flags, 0, 0, 0))


def telemetry(mode=1):
    return seal(struct.pack('<2sB9fH', b'SP', mode, 1., 0., 0., 0., 0., 0., 0., 0., 0., 0))


class TransportTests(unittest.TestCase):
    def test_crc_golden(self):
        self.assertEqual(crc16_sp(b'123456789'), 0x6f91)

    def test_mixed_fragmentation_corruption(self):
        from sentry_vision_transport import crc16_nav
        nav_body = struct.pack('<2sBBHI4H4hB', b'\xa5\x5a', 1, 0x81, 1, 1, *([0]*8), 0)
        nav = nav_body + struct.pack('<H', crc16_nav(nav_body))
        bad = bytearray(telemetry()); bad[-1] ^= 1
        stream = b'noiseS' + bytes(bad) + status() + nav + telemetry()
        for split in range(1, len(stream)):
            parser = MixedParser()
            got = parser.feed(stream[:split]) + parser.feed(stream[split:])
            self.assertEqual([f for _, f in got], [status(), nav, telemetry()])
        parser = MixedParser()
        self.assertEqual(parser.feed(b'x'*20000 + status()), [('status', status())])
        self.assertLessEqual(len(parser.buffer), 43)

    def test_validation(self):
        for mode in (0, 1, 2): self.assertTrue(validate_sp(command(mode), True))
        for mode in (3, 255): self.assertFalse(validate_sp(command(mode), True))
        for bad in (float('nan'), float('inf'), 13.):
            self.assertFalse(validate_sp(command(values=(bad, 0., 0., 0., 0., 0.)), True))
        self.assertFalse(validate_sp(command(values=(0., 101., 0., 0., 0., 0.)), True))
        self.assertFalse(validate_sp(status(64), False))
        self.assertFalse(validate_sp(status(flags=64), False))

    def test_actual_ros_reader_uses_shared_parser(self):
        # Execute the actual ROS bridge method with only ROS message dependencies
        # stubbed; the production parser, PTY, queue, and telemetry unpacking run.
        modules = {}
        for name in ('rclpy', 'rclpy.node', 'geometry_msgs', 'geometry_msgs.msg',
                     'nav_msgs', 'nav_msgs.msg', 'std_msgs', 'std_msgs.msg',
                     'serial', 'serial.tools', 'serial.tools.list_ports'):
            modules[name] = types.ModuleType(name)
        modules['rclpy.node'].Node = object
        for name in ('Twist', 'Vector3'): setattr(modules['geometry_msgs.msg'], name, type(name, (), {}))
        modules['nav_msgs.msg'].Odometry = type('Odometry', (), {})
        for name in ('Bool', 'Float64MultiArray'): setattr(modules['std_msgs.msg'], name, type(name, (), {}))
        modules['serial'].SerialException = type('SerialException', (Exception,), {})
        original = {name: sys.modules.get(name) for name in modules}
        sys.modules.update(modules)
        try:
            spec = importlib.util.spec_from_file_location('actual_bridge', Path(sys.path[0]) / 'sentry_nav2_bridge.py')
            bridge_module = importlib.util.module_from_spec(spec); spec.loader.exec_module(bridge_module)
            bridge = bridge_module.SentryNav2Bridge.__new__(bridge_module.SentryNav2Bridge)
            from sentry_vision_transport import crc16_nav
            body = struct.pack('<2sBBHI4H4hB', b'\xa5\x5a', 1, 0x81, 1, 2, *([0]*8), 0)
            nav = body + struct.pack('<H', crc16_nav(body))
            class Serial:
                def __init__(self): self.data = b''; self.out = bytearray()
                @property
                def in_waiting(self): return len(self.data)
                def read(self, count): data,self.data=self.data[:count],self.data[count:]; return data
                def write(self, data): self.out.extend(data[:2]); return min(2,len(data))
            with tempfile.TemporaryDirectory() as directory:
                transport = SharedVisionTransport(directory+'/vision')
                bridge._vision = transport; bridge._serial = Serial()
                bridge._rx_bytes_total = bridge._valid_frame_count = 0
                bridge._last_motor_status = 0
                called=[]; bridge._handle_telemetry=lambda *args: called.append(args)
                bridge._disconnect_serial=lambda reason: self.fail(reason)
                stream = telemetry()+status()+nav
                try:
                    for byte in stream:
                        bridge._serial.data = bytes((byte,)); bridge._read_telemetry()
                    self.assertEqual(called, [(2, (0,)*4, (0,)*4)])
                    self.assertEqual(transport.status, status())
                finally: transport.close()
        finally:
            for name,value in original.items():
                if value is None: sys.modules.pop(name,None)
                else: sys.modules[name]=value

    def test_idle_unavailable_attitude_does_not_authorize(self):
        with tempfile.TemporaryDirectory() as directory:
            t=SharedVisionTransport(directory+'/vision')
            try:
                idle=seal(struct.pack('<2sB9fH', b'SP',0,*([0.]*9),0))
                t.feed_usb(status()+idle,.1)
                self.assertFalse(t.fresh(.1))
            finally: t.close()

    def test_wire_field_offsets(self):
        self.assertTrue(validate_sp(command(values=(1., 90., 900., 1., 90., 900.)), True))
        wire = seal(struct.pack('<2sB9fH', b'SP', 1, 1.,0.,0.,0.,0.,90.,0.,90.,30.,42))
        self.assertTrue(validate_sp(wire))
        mode3 = bytearray(wire[:-2]); mode3[2] = 3
        self.assertTrue(validate_sp(seal(mode3)))
        incoherent = seal(struct.pack('<2sB9fH', b'SP', 1, 1.,0.,0.,0.,1.,0.,0.,0.,30.,42))
        self.assertFalse(validate_sp(incoherent))

    def test_capabilities_command_timeout_and_client_disconnect(self):
        with tempfile.TemporaryDirectory() as directory:
            t=SharedVisionTransport(directory+'/vision')
            client=os.open(t.path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
            try:
                t.feed_usb(status()+telemetry(),.1)
                os.write(client,command(values=(1.,0.,0.,0.,0.,0.))); t.poll(.11)
                self.assertFalse(t.active)  # no yaw actuator capability
                t.feed_usb(status(caps=9)+telemetry(),.12)
                os.write(client,command(values=(1.,0.,0.,0.,0.,0.))); t.poll(.13)
                self.assertTrue(t.active)
                t.feed_usb(status(caps=9)+telemetry(),.4)
                t.poll(.4); self.assertFalse(t.active)  # command alone expired
                os.write(client,command(2)); t.poll(.41)
                self.assertFalse(t.active)  # fire unavailable
                os.write(client,command()); t.poll(.42); self.assertTrue(t.active)
                os.close(client); client=None; t.poll(.43)
                self.assertFalse(t.active)
                self.assertIsNone(t.status_time)
            finally:
                if client is not None: os.close(client)
                t.close()

    def test_timeout_status_allows_fresh_command_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            t=SharedVisionTransport(directory+'/vision')
            client=os.open(t.path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
            try:
                t.feed_usb(status(flags=33|8|4)+telemetry(),.1)
                os.write(client,command()); t.poll(.11)
                self.assertTrue(t.active)
                t.feed_usb(status(flags=33|16)+telemetry(),.12)
                self.assertFalse(t.active)
            finally: os.close(client); t.close()

    def test_excess_command_backlog_is_flushed(self):
        with tempfile.TemporaryDirectory() as directory:
            t=SharedVisionTransport(directory+'/vision')
            client=os.open(t.path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
            try:
                t.feed_usb(status()+telemetry(),.1)
                burst=command()*200
                self.assertEqual(os.write(client,burst),len(burst))
                t.poll(.11)
                self.assertFalse(t.active)
                t.poll(.12)
                self.assertFalse(t.active)  # dropped backlog cannot replay later
            finally: os.close(client); t.close()

    def test_command_first_fragment_deadline_cannot_slide(self):
        with tempfile.TemporaryDirectory() as directory:
            t=SharedVisionTransport(directory+'/vision')
            client=os.open(t.path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
            try:
                packet=command()
                for now,part in ((.1,packet[:10]),(.3,packet[10:20]),(.5,packet[20:])):
                    t.feed_usb(status()+telemetry(),now)
                    os.write(client,part); t.poll(now)
                self.assertFalse(t.active)
                os.write(client,packet); t.poll(.51)
                self.assertTrue(t.active)
            finally: os.close(client); t.close()

    def test_usb_first_fragment_deadline_cannot_slide(self):
        t=SharedVisionTransport(None)
        try:
            packet=telemetry()
            for now,part in ((.1,packet[:15]),(.3,packet[15:30]),(.5,packet[30:])):
                t.feed_usb(part,now)
            self.assertIsNone(t.telemetry_time)
            t.feed_usb(status()+packet,.51)
            self.assertTrue(t.fresh(.51))
        finally: t.close()

    def test_invalid_feedback_revokes_and_valid_following_recovers(self):
        nan_telemetry=seal(struct.pack('<2sB9fH',b'SP',1,1.,0.,0.,0.,float('nan'),0.,0.,0.,0.,0))
        bad_crc=bytearray(telemetry()); bad_crc[-1] ^= 1
        for malformed in (nan_telemetry, bytes(bad_crc), status(caps=64), status(flags=64)):
            with self.subTest(malformed=malformed):
                t=SharedVisionTransport(None)
                try:
                    t.feed_usb(status()+telemetry(),.1)
                    t.active=True
                    t.feed_usb(malformed,.11)
                    self.assertFalse(t.active)
                    self.assertFalse(t.fresh(.11))
                    self.assertIsNone(t.telemetry_time)
                    self.assertIsNone(t.status_time)
                    self.assertTrue(any(frame[0] == command(0) for frame in t.tx.pending))
                    t.feed_usb(status()+telemetry(),.12)
                    self.assertTrue(t.fresh(.12))
                    # Events retain order even within a single mixed USB read.
                    t.active=True
                    t.feed_usb(malformed+status()+telemetry(),.13)
                    self.assertFalse(t.active)
                    self.assertTrue(t.fresh(.13))
                    t.feed_usb(status()+telemetry()+malformed,.14)
                    self.assertFalse(t.fresh(.14))
                finally: t.close()

    def test_idle_work_mode_releases_commands_without_invalidating_sensing(self):
        with tempfile.TemporaryDirectory() as directory:
            t=SharedVisionTransport(directory+'/vision')
            client=os.open(t.path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
            try:
                t.feed_usb(status()+telemetry(),.1)
                os.write(client,command()); t.poll(.11)
                self.assertTrue(t.active)
                t.feed_usb(status()+telemetry(mode=0),.12)
                self.assertTrue(t.fresh(.12))
                self.assertFalse(t.active)
                self.assertTrue(any(frame[0] == command(0) for frame in t.tx.pending))
                os.write(client,command()); t.poll(.13)
                self.assertFalse(t.active)
                self.assertTrue(t.allows(command(0),.13))
                t.feed_usb(status()+telemetry(mode=3),.14)
                os.write(client,command()); t.poll(.15)
                self.assertTrue(t.active)
            finally: os.close(client); t.close()

    def test_short_blocked_and_stale_writes(self):
        q = TxQueue(); q.put(b'abcdef', 0., 'nav')
        out = bytearray()
        def short(data): out.extend(data[:2]); return min(2, len(data))
        q.flush(short, .1); self.assertEqual(out, b'ab')
        def blocked(data): raise BlockingIOError(errno.EAGAIN, 'blocked')
        q.flush(blocked, .1)
        q.flush(short, .1); q.flush(short, .1)
        self.assertEqual(out, b'abcdef')
        q.put(b'stale', 0., 'nav'); q.flush(short, 1.)
        self.assertEqual(out, b'abcdef')
        q.put(b'partial', 1., 'nav'); q.flush(short, 1.)
        with self.assertRaises(TimeoutError): q.flush(short, 2.)

    def test_pty_gating_reconnect_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            path = directory + '/vision'
            t = SharedVisionTransport(path)
            t.reset(); client = os.open(path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
            try:
                t.tx.clear(); os.write(client, command()); t.poll(.1)
                self.assertFalse(t.active)
                t.feed_usb(status() + telemetry(), .2)
                os.write(client, command()); t.poll(.21)
                self.assertTrue(t.active)
                t.poll(1.); self.assertFalse(t.active)
                t.reset(); self.assertFalse(t.active)
                t.tx.clear(); t.poll(1.1); self.assertFalse(t.tx.pending)
            finally: os.close(client); t.close()
            self.assertFalse(os.path.lexists(path))
            Path(path).write_text('owned by user')
            with self.assertRaises(FileExistsError): SharedVisionTransport(path)
            self.assertEqual(Path(path).read_text(), 'owned by user')

if __name__ == '__main__': unittest.main()

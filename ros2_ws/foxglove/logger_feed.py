"""Read the existing RTT logger's text and telemetry without opening a debug probe."""

from __future__ import annotations

import asyncio
from collections.abc import Callable, Iterator
import re
import struct
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from scripts.rtt_common.telemetry import TelemetryFrame

try:
    from websockets.exceptions import WebSocketException
    from websockets.legacy.client import connect
except ImportError:
    connect = None


# Match scripts/dashboard/rtt_ws_bridge.py. Each packet contains one chunk,
# which may begin or end in the middle of a firmware log line.
PACKET_HEADER = struct.Struct("<HBBIQ")
PACKET_MAGIC = 0x5753
PACKET_VERSION = 1
STREAM_LOG = 0
STREAM_DASHBOARD = 1
MAX_PAYLOAD = 4096
MAX_LINE = 4096
RECONNECT_DELAY = 0.5
ANSI_ESCAPE = re.compile(
    r"\x1b(?:\[[0-?]*[ -/]*[@-~]|\][^\x07\x1b]*(?:\x07|\x1b\\)|[ -/]*[@-Z\\-_])"
)
CONTROL_CHARACTER = re.compile(r"[\x00-\x08\x0b-\x1f\x7f-\x9f]")
FIRMWARE_LEVELS = {"I:": "info", "W:": "warning", "E:": "error"}


def _parse_packet(packet: bytes | str) -> tuple[int, int, bytes] | None:
    if not isinstance(packet, bytes) or len(packet) < PACKET_HEADER.size:
        return None
    magic, version, stream, length, host_rx_ms = PACKET_HEADER.unpack_from(packet)
    if (magic, version) != (PACKET_MAGIC, PACKET_VERSION):
        return None
    if length > MAX_PAYLOAD or len(packet) != PACKET_HEADER.size + length:
        return None
    return stream, host_rx_ms, packet[PACKET_HEADER.size:]


class _LogLines:
    def __init__(self) -> None:
        self._partial = bytearray()
        self._truncated = False

    def feed(self, payload: bytes) -> Iterator[tuple[str, str]]:
        parts = payload.split(b"\n")
        for index, part in enumerate(parts):
            remaining = MAX_LINE - len(self._partial)
            self._partial.extend(part[:remaining])
            self._truncated |= len(part) > remaining
            if index == len(parts) - 1:
                break

            line = self._partial.decode("utf-8", errors="replace")
            line = CONTROL_CHARACTER.sub("", ANSI_ESCAPE.sub("", line)).strip()
            if line:
                level = FIRMWARE_LEVELS.get(line[:2])
                if level is not None:
                    line = line[2:].lstrip()
                if self._truncated:
                    line += " [truncated]"
                if line:
                    yield level or "info", line
            self._partial.clear()
            self._truncated = False


class LoggerFeed:
    """Deliver text lines and optional telemetry from a local logger on a background thread."""

    def __init__(
        self,
        port: int,
        log: Callable[[str, str], None],
        telemetry: Callable[[TelemetryFrame], None] | None = None,
    ) -> None:
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("Logger port must be between 1 and 65535")
        self._url = f"ws://127.0.0.1:{port}/"
        self._log = log
        self._telemetry = telemetry
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._connected = threading.Event()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._task: asyncio.Task | None = None
        self._missing_reported = False

    @property
    def connected(self) -> bool:
        """Whether the local WebSocket connection is open."""
        return self._connected.is_set() and not self._stopping.is_set()

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            if connect is not None:
                self._stopping.clear()
                self._thread = threading.Thread(
                    target=self._run, name="rtt-log-feed", daemon=True,
                )
                self._thread.start()
                return
            if self._missing_reported:
                return
            self._missing_reported = True
        self._log("warning", "RTT log feed unavailable: Python websockets package is missing")

    def stop(self) -> None:
        with self._lock:
            self._stopping.set()
            thread = self._thread
            if self._loop is not None and self._task is not None:
                self._loop.call_soon_threadsafe(self._task.cancel)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def _run(self) -> None:
        try:
            asyncio.run(self._read())
        except asyncio.CancelledError:
            pass

    async def _read(self) -> None:
        telemetry = self._telemetry
        parser_factory = None
        if telemetry is not None:
            from scripts.rtt_common.telemetry import FrameParser

            parser_factory = FrameParser

        with self._lock:
            self._loop = asyncio.get_running_loop()
            self._task = asyncio.current_task()
        try:
            while not self._stopping.is_set():
                try:
                    # The legacy client connects directly to localhost without
                    # reading proxy environment variables, including on v15+.
                    async with connect(
                        self._url, open_timeout=1.0, close_timeout=0.25,
                        ping_interval=5.0, ping_timeout=2.0, max_queue=8,
                        max_size=PACKET_HEADER.size + MAX_PAYLOAD,
                    ) as websocket:
                        self._connected.set()
                        lines = _LogLines()
                        parser = parser_factory() if parser_factory is not None else None
                        async for packet in websocket:
                            parsed = _parse_packet(packet)
                            if parsed is None:
                                continue
                            stream, host_rx_ms, payload = parsed
                            if stream == STREAM_LOG:
                                for level, line in lines.feed(payload):
                                    self._log(level, line)
                            elif stream == STREAM_DASHBOARD and parser is not None and telemetry is not None:
                                frames = parser.feed(payload, host_rx_ms)
                                if frames:
                                    telemetry(frames[-1])
                except (OSError, asyncio.TimeoutError, WebSocketException):
                    pass
                finally:
                    self._connected.clear()
                if not self._stopping.is_set():
                    await asyncio.sleep(RECONNECT_DELAY)
        finally:
            with self._lock:
                self._task = None
                self._loop = None

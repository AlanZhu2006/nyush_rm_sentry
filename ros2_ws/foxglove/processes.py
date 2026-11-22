"""Run and stop the NUC operator's Just recipes without a shell."""

import logging
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
from typing import BinaryIO, Callable


MAX_LOG_LINE_BYTES = 4096


def _session_groups(session_id: int) -> set[int]:
    """Include groups created by job control inside the Linux process session."""
    groups = set()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            stat = (entry / 'stat').read_text(errors='replace')
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        # Process names can contain spaces and parentheses.
        fields = stat.rsplit(')', 1)[-1].split()
        if len(fields) > 3 and fields[0] not in ('Z', 'X') and int(fields[3]) == session_id:
            groups.add(int(fields[2]))
    return groups


def _signal_group(group: int, sig: signal.Signals) -> None:
    try:
        os.killpg(group, sig)
    except ProcessLookupError:
        pass


class ProcessManager:
    def __init__(self, project: Path, env: dict[str, str], log: Callable[[str, str], None]):
        self._project = project.resolve()
        self._env = dict(env)
        self._log = log
        self._processes: dict[str, subprocess.Popen[bytes]] = {}
        self._lock = threading.Lock()
        self._closed = False

    def start(self, key: str, args: list[str], env_overrides: dict[str, str] | None = None) -> None:
        with self._lock:
            if self._closed:
                raise RuntimeError('Process manager is closed')
            previous = self._processes.get(key)
            if previous is not None and self._is_running(previous):
                raise RuntimeError(f'{key} is already running')
            env = self._env | (env_overrides or {})
            runner = shutil.which('just', path=env.get('PATH'))
            if not runner:
                raise FileNotFoundError('Could not find just in PATH')
            process = subprocess.Popen(
                [runner, '--justfile', str(self._project / 'justfile'), *args],
                cwd=self._project, env=env, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True,
            )
            self._processes[key] = process
            if process.stdout is not None:
                threading.Thread(
                    target=self._read_output, args=(key, process.stdout), daemon=True,
                ).start()

    def _read_output(self, key: str, output: BinaryIO) -> None:
        report = self._log
        with output:
            while fragment := output.readline(MAX_LOG_LINE_BYTES + 1):
                truncated = len(fragment.rstrip(b'\r\n')) > MAX_LOG_LINE_BYTES
                line = fragment[:MAX_LOG_LINE_BYTES].rstrip(b'\r\n').decode(errors='replace')
                if truncated:
                    line = line[:-1] + '…'
                if report is not None:
                    try:
                        report(key, line)
                    except Exception:
                        logging.getLogger(__name__).exception('Output callback failed for %s', key)
                        # A broken callback must not block the child's output pipe.
                        report = None
                while fragment and not fragment.endswith(b'\n'):
                    fragment = output.readline(MAX_LOG_LINE_BYTES + 1)

    def poll(self, key: str) -> int | None:
        with self._lock:
            process = self._processes.get(key)
        return process.poll() if process is not None else None

    @staticmethod
    def _is_running(process: subprocess.Popen[bytes]) -> bool:
        return process.poll() is None or bool(_session_groups(process.pid))

    def running(self, key: str) -> bool:
        with self._lock:
            process = self._processes.get(key)
        return process is not None and self._is_running(process)

    @staticmethod
    def _wait_for_exit(process: subprocess.Popen[bytes], timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while ProcessManager._is_running(process):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            time.sleep(min(0.05, remaining))
        return True

    def stop(self, key: str, grace: float = 12) -> int | None:
        with self._lock:
            process = self._processes.get(key)
        if process is None:
            return None

        # The mapping wrapper traps SIGINT and stops its own ROS launch group.
        # Give it time to finish even if the just parent exits first.
        if process.pid in _session_groups(process.pid):
            _signal_group(process.pid, signal.SIGINT)
        if not self._wait_for_exit(process, grace):
            for sig in (signal.SIGTERM, signal.SIGKILL):
                for group in _session_groups(process.pid):
                    _signal_group(group, sig)
                if self._wait_for_exit(process, 2):
                    break
            else:
                raise TimeoutError(f'{key} did not stop after SIGKILL')
        return process.poll()

    def close(self) -> None:
        with self._lock:
            self._closed = True
            keys = tuple(reversed(self._processes))
        failure = None
        for key in keys:
            try:
                self.stop(key)
            except (OSError, TimeoutError) as error:
                failure = failure or error
        if failure is not None:
            raise failure

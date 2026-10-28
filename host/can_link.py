"""
can_link.py -- the Jetson's CAN interface, with two backends.

  SimLink  : spawns tools/rover_sim, the REAL C++ controller compiled natively,
             and exchanges frames over a pipe. No hardware, no CAN drivers, and
             crucially no second implementation of the safety logic.
  SocketCanLink : python-can on a real interface (can0, or vcan0 for a Linux
             virtual bus). Imported lazily so the sim path needs no install.

Both expose the same two calls, so jetson_test.py does not know or care which
it is talking to:
    send(can_id, payload_bytes)
    recv() -> list of (can_id, payload_bytes) received since the last call
"""

import os
import queue
import shutil
import subprocess
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol_root  # noqa: E402
from rover_config import CAN_BITRATE_HZ  # noqa: E402

# Resolved through protocol_root so that every reader of firmware/src -- this
# module, the config-sync test and the golden-vector test -- agrees on which
# checkout is in play. They did not always, and the suite went green over a
# version mismatch as a result; see host/protocol_root.py.
REPO = protocol_root.protocol_root()
SIM_SOURCES = [
    protocol_root.tools("rover_sim.cpp"),
    protocol_root.firmware_src("rover_controller.cpp"),
    protocol_root.firmware_src("rover_protocol.cpp"),
    protocol_root.firmware_src("rover_drive.cpp"),
]
# Headers count too: a constant changed only in rover_config.h must still
# rebuild the simulator, or the tests run against stale message sizes.
SIM_HEADERS = [
    protocol_root.firmware_src(name)
    for name in ("rover_config.h", "rover_protocol.h", "rover_controller.h",
                 "rover_drive.h")
]
SIM_BINARY = os.path.join(protocol_root.build_dir(), "rover_sim")


def build_sim(force=False):
    """Compile the simulator if needed. Returns its path."""
    # Checked by name, and before the staleness comparison below, because the
    # alternative symptoms are all worse: an OSError from getmtime, a compiler
    # error naming a header nobody pointed at, or -- worst -- a cached binary
    # from an earlier run that still looks perfectly good.
    missing = [p for p in SIM_SOURCES + SIM_HEADERS if not os.path.exists(p)]
    if missing:
        hint = ("" if not protocol_root.is_overridden() else
                "; %s is set, and must point at the root of a "
                "protocol-repository checkout" % protocol_root.ENV_VAR)
        raise RuntimeError("firmware sources missing under %s: %s%s"
                           % (REPO, ", ".join(os.path.relpath(m, REPO)
                                              for m in missing), hint))
    if not force and os.path.exists(SIM_BINARY):
        newest_src = max(os.path.getmtime(s) for s in SIM_SOURCES + SIM_HEADERS)
        if os.path.getmtime(SIM_BINARY) >= newest_src:
            return SIM_BINARY
    cxx = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    if cxx is None:
        raise RuntimeError("no C++ compiler found; the simulator is compiled from "
                           "firmware/src so it cannot drift from the firmware")
    os.makedirs(os.path.dirname(SIM_BINARY), exist_ok=True)
    subprocess.run([cxx, "-std=c++17", "-O1"] + SIM_SOURCES + ["-o", SIM_BINARY],
                   check=True, capture_output=True)
    return SIM_BINARY


class SimLink:
    """Talks to the compiled C++ controller over a pipe.

    Frames are collected by a reader thread rather than by polling the pipe
    with selectors. Polling the file descriptor is wrong here: Python's
    buffered reader pulls several lines into its own buffer at once, so the
    OS-level fd reports "nothing to read" while complete frames are already
    sitting in userspace waiting. That silently drops telemetry.
    """

    def __init__(self):
        self.proc = subprocess.Popen(
            [build_sim()], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        self._frames = queue.Queue()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        for line in self.proc.stdout:          # blocks in the thread, not the caller
            parts = line.split()
            if len(parts) == 2:
                try:
                    self._frames.put((int(parts[0], 16), bytes.fromhex(parts[1])))
                except ValueError:
                    pass                       # malformed line from a crashed sim

    def send(self, can_id, payload):
        try:
            self.proc.stdin.write("%03x %s\n" % (can_id, payload.hex()))
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass   # simulator exited; recv() will report nothing further

    def recv(self):
        frames = []
        while True:
            try:
                frames.append(self._frames.get_nowait())
            except queue.Empty:
                return frames

    def close(self):
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self._reader.join(timeout=1)


class SocketCanLink:
    """python-can on a real (or virtual) CAN interface."""

    def __init__(self, channel="can0", bitrate=CAN_BITRATE_HZ):
        import can   # lazy: the sim path must not require python-can
        self.bus = can.Bus(channel=channel, interface="socketcan", bitrate=bitrate)

    def send(self, can_id, payload):
        import can
        self.bus.send(can.Message(arbitration_id=can_id, data=payload,
                                  is_extended_id=False))

    def recv(self):
        frames = []
        while True:
            msg = self.bus.recv(timeout=0)
            if msg is None:
                return frames
            frames.append((msg.arbitration_id, bytes(msg.data)))

    def close(self):
        self.bus.shutdown()

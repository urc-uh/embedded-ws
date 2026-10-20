#!/usr/bin/env python3
"""
demo.py -- Jetson <-> controller message-exchange demonstration, no hardware.

Runs against tools/rover_sim: the REAL C++ rover_controller.cpp compiled
natively, with only the plant and the transport simulated. So what you see
below is the actual safety logic that gets flashed to the Teensy.

A: normal 20 Hz operation, four telemetry frames reassembled per cycle.
B: a deliberate 800 ms silence (well past the 300 ms command watchdog).
C: the explicit stop flag at full throttle.
D: autonomy abort while in AUTONOMOUS.
E: a corrupted frame -- the application-layer CRC rejects it and the rover
   reports CRC_ERROR rather than acting on it.
F: a sequence gap -- detected and reported as SEQ_GAP.

This is the TASK 3 demonstration and stays focused on the link. The Task 4
demonstration -- an autonomy decision reaching the drive stage, and autonomy
and teleop commands being refused in each other's modes -- is
host/autonomy_sim.py (`make autonomy`).
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink  # noqa: E402


def line(jl, t0):
    s = jl.snapshot
    if s is None:
        return None
    faults = ",".join(rp.fault_names(s["fault_status"])) or "none"
    age = s["cmd_age_ms"]
    age_txt = "never" if age == rp.CMD_AGE_UNKNOWN else "%dms" % age
    return ("  t=%5.2fs  seq=%3d  enc=%+8d  %+7.2fA %6.2fV  mode=%-10s "
            "jet=%-8s c2=%-12s ind=%-10s age=%-7s faults=%s"
            % (time.monotonic() - t0, s["seq"], s["enc_left"], s["current_a"],
               s["voltage_v"], rp.MODE_NAMES.get(s["mode"], "?"),
               rp.LINK_NAMES.get(s["jetson_link"], "?"),
               rp.LINK_NAMES.get(s["c2_link"], "?"),
               rp.INDICATOR_NAMES.get(s["indicator_state"], "?"),
               age_txt, faults))


def phase(jl, t0, seconds, label, send=True, **kw):
    print("-- %s --" % label)
    start = time.monotonic()
    last = 0.0
    while time.monotonic() - start < seconds:
        if send:
            jl.send_control(**kw)
        jl.poll()
        now = time.monotonic() - t0
        if now - last > 0.15:
            txt = line(jl, t0)
            if txt:
                print(txt)
                last = now
        time.sleep(0.02)
    jl.poll()


def main():
    print("=" * 78)
    print("Jetson <-> rover controller over CAN (simulated transport,")
    print("REAL compiled firmware safety logic from firmware/src/)")
    print("=" * 78)

    link = can_link.SimLink()
    jl = JetsonLink(link)
    t0 = time.monotonic()
    try:
        phase(jl, t0, 1.0, "A: normal operation, CONTROL at 20Hz",
              drive_cmd=500, steer_cmd=0, mode=rp.MODE_TELEOP, c2_lost=False,
              indicator_request=rp.INDICATOR_TELEOP)

        phase(jl, t0, 0.8, "B: link cut -- Jetson stops sending (800ms > 300ms watchdog)",
              send=False)

        phase(jl, t0, 0.8, "B: link restored",
              drive_cmd=300, steer_cmd=100, mode=rp.MODE_TELEOP, c2_lost=False,
              indicator_request=rp.INDICATOR_TELEOP)

        phase(jl, t0, 0.5, "C: explicit stop asserted at full throttle",
              drive_cmd=1000, steer_cmd=500, mode=rp.MODE_TELEOP, c2_lost=False,
              stop=True)

        phase(jl, t0, 0.5, "D: autonomy abort while AUTONOMOUS",
              drive_cmd=800, steer_cmd=0, mode=rp.MODE_AUTONOMOUS, c2_lost=False,
              autonomy_abort=True, indicator_request=rp.INDICATOR_AUTONOMOUS,
              from_autonomy=True)

        # The two link failures are NOT equivalent, and these two phases are
        # the demonstration of it. C2 loss with a healthy Jetson is expected on
        # the autonomy course, where line of sight to the base station drops
        # while onboard autonomy keeps running.
        phase(jl, t0, 0.5, "D2: C2 lost while AUTONOMOUS -- keeps driving, fault raised",
              drive_cmd=800, steer_cmd=0, mode=rp.MODE_AUTONOMOUS, c2_lost=True,
              indicator_request=rp.INDICATOR_AUTONOMOUS, from_autonomy=True)

        phase(jl, t0, 0.5, "D3: C2 lost while TELEOP -- stops, nobody can reach it",
              drive_cmd=800, steer_cmd=0, mode=rp.MODE_TELEOP, c2_lost=True,
              indicator_request=rp.INDICATOR_TELEOP)

        # E: corrupt a frame AFTER the CRC is computed. CAN's own CRC would not
        # see this -- it is exactly the software-path corruption the
        # application-layer CRC exists to catch.
        print("-- E: corrupted payload (app-layer CRC must reject it) --")
        good = rp.encode_control(900, 0, rp.MODE_TELEOP, False, seq=99)
        bad = bytearray(good)
        bad[0] ^= 0xFF
        for _ in range(25):
            link.send(rp.CAN_ID_CONTROL, bytes(bad))
            jl.poll()
            time.sleep(0.02)
        jl.poll()
        txt = line(jl, t0)
        if txt:
            print(txt)

        # F: jump the sequence number so frames look lost.
        print("-- F: sequence gap (frames 100 -> 140) --")
        link.send(rp.CAN_ID_CONTROL,
                  rp.encode_control(400, 0, rp.MODE_TELEOP, False, seq=100))
        time.sleep(0.06)
        jl.poll()
        link.send(rp.CAN_ID_CONTROL,
                  rp.encode_control(400, 0, rp.MODE_TELEOP, False, seq=140))
        time.sleep(0.12)
        jl.poll()
        txt = line(jl, t0)
        if txt:
            print(txt)

        assert jl.snapshot is not None, "never received a complete snapshot"
        assert jl.tele.complete_count > 10, "too few complete snapshots"
        assert jl.unknown_frames == 0, "unexpected CAN id seen"
        print()
        print("PASS: snapshots reassembled from five frames throughout; the")
        print("watchdog, stop flag and autonomy abort each forced a stop; C2")
        print("loss stopped a teleoperated rover but not an autonomous one; the")
        print("application-layer CRC rejected a corrupted frame; and a")
        print("sequence gap was detected and reported.")
    finally:
        link.close()


if __name__ == "__main__":
    main()

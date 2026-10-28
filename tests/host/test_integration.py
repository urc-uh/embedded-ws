"""
test_integration.py -- end-to-end Jetson <-> controller over the simulated bus.

The "controller" here is tools/rover_sim, which links the REAL
firmware/src/rover_controller.cpp. There is no Python reimplementation of the
safety logic to drift from the firmware, which is the mistake this project
already made once and fixed.

These tests involve a real subprocess and real time, so they use generous
margins (the watchdog is 300 ms; cuts are 700 ms+) and assert on eventual
state rather than exact frame counts. A flaky safety test gets ignored, which
is worse than no test.
"""

import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import can_link  # noqa: E402
import rover_protocol as rp  # noqa: E402
from jetson_test import JetsonLink  # noqa: E402


class RoverFixture(unittest.TestCase):
    def setUp(self):
        try:
            self.link = can_link.SimLink()
        except RuntimeError as exc:
            self.skipTest(str(exc))
        self.jl = JetsonLink(self.link)

    def tearDown(self):
        self.link.close()

    def drive(self, seconds, send=True, **kw):
        """Send CONTROL at 20 Hz (or stay silent) for `seconds`, polling."""
        kw.setdefault("drive_cmd", 400)
        kw.setdefault("steer_cmd", 100)
        kw.setdefault("mode", rp.MODE_TELEOP)
        kw.setdefault("c2_lost", False)
        # Default to the producer that OWNS the mode being commanded, so a
        # test that is not about authority does not have to think about it --
        # and the ones that are say so explicitly.
        kw.setdefault("from_autonomy", kw["mode"] == rp.MODE_AUTONOMOUS)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if send:
                self.jl.send_control(**kw)
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()

    def send_raw(self, frame, seconds):
        """Put arbitrary bytes on the CONTROL id, the way a faulty or
        mismatched sender would."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.link.send(rp.CAN_ID_CONTROL, frame)
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()

    def snap(self):
        self.assertIsNotNone(self.jl.snapshot,
                             "no complete telemetry snapshot was assembled")
        return self.jl.snapshot

    def faults(self):
        return rp.fault_names(self.snap()["fault_status"])


class TestSnapshotAssembly(RoverFixture):
    """The five telemetry frames must reassemble into coherent snapshots."""

    def test_all_five_frames_arrive_and_assemble(self):
        self.drive(0.6)
        self.assertGreater(self.jl.tele.complete_count, 5)
        for can_id in rp.TELEM_IDS:
            self.assertGreater(self.jl.tele.frames_by_id[can_id], 5,
                               "no frames on id 0x%03x" % can_id)
        self.assertEqual(self.jl.tele.decode_errors, {})
        self.assertEqual(self.jl.unknown_frames, 0)

    def test_snapshots_are_not_torn(self):
        # All five frames of a cycle share one sequence number. Over a clean
        # link every snapshot should be coherent.
        self.drive(1.0)
        self.assertEqual(self.jl.tele.torn_count, 0,
                         "snapshots were torn across cycles on a clean link")

    def test_the_snapshot_sequence_number_advances(self):
        self.drive(0.4)
        first = self.snap()["seq"]
        self.drive(0.4)
        self.assertNotEqual(self.snap()["seq"], first)

    def test_a_snapshot_carries_every_required_field(self):
        # The task enumerates these; this is the end-to-end check that each one
        # actually reaches the Jetson rather than merely being defined.
        self.drive(0.6)
        s = self.snap()
        for field in ("seq", "enc_left", "enc_right", "steer_fb", "current_ca",
                      "voltage_cv", "mode", "fault_status", "cmd_age_ms",
                      "jetson_link", "c2_link", "controller_health",
                      "indicator_state",
                      # Task 4's feedback loop adds these three. The accepted
                      # commands are what let the Jetson tell "my command got
                      # through" from "the rover happens to be moving".
                      "drive_applied", "steer_applied", "ack_seq",
                      "cmd_status", "last_decode", "return_active"):
            self.assertIn(field, s, "telemetry is missing %r" % field)


class TestEndToEnd(RoverFixture):
    def test_normal_operation(self):
        self.drive(0.6, indicator_request=rp.INDICATOR_TELEOP)
        s = self.snap()
        self.assertEqual(self.faults(), [])
        self.assertGreater(s["enc_left"], 0, "rover never moved")
        self.assertGreater(s["current_a"], 0)
        self.assertGreater(s["voltage_v"], 20.0)
        self.assertEqual(s["mode"], rp.MODE_TELEOP)
        self.assertEqual(s["jetson_link"], rp.LINK_OK)
        self.assertEqual(s["indicator_state"], rp.INDICATOR_TELEOP)

    def test_link_cut_trips_comm_timeout_then_recovers(self):
        self.drive(0.6)
        self.assertEqual(self.faults(), [])
        moving = self.snap()["enc_left"]

        self.drive(0.7, send=False)
        self.assertIn("COMM_TIMEOUT", self.faults())
        self.assertEqual(self.snap()["current_ca"], 0)
        self.assertEqual(self.snap()["jetson_link"], rp.LINK_LOST)
        # A faulted rover must not display a cheerful indicator.
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_FAULT)
        frozen = self.snap()["enc_left"]
        self.assertGreater(frozen, moving)

        self.drive(0.4, send=False)
        self.assertEqual(self.snap()["enc_left"], frozen,
                         "kept moving while commands were stale")

        self.drive(0.6)
        self.assertEqual(self.faults(), [], "fault did not clear on its own")
        self.assertGreater(self.snap()["enc_left"], frozen)

    def test_stop_flag_is_honoured(self):
        self.drive(0.5)
        running = self.snap()["enc_left"]
        self.drive(0.5, drive_cmd=1000, steer_cmd=500, stop=True)
        halted = self.snap()["enc_left"]
        self.assertEqual(self.snap()["current_ca"], 0)
        self.drive(0.4, drive_cmd=1000, steer_cmd=500, stop=True)
        self.assertEqual(self.snap()["enc_left"], halted,
                         "moved while stop was asserted")
        # Real subprocess, real latency: the previous command is still in
        # effect for an iteration or two. Bounded coast then a full freeze is
        # the property that matters, not a single-cycle stop.
        self.assertLess(halted - running, 500, "coasted far too long")
        self.drive(0.5)
        self.assertGreater(self.snap()["enc_left"], halted)

    def test_disabled_mode_never_moves(self):
        self.drive(0.6, drive_cmd=1000, steer_cmd=1000, mode=rp.MODE_SAFE)
        self.assertEqual(self.snap()["enc_left"], 0)
        self.assertEqual(self.snap()["current_ca"], 0)
        self.assertEqual(self.snap()["mode"], rp.MODE_SAFE)


class TestNewCommandFields(RoverFixture):
    def test_autonomy_abort_stops_an_autonomous_rover(self):
        self.drive(0.5, drive_cmd=800, mode=rp.MODE_AUTONOMOUS)
        moving = self.snap()["enc_left"]
        self.assertGreater(moving, 0)
        self.drive(0.5, drive_cmd=800, mode=rp.MODE_AUTONOMOUS,
                   autonomy_abort=True)
        halted = self.snap()["enc_left"]
        self.drive(0.4, drive_cmd=800, mode=rp.MODE_AUTONOMOUS,
                   autonomy_abort=True)
        self.assertEqual(self.snap()["enc_left"], halted,
                         "kept driving autonomously after an abort")
        self.assertEqual(self.snap()["current_ca"], 0)

    def test_autonomy_abort_does_not_stop_a_teleoperated_rover(self):
        # In TELEOP the operator is already driving; there is no autonomous
        # task to abort, and stopping would surprise them mid-manoeuvre.
        self.drive(0.6, drive_cmd=600, mode=rp.MODE_TELEOP, autonomy_abort=True)
        self.assertGreater(self.snap()["enc_left"], 0)
        self.assertGreater(self.snap()["current_ca"], 0)

    def test_the_indicator_reports_what_the_rover_is_actually_doing(self):
        # URC scores this light on the rover's real state, so the controller's
        # own mode overrides the Jetson's request. Comparing request against
        # state is how an operator sees whether it was honoured.
        for requested in (rp.INDICATOR_TELEOP, rp.INDICATOR_AUTONOMOUS,
                          rp.INDICATOR_ARRIVED):
            self.drive(0.4, mode=rp.MODE_TELEOP, indicator_request=requested)
            self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_TELEOP,
                             "a teleoperated rover showed %d" % requested)

        self.drive(0.4, mode=rp.MODE_AUTONOMOUS,
                   indicator_request=rp.INDICATOR_TELEOP)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_AUTONOMOUS)

        # ARRIVED is the one request the controller cannot derive itself.
        self.drive(0.4, mode=rp.MODE_AUTONOMOUS,
                   indicator_request=rp.INDICATOR_ARRIVED)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_ARRIVED)

        self.drive(0.4, mode=rp.MODE_SAFE,
                   indicator_request=rp.INDICATOR_AUTONOMOUS)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_OFF)

    def test_mode_is_echoed_back(self):
        # Without the echo a dropped or misread mode change is invisible.
        for mode in (rp.MODE_TELEOP, rp.MODE_AUTONOMOUS, rp.MODE_SAFE):
            self.drive(0.4, mode=mode)
            self.assertEqual(self.snap()["mode"], mode)


class TestLinkSeparation(RoverFixture):
    """C2 loss and Jetson loss must never be treated as equivalent."""

    def test_c2_link_is_reported_not_reported_before_any_frame(self):
        # Honest "nobody told us" rather than a misleading OK. The controller
        # has no radio, so this is the only truthful answer until the Jetson
        # forwards its verdict.
        self.drive(0.3, send=False)
        snap = self.snap()
        self.assertEqual(snap["c2_link"], rp.LINK_NOT_REPORTED)

    def test_the_forwarded_c2_verdict_reaches_telemetry(self):
        self.drive(0.6, c2_lost=False)
        self.assertEqual(self.snap()["c2_link"], rp.LINK_OK)
        self.drive(0.6, mode=rp.MODE_AUTONOMOUS, c2_lost=True)
        self.assertEqual(self.snap()["c2_link"], rp.LINK_LOST)
        self.assertTrue(self.snap()["fault_status"] & rp.FAULT_C2_LINK_LOST)

    def test_a_healthy_jetson_link_does_not_imply_a_healthy_c2_link(self):
        self.drive(0.6, mode=rp.MODE_AUTONOMOUS, c2_lost=True)
        snap = self.snap()
        self.assertEqual(snap["jetson_link"], rp.LINK_OK)
        self.assertEqual(snap["c2_link"], rp.LINK_LOST,
                         "C2 was reported OK purely because the Jetson link was")

    def test_c2_loss_stops_teleop_but_not_autonomy(self):
        # The asymmetry the autonomy course requires: line of sight to the
        # base station drops while onboard autonomy keeps running.
        self.drive(0.6, drive_cmd=800, mode=rp.MODE_AUTONOMOUS, c2_lost=True)
        self.assertNotEqual(self.snap()["current_a"], 0.0,
                            "an autonomous rover was stopped by C2 loss alone")

        self.drive(0.6, drive_cmd=800, mode=rp.MODE_TELEOP, c2_lost=True)
        self.assertEqual(self.snap()["current_a"], 0.0,
                         "a teleoperated rover kept driving with C2 lost")

    def test_a_stale_c2_bit_is_reported_as_not_reported(self):
        # Once the Jetson goes silent the last c2_lost value is stale. A clear
        # bit from before the silence does not mean C2 is fine now.
        self.drive(0.4, mode=rp.MODE_AUTONOMOUS, c2_lost=False)
        self.assertEqual(self.snap()["c2_link"], rp.LINK_OK)
        self.drive(0.6, send=False)              # past the 300 ms watchdog
        snap = self.snap()
        self.assertTrue(snap["fault_status"] & rp.FAULT_COMM_TIMEOUT)
        self.assertEqual(snap["c2_link"], rp.LINK_NOT_REPORTED)
        self.assertFalse(snap["fault_status"] & rp.FAULT_C2_LINK_LOST,
                         "a stale C2 verdict was reported as a live fault")

    def test_the_two_link_fields_are_independent_on_the_wire(self):
        for jl in (rp.LINK_OK, rp.LINK_DEGRADED, rp.LINK_LOST):
            for c2 in (rp.LINK_OK, rp.LINK_DEGRADED, rp.LINK_LOST,
                       rp.LINK_NOT_REPORTED):
                frame = rp.encode_telemetry_state(
                    rp.MODE_TELEOP, jl, c2, rp.CTRL_HEALTH_OK,
                    rp.INDICATOR_TELEOP, seq=1)
                _, d = rp.decode_telemetry_state(frame)
                self.assertEqual(d["jetson_link"], jl)
                self.assertEqual(d["c2_link"], c2)


class TestBusHygiene(RoverFixture):
    def test_foreign_can_id_does_not_command_the_rover(self):
        self.drive(0.4)
        end = time.monotonic() + 0.7
        while time.monotonic() < end:
            self.link.send(0x321, rp.encode_control(1000, 0, rp.MODE_TELEOP, False))
            self.jl.poll()
            time.sleep(0.02)
        self.jl.poll()
        self.assertIn("COMM_TIMEOUT", self.faults(),
                      "a foreign id kept the command watchdog alive")
        self.assertEqual(self.snap()["current_ca"], 0)

    def test_a_corrupted_frame_is_rejected_and_reported(self):
        # Corrupted AFTER the CRC is computed -- exactly the software-path
        # failure CAN's own CRC cannot see.
        self.drive(0.4)
        good = rp.encode_control(900, 0, rp.MODE_TELEOP, False, seq=50)
        bad = bytearray(good)
        bad[0] ^= 0xFF
        self.send_raw(bytes(bad), 0.7)
        self.assertIn("CRC_ERROR", self.faults())
        self.assertIn("COMM_TIMEOUT", self.faults(),
                      "a CRC-invalid frame refreshed the command watchdog")
        self.assertEqual(self.snap()["current_ca"], 0)

    def test_undefined_mode_is_rejected_and_does_not_move_the_rover(self):
        payload = struct.pack(rp.CONTROL_FMT, 1000, 500, 7, 0)
        body = payload + bytes((1,))
        frame = body + bytes((rp.frame_crc8(rp.CAN_ID_CONTROL, body),))
        self.send_raw(frame, 0.7)
        self.assertEqual(self.snap()["enc_left"], 0,
                         "rover moved in an undefined mode")
        self.assertIn("COMM_TIMEOUT", self.faults())

    def test_a_sequence_gap_is_detected_and_reported(self):
        self.drive(0.4)
        for seq in (10, 60):          # a 50-frame jump
            self.link.send(rp.CAN_ID_CONTROL,
                           rp.encode_control(400, 0, rp.MODE_TELEOP, False, seq=seq))
            time.sleep(0.08)
            self.jl.poll()
        self.assertIn("SEQ_GAP", self.faults())
        self.assertEqual(self.snap()["jetson_link"], rp.LINK_DEGRADED,
                         "a sequence gap did not degrade the reported link")


class TestCommandAuthority(RoverFixture):
    """Task 4's core claim: manual and autonomous commands do not get mixed up.

    These run against the real compiled controller over a real pipe, so what
    they check is the behaviour a rover would show, not a predicate.
    """

    def test_an_autonomy_command_is_not_acted_on_while_teleoperated(self):
        self.drive(0.6, drive_cmd=600, steer_cmd=0, mode=rp.MODE_TELEOP,
                   from_autonomy=True)
        s = self.snap()
        self.assertEqual(s["enc_left"], 0, "an unauthorised command drove the rover")
        self.assertEqual(s["drive_applied"], 0)
        self.assertEqual(s["cmd_status"], rp.CMD_STATUS_WRONG_SOURCE)
        self.assertIn("CMD_SOURCE", self.faults())
        # Refused, not disconnected: the frame was valid and the link is fine.
        self.assertNotIn("COMM_TIMEOUT", self.faults())
        self.assertEqual(s["jetson_link"], rp.LINK_OK)

    def test_the_same_command_drives_once_the_mode_matches(self):
        # The demonstration the task asks for, end to end: identical command,
        # only the mode differs.
        self.drive(0.6, drive_cmd=600, steer_cmd=0, mode=rp.MODE_TELEOP,
                   from_autonomy=True)
        refused = self.snap()["enc_left"]
        self.assertEqual(refused, 0)

        self.drive(0.6, drive_cmd=600, steer_cmd=0, mode=rp.MODE_AUTONOMOUS,
                   from_autonomy=True)
        s = self.snap()
        self.assertGreater(s["enc_left"], 0, "the authorised command did not drive")
        self.assertEqual(s["drive_applied"], 600)
        self.assertEqual(s["cmd_status"], rp.CMD_STATUS_APPLIED)
        self.assertNotIn("CMD_SOURCE", self.faults())

    def test_a_teleop_command_is_not_acted_on_while_autonomous(self):
        self.drive(0.6, drive_cmd=600, steer_cmd=0, mode=rp.MODE_AUTONOMOUS,
                   from_autonomy=False)
        self.assertEqual(self.snap()["enc_left"], 0)
        self.assertEqual(self.snap()["cmd_status"], rp.CMD_STATUS_WRONG_SOURCE)

    def test_stop_still_overrides_an_authorised_autonomy_command(self):
        self.drive(0.5, drive_cmd=800, mode=rp.MODE_AUTONOMOUS)
        moving = self.snap()["enc_left"]
        self.assertGreater(moving, 0)
        self.drive(0.5, drive_cmd=800, mode=rp.MODE_AUTONOMOUS, stop=True)
        halted = self.snap()["enc_left"]
        self.drive(0.4, drive_cmd=800, mode=rp.MODE_AUTONOMOUS, stop=True)
        self.assertEqual(self.snap()["enc_left"], halted)
        self.assertEqual(self.snap()["cmd_status"], rp.CMD_STATUS_STOPPED)

    def test_a_stop_from_the_unauthorised_source_still_stops_the_rover(self):
        # Safety flags are never gated by authority: an operator who can see
        # the rover must be able to stop it even while autonomy owns the mode.
        self.drive(0.5, drive_cmd=800, mode=rp.MODE_AUTONOMOUS)
        self.assertGreater(self.snap()["enc_left"], 0)
        self.drive(0.5, drive_cmd=800, mode=rp.MODE_AUTONOMOUS,
                   from_autonomy=False, stop=True)
        halted = self.snap()["enc_left"]
        self.drive(0.4, drive_cmd=800, mode=rp.MODE_AUTONOMOUS,
                   from_autonomy=False, stop=True)
        self.assertEqual(self.snap()["enc_left"], halted)
        self.assertEqual(self.snap()["cmd_status"], rp.CMD_STATUS_STOPPED)

    def test_the_refusal_does_not_latch(self):
        # A safety gate that cannot be left is a rover that has to be
        # power-cycled at a competition.
        self.drive(0.5, drive_cmd=600, mode=rp.MODE_TELEOP, from_autonomy=True)
        self.assertIn("CMD_SOURCE", self.faults())
        self.drive(0.5, drive_cmd=600, mode=rp.MODE_TELEOP, from_autonomy=False)
        self.assertNotIn("CMD_SOURCE", self.faults())
        self.assertEqual(self.snap()["cmd_status"], rp.CMD_STATUS_APPLIED)
        self.assertGreater(self.snap()["enc_left"], 0)

    def test_an_unauthorised_arrival_claim_does_not_light_the_indicator(self):
        # Flashing green is worth 25 points under rule 1.e.xvii.
        self.drive(0.5, drive_cmd=0, mode=rp.MODE_AUTONOMOUS,
                   from_autonomy=False, indicator_request=rp.INDICATOR_ARRIVED)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_AUTONOMOUS)
        self.drive(0.5, drive_cmd=0, mode=rp.MODE_AUTONOMOUS,
                   from_autonomy=True, indicator_request=rp.INDICATOR_ARRIVED)
        self.assertEqual(self.snap()["indicator_state"], rp.INDICATOR_ARRIVED)

    def test_a_stale_source_mismatch_is_not_reported(self):
        # Once the watchdog trips, the controller cannot vouch for who sent
        # the last frame any more than for what it said.
        self.drive(0.5, drive_cmd=600, mode=rp.MODE_TELEOP, from_autonomy=True)
        self.assertIn("CMD_SOURCE", self.faults())
        self.drive(0.7, send=False)
        self.assertIn("COMM_TIMEOUT", self.faults())
        self.assertNotIn("CMD_SOURCE", self.faults())


class TestAcknowledgement(RoverFixture):
    """TELEM_CMD_ECHO: the Jetson has to be able to prove a command landed."""

    def test_the_controller_acknowledges_the_sequence_it_acted_on(self):
        self.drive(0.6, drive_cmd=400)
        s = self.snap()
        # The ack has to name a frame we actually sent, and a recent one --
        # 20 Hz commands and 20 Hz telemetry should never be far apart.
        self.assertLessEqual(rp.seq_delta(s["ack_seq"], self.jl.tx_seq), 10,
                             "the acknowledged sequence number is far behind")
        self.assertEqual(s["cmd_status"], rp.CMD_STATUS_APPLIED)
        self.assertEqual(s["last_decode"], rp.DECODE_OK)

    def test_the_echo_reports_why_the_last_frame_was_rejected(self):
        # A rejected frame and an applied command can coexist: the reason the
        # last frame bounced is its own field, not a verdict on the command.
        self.drive(0.4, drive_cmd=400)
        good = rp.encode_control(400, 0, rp.MODE_TELEOP, False, seq=90)
        bad = bytearray(good)
        bad[0] ^= 0xFF
        self.send_raw(bytes(bad), 0.2)
        self.assertEqual(self.snap()["last_decode"], rp.DECODE_BAD_CRC)

    def test_the_echo_tracks_what_the_drive_stage_was_handed(self):
        for drive_cmd, steer_cmd in ((0, 0), (250, -100), (1000, 0), (-700, 400)):
            self.drive(0.4, drive_cmd=drive_cmd, steer_cmd=steer_cmd)
            s = self.snap()
            self.assertEqual(s["drive_applied"], drive_cmd)
            self.assertEqual(s["steer_applied"], steer_cmd)

    def test_a_return_request_is_acknowledged_without_stopping_the_rover(self):
        # Reported, not obeyed: driving back is route planning, and
        # PROTOCOL.md section 9 leaves the semantics as a team decision.
        self.drive(0.5, drive_cmd=500, mode=rp.MODE_AUTONOMOUS,
                   return_request=True)
        s = self.snap()
        self.assertTrue(s["return_active"])
        self.assertEqual(s["cmd_status"], rp.CMD_STATUS_APPLIED)
        self.assertEqual(s["drive_applied"], 500)
        self.assertGreater(s["enc_left"], 0, "the return request stopped the rover")
        self.assertEqual(s["indicator_state"], rp.INDICATOR_AUTONOMOUS)

    def test_controller_health_is_no_longer_a_constant(self):
        # The drive module reports its own state, so this field finally
        # carries something. OK while driving normally.
        self.drive(0.6, drive_cmd=400, steer_cmd=0)
        self.assertEqual(self.snap()["controller_health"], rp.CTRL_HEALTH_OK)

    def test_a_saturating_mix_is_reported_as_degraded(self):
        # drive 800 + steer 400 asks for 1200 on one side. The module scales
        # the pair down to keep the turn, and says it could not deliver both.
        self.drive(0.6, drive_cmd=800, steer_cmd=400, mode=rp.MODE_AUTONOMOUS)
        self.assertEqual(self.snap()["controller_health"],
                         rp.CTRL_HEALTH_DEGRADED)
        # Still driving -- DEGRADED is information, not a fault.
        self.assertGreater(self.snap()["enc_left"], 0)
        self.assertEqual(self.faults(), [])


class TestDriveStage(RoverFixture):
    """The mixing actually reaches the plant, rather than stopping at a struct."""

    def test_steering_splits_the_two_encoders(self):
        # Straight ahead: both sides equal. Turning: they diverge, and in the
        # commanded direction.
        self.drive(0.6, drive_cmd=400, steer_cmd=0)
        s = self.snap()
        self.assertEqual(s["enc_left"], s["enc_right"],
                         "the sides differ with no steering commanded")

        self.drive(0.6, drive_cmd=400, steer_cmd=300)
        s = self.snap()
        self.assertGreater(s["enc_left"], s["enc_right"],
                           "a right turn did not out-run the left side")

    def test_the_steering_actuator_follows_the_command(self):
        self.drive(0.8, drive_cmd=0, steer_cmd=500)
        self.assertAlmostEqual(self.snap()["steer_fb"], 500, delta=25)
        self.drive(0.8, drive_cmd=0, steer_cmd=-500)
        self.assertAlmostEqual(self.snap()["steer_fb"], -500, delta=25)

    def test_nothing_is_energized_under_a_stop(self):
        self.drive(0.5, drive_cmd=1000, steer_cmd=1000, stop=True)
        s = self.snap()
        self.assertEqual(s["drive_applied"], 0)
        self.assertEqual(s["steer_applied"], 0)
        self.assertEqual(s["current_ca"], 0)


class TestProtocolRootOverride(unittest.TestCase):
    """Where can_link looks for the firmware it compiles.

    The autonomy program is going to live in its own repository, so the Jetson
    code has to be able to point at a checkout of the protocol repository
    rather than assume the firmware sits one directory above it. These run in
    subprocesses because the root is resolved at import time, and a reload
    would leave the module pointing somewhere else for every test after it.
    """

    SNIPPET = ("import sys; sys.path.insert(0, %r);"
               " import can_link; print(can_link.REPO)")

    def _run(self, snippet, env_root=None, cwd=None, argv=None):
        env = dict(os.environ)
        env.pop("ROVER_PROTOCOL_ROOT", None)
        if env_root is not None:
            env["ROVER_PROTOCOL_ROOT"] = env_root
        return subprocess.run(argv or [sys.executable, "-c", snippet],
                              env=env, cwd=cwd, capture_output=True, text=True)

    def test_without_the_variable_the_root_is_the_enclosing_repository(self):
        done = self._run(self.SNIPPET % os.path.join(REPO, "host"), cwd="/")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertEqual(done.stdout.strip(), os.path.realpath(REPO))

    def test_the_variable_moves_the_root_somewhere_else_entirely(self):
        # Pointing the variable AT this repository would prove nothing: the
        # default derivation lands on the same path, so the assertion would
        # pass either way. So the root here is a foreign directory that merely
        # has the right shape, which is also what a protocol checkout inside a
        # separate autonomy repository looks like.
        snippet = (self.SNIPPET % os.path.join(REPO, "host")) + \
            "; print(can_link.build_sim())"
        with tempfile.TemporaryDirectory() as elsewhere:
            for name in ("firmware", "tools"):
                os.symlink(os.path.join(REPO, name),
                           os.path.join(elsewhere, name))
            done = self._run(snippet, env_root=elsewhere, cwd="/")
            self.assertEqual(done.returncode, 0, done.stderr)
            root, binary = done.stdout.split()
            self.assertEqual(root, elsewhere)
            self.assertNotEqual(root, REPO)
            # Compiled from the foreign root, into the foreign root -- so a
            # read-only protocol checkout is the one layout this cannot serve,
            # and it fails loudly rather than reusing the wrong binary.
            self.assertTrue(binary.startswith(elsewhere), binary)
            self.assertTrue(os.access(binary, os.X_OK))

    def test_a_wrong_root_fails_by_name_and_says_which_variable_is_wrong(self):
        # The symptom to avoid is a compiler error about a header nobody
        # pointed at. The message has to name the file AND the variable, or the
        # next person edits the wrong thing.
        snippet = (self.SNIPPET % os.path.join(REPO, "host")) + \
            "; can_link.build_sim()"
        with tempfile.TemporaryDirectory() as empty:
            done = self._run(snippet, env_root=empty, cwd="/")
        self.assertNotEqual(done.returncode, 0)
        self.assertIn("RuntimeError", done.stderr)
        self.assertIn("rover_sim.cpp", done.stderr)
        self.assertIn("ROVER_PROTOCOL_ROOT", done.stderr)

    def test_the_pinning_follows_the_firmware_that_is_actually_compiled(self):
        # The regression test for a real bug. When only can_link.py honoured the
        # override, the integration suite drove the FOREIGN firmware while
        # test_config_sync.py and test_golden_vectors.py read the LOCAL headers.
        # The whole host suite then reported OK against a firmware declaring a
        # different PROTOCOL_VERSION than the Python codec talking to it -- the
        # one failure section 8 of PROTOCOL.md exists to catch, hidden by the
        # very flag added to keep the two honest across checkouts.
        with tempfile.TemporaryDirectory() as foreign:
            for name in ("firmware", "tools"):
                shutil.copytree(os.path.join(REPO, name),
                                os.path.join(foreign, name))
            header = os.path.join(foreign, "firmware", "src", "rover_config.h")
            with open(header) as fh:
                text = fh.read()
            bumped = text.replace("PROTOCOL_VERSION = %d" % rp.PROTOCOL_VERSION,
                                  "PROTOCOL_VERSION = %d" % (rp.PROTOCOL_VERSION + 90))
            self.assertNotEqual(bumped, text, "could not find the version to bump")
            with open(header, "w") as fh:
                fh.write(bumped)

            done = self._run(None, env_root=foreign, cwd=REPO, argv=[
                sys.executable, os.path.join(REPO, "tests", "host",
                                             "test_config_sync.py")])

        self.assertNotEqual(
            done.returncode, 0,
            "config sync passed against a firmware declaring a different "
            "PROTOCOL_VERSION, so the pinning is reading the wrong checkout")
        self.assertIn("PROTOCOL_VERSION differs", done.stderr)

    def test_an_already_built_binary_cannot_mask_a_wrong_root(self):
        # The nastiest version of the mistake: a stale binary left in place
        # looks perfectly good, so the sources are checked BEFORE the staleness
        # comparison rather than after it.
        snippet = (self.SNIPPET % os.path.join(REPO, "host")) + \
            "; can_link.build_sim()"
        with tempfile.TemporaryDirectory() as fake:
            build = os.path.join(fake, "tests", "cpp", "build")
            os.makedirs(build)
            with open(os.path.join(build, "rover_sim"), "w") as fh:
                fh.write("not a real simulator")
            done = self._run(snippet, env_root=fake, cwd="/")
        self.assertNotEqual(done.returncode, 0,
                            "a leftover binary was accepted without its sources")
        self.assertIn("firmware sources missing", done.stderr)


if __name__ == "__main__":
    unittest.main()

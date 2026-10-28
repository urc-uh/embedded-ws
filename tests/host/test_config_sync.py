"""
test_config_sync.py -- pin host/rover_config.py to firmware/src/rover_config.h.

The constants exist in both languages because the Teensy runs C++ and the
Jetson runs Python. If someone changes a timeout, an id or a frame size on one
side only, the two ends would silently disagree on a moving rover. This test
turns that into a CI failure instead.

It reads the C++ header as text rather than compiling it, so it needs no
toolchain and runs anywhere the host tests run.
"""

import os
import re
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import c2_link  # noqa: E402
import protocol_root  # noqa: E402
import rover_config as cfg  # noqa: E402
import rover_protocol as rp  # noqa: E402

# Resolved, not derived from __file__: this test's whole job is to pin the
# Python constants against the C++ ones, and it can only do that if it reads
# the header belonging to the firmware can_link.py actually compiles. Deriving
# it locally while can_link compiled an overridden checkout is how this suite
# once passed over a PROTOCOL_VERSION mismatch.
HEADER_PATH = protocol_root.firmware_src("rover_config.h")

# Python name -> C++ name, for every constant that exists on both sides.
SHARED = {
    "PROTOCOL_VERSION":      "PROTOCOL_VERSION",
    "CAN_BITRATE_HZ":        "CAN_BITRATE_HZ",
    "WATCHDOG_TIMEOUT_MS":   "DEFAULT_WATCHDOG_TIMEOUT_MS",
    "TELEMETRY_PERIOD_MS":   "DEFAULT_TELEMETRY_PERIOD_MS",
    "LINK_DEGRADED_HOLD_MS": "LINK_DEGRADED_HOLD_MS",
    "CAN_ID_CONTROL":        "CAN_ID_CONTROL",
    "CAN_ID_TELEM_DRIVE_L":  "CAN_ID_TELEM_DRIVE_L",
    "CAN_ID_TELEM_DRIVE_R":  "CAN_ID_TELEM_DRIVE_R",
    "CAN_ID_TELEM_POWER":    "CAN_ID_TELEM_POWER",
    "CAN_ID_TELEM_STATE":    "CAN_ID_TELEM_STATE",
    "CAN_ID_TELEM_CMD_ECHO": "CAN_ID_TELEM_CMD_ECHO",
    "MODE_SAFE":             "MODE_SAFE",
    "MODE_TELEOP":           "MODE_TELEOP",
    "MODE_AUTONOMOUS":       "MODE_AUTONOMOUS",
    "MODE_FAULT":            "MODE_FAULT",
    "CMD_MIN":               "CMD_MIN",
    "CMD_MAX":               "CMD_MAX",
    "PWM_NEUTRAL_US":        "PWM_NEUTRAL_US",
    "PWM_SPAN_US":           "PWM_SPAN_US",
}

# Frame geometry: stated on both sides, because the sequence and CRC bytes are
# not part of any struct format and so cannot be derived from one.
FRAME_GEOMETRY = ("FRAME_DLC", "FRAME_PAYLOAD", "FRAME_SEQ_OFFSET",
                  "FRAME_CRC_OFFSET")

# No Python counterpart, so the comparison above cannot catch these drifting
# back out into rover_firmware.cpp.
FIRMWARE_ONLY = ("HW_WATCHDOG_MS", "OVER_CURRENT_CA", "UNDERVOLTAGE_CV")


def cpp_constants():
    """Every `NAME = <integer literal>` in rover_config.h, comments stripped."""
    with open(HEADER_PATH) as f:
        text = re.sub(r"//[^\n]*", "", f.read())
    found = {}
    for name, value in re.findall(
            r"\b([A-Z][A-Z0-9_]*)\s*=\s*(-?\s*(?:0[xX][0-9A-Fa-f]+|\d+))\b", text):
        found[name] = int(value.replace(" ", ""), 0)
    return found


class TestConfigSync(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cpp = cpp_constants()

    def test_header_was_parsed(self):
        # Guards the test itself: a regex that matched nothing would make
        # every comparison below vacuous.
        for cpp_name in SHARED.values():
            self.assertIn(cpp_name, self.cpp,
                          "%s not found in rover_config.h" % cpp_name)

    def test_python_config_matches_cpp_config(self):
        for py_name, cpp_name in SHARED.items():
            self.assertEqual(
                getattr(cfg, py_name), self.cpp[cpp_name],
                "%s differs: rover_config.py=%r, rover_config.h=%r -- the "
                "Jetson and the Teensy disagree"
                % (py_name, getattr(cfg, py_name), self.cpp[cpp_name]))

    def test_frame_geometry_matches_cpp(self):
        # A field added on one side only shows up here first.
        for name in FRAME_GEOMETRY:
            self.assertEqual(getattr(rp, name), self.cpp[name],
                             "%s differs between the two sides" % name)

    def test_payload_layouts_fit_the_frame(self):
        # Every struct format must pack into exactly the payload bytes, with
        # the sequence number and CRC taking the rest of the frame.
        import struct
        for name in ("CONTROL_FMT", "DRIVE_L_FMT", "DRIVE_R_FMT",
                     "POWER_FMT", "STATE_FMT", "ECHO_FMT"):
            self.assertEqual(struct.calcsize(getattr(rp, name)),
                             self.cpp["FRAME_PAYLOAD"],
                             "%s is not FRAME_PAYLOAD bytes" % name)
        self.assertEqual(self.cpp["FRAME_PAYLOAD"] + 2, self.cpp["FRAME_DLC"])

    def test_firmware_only_constants_are_in_the_config_header(self):
        for name in FIRMWARE_ONLY:
            self.assertIn(name, self.cpp,
                          "%s belongs in rover_config.h" % name)

    def test_mode_max_is_the_highest_commandable_mode(self):
        # C++ defines MODE_MAX as MODE_AUTONOMOUS (checked by name above).
        # MODE_FAULT is higher but is never sent, so it does not count.
        self.assertEqual(cfg.MODE_MAX, max(rp.COMMANDABLE_MODES))

    def test_link_timeout_is_longer_than_a_telemetry_period(self):
        # Otherwise the Jetson would report the link down between two
        # perfectly on-time telemetry frames.
        self.assertGreater(cfg.LINK_TIMEOUT_S * 1000, cfg.TELEMETRY_PERIOD_MS)

    def test_watchdog_tolerates_a_missed_control_frame(self):
        # The controller must not stop the rover over a single late frame.
        self.assertGreater(cfg.WATCHDOG_TIMEOUT_MS, 2 * 1000 // cfg.CONTROL_RATE_HZ)

    def test_c2_timeout_is_longer_than_the_command_watchdog(self):
        # A radio drops packets far more often than a CAN bus, so the C2
        # timeout must not be as tight as the command watchdog. If these ever
        # converge, a single radio hiccup starts stopping the rover in TELEOP.
        self.assertGreater(cfg.C2_TIMEOUT_S * 1000, cfg.WATCHDOG_TIMEOUT_MS)

    def test_c2_monitor_defaults_to_the_configured_timeout(self):
        # The constant is worthless if the only caller hardcodes its own.
        self.assertEqual(c2_link.C2Monitor().C2_timeout_s, cfg.C2_TIMEOUT_S)

    def test_degraded_hold_outlasts_a_telemetry_period(self):
        # A gap or CRC error must stay visible for more than one frame, or an
        # operator watching at 20 Hz would never see it.
        self.assertGreater(cfg.LINK_DEGRADED_HOLD_MS, cfg.TELEMETRY_PERIOD_MS)

    def test_command_range_is_symmetric_and_nonzero(self):
        self.assertEqual(cfg.CMD_MIN, -cfg.CMD_MAX)
        self.assertGreater(cfg.CMD_MAX, 0)

    def test_pwm_full_scale_stays_inside_the_servo_pulse_range(self):
        # 1000-2000 us is what servo and ESC inputs expect. A span that pushed
        # either end outside it would be rejected or, worse, misread by the
        # hardware as a different command.
        self.assertGreaterEqual(cfg.PWM_NEUTRAL_US - cfg.PWM_SPAN_US, 1000)
        self.assertLessEqual(cfg.PWM_NEUTRAL_US + cfg.PWM_SPAN_US, 2000)
        self.assertGreater(cfg.PWM_SPAN_US, 0)

    def test_every_can_id_is_distinct_and_control_outranks_telemetry(self):
        # On CAN the id IS the priority, lowest wins. A command arriving late
        # can hurt the rover; late telemetry only annoys an operator.
        ids = (cfg.CAN_ID_CONTROL,) + rp.TELEM_IDS
        self.assertEqual(len(set(ids)), len(ids), "two frames share a CAN id")
        self.assertEqual(min(ids), cfg.CAN_ID_CONTROL)
        self.assertLessEqual(max(ids), 0x7FF, "not an 11-bit standard id")


if __name__ == "__main__":
    unittest.main()

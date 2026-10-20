"""
test_golden_vectors.py -- pin the C++ and Python codecs together, byte for byte.

The codec is the only part of this protocol that exists in two languages. This
test compiles the real C++ encoder, runs it, and asserts Python produces
identical output for identical inputs -- not just for encoded frames, but for
the CRC, the sequence arithmetic and every validation predicate.

If it fails, the Teensy and the Jetson disagree about the wire format or about
which values are legal. That is not a style issue: it means commands would be
misread, or silently rejected, on a moving rover.
"""

import os
import subprocess
import sys
import unittest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "host"))

import protocol_root  # noqa: E402
import rover_protocol as p  # noqa: E402


def build_and_run_cpp():
    # Resolved rather than derived from __file__, for the same reason as
    # test_config_sync.py: pinning the local Python against a different
    # checkout's C++ than the one under test proves nothing.
    out_dir = protocol_root.build_dir()
    os.makedirs(out_dir, exist_ok=True)
    binary = os.path.join(out_dir, "golden_vectors")
    subprocess.run(
        [os.environ.get("CXX", "g++"), "-std=c++17", "-O1",
         protocol_root.tools("golden_vectors.cpp"),
         protocol_root.firmware_src("rover_protocol.cpp"),
         "-o", binary],
        check=True, capture_output=True)
    result = subprocess.run([binary], check=True, capture_output=True, text=True)
    return [ln.split("|") for ln in result.stdout.splitlines() if ln.strip()]


class GoldenBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.rows = build_and_run_cpp()
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            raise unittest.SkipTest("C++ toolchain unavailable: %s" % exc)

    def rowsOf(self, kind):
        found = [r for r in self.rows if r[0] == kind]
        self.assertTrue(found, "C++ emitted no %s vectors" % kind)
        return found


class TestCrcAgreement(GoldenBase):
    def test_crc8_known_answer_matches(self):
        for _, text, expected in self.rowsOf("CRC8"):
            self.assertEqual("%02x" % p.crc8(text.encode()), expected)

    def test_crc8_check_value_is_sae_j1850(self):
        """Pinned independently of the C++ output, so a matching pair of wrong
        implementations still fails. CRC-8/SAE-J1850 check value is 0x4B."""
        self.assertEqual(p.crc8(b"123456789"), 0x4B)

    def test_frame_crc_matches_including_the_id_seed(self):
        for _, can_id, payload_hex, expected in self.rowsOf("FRAMECRC"):
            got = p.frame_crc8(int(can_id), bytes.fromhex(payload_hex))
            self.assertEqual("%02x" % got, expected,
                             "frame_crc8 disagrees for id %s" % can_id)

    def test_the_id_seed_actually_changes_the_crc(self):
        ids = (p.CAN_ID_CONTROL,) + p.TELEM_IDS
        crcs = {p.frame_crc8(cid, bytes(range(6))) for cid in ids}
        self.assertEqual(len(crcs), len(ids),
                         "the CAN id is not affecting the CRC")


class TestPredicateAgreement(GoldenBase):
    def test_mode_predicates_agree_for_all_256_values(self):
        rows = self.rowsOf("MODE")
        self.assertEqual(len(rows), 256)
        for _, value, flags in rows:
            mode = int(value)
            self.assertEqual(p.is_commandable_mode(mode), flags[0] == "1",
                             "is_commandable_mode disagrees for mode=%d" % mode)
            self.assertEqual(p.mode_permits_motion(mode), flags[1] == "1",
                             "mode_permits_motion disagrees for mode=%d" % mode)

    def test_only_manual_and_autonomous_ever_permit_motion(self):
        for mode in range(256):
            if mode not in (p.MODE_TELEOP, p.MODE_AUTONOMOUS):
                self.assertFalse(p.mode_permits_motion(mode),
                                 "mode=%d permits motion but must not" % mode)

    def test_indicator_predicate_agrees_for_all_256_values(self):
        rows = self.rowsOf("INDICATOR")
        self.assertEqual(len(rows), 256)
        for _, value, known in rows:
            self.assertEqual(p.is_known_indicator(int(value)), known == "1",
                             "is_known_indicator disagrees for %s" % value)

    def test_command_range_agrees_across_languages(self):
        """Both sides must agree on exactly where the valid drive/steer range
        ends. A host that sends 1001 thinking it is legal would stop the rover
        with a PROTOCOL_ERROR it cannot explain."""
        rows = self.rowsOf("RANGE")
        self.assertGreater(len(rows), 0, "C++ emitted no RANGE cases")
        for _, value, valid in rows:
            self.assertEqual(p.is_valid_command(int(value)), valid == "1",
                             "is_valid_command disagrees for %s" % value)

    def test_out_of_range_commands_are_rejected_by_the_python_decoder(self):
        for v in (p.CMD_MIN - 1, p.CMD_MAX + 1, -32768, 32767):
            for drive, steer in ((v, 0), (0, v)):
                frame = p.encode_control(drive, steer, p.MODE_TELEOP, False)
                self.assertEqual(p.decode_control(frame)[0], p.DECODE_BAD_RANGE,
                                 "Python accepted %d,%d" % (drive, steer))
        for v in (p.CMD_MIN, 0, p.CMD_MAX):
            frame = p.encode_control(v, v, p.MODE_TELEOP, False)
            self.assertEqual(p.decode_control(frame)[0], p.DECODE_OK)

    def test_sequence_arithmetic_agrees_across_the_wrap(self):
        for _, pair, expected in self.rowsOf("SEQDELTA"):
            prev, cur = (int(x) for x in pair.split(","))
            self.assertEqual(p.seq_delta(prev, cur), int(expected),
                             "seq_delta(%d,%d) disagrees" % (prev, cur))


class TestAuthorityAgreement(GoldenBase):
    """The mux rule exists on both sides, so it is pinned like the codec."""

    def test_authority_agrees_for_every_mode_and_both_sources(self):
        rows = self.rowsOf("AUTHORITY")
        self.assertEqual(len(rows), 512, "expected both sources x 256 modes")
        for _, args, owns in rows:
            src, mode = (int(a) for a in args.split(","))
            self.assertEqual(p.source_owns_mode(bool(src), mode), owns == "1",
                             "source_owns_mode disagrees for src=%d mode=%d"
                             % (src, mode))

    def test_no_undefined_mode_is_ever_owned_by_a_motion_command(self):
        # Independent of the C++ output, so a matching pair of wrong
        # implementations still fails. The only modes where the question has a
        # real answer are the two that permit motion.
        for mode in range(256):
            if mode == p.MODE_AUTONOMOUS:
                self.assertTrue(p.source_owns_mode(True, mode))
                self.assertFalse(p.source_owns_mode(False, mode))
            elif mode == p.MODE_TELEOP:
                self.assertTrue(p.source_owns_mode(False, mode))
                self.assertFalse(p.source_owns_mode(True, mode))
            else:
                # Nothing may move, so neither source is wrong -- and nothing
                # is authorised to move either, which modePermitsMotion says.
                self.assertFalse(p.mode_permits_motion(mode))

    def test_the_command_status_byte_packs_identically(self):
        rows = self.rowsOf("CMDSTATUS")
        for _, args, packed_hex in rows:
            status, decode, ret = (int(a) for a in args.split(","))
            got = p.pack_cmd_status(status, decode, bool(ret))
            self.assertEqual("%02x" % got, packed_hex,
                             "pack_cmd_status disagrees for %s" % args)
            self.assertEqual(p.cmd_status_of(got), status)
            self.assertEqual(p.decode_result_of(got), decode)
            self.assertEqual(p.return_active_of(got), bool(ret))
            self.assertEqual(got & p.CMD_ECHO_RESERVED_MASK, 0)


class TestFrameAgreement(GoldenBase):
    def encode(self, kind, args):
        n = [int(a) for a in args.split(",")]
        if kind == "CONTROL":
            return p.encode_control(n[0], n[1], n[2], bool(n[6]),
                                    stop=bool(n[3]), seq=n[9],
                                    autonomy_abort=bool(n[4]),
                                    return_request=bool(n[5]),
                                    from_autonomy=bool(n[7]),
                                    indicator_request=n[8])
        if kind == "DRIVE_L":
            return p.encode_telemetry_drive_l(n[0], n[1], seq=n[2])
        if kind == "DRIVE_R":
            return p.encode_telemetry_drive_r(n[0], n[1], seq=n[2])
        if kind == "POWER":
            return p.encode_telemetry_power(n[0], n[1], n[2], seq=n[3])
        if kind == "STATE":
            return p.encode_telemetry_state(n[0], n[1], n[2], n[3], n[4], seq=n[5])
        if kind == "CMD_ECHO":
            return p.encode_telemetry_cmd_echo(
                n[0], n[1], n[2],
                p.pack_cmd_status(n[3], n[4], bool(n[5])), seq=n[6])
        self.fail("unknown vector kind %r" % kind)

    FRAME_KINDS = ("CONTROL", "DRIVE_L", "DRIVE_R", "POWER", "STATE", "CMD_ECHO")

    def test_every_frame_encodes_identically(self):
        checked = 0
        for kind in self.FRAME_KINDS:
            for _, args, expected in self.rowsOf(kind):
                actual = self.encode(kind, args).hex()
                self.assertEqual(
                    actual, expected,
                    "\n  %s(%s)\n    C++    : %s\n    Python : %s\n"
                    "  The two sides of the link disagree about the wire format."
                    % (kind, args, expected, actual))
                checked += 1
        self.assertGreater(checked, 15, "suspiciously few frames compared")

    def test_python_decodes_what_cpp_encoded(self):
        decoders = {
            "CONTROL": lambda f: p.decode_control(f),
            "DRIVE_L": lambda f: p.decode_telemetry_drive_l(f),
            "DRIVE_R": lambda f: p.decode_telemetry_drive_r(f),
            "POWER":   lambda f: p.decode_telemetry_power(f),
            "STATE":   lambda f: p.decode_telemetry_state(f),
            "CMD_ECHO": lambda f: p.decode_telemetry_cmd_echo(f),
        }
        for kind in self.FRAME_KINDS:
            for _, args, frame_hex in self.rowsOf(kind):
                result, decoded = decoders[kind](bytes.fromhex(frame_hex))
                self.assertEqual(result, p.DECODE_OK,
                                 "Python rejected a valid C++ %s frame (%s): %s"
                                 % (kind, args, p.DECODE_NAMES[result]))
                self.assertIsNotNone(decoded)

    def test_sequence_number_survives_the_round_trip(self):
        for _, args, frame_hex in self.rowsOf("CONTROL"):
            expected_seq = int(args.split(",")[9])
            _, decoded = p.decode_control(bytes.fromhex(frame_hex))
            self.assertEqual(decoded["seq"], expected_seq)

    def test_the_command_source_survives_the_round_trip(self):
        seen = set()
        for _, args, frame_hex in self.rowsOf("CONTROL"):
            expected = bool(int(args.split(",")[7]))
            _, decoded = p.decode_control(bytes.fromhex(frame_hex))
            self.assertEqual(decoded["from_autonomy"], expected,
                             "the source bit disagrees for CONTROL(%s)" % args)
            seen.add(expected)
        self.assertEqual(seen, {False, True},
                         "the vectors never exercise both command sources")

    def test_the_echo_status_byte_unpacks_to_what_cpp_packed(self):
        for _, args, frame_hex in self.rowsOf("CMD_ECHO"):
            n = [int(a) for a in args.split(",")]
            _, decoded = p.decode_telemetry_cmd_echo(bytes.fromhex(frame_hex))
            self.assertEqual(decoded["drive_applied"], n[0])
            self.assertEqual(decoded["steer_applied"], n[1])
            self.assertEqual(decoded["ack_seq"], n[2])
            self.assertEqual(decoded["cmd_status"], n[3])
            self.assertEqual(decoded["last_decode"], n[4])
            self.assertEqual(decoded["return_active"], bool(n[5]))

    def test_every_frame_is_exactly_eight_bytes(self):
        # 8 bytes is Classic CAN's limit. Exceeding it would force CAN FD
        # transceivers onto every node of the bus.
        for kind in self.FRAME_KINDS:
            for _, args, frame_hex in self.rowsOf(kind):
                self.assertEqual(len(frame_hex) // 2, p.FRAME_DLC,
                                 "%s(%s) is not 8 bytes" % (kind, args))


class TestPythonValidation(unittest.TestCase):
    """Python must reject what C++ rejects, for the same reasons."""

    def test_non_commandable_mode_is_rejected(self):
        # MODE_FAULT (3) is defined, but only the controller enters it.
        for mode in (p.MODE_FAULT, 4, 42, 128, 255):
            frame = p.encode_control(1000, 500, mode, False)
            result, decoded = p.decode_control(frame)
            self.assertEqual(result, p.DECODE_BAD_MODE)
            self.assertIsNone(decoded)

    def test_a_corrupted_byte_is_rejected(self):
        good = p.encode_control(500, -200, p.MODE_TELEOP, False, seq=3)
        for byte in range(p.FRAME_CRC_OFFSET):
            bad = bytearray(good)
            bad[byte] ^= 0xFF
            result, _ = p.decode_control(bytes(bad))
            self.assertNotEqual(result, p.DECODE_OK,
                                "byte %d corrupted but accepted" % byte)

    def test_a_frame_on_the_wrong_id_is_rejected(self):
        frame = p.encode_control(500, 0, p.MODE_TELEOP, False, seq=1)
        self.assertEqual(p.decode_control(frame)[0], p.DECODE_OK)
        self.assertEqual(p.decode_control(frame, can_id=p.CAN_ID_TELEM_POWER)[0],
                         p.DECODE_BAD_CRC)

    def test_wrong_length_is_rejected(self):
        frame = p.encode_control(0, 0, p.MODE_TELEOP, False)
        for n in range(0, 8):
            self.assertEqual(p.decode_control(frame[:n])[0], p.DECODE_BAD_DLC)

    def test_no_flags_byte_value_is_rejected_any_more(self):
        """v0.5 rejected bit 7 as reserved; v0.6 gave it to from_autonomy, so
        the only part of the flags byte that can still be out of range is the
        indicator field inside it. The guard that the check was really
        removed, rather than left checking a mask someone resets later."""
        for raw in range(256):
            frame = p.encode_control(0, 0, p.MODE_TELEOP, False, raw_flags=raw)
            ind = (raw & p.CTRL_INDICATOR_MASK) >> p.CTRL_INDICATOR_SHIFT
            expected = (p.DECODE_OK if p.is_known_indicator(ind)
                        else p.DECODE_BAD_INDICATOR)
            self.assertEqual(p.decode_control(frame)[0], expected,
                             "flags=0x%02x decoded unexpectedly" % raw)

    def test_the_c2_and_source_bits_are_fields_not_reserved_bits(self):
        """Bits 6 and 7 are real fields, so setting them must NOT reject the
        frame -- the guard against a stray mask writing into a live field."""
        flags = p.pack_control_flags(True, stop=True, from_autonomy=True)
        self.assertEqual(flags & p.CTRL_FLAG_C2_LOST, p.CTRL_FLAG_C2_LOST)
        self.assertEqual(flags & p.CTRL_FLAG_FROM_AUTONOMY,
                         p.CTRL_FLAG_FROM_AUTONOMY)
        result, decoded = p.decode_control(
            p.encode_control(0, 0, p.MODE_AUTONOMOUS, False, raw_flags=flags))
        self.assertEqual(result, p.DECODE_OK)
        self.assertTrue(decoded["c2_lost"])
        self.assertTrue(decoded["from_autonomy"])

    def test_a_mismatched_source_is_not_a_codec_error(self):
        """Authority is the controller's decision, not the codec's. If decode
        ever started rejecting these, a Jetson-side mux bug would stop the
        rover instead of being reported."""
        for mode, src in ((p.MODE_TELEOP, True), (p.MODE_AUTONOMOUS, False)):
            frame = p.encode_control(700, 100, mode, False, from_autonomy=src)
            result, decoded = p.decode_control(frame)
            self.assertEqual(result, p.DECODE_OK)
            self.assertFalse(p.source_owns_mode(decoded["from_autonomy"],
                                                decoded["mode"]))

    def test_undefined_indicator_request_is_rejected(self):
        for ind in (5, 6, 7):
            frame = p.encode_control(0, 0, p.MODE_TELEOP, False, indicator_request=ind)
            self.assertEqual(p.decode_control(frame)[0], p.DECODE_BAD_INDICATOR)

    def test_the_command_source_fault_bit_is_named(self):
        """0x0400 was the first of the bits PROTOCOL.md 3.5.2 held in reserve
        until someone was actually building the thing that needed one."""
        self.assertEqual(p.FAULT_CMD_SOURCE, 0x0400)
        self.assertEqual(p.fault_names(0x0400), ["CMD_SOURCE"])

    def test_the_c2_fault_bit_is_named(self):
        """0x0080 is FAULT_C2_LINK_LOST, which v0.4 had left reserved."""
        self.assertEqual(p.FAULT_C2_LINK_LOST, 0x0080)
        self.assertEqual(p.fault_names(0x0080), ["C2_LINK_LOST"])

    def test_c2_lost_survives_a_round_trip_in_every_mode(self):
        for mode in (p.MODE_SAFE, p.MODE_TELEOP, p.MODE_AUTONOMOUS):
            for c2 in (False, True):
                frame = p.encode_control(100, 0, mode, c2)
                result, decoded = p.decode_control(frame)
                self.assertEqual(result, p.DECODE_OK)
                self.assertEqual(decoded["c2_lost"], c2)


if __name__ == "__main__":
    unittest.main()

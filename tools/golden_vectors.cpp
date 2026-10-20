// golden_vectors.cpp -- emit encoded frames and predicate tables as text, for
// cross-language pinning.
//
// The codec is the one thing that must exist in both C++ (Teensy) and Python
// (Jetson), because neither can import the other. This program prints what the
// C++ side produces; tests/host/test_golden_vectors.py computes the same in
// Python and asserts they are identical.
//
// That turns "we were careful to keep them in sync" into a CI failure the
// moment someone changes a field width, order, endianness, CRC or predicate on
// one side only.
//
// Line formats:
//   CRC8|<ascii>|<hex>                   known-answer CRC
//   FRAMECRC|<can_id>|<payload hex>|<hex>
//   SEQDELTA|<prev>,<cur>|<delta>
//   MODE|<value>|<known><motion>         two 0/1 digits
//   INDICATOR|<value>|<known>
//   RANGE|<value>|<valid>
//   AUTHORITY|<from_autonomy>,<mode>|<owns>
//   CMDSTATUS|<status>,<decode>,<return>|<packed hex>
//   <NAME>|<args>|<frame hex>            encoded frames

#include <cstdio>
#include "../firmware/src/rover_protocol.h"

using namespace rover;

static void emitHex(const char* name, const char* args,
                    const uint8_t* buf, uint8_t len) {
    std::printf("%s|%s|", name, args);
    for (uint8_t i = 0; i < len; ++i) std::printf("%02x", buf[i]);
    std::printf("\n");
}

int main() {
    uint8_t buf[8];
    char args[160];

    // --- CRC known-answer and id seeding --------------------------------
    const char* kav = "123456789";
    std::printf("CRC8|%s|%02x\n", kav, crc8(reinterpret_cast<const uint8_t*>(kav), 9));

    const uint8_t sample[6] = {1, 2, 3, 4, 5, 6};
    const uint32_t ids[] = {CAN_ID_CONTROL, CAN_ID_TELEM_DRIVE_L,
                            CAN_ID_TELEM_DRIVE_R, CAN_ID_TELEM_POWER,
                            CAN_ID_TELEM_STATE, CAN_ID_TELEM_CMD_ECHO,
                            0x7FF, 0x000};
    for (uint32_t id : ids) {
        std::printf("FRAMECRC|%u|010203040506|%02x\n",
                    static_cast<unsigned>(id), frameCrc8(id, sample, 6));
    }

    // --- sequence arithmetic across the wrap -----------------------------
    const uint8_t seq_pairs[][2] = {{0,1},{1,1},{254,255},{255,0},{254,1},{0,255},{200,10}};
    for (const auto& sp : seq_pairs) {
        std::printf("SEQDELTA|%u,%u|%u\n", sp[0], sp[1], seqDelta(sp[0], sp[1]));
    }

    // --- validation predicates, exhaustive over the byte -----------------
    for (int m = 0; m <= 255; ++m) {
        std::printf("MODE|%d|%d%d\n", m,
                    isCommandableMode(static_cast<uint8_t>(m)) ? 1 : 0,
                    modePermitsMotion(static_cast<uint8_t>(m)) ? 1 : 0);
    }
    for (int i = 0; i <= 255; ++i) {
        std::printf("INDICATOR|%d|%d\n", i,
                    isKnownIndicator(static_cast<uint8_t>(i)) ? 1 : 0);
    }
    // Command range at and around the boundaries, so the Jetson cannot think
    // a value is sendable while the controller rejects it. Not exhaustive --
    // the field is 16 bits -- but off by one is the whole bug here.
    const int16_t range_cases[] = {-32768, -32767, -1002, -1001, -1000, -999,
                                   -1, 0, 1, 999, 1000, 1001, 1002, 32766, 32767};
    for (int16_t v : range_cases) {
        std::printf("RANGE|%d|%d\n", v, isValidCommand(v) ? 1 : 0);
    }

    // Command authority, exhaustive over the mode byte for both sources. The
    // failure this pins is an undefined mode reading as owned by whoever
    // asked -- a whitelist that quietly became a blacklist on one side only.
    for (int src = 0; src <= 1; ++src) {
        for (int m = 0; m <= 255; ++m) {
            std::printf("AUTHORITY|%d,%d|%d\n", src, m,
                        sourceOwnsMode(src != 0, static_cast<uint8_t>(m)) ? 1 : 0);
        }
    }

    // The command-status byte. Two 3-bit fields and a flag sharing one byte is
    // exactly the kind of packing that goes wrong on one side only.
    const uint8_t statuses[] = {CMD_STATUS_APPLIED, CMD_STATUS_STOPPED,
                                CMD_STATUS_WRONG_SOURCE, CMD_STATUS_NO_COMMAND};
    const DecodeResult decodes[] = {DECODE_OK, DECODE_BAD_DLC, DECODE_BAD_CRC,
                                    DECODE_BAD_MODE, DECODE_BAD_FLAGS,
                                    DECODE_BAD_INDICATOR, DECODE_BAD_RANGE};
    for (uint8_t st : statuses) {
        for (DecodeResult dr : decodes) {
            for (int ret = 0; ret <= 1; ++ret) {
                std::printf("CMDSTATUS|%u,%u,%d|%02x\n", st,
                            static_cast<unsigned>(dr), ret,
                            packCmdStatus(st, dr, ret != 0));
            }
        }
    }

    // --- CONTROL ---------------------------------------------------------
    struct Ctl { int16_t d, s; uint8_t m; bool stop, abort, ret, c2, aut; uint8_t ind, seq; };
    const Ctl ctls[] = {
        {     0,     0, MODE_SAFE,   true,  false, false, false, false, INDICATOR_OFF,        0},
        {   500,  -200, MODE_TELEOP,     false, false, false, false, false, INDICATOR_TELEOP,     1},
        {  1000, -1000, MODE_AUTONOMOUS, false, false, false, false, true,  INDICATOR_AUTONOMOUS, 42},
        { -1000,  1000, MODE_TELEOP,     true,  true,  true,  true,  false, INDICATOR_ARRIVED,  255},
        // These two were -32768,32767 and -21846,170 before command-range
        // validation. Both are outside CMD_MIN..CMD_MAX, so they no longer
        // decode; the boundary itself is pinned by the RANGE table above and
        // the extreme-int16 ENCODING by the_encoder_still_writes_any_int16.
        {  -999,   999, MODE_TELEOP,     false, false, true,  false, false, INDICATOR_FAULT,    128},
        {  -846,   170, MODE_AUTONOMOUS, false, true,  false, true,  true,  INDICATOR_OFF,        7},
        {   250,   -50, MODE_AUTONOMOUS, false, false, false, true,  true,  INDICATOR_AUTONOMOUS, 64},
        {     0,     0, MODE_TELEOP,     false, false, false, true,  false, INDICATOR_TELEOP,    65},
        // Both mismatched combinations, which are legal on the wire and
        // refused by the controller rather than by the codec. If either side
        // ever started rejecting them at decode, these two rows would be the
        // first thing to fail.
        {   700,   100, MODE_TELEOP,     false, false, false, false, true,  INDICATOR_TELEOP,    66},
        {   700,   100, MODE_AUTONOMOUS, false, false, false, false, false, INDICATOR_ARRIVED,   67},
    };
    for (const Ctl& c : ctls) {
        ControlMsg m;
        m.drive_cmd = c.d; m.steer_cmd = c.s; m.mode = c.m;
        m.stop = c.stop; m.autonomy_abort = c.abort; m.return_request = c.ret;
        m.c2_lost = c.c2; m.from_autonomy = c.aut;
        m.indicator_request = c.ind; m.seq = c.seq;
        uint8_t n = encodeControl(m, buf);
        std::snprintf(args, sizeof(args), "%d,%d,%u,%d,%d,%d,%d,%d,%u,%u",
                      c.d, c.s, c.m, c.stop ? 1 : 0, c.abort ? 1 : 0,
                      c.ret ? 1 : 0, c.c2 ? 1 : 0, c.aut ? 1 : 0, c.ind, c.seq);
        emitHex("CONTROL", args, buf, n);
    }

    // --- TELEM_DRIVE_L ---------------------------------------------------
    struct DL { long long enc; int16_t fb; uint8_t seq; };
    const DL dls[] = {{0,0,0}, {123456,-1000,1}, {INT32_MAX_V,1000,250},
                      {INT32_MIN_V,-1,251}, {2147483648LL,0,9}};
    for (const DL& d : dls) {
        TelemetryDriveL m; m.enc_left = wrapI32(d.enc); m.steer_fb = d.fb;
        uint8_t n = encodeTelemetryDriveL(m, d.seq, buf);
        std::snprintf(args, sizeof(args), "%lld,%d,%u", d.enc, d.fb, d.seq);
        emitHex("DRIVE_L", args, buf, n);
    }

    // --- TELEM_DRIVE_R ---------------------------------------------------
    struct DR { long long enc; uint16_t age; uint8_t seq; };
    const DR drs[] = {{0,0,0}, {-123456,400,3}, {INT32_MAX_V,CMD_AGE_MAX,4},
                      {0,CMD_AGE_UNKNOWN,5}};
    for (const DR& d : drs) {
        TelemetryDriveR m; m.enc_right = wrapI32(d.enc); m.cmd_age_ms = d.age;
        uint8_t n = encodeTelemetryDriveR(m, d.seq, buf);
        std::snprintf(args, sizeof(args), "%lld,%u,%u", d.enc, d.age, d.seq);
        emitHex("DRIVE_R", args, buf, n);
    }

    // --- TELEM_POWER -----------------------------------------------------
    struct PW { int16_t ca, cv; uint16_t f; uint8_t seq; };
    const PW pws[] = {
        {0, 0, 0, 0},
        {-2500, 2400, FAULT_OVER_CURRENT, 6},
        {32767, -32768, 0xFFFF, 7},
        {-10000, 1200, static_cast<uint16_t>(FAULT_COMM_TIMEOUT | FAULT_SEQ_GAP
                                             | FAULT_CRC_ERROR), 8},
    };
    for (const PW& w : pws) {
        TelemetryPower m; m.current_ca = w.ca; m.voltage_cv = w.cv;
        m.fault_status = w.f;
        uint8_t n = encodeTelemetryPower(m, w.seq, buf);
        std::snprintf(args, sizeof(args), "%d,%d,%u,%u", w.ca, w.cv, w.f, w.seq);
        emitHex("POWER", args, buf, n);
    }

    // --- TELEM_STATE -----------------------------------------------------
    struct ST { uint8_t mode, jl, c2, h, ind, seq; };
    const ST sts[] = {
        {MODE_SAFE, LINK_LOST, LINK_NOT_REPORTED, CTRL_HEALTH_NOT_REPORTED,
         INDICATOR_OFF, 0},
        {MODE_TELEOP, LINK_OK, LINK_OK, CTRL_HEALTH_OK, INDICATOR_TELEOP, 11},
        {MODE_AUTONOMOUS, LINK_DEGRADED, LINK_LOST, CTRL_HEALTH_DEGRADED,
         INDICATOR_AUTONOMOUS, 12},
        {MODE_AUTONOMOUS, LINK_OK, LINK_NOT_REPORTED, CTRL_HEALTH_FAULT,
         INDICATOR_FAULT, 13},
    };
    for (const ST& s : sts) {
        TelemetryState m; m.mode = s.mode; m.jetson_link = s.jl; m.c2_link = s.c2;
        m.controller_health = s.h; m.indicator_state = s.ind; m.reserved = 0;
        uint8_t n = encodeTelemetryState(m, s.seq, buf);
        std::snprintf(args, sizeof(args), "%u,%u,%u,%u,%u,%u",
                      s.mode, s.jl, s.c2, s.h, s.ind, s.seq);
        emitHex("STATE", args, buf, n);
    }

    // --- TELEM_CMD_ECHO --------------------------------------------------
    struct EC { int16_t d, s; uint8_t ack, st; DecodeResult dr; bool ret; uint8_t seq; };
    const EC ecs[] = {
        {    0,     0,   0, CMD_STATUS_NO_COMMAND,   DECODE_OK,        false,   0},
        {  500,  -200,   7, CMD_STATUS_APPLIED,      DECODE_OK,        false,  21},
        {    0,     0,  99, CMD_STATUS_WRONG_SOURCE, DECODE_OK,        false,  22},
        {    0,     0, 255, CMD_STATUS_STOPPED,      DECODE_BAD_CRC,   false,  23},
        { 1000, -1000, 128, CMD_STATUS_APPLIED,      DECODE_BAD_RANGE, true,   24},
        {-1000,  1000,   1, CMD_STATUS_APPLIED,      DECODE_OK,        true,  255},
    };
    for (const EC& e : ecs) {
        TelemetryCmdEcho m;
        m.drive_applied = e.d; m.steer_applied = e.s; m.ack_seq = e.ack;
        m.status = packCmdStatus(e.st, e.dr, e.ret);
        uint8_t n = encodeTelemetryCmdEcho(m, e.seq, buf);
        std::snprintf(args, sizeof(args), "%d,%d,%u,%u,%u,%d,%u",
                      e.d, e.s, e.ack, e.st, static_cast<unsigned>(e.dr),
                      e.ret ? 1 : 0, e.seq);
        emitHex("CMD_ECHO", args, buf, n);
    }
    return 0;
}

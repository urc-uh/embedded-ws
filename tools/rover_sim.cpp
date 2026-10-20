// rover_sim.cpp -- a simulated rover controller for host-side testing.
//
// WHY THIS IS C++ AND NOT PYTHON
//   The obvious thing would be a Python mock of the controller. That would
//   recreate the exact problem this project already fixed once: two
//   hand-maintained copies of a safety rule, where the simulator can pass
//   while the real firmware misbehaves, and the tests check the wrong copy.
//
//   So this links the REAL rover_controller.cpp -- the same source the Teensy
//   compiles. Only the plant (fake encoders, fake current and voltage) and the
//   transport (a pipe instead of a CAN bus) are simulated.
//
// PIPE PROTOCOL (stands in for the CAN bus)
//   stdin :  "<id_hex> <frame_hex>\n"   one received frame per line
//   stdout:  "<id_hex> <frame_hex>\n"   one transmitted frame per line
//   stdin closing ends the run.
//
// PRINTED DRIVE OUTPUT
//   `rover_sim -v` additionally prints what the drive stage is doing, one line
//   per telemetry cycle, TO STDERR. Task 4 lists a printed output as one
//   acceptable form for the drive/steering module, and stderr is the only
//   place it can go: stdout is the bus.

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <chrono>
#include <string>
#include <poll.h>
#include <unistd.h>

#include "../firmware/src/rover_controller.h"
#include "../firmware/src/rover_drive.h"

using namespace rover;

static std::chrono::steady_clock::time_point g_start;
static uint32_t realMillis() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now() - g_start).count());
}

static void emitFrame(uint32_t id, const uint8_t* buf, uint8_t len) {
    std::printf("%03x ", id);
    for (uint8_t i = 0; i < len; ++i) std::printf("%02x", buf[i]);
    std::printf("\n");
    std::fflush(stdout);        // unbuffered: the host reads these live
}

int main(int argc, char** argv) {
    bool verbose = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-v") == 0) verbose = true;
    }

    g_start = std::chrono::steady_clock::now();
    RoverController ctl(realMillis, DEFAULT_WATCHDOG_TIMEOUT_MS,
                        DEFAULT_TELEMETRY_PERIOD_MS);
    // The REAL drive stage, not a stand-in: tools/rover_sim links the same
    // rover_drive.cpp the Teensy compiles, so the host tests exercise the
    // actual mixing, clamping and stop behaviour rather than a Python idea of
    // them. Only the pins at the far end are missing.
    DriveModule drv;

    int32_t enc_left = 0, enc_right = 0;
    int16_t steer_fb = 0;
    std::string pending;
    char chunk[512];

    for (;;) {
        // Non-blocking stdin so the telemetry timer keeps running while the
        // host is quiet -- precisely the condition the command watchdog exists
        // to detect.
        struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
        if (poll(&pfd, 1, 2) > 0 && (pfd.revents & (POLLIN | POLLHUP))) {
            ssize_t n = read(STDIN_FILENO, chunk, sizeof(chunk));
            if (n <= 0) break;                       // host closed the pipe
            pending.append(chunk, static_cast<size_t>(n));
            size_t nl;
            while ((nl = pending.find('\n')) != std::string::npos) {
                std::string line = pending.substr(0, nl);
                pending.erase(0, nl + 1);
                unsigned id = 0; char hex[32] = {0};
                if (std::sscanf(line.c_str(), "%x %31s", &id, hex) == 2) {
                    uint8_t rx[8] = {0};
                    size_t len = std::strlen(hex) / 2;
                    if (len > 8) len = 8;
                    for (size_t i = 0; i < len; ++i) {
                        unsigned byte = 0;
                        std::sscanf(hex + i * 2, "%2x", &byte);
                        rx[i] = static_cast<uint8_t>(byte);
                    }
                    ctl.ingestFrame(id, rx, static_cast<uint8_t>(len));
                }
            }
        }

        // The Task 4 chain, in order: the controller has already validated the
        // frame and checked the mode; it now says what may be driven, and the
        // drive module turns that into outputs.
        int16_t drive, steer;
        ctl.commandedOutputs(drive, steer);          // zeroed under stop or
                                                     // an authority mismatch
        const bool stopped = ctl.effectiveStop();
        const DriveOutputs& out = drv.update(drive, steer, !stopped);

        // Fake plant, driven by the DRIVE MODULE's output rather than by the
        // raw command, so the mix is part of what the host tests observe. The
        // fail-safe has already been applied twice over, so there is no third
        // copy of the stop rule here.
        enc_left  = wrapI32(static_cast<int64_t>(enc_left)  + out.left_effort / 10);
        enc_right = wrapI32(static_cast<int64_t>(enc_right) + out.right_effort / 10);
        if (steer_fb < out.steer_effort) {
            steer_fb = static_cast<int16_t>((steer_fb + 25 < out.steer_effort)
                                            ? steer_fb + 25 : out.steer_effort);
        } else if (steer_fb > out.steer_effort) {
            steer_fb = static_cast<int16_t>((steer_fb - 25 > out.steer_effort)
                                            ? steer_fb - 25 : out.steer_effort);
        }
        const int32_t effort_mag =
            (std::abs(out.left_effort) + std::abs(out.right_effort)) / 2;
        const int16_t current_ca =
            out.energized ? static_cast<int16_t>(effort_mag * 3 / 2) : 0;
        // Fake pack voltage: 24.00 V nominal, sagging a little under load.
        const int16_t voltage_cv = static_cast<int16_t>(2400 - effort_mag / 40);

        if (ctl.telemetryDue()) {
            const uint8_t seq = ctl.telemetrySeq();   // one seq for all four
            uint8_t buf[8];

            TelemetryDriveL dl = ctl.buildDriveL(enc_left, steer_fb);
            emitFrame(CAN_ID_TELEM_DRIVE_L, buf, encodeTelemetryDriveL(dl, seq, buf));

            TelemetryDriveR dr = ctl.buildDriveR(enc_right);
            emitFrame(CAN_ID_TELEM_DRIVE_R, buf, encodeTelemetryDriveR(dr, seq, buf));

            TelemetryPower pw = ctl.buildPower(current_ca, voltage_cv);
            emitFrame(CAN_ID_TELEM_POWER, buf, encodeTelemetryPower(pw, seq, buf));

            // controller_health now comes from the drive stage, which is the
            // layer that actually knows. Still not real ESC telemetry -- it
            // reports the module's own state -- but it is no longer a constant
            // NOT_REPORTED. c2_link the controller derives itself from the
            // c2_lost bit the Jetson forwards; see c2LinkState().
            TelemetryState st = ctl.buildState(drv.health());
            emitFrame(CAN_ID_TELEM_STATE, buf, encodeTelemetryState(st, seq, buf));

            // The acknowledgement: what was accepted, which frame it came
            // from, and why it is or is not being acted on.
            TelemetryCmdEcho ce = ctl.buildCmdEcho(drive, steer);
            emitFrame(CAN_ID_TELEM_CMD_ECHO, buf, encodeTelemetryCmdEcho(ce, seq, buf));

            if (verbose) {
                // Printed drive output, on stderr because stdout is the bus.
                std::fprintf(stderr,
                             "drive: %-12s ack=%3u cmd=(%+5d,%+5d) "
                             "out=(L%+5d R%+5d S%+5d) pwm=(%4u %4u %4u) %s\n",
                             cmdStatusName(ctl.commandStatus()), ce.ack_seq,
                             drive, steer, out.left_effort, out.right_effort,
                             out.steer_effort, out.left_pwm_us, out.right_pwm_us,
                             out.steer_pwm_us,
                             out.energized ? "ENERGIZED" : "neutral");
            }
        }
    }
    return 0;
}

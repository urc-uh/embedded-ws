// rover_firmware.cpp -- Rover embedded controller firmware.
// Target: Teensy 4.1 (NXP i.MX RT1062, Cortex-M7 @ 600 MHz), Teensyduino core.
//
// BUILD
//   PlatformIO: `pio run` to compile, `pio run -t upload` to flash. See
//   platformio.ini in the repo root; it names the board and both libraries:
//     FlexCAN_T4   https://github.com/tonton81/FlexCAN_T4
//     WDT_T4       https://github.com/tonton81/WDT_T4  (header: Watchdog_t4.h)
//   Sources under src/ are compiled automatically.
//
//   This is a .cpp, not a .ino, so it is a plain translation unit: nothing
//   rewrites it before the compiler sees it, and Arduino.h is included
//   explicitly below rather than injected. That is why setup() and loop()
//   appear at the bottom -- every function above them is defined before it is
//   used, with no generated forward declarations to depend on. Keep it that
//   way: a new helper goes above its first caller.
//
// WIRING -- READ BEFORE POWERING ANYTHING
//   * Teensy 4.1 is 3.3 V and NOT 5 V tolerant. A 5 V encoder or sensor wired
//     direct to a pin destroys the board. Level-shift everything.
//   * CAN needs a TRANSCEIVER per bus; the MCU cannot drive a differential
//     bus itself. CAN3 is the FD-capable bus on Teensy 4.1.
//   * The bus needs 120 ohm termination at BOTH physical ends. Exactly one
//     resistor, or three, is the most common CAN bring-up failure.
//
// WHY THIS FILE IS SHORT
//   All the protocol and safety logic lives in src/rover_protocol.* and
//   src/rover_controller.*, which are pure C++ with no Arduino dependency and
//   are unit-tested natively (`make -C tests/cpp test`). This file
//   is only the hardware wiring around them. If you are reviewing safety
//   behaviour, read rover_controller.h, not this.
//
// SAFETY STRUCTURE
//   1. Hardware watchdog (WDT_T4). If this loop stops feeding it, the Teensy
//      resets, which drives every pin to a safe state and de-energizes the
//      drives. This is the backstop for a hang or a crash.
//   2. Boot-fault reporting. The reset itself is invisible to the Jetson
//      except as a gap in telemetry, so the NEXT boot inspects the reset
//      status register and raises FAULT_FIRMWARE_FAULT for the whole session.
//   3. Command watchdog, in RoverController: no valid CONTROL frame within
//      300 ms forces a stop and raises FAULT_COMM_TIMEOUT.
//   4. Commands are zeroed by commandedOutputs() before they ever reach the
//      motor layer, so a future edit to setMotorOutputs() cannot accidentally
//      act on a stale command. DriveModule then refuses to energize anything
//      when told not to, so the stop survives an edit to either layer.
//   5. Command authority (v0.6). An autonomy-generated command may not drive
//      a teleoperated rover and vice versa; see commandAuthorized(). The mux
//      is here rather than on the Jetson so that a Jetson-side bug cannot
//      drive an autonomous rover from the teleop path.
//
//   Note one honest difference from the CircuitPython version: there is no
//   try/except guard here, because Arduino builds run without exceptions. The
//   hardware watchdog is the whole story for an unexpected fault, which is
//   why arming it is not optional.

#include <Arduino.h>

#include <FlexCAN_T4.h>
#include <Watchdog_t4.h>

#include "src/rover_controller.h"
#include "src/rover_drive.h"
#include "src/rover_protocol.h"

using namespace rover;

// ---------------------------------------------------------------------------
// Configuration
//
// Protocol and timing constants (CAN_BITRATE_HZ, HW_WATCHDOG_MS, the
// timeouts, the CAN ids, OVER_CURRENT_CA, UNDERVOLTAGE_CV) live in
// src/rover_config.h. Only board-specific wiring stays here.
// ---------------------------------------------------------------------------

static const int LED_PIN = 13;   // Teensy onboard LED

// Drive-stage output pins. PLACEHOLDERS -- nothing is wired to them yet, and
// they need checking against the Teensy 4.1 PWM-capable pin list and against
// whatever the drivetrain turns out to be. See setMotorOutputs().
static const int PIN_DRIVE_L = 2;
static const int PIN_DRIVE_R = 3;
static const int PIN_STEER   = 4;

// Servo/ESC pulse rate. 50 Hz is the universally safe default; many modern
// ESCs accept far more, which is worth having once the hardware is known,
// because 50 Hz puts up to 20 ms of latency on the end of a 20 Hz loop.
static const int PWM_FREQUENCY_HZ = 50;
static const int PWM_RESOLUTION_BITS = 12;
// LED_PIN is a telemetry heartbeat blink ONLY. It is not the status indicator.
//
// TODO (hardware): the URC status light has no pin assignment and no driver
// yet. RoverController::indicatorState() already decides what it should show
// and TELEM_STATE reports it, so the whole chain exists except the output --
// indicator_state is currently a value the Jetson can read and nobody can see.
//
// This costs POINTS, not just compliance: rule 1.e.xvii scores a target as
// reached only on "autonomously stopping within 1m of the target location and
// indicating its arrival", 25 points per target. So it is its own hardware
// item, not part of the setMotorOutputs() stub.
//
// Rule 1.e.ii also constrains the hardware, not only the colours: the
// indicator must be ON THE BACK of the rover and VISIBLE IN BRIGHT DAYLIGHT
// (it suggests an LED array or high-power LED). Every event runs in full
// daylight, so a bench-sized indicator will not pass.
//
// Mapping (PROTOCOL.md 3.5.1):
//   INDICATOR_OFF        off
//   INDICATOR_TELEOP     blue
//   INDICATOR_AUTONOMOUS red
//   INDICATOR_ARRIVED    flashing green   (this firmware owns the flash rate)
//   INDICATOR_FAULT      yellow, solid    (placeholder -- see PROTOCOL.md 9)
// Flip to false once real encoders / current sensing / steering feedback are
// wired in. See readSensors() for where the real code goes.
static const bool SIMULATE_SENSORS = true;

// CAN3 is the FD-capable bus on Teensy 4.1. Classic frames are used here so
// the protocol also runs on CAN1/CAN2 and alongside Classic-only motor
// controllers; switching this to CAN1 or CAN2 needs no other change.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
WDT_T4<WDT1> wdt;

static RoverController* ctl = nullptr;
static DriveModule drive_stage;
static uint16_t g_boot_faults = 0;

// ---------------------------------------------------------------------------
// Sensor / motor I/O -- placeholder implementations.
//
// These are the ONLY functions that should need touching once real hardware
// is available. Everything above is protocol and safety and should not change.
//
// Teensy 4.1 gives you 55 digital I/O and 18 analog inputs, so unlike the
// previous board you can wire encoders and current sense directly. Putting
// the motor controllers on CAN instead is still usually the better call --
// they report their own encoder counts and current over the bus.
// ---------------------------------------------------------------------------

static int32_t sim_enc_left  = 0;
static int32_t sim_enc_right = 0;
static int16_t sim_steer_fb  = 0;

struct SensorReading {
    int32_t  enc_left;
    int32_t  enc_right;
    int16_t  steer_fb;
    int16_t  current_ca;   // SIGNED centiamps  (1 cA = 10 mA)
    int16_t  voltage_cv;   // SIGNED centivolts (1 cV = 10 mV)
    uint16_t fault_bits;
    uint8_t  controller_health;  // CTRL_HEALTH_*
    int16_t  drive_applied;      // what the drive stage was handed this cycle
    int16_t  steer_applied;
};

// Takes the DRIVE STAGE's output rather than the raw command, so the
// simulated plant reflects the mix the real hardware would receive -- the
// same shape as tools/rover_sim.cpp, and the reason the two agree.
static SensorReading readSensors(const DriveOutputs& out) {
    SensorReading r;
    if (!SIMULATE_SENSORS) {
        // --- real sensor reads go here ---
        //   enc_*       : i.MX RT quadrature encoder peripherals, or CAN
        //   steer_fb    : analogRead() scaled into -1000..1000
        //   current_ca  : analogRead() scaled to SIGNED centiamps (1 cA=10 mA)
        //   voltage_cv  : analogRead() on the pack divider, SIGNED centivolts
        //   fault_bits  : FAULT_OVER_CURRENT / FAULT_ENCODER_FAULT / etc.
        //   controller_health : CTRL_HEALTH_* from motor-driver telemetry,
        //                 which on a CAN bus arrives as its own messages
        r.enc_left = 0; r.enc_right = 0; r.steer_fb = 0;
        r.current_ca = 0; r.voltage_cv = 0; r.fault_bits = 0;
        r.controller_health = drive_stage.health();
        r.drive_applied = 0; r.steer_applied = 0;   // filled in by loop()
        return r;
    }

    const int16_t target = out.steer_effort;

    // wrapI32 takes int64 and wraps explicitly. Letting an int32 counter
    // overflow would be UNDEFINED BEHAVIOUR in C++ -- not a wrap, not an
    // exception, but a compiler free to do anything at all.
    sim_enc_left  = wrapI32(static_cast<int64_t>(sim_enc_left)  + out.left_effort / 10);
    sim_enc_right = wrapI32(static_cast<int64_t>(sim_enc_right) + out.right_effort / 10);

    if (sim_steer_fb < target)      sim_steer_fb = min(target, (int16_t)(sim_steer_fb + 25));
    else if (sim_steer_fb > target) sim_steer_fb = max(target, (int16_t)(sim_steer_fb - 25));

    const int32_t effort_mag =
        (abs(out.left_effort) + abs(out.right_effort)) / 2;

    r.enc_left   = sim_enc_left;
    r.enc_right  = sim_enc_right;
    r.steer_fb   = sim_steer_fb;
    r.current_ca = out.energized ? static_cast<int16_t>(effort_mag * 3 / 2) : 0;
    // Fake 24 V pack sagging a little under load.
    r.voltage_cv = static_cast<int16_t>(2400 - effort_mag / 40);
    r.fault_bits = 0;
    if (r.current_ca > OVER_CURRENT_CA) r.fault_bits |= FAULT_OVER_CURRENT;
    if (r.voltage_cv < UNDERVOLTAGE_CV) r.fault_bits |= FAULT_UNDERVOLTAGE;
    // The drive stage reports its own health now. Still not real ESC
    // telemetry -- that arrives as its own CAN messages and overrides this --
    // but no longer a constant NOT_REPORTED.
    r.controller_health = drive_stage.health();
    r.drive_applied = 0;                        // filled in by loop()
    r.steer_applied = 0;
    return r;
}

// Convert a pulse width to the duty value analogWrite() wants at the
// configured frequency and resolution. Kept here rather than in rover_drive.*
// because it is board configuration, not protocol: the module speaks
// microseconds, which every servo and ESC datasheet also speaks.
static uint32_t pulseToDuty(uint16_t pulse_us) {
    const uint32_t full_scale = (1u << PWM_RESOLUTION_BITS) - 1u;
    const uint32_t period_us = 1000000u / PWM_FREQUENCY_HZ;
    return (static_cast<uint32_t>(pulse_us) * full_scale) / period_us;
}

static void setMotorOutputs(const DriveOutputs& out) {
    // The module has already applied the stop: every channel reads neutral
    // when it was told not to energize, so there is no second copy of the
    // rule here and no way for this function to disagree with it.
    //
    // VERIFY ON HARDWARE. analogWrite() produces a continuous PWM waveform,
    // which is what a brushed H-bridge or a PWM-input ESC wants. A hobby servo
    // or a PPM-input ESC wants a 1-2 ms pulse in a 20 ms frame instead, which
    // is the same numbers through a servo library rather than this. Check
    // which the drive hardware expects BEFORE connecting anything that moves.
    analogWrite(PIN_DRIVE_L, pulseToDuty(out.left_pwm_us));
    analogWrite(PIN_DRIVE_R, pulseToDuty(out.right_pwm_us));
    analogWrite(PIN_STEER,   pulseToDuty(out.steer_pwm_us));
}

// ---------------------------------------------------------------------------
// Boot fault detection
// ---------------------------------------------------------------------------

static uint16_t detectBootFault() {
    // SRC_SRSR is the i.MX RT reset status register; a watchdog reset means
    // the previous run of this firmware stopped feeding the watchdog, i.e. it
    // hung or crashed. The Jetson needs to know: telemetry resuming after an
    // unexplained gap otherwise looks like a flaky bus rather than a
    // controller that restarted mid-drive.
    //
    // VERIFY ON HARDWARE before trusting this bit mask. It is written to fail
    // SAFE -- an unrecognised reset cause reports nothing rather than raising
    // a false alarm -- but a missed watchdog reset is a silent gap, so confirm
    // it by deliberately hanging the loop once on the bench.
    const uint32_t srsr = SRC_SRSR;
    SRC_SRSR = srsr;                        // write-1-to-clear for next boot
    const uint32_t WDOG_BITS = (1u << 4) | (1u << 7);   // WDOG and WDOG3
    return (srsr & WDOG_BITS) ? FAULT_FIRMWARE_FAULT : 0;
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

// Four frames per cycle, because each must fit Classic CAN's 8 bytes once the
// per-frame sequence number and CRC are accounted for. All four carry the SAME
// sequence number, which is how the Jetson tells a coherent snapshot from one
// torn across two cycles. See rover_protocol.h for why not one CAN FD frame.
static void sendTelemetry(const SensorReading& s) {
    const uint8_t seq = ctl->telemetrySeq();
    CAN_message_t frame;

    TelemetryDriveL dl = ctl->buildDriveL(s.enc_left, s.steer_fb);
    frame.id  = CAN_ID_TELEM_DRIVE_L;
    frame.len = encodeTelemetryDriveL(dl, seq, frame.buf);
    Can.write(frame);

    TelemetryDriveR dr = ctl->buildDriveR(s.enc_right);
    frame.id  = CAN_ID_TELEM_DRIVE_R;
    frame.len = encodeTelemetryDriveR(dr, seq, frame.buf);
    Can.write(frame);

    TelemetryPower pw =
        ctl->buildPower(s.current_ca, s.voltage_cv, s.fault_bits, g_boot_faults);
    frame.id  = CAN_ID_TELEM_POWER;
    frame.len = encodeTelemetryPower(pw, seq, frame.buf);
    Can.write(frame);

    // c2_link comes from the c2_lost bit the Jetson forwards, via
    // c2LinkState(). It is deliberately NOT derived from the Jetson link --
    // the two are separate links and must never be treated as equivalent --
    // and it reads LINK_NOT_REPORTED while that bit is absent or stale, which
    // is an honest "we do not know" rather than a misleading LINK_OK.
    TelemetryState st = ctl->buildState(s.controller_health);
    frame.id  = CAN_ID_TELEM_STATE;
    frame.len = encodeTelemetryState(st, seq, frame.buf);
    Can.write(frame);

    // The acknowledgement: what the controller accepted, which frame it came
    // from, and why it is or is not being acted on. This is what lets the
    // Jetson tell "my command got through" from "the rover is moving".
    TelemetryCmdEcho ce = ctl->buildCmdEcho(s.drive_applied, s.steer_applied);
    frame.id  = CAN_ID_TELEM_CMD_ECHO;
    frame.len = encodeTelemetryCmdEcho(ce, seq, frame.buf);
    Can.write(frame);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Bring the drive pins up at neutral BEFORE anything else can run. A pin
    // left floating through setup() is a pin an ESC may read as a command.
    analogWriteResolution(PWM_RESOLUTION_BITS);
    for (int pin : {PIN_DRIVE_L, PIN_DRIVE_R, PIN_STEER}) {
        pinMode(pin, OUTPUT);
        analogWriteFrequency(pin, PWM_FREQUENCY_HZ);
        analogWrite(pin, pulseToDuty(PWM_NEUTRAL_US));
    }

    Serial.begin(115200);                   // USB console, for diagnostics only
    g_boot_faults = detectBootFault();

    Can.begin();
    Can.setBaudRate(CAN_BITRATE_HZ);
    // Accept only the Jetson's CONTROL id. Filtering in hardware keeps
    // motor-controller and payload traffic on a shared bus from ever reaching
    // the CPU, which matters far more at 1 Mbps than at 20 Hz.
    Can.setMBFilter(REJECT_ALL);
    Can.setMBFilter(MB0, CAN_ID_CONTROL);
    Can.enableMBInterrupts();

    static RoverController controller(millis, DEFAULT_WATCHDOG_TIMEOUT_MS,
                                      DEFAULT_TELEMETRY_PERIOD_MS);
    ctl = &controller;

    WDT_timings_t wdt_config;
    // Both fields are doubles in SECONDS. `trigger` defaults to 5 s, which
    // would sit ABOVE our 1 s timeout, so set it explicitly: it is when the
    // optional warning callback fires and must be below the reset timeout.
    wdt_config.trigger = (HW_WATCHDOG_MS / 1000.0) / 2.0;
    wdt_config.timeout = HW_WATCHDOG_MS / 1000.0;
    wdt.begin(wdt_config);

    Serial.printf("rover firmware up: CAN3 @ %lu bps, hw watchdog %lu ms%s\n",
                  (unsigned long)CAN_BITRATE_HZ, (unsigned long)HW_WATCHDOG_MS,
                  g_boot_faults ? ", RECOVERED FROM WATCHDOG RESET" : "");
}

void loop() {
    wdt.feed();

    // Drain everything waiting. read() returns false when the queue is empty.
    CAN_message_t rx;
    while (Can.read(rx)) {
        ctl->ingestFrame(rx.id, rx.buf, rx.len);
    }

    // The Task 4 chain, in order: the frame has been validated and the mode
    // and authority checked above; the controller now says what may be
    // driven, and the drive stage turns that into pin outputs.
    int16_t drive, steer;
    ctl->commandedOutputs(drive, steer);        // zeroed under stop OR under
                                                // a command-authority mismatch
    const bool stopped = ctl->effectiveStop();
    const DriveOutputs& out = drive_stage.update(drive, steer, !stopped);

    SensorReading s = readSensors(out);
    // Not sensor readings, so they are filled in here rather than inside
    // readSensors(): they are what the controller AUTHORISED, which is what
    // the acknowledgement frame is about.
    s.drive_applied = drive;
    s.steer_applied = steer;
    setMotorOutputs(out);

    if (ctl->telemetryDue()) {
        sendTelemetry(s);
        digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));
    }
}

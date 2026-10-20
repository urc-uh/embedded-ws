# URC Rover — Jetson ↔ Teensy Link (Tasks 3 & 4)

Communication link between the rover's **onboard computer** (Jetson Orin Nano,
candidate) and its **embedded controller** (**Teensy 4.1**, C++ / Teensyduino),
over **CAN** — and the autonomy-to-embedded interface built on top of it.

Framed, prioritised CAN messages with watchdog-based fail-safes. Full
specification: **[PROTOCOL.md](PROTOCOL.md)**.

**Task 3** proved a Jetson command reaches the controller intact. **Task 4**
proves a high-level autonomy decision travels that interface, reaches the
drive stage, and that the Jetson can read back that it did — including that
autonomy and teleop commands are refused in each other's modes.

## Status

| | |
|---|---|
| Protocol, safety logic, firmware | Implemented, **224 tests** passing |
| Cross-language codec | Pinned byte-for-byte, plus every validation predicate |
| Autonomy → drive-stage loop (Task 4) | Closed and verified end to end (`make autonomy`) |
| **Validated on real hardware** | **Not yet — see [Known gaps](#known-gaps)** |

---

## Quick start

**Nothing to install.** Needs a C++17 compiler and Python 3.9+, both of which
you already have. The host tests compile the *real firmware sources* rather
than mocking them, so there is no dependency to manage.

```bash
git clone https://github.com/schradivarius/URC-MCU-Jetson-Comm-Protocol.git
cd URC-MCU-Jetson-Comm-Protocol

make test      # 126 C++ tests + 98 host tests
make demo      # no-hardware message-exchange demonstration (Task 3)
make autonomy  # mock autonomy stack driving the firmware (Task 4)
```

If all three succeed, your environment is good.

---

## Layout

Two languages, because the two ends of the link are different machines:

```
firmware/                    C++ — runs on the Teensy 4.1
  rover_firmware.cpp           hardware wiring ONLY (CAN, watchdog, pins)
  src/rover_config.h           every protocol + timing constant ─┐
  src/rover_protocol.*         message definitions + codec      ─┤ pure C++,
  src/rover_controller.*       command/safety state machine      │ no Arduino
  src/rover_drive.*            drive/steering output stage      ─┘

host/                        Python — runs on the Jetson
  rover_config.py              constants (pinned to rover_config.h)
  rover_protocol.py            codec (pinned to the C++ one)
  protocol_root.py             which checkout firmware/src comes from
  can_link.py                  sim + python-can backends
  c2_link.py                   C2 (base-station) link-loss detection
  jetson_test.py               live harness
  demo.py                      the Task 3 demonstration
  autonomy_sim.py              mock autonomy stack — the Task 4 mission

tools/
  rover_sim.cpp                simulator: REAL controller + drive stage, fake plant
  golden_vectors.cpp           emits vectors for cross-language pinning

tests/cpp/                   126 tests — firmware core
tests/host/                  98 tests — config sync, golden vectors, integration,
                                        the autonomy mission
```

The files under `firmware/src/` have **no Arduino dependency and no hardware
calls**, which is what lets the safety-critical logic be unit-tested natively
with a fake clock. `rover_firmware.cpp` is only the wiring around them. **If
you are reviewing safety behaviour, read `rover_controller.h`.**

The autonomy program can also live in a repository of its own — it is a
consumer of this protocol, not part of it. Two files move, the Python codec
stays, and `ROVER_PROTOCOL_ROOT` points the Jetson code at a checkout of this
one. [PROTOCOL.md §10.2](PROTOCOL.md) says why copying the codec instead would
defeat the cross-language pinning.

---

## Running the tests

```bash
make test        # everything
make test-cpp    # firmware core only — no Python needed
make test-host   # host side only — compiles the C++ core
```

| Suite | Tests | Answers |
|---|---:|---|
| `tests/cpp/test_protocol.cpp` | 41 | Wire format, CRC-8, and every validation predicate |
| `tests/cpp/test_controller.cpp` | 68 | **Does the rover stop when it should?** And: does it refuse a command from the wrong half of the stack? |
| `tests/cpp/test_drive.cpp` | 17 | Does a command become the right motor output — and never one it was told not to? |
| `tests/host/test_config_sync.py` | 14 | Do the C++ and Python configs hold the same constants? |
| `tests/host/test_golden_vectors.py` | 30 | Do C++ and Python agree byte-for-byte? |
| `tests/host/test_integration.py` | 44 | Do both ends actually talk to each other? |
| `tests/host/test_autonomy_mission.py` | 10 | Does an autonomy decision reach the drive stage — and would we notice if it stopped? |

Every test is named for the failure it prevents, so the reason it exists
outlives anyone's memory of writing it. **Start a safety review at
`test_controller.cpp`.**

Three of these were verified to *fail* against a deliberately broken
implementation, rather than merely assumed to work:

- `watchdog_is_correct_across_millis_wraparound` — fails against a naive
  timestamp comparison
- `test_python_encoder_matches_cpp_byte_for_byte` — fails against an injected
  endianness flip
- `wrong_dlc_does_not_refresh_the_watchdog` — the version-mismatch guard
- `a_bad_crc_does_not_refresh_the_watchdog` — the application-layer CRC's whole
  purpose: corruption in the software path, which CAN's own CRC cannot see
- `the_c2_link_is_never_derived_from_the_jetson_link` and
  `c2_loss_stops_a_teleoperated_rover_but_not_an_autonomous_one` — the 2027
  course has areas with no C2 line-of-sight while autonomy keeps working;
  collapsing the two links would stop the rover exactly where it should keep
  going
- `a_stale_c2_bit_is_reported_as_unknown_not_as_ok_or_lost` — once the Jetson
  goes silent the last forwarded C2 verdict is worthless, so the controller
  stops asserting it rather than latching a claim it cannot vouch for
- `a_non_stopping_fault_does_not_blank_the_indicator` — URC requires red while
  autonomous, so an undervoltage warning must not take the light away from
  what the rover is actually doing
- `undefined_mode_does_not_permit_motion` — fails against the pre-fix
  `mode == MODE_DISABLED` check (that mode is `MODE_SAFE` now), which permitted full throttle on any
  undefined mode ([issue #4](https://github.com/schradivarius/URC-MCU-Jetson-Comm-Protocol/issues/4))
- `the_same_command_is_accepted_once_the_mode_matches` — one command, refused
  in `TELEOP` and applied in `AUTONOMOUS`, with nothing else changed: the Task
  4 claim that manual and autonomous commands do not get mixed up
- `test_an_impossible_expectation_fails_the_run` and
  `test_every_step_passed_on_telemetry_it_caused` — the negative controls for
  the autonomy mission. A harness that cannot fail is not evidence, and a step
  that passes on the *previous* step's telemetry is the classic way an
  integration test goes green while proving nothing

CI additionally builds with **clang** as a second toolchain and runs the suite
under **UBSan + ASan**, because signed overflow in the encoder accumulators
would be undefined behaviour rather than a wrap.

---

## Running against hardware

### A virtual CAN bus (no hardware, real CAN stack)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
pip install python-can
python3 host/jetson_test.py --channel vcan0
```

### The real Teensy

**PlatformIO.** `platformio.ini` is in the repo root and names the board and
both libraries, so there is nothing to install by hand:

```bash
pio run              # compile for the Teensy 4.1
pio run -t upload    # compile and flash
pio device monitor   # the boot banner, 115200
```

Then, from the Jetson side:

```bash
python3 host/jetson_test.py --channel can0 --bitrate 500000
```

> **Why not the Arduino IDE.** The entry point is `firmware/rover_firmware.cpp`,
> a plain translation unit rather than an Arduino `.ino` sketch. The IDE needs a
> `.ino` to open a sketch at all, so it cannot build this tree as it stands. The
> gain is that nothing rewrites the file before the compiler sees it: a sketch
> gets forward declarations injected near the top, which land ahead of the
> structs they name in a file shaped like this one. If you do want the IDE back,
> add a `firmware/rover_firmware.ino` containing the single line
> `#include "rover_firmware.cpp"` — but then keep both toolchains honest, because
> only one of them is exercised here.

First flash is worth doing with **nothing connected to the drive pins or the
CAN bus** — the boot banner alone exercises the build, the boot-fault check and
the watchdog arming, with nothing able to move.

> **Before powering anything.**
> - Teensy 4.1 is **3.3 V and NOT 5 V tolerant.** A 5 V sensor on a pin
>   destroys the board. Level-shift everything.
> - Each CAN bus needs a **transceiver** — the MCU cannot drive a differential
>   bus directly.
> - The bus needs **120 Ω termination at both physical ends.** Exactly one
>   resistor, or three, is the single most common CAN bring-up failure.

The firmware prints a banner over USB serial on boot:

```
rover firmware up: CAN3 @ 500000 bps, hw watchdog 1000 ms
```

`RECOVERED FROM WATCHDOG RESET` appended means the previous run hung or
crashed — see `PROTOCOL.md` §5.2.

---

## Known gaps

Stated plainly, so nobody builds on an unchecked assumption:

- **Never run on a real Teensy or a real CAN bus.** Every library call was
  verified against the `FlexCAN_T4` and `WDT_T4` headers, and the whole
  firmware — `rover_firmware.cpp` plus everything in `src/` — compiles *and
  links* under a host compiler at `-Wall -Wextra -Werror` against stub
  `Arduino.h`, `FlexCAN_T4.h` and `Watchdog_t4.h`. What that does not cover is
  the real `arm-none-eabi` build against the actual Teensyduino and library
  headers, which has never run, and the board, which has never been flashed.
  Bench validation is owed.
- **`SRC_SRSR` watchdog bit mask is unverified.** It fails safe — an
  unrecognised reset cause reports nothing — but confirm it by deliberately
  hanging the loop once on the bench.
- **Sensor I/O is stubbed** behind `readSensors()`. The drive stage is real and
  unit-tested (`rover_drive.*`), but its output pins are placeholders and the
  PWM mode is unverified: `analogWrite()` suits a brushed H-bridge or a
  PWM-input ESC, while a hobby servo or a PPM-input ESC wants a 1–2 ms pulse in
  a 20 ms frame. **Establish which the hardware expects before connecting
  anything that moves.**
- **`return_request` is acknowledged, not acted on.** The controller reports
  that it arrived; whether it should *do* anything is an open team decision in
  `PROTOCOL.md` §9, because it changes `effectiveStop()`.
- **Two encoders, one aggregate current reading.** Both need to grow, and both
  change the payload when they do.
- **CAN FD, the second and third CAN buses, and a dedicated e-stop frame** are
  all unused. The `0x000`–`0x0FF` id block is reserved for that e-stop.

## CI

Every push runs the C++ suite under g++ **and** clang **and** sanitizers, plus
the host suite on Python 3.9 / 3.11 / 3.12 —
[`.github/workflows/ci.yml`](.github/workflows/ci.yml).

## License

GPL-3.0 — see [LICENSE](LICENSE).

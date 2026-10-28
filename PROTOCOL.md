# Rover Onboard Computer ↔ Embedded Controller Protocol (v0.6)

**Scope:** Onboard computer (Jetson Orin Nano, candidate) ↔ embedded controller
(**Teensy 4.1**, NXP i.MX RT1062, C++ / Teensyduino), over **CAN**.

> **v0.4 — structured command and telemetry packets.** Added a sequence number
> and an application-layer CRC-8 in both directions, plus the autonomy-abort,
> return-request and status-indicator fields, voltage, operating-mode echo,
> per-link health and low-level controller health. `fault_status` widened to
> `uint16`. Telemetry became **four frames** carrying one shared sequence
> number.
>
> **v0.5 — C2 and the status indicator, reconciled.** v0.4 was written against
> a `main` that predated two merged changes (PRs #13 and #15), so it left the
> C2 link as a placeholder and used its own indicator policy. v0.5 folds both
> in and nothing is a placeholder any more:
>
> * `c2_lost` is a real CONTROL field again, at **flags bit 6**, so the Jetson
>   forwards its verdict on a link the controller cannot see.
> * `FAULT_C2_LINK_LOST` claims the `0x0080` bit v0.4 had reserved for it.
> * `TELEM_STATE.c2_link` is populated from that bit, and goes
>   `LINK_NOT_REPORTED` when the value is stale. That replaces v0.3.1's
>   receiver-side `C2_UNKNOWN` convention: the staleness rule now lives in the
>   **sender**, so a receiver that forgets it cannot be misled (§3.5).
> * The indicator follows the controller's own mode, per v0.3.1's rule table,
>   with `INDICATOR_FAULT` added for the cases where nothing is driving the
>   rover (§3.5). v0.4 forced `FAULT` on *any* fault bit, which would have
>   taken the light away from red mid-autonomy over an undervoltage warning.
>
> **v0.6 — autonomy to the drive stage, and proof it arrived.** Task 3 proved a
> command can reach the controller intact. Task 4 is about it reaching the
> *drive stage*, and about the Jetson being able to prove that it did. Two
> things were missing for that, and v0.6 adds exactly those two:
>
> * **A command source.** `flags` **bit 7**, previously reserved, is
>   `from_autonomy`: did the onboard autonomy stack produce this frame's
>   drive/steer, or the human operator? The controller refuses *motion* from a
>   producer that does not own the current mode — a mux, deliberately on the
>   MCU (§3.1.5). `FAULT_CMD_SOURCE` (`0x0400`) reports the refusal.
> * **An acknowledgement frame.** `TELEM_CMD_ECHO` (`0x204`) carries the drive
>   and steer actually handed to the drive stage, the sequence number they came
>   from, and a verdict byte saying whether they are being applied, overridden
>   by a stop, or refused for lacking authority (§3.6). Telemetry is therefore
>   **five frames** per cycle, not four.
>
> Two consequences worth knowing before reading on. The flags byte now has **no
> reserved bits**, so `DECODE_BAD_FLAGS` is kept in the enum but no longer
> produced and the next field goes in the mode byte (§3.1.2). And
> `controller_health` is no longer a constant `NOT_REPORTED`: the new drive
> stage reports its own state (§3.5).
>
> Full field tables with types, units, ranges, meanings and update rates are in
> section 3.

---

## 1. Design goals

- **No hardware dependency for development.** The protocol, the safety logic
  and the full exchange run and are tested on a laptop with no board, no CAN
  interface and no drivers (section 7).
- **Fail-safe on silence**, and fail *closed* on anything unrecognized.
- **End-to-end protected**, not merely wire-protected (section 4).
- **One implementation of the safety rule**, with the codec pinned across the
  two languages that must both speak it (section 8).
- **Portable across CAN flavours.** Every frame is 8 bytes, so this runs on
  Classic CAN or CAN FD, alongside Classic-only motor controllers.

## 2. Link basics

| Parameter | Value |
|---|---|
| Physical layer | CAN 2.0B, differential twisted pair, **120 Ω termination at both ends** |
| Bit rate | 500 kbps (placeholder; 1 Mbps is fine at these frame sizes) |
| Bus | Teensy 4.1 **CAN3** (FD-capable) — CAN1/CAN2 work unchanged |
| Frame format | Standard, 11-bit identifiers, **every frame DLC 8** |
| CONTROL rate | 20 Hz (1 frame per cycle) |
| TELEMETRY rate | 20 Hz (5 frames per cycle = 100 frames/s) |
| Byte order | Little-endian, written byte-by-byte (not struct-punned) |

At 500 kbps an 8-byte frame is roughly 220 µs of airtime. Six frames per cycle
is ~1.3 ms of a 50 ms period — about **3% bus utilisation**. Throughput is
nowhere near the constraint; CAN was chosen for determinism and noise immunity,
not speed. Say so plainly when anyone asks.

### 2.1 Identifiers *are* priorities

CAN arbitration is bitwise and dominant-low, so the **numerically lowest id
wins the bus** and the loser backs off without its message being corrupted.
This table is a priority ordering, not just a set of names.

| ID | Message | Direction | DLC | Rate |
|---|---|---|---|---|
| `0x000`–`0x0FF` | *reserved* — headroom for a dedicated e-stop frame | — | — | — |
| **`0x100`** | `CONTROL` | Jetson → controller | 8 | 20 Hz |
| **`0x200`** | `TELEM_DRIVE_L` | controller → Jetson | 8 | 20 Hz |
| **`0x201`** | `TELEM_DRIVE_R` | controller → Jetson | 8 | 20 Hz |
| **`0x202`** | `TELEM_POWER` | controller → Jetson | 8 | 20 Hz |
| **`0x203`** | `TELEM_STATE` | controller → Jetson | 8 | 20 Hz |
| **`0x204`** | `TELEM_CMD_ECHO` | controller → Jetson | 8 | 20 Hz |

Commands outrank telemetry because a late command can hurt the rover and late
telemetry only annoys an operator.

### 2.2 Frame envelope

Every frame, both directions, has the same shape:

```
 byte:  0    1    2    3    4    5      6      7
      +----+----+----+----+----+----+-------+-------+
      |     6-byte message payload     |  SEQ  | CRC8  |
      +----+----+----+----+----+----+-------+-------+
```

| Field | Type | Units | Range | Meaning |
|---|---|---|---|---|
| `seq` | uint8 | frames | 0–255, **wraps** | Increments once per transmitted cycle. Receiver uses the delta: 1 healthy, 0 duplicate, >1 frames lost. |
| `crc8` | uint8 | — | 0–255 | CRC-8/SAE-J1850 over the CAN id then bytes 0–6. See section 4. |

**Why five telemetry frames and not one CAN FD frame.** 6 payload bytes × 5
frames = 30 bytes, which is what the field list needs. One 64-byte FD frame
would hold it all atomically, but would require FD-capable transceivers on
every node of the bus including Classic-only motor controllers. The field
definitions would not change if you later move telemetry to FD.

**Snapshot tearing.** All five telemetry frames of one cycle carry the **same
`seq`**. The Jetson therefore knows whether the five frames it holds came from
one cycle or straddle two, and can discard or flag a torn snapshot instead of
silently mixing a fresh encoder reading with a stale fault word.
`host/jetson_test.py` counts torn snapshots; a clean link produces zero.

**There is no version field on the wire** (§6), so a controller running an
older revision simply sends fewer frames and the Jetson's snapshot never
completes. That is silent, so `TelemetryAssembler.missing_ids()` names the ids
nothing has ever arrived on and `jetson_test.py` prints them — which ids showed
up *is* the version diagnostic.

## 3. Message definitions

### 3.0 Requirement coverage

Every field the Task 3 and Task 4 briefs enumerate, and where it lives.
Nothing in either direction is "to be added later".

**Jetson → MCU command packet** — one structured message, `CONTROL` (§3.1):

| Required field | Where | Type |
|---|---|---|
| sequence number | `CONTROL` byte 6 | uint8 |
| operating mode | `CONTROL` byte 4 | uint8 enum |
| desired drive command | `CONTROL` bytes 0–1 | int16 |
| desired steering command | `CONTROL` bytes 2–3 | int16 |
| stop command | `flags` bit 0 | bool |
| autonomy-abort flag | `flags` bit 1 | bool |
| return-request flag | `flags` bit 2 | bool |
| status-indicator request | `flags` bits 3–5 | uint8 enum |
| command source (Task 4) | `flags` bit 7 | bool |
| checksum / CRC | `CONTROL` byte 7 | uint8 CRC-8 |

**MCU → Jetson telemetry packet** — one logical packet, sent as five frames of
a shared cycle (§2.2, §3.2–3.6):

| Required field | Where | Type |
|---|---|---|
| sequence number | byte 6 of all five frames | uint8 |
| encoder values | `TELEM_DRIVE_L` / `_R` bytes 0–3 | int32 ×2 |
| steering feedback | `TELEM_DRIVE_L` bytes 4–5 | int16 |
| current status | `TELEM_POWER` bytes 0–1 | int16 |
| voltage status | `TELEM_POWER` bytes 2–3 | int16 |
| current operating mode | `TELEM_STATE` byte 0 | uint8 enum |
| fault flags | `TELEM_POWER` bytes 4–5 | uint16 bitfield |
| command age | `TELEM_DRIVE_R` bytes 4–5 | uint16 |
| Jetson-link health | `TELEM_STATE` byte 1 | uint8 enum |
| C2-link state (as forwarded) | `TELEM_STATE` byte 2 | uint8 enum |
| low-level controller health | `TELEM_STATE` byte 3 | uint8 enum |
| status-indicator state | `TELEM_STATE` byte 4 | uint8 enum |
| accepted drive command (Task 4) | `TELEM_CMD_ECHO` bytes 0–1 | int16 |
| accepted steering command (Task 4) | `TELEM_CMD_ECHO` bytes 2–3 | int16 |
| acknowledged sequence number (Task 4) | `TELEM_CMD_ECHO` byte 4 | uint8 |
| command verdict (Task 4) | `TELEM_CMD_ECHO` byte 5, bits 0–2 | uint8 enum |
| checksum / CRC | byte 7 of all five frames | uint8 CRC-8 |

Task 4's feedback loop asks for six fields by name. Five were already here —
current mode, controller health, command age and fault state, plus the
sequence number that ties a cycle together. The two that were not are the
**accepted drive and steering commands**, and their absence was the real gap:
encoders and steering feedback say what the rover *did*, which an autonomy
stack cannot use to tell "my command got through" from "the rover happens to
be moving". `TELEM_CMD_ECHO` answers the second question.

**Every field in both directions updates at 20 Hz**, the rate of its frame. The
rate is per frame, not per field: there is no slow-changing field sent less
often, because at 8 bytes a frame the bus is ~3% utilised (§2) and a second rate
would only add a staleness question nobody needs to ask.

### 3.1 `CONTROL` — `0x100`, DLC 8, 20 Hz, Jetson → controller

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0–1 | `drive_cmd` | int16 | 0.1 % of full drive effort | −1000 … +1000 | Desired drive. Positive is forward. A value outside the range **rejects the whole frame** — see 3.1.4. |
| 2–3 | `steer_cmd` | int16 | 0.1 % of full steering range | −1000 … +1000 | Desired steering. Positive is right. Range-checked exactly like `drive_cmd`. |
| 4 | `mode` | uint8 | enum | **0, 1, 2 only** | `0=SAFE, 1=TELEOP, 2=AUTONOMOUS`. Any other value, including `3=FAULT`, **rejects the frame** — see 3.1.1. |
| 5 | `flags` | uint8 | bitfield | see below | Six fields packed, no spare bits; see 3.1.2. |
| 6 | `seq` | uint8 | frames | 0–255 wraps | See 2.2. |
| 7 | `crc8` | uint8 | — | 0–255 | See 4. |

#### 3.1.1 `mode` validation

`mode` is a uint8 carrying 256 possible values where four are defined and
three may be commanded. Two separate questions follow, and conflating them was
a real bug:

| Question | Answer |
|---|---|
| May the Jetson *send* this value? (`isCommandableMode`) | `0`, `1`, `2`. **`SAFE` is valid** — a legitimate command meaning "do not move". `3=FAULT` is not: only the controller enters it. |
| May the rover *move*? (`modePermitsMotion`) | `TELEOP` or `AUTONOMOUS` only — a **whitelist**. |

**Commanded mode vs. active mode.** `SAFE` means stopped *on purpose*;
`FAULT` means stopped because a link the mode needs is gone. The controller
works out which mode it is actually executing with
`RoverController::activeMode()`:

| Situation | Active mode |
|---|---|
| No valid `CONTROL` yet, or none for 300 ms (`COMM_TIMEOUT`) | `FAULT` |
| `TELEOP` commanded, but `c2_lost = 1` | `FAULT` |
| Anything else | the commanded mode |

`FAULT` is not latched: once the link is back, the active mode is whatever was
last commanded. `effectiveStop()` and `indicatorState()` are both derived from
it, so the stop decision, the mode and the rear light cannot disagree.
`TELEM_STATE.mode` still echoes the *commanded* mode.

A non-commandable mode **rejects the whole frame**, exactly like a bad CRC: the
sender disagrees with us about the protocol. Deliberately **not**
forward-compatible — a newer peer sending a mode this firmware does not
implement must stop this rover. Adding a fourth mode means updating every
controller on the bus *before* a host may send it.

#### 3.1.2 `flags` byte (byte 5)

| Bits | Field | Type | Range | Meaning |
|---|---|---|---|---|
| 0 | `stop` | bool | 0/1 | Stop now, regardless of `mode`. Takes effect on the cycle it arrives — no coast frame. Separate from `mode` so an e-stop can be asserted **and released** without a mode round trip. |
| 1 | `autonomy_abort` | bool | 0/1 | Abandon the current autonomous task. **Forces a stop while `mode == AUTONOMOUS`**; in `TELEOP` the operator is already driving, so it has no autonomous task to abort and does not stop. ⚠️ *This behaviour is a judgement call, not spelled out by the requirement — flagged for review.* |
| 2 | `return_request` | bool | 0/1 | Begin the return-to-base behaviour. Carried, and since v0.6 **acknowledged** in `TELEM_CMD_ECHO` bit 6 while `mode == AUTONOMOUS`; the controller still takes no low-level action on it — see §3.1.6. |
| 3–5 | `indicator_request` | uint8 | **0–4** | Requested status-indicator state. `0=OFF, 1=TELEOP, 2=AUTONOMOUS, 3=ARRIVED, 4=FAULT`. 5–7 are reachable on the wire and **reject the frame**. The controller honours this only where it agrees with its own mode — see §3.5. |
| 6 | `c2_lost` | bool | 0/1 | `1` = the Jetson has lost the base-station (C2) link. The Jetson's report on a link the controller cannot see — see below. |
| 7 | `from_autonomy` | bool | 0/1 | **v0.6.** `1` = the onboard autonomy stack produced this frame's `drive_cmd`/`steer_cmd`; `0` = the human operator did. Gates **motion and the arrival claim only** — see §3.1.5. Reserved before v0.6. |

**The flags byte is now full.** The next field to be added goes in the `mode`
byte, whose top five bits are unused (`MODE_MAX` is 2, and `MODE_FAULT` is 3) and which already
rejects anything that may not be commanded through `isCommandableMode()`. That is why
`DECODE_BAD_FLAGS` is **kept in the `DecodeResult` enum but no longer
produced**: a receiver still meets it from a v0.5 sender, and the next revision
to reserve a bit revives it. Renumbering a wire enum to close a gap is how two
ends come to disagree about what `5` means.

**Version skew in both directions fails safe**, which is worth stating because
there is no version field on the wire (§6):

| Pairing | What happens |
|---|---|
| v0.6 Jetson → v0.5 controller | A teleop frame (bit 7 clear) is accepted normally. An **autonomy** frame (bit 7 set) hits v0.5's reserved-bit check, is rejected as `DECODE_BAD_FLAGS`, does not refresh the watchdog, and the rover stops. |
| v0.5 Jetson → v0.6 controller | Every frame reads as teleop-sourced. `TELEOP` works; `AUTONOMOUS` refuses motion and raises `FAULT_CMD_SOURCE`, and the rover sits still. |

Neither direction moves a rover on a misunderstood command, which is the only
property that matters here.

#### 3.1.3 `c2_lost` — the one field the controller cannot determine itself

The controller detects a silent Jetson on its own (the command watchdog), but
it has **no radio** and therefore no view of the base station ↔ Jetson link. So
the Jetson watches that link (`C2Monitor` in `host/c2_link.py`: no base-station
heartbeat within `C2_timeout_s`, default 1.0 s) and forwards the verdict in
every `CONTROL` frame.

**The two link failures are deliberately not equivalent.** C2 loss with a
healthy Jetson is *expected* on the autonomy course. **Rule 1.e.xvi puts it
there on purpose:** the second route-finding target is *"intentionally placed
behind the hill out of radio communications line of sight with the C2
station."* Line of sight to the
base station drops while onboard autonomy keeps running.

| Mode | `c2_lost = 1` → | Why |
|---|---|---|
| `TELEOP` | **stop** | The operator's commands can no longer arrive, so nobody is driving |
| `AUTONOMOUS` | **keep driving** | The Jetson is driving, and is provably alive or the command watchdog would already have stopped the rover |
| `SAFE` | stopped anyway | — |

`FAULT_C2_LINK_LOST` is raised in **every** mode: during an autonomous run it
is the only sign that the rover is out of contact, which an operator needs
whether or not it changes the driving. The per-mode *stop* policy is an initial
choice and may be revised; the reporting is not negotiable.

Two implementation details that exist to stop this field lying:

- **`encode_control()` has no default for `c2_lost`.** A caller that forgot it
  would silently report the link as healthy, which is the one direction of
  error that lets a rover keep driving in `TELEOP` with nobody able to reach
  it. A missing argument is a loud failure instead.
- **The controller boots with `c2_lost = 1`** and holds it until a valid frame
  says otherwise, matching every other boot default (assume the worst). It is
  only ever *visible* through `c2LinkState()`, which reports
  `LINK_NOT_REPORTED` until a real frame arrives, so the boot default cannot
  masquerade as forwarded state.

#### 3.1.4 `drive_cmd` / `steer_cmd` range validation

Both fields are int16 on the wire, so they can carry −32768…+32767 while only
−1000…+1000 means anything. A value outside that range is **not** "more than
full throttle" — it is a sender that disagrees with us about the scale or the
layout, the same class of fault as an undefined `mode` or a set reserved bit.

So the decoder **rejects the whole frame** (`DECODE_BAD_RANGE`) rather than
clamping it. Clamping is the dangerous option: it would quietly turn a
misunderstood command into full throttle. Rejecting is safe, because a rejected
frame does not refresh the command watchdog — the rover stops, and
`PROTOCOL_ERROR` says why (§4.2).

This does **not** replace clamping in the motor layer. That is a different job:
the motor layer protects the hardware from a command we understood, while this
protects the rover from a command we did not.

±1000 is full scale and **legal** — off by one here is the whole bug, so the
boundary is pinned on both sides by the `RANGE` table in
`tools/golden_vectors.cpp`.

#### 3.1.5 `from_autonomy` — keeping manual and autonomous commands apart

Both producers live inside the Jetson and both emit on `0x100`, so without a
tag the controller cannot tell an autonomy-generated command from a
teleoperated one. That is the mix-up Task 4 asks to be ruled out: a stale
autonomy command executing after the operator has taken over, or the reverse.

**The rule.** A producer may command *motion* only in the mode it owns:

| `mode` | Owner | A frame from the other producer |
|---|---|---|
| `AUTONOMOUS` | the autonomy stack (`from_autonomy = 1`) | accepted, motion refused |
| `TELEOP` | the human operator (`from_autonomy = 0`) | accepted, motion refused |
| `SAFE` | nobody — nothing may move, so there is nothing to own | no mismatch possible |

**Refused, not rejected.** A mismatched frame is *well formed*. It decodes, it
refreshes the command watchdog, its `mode` is echoed and its safety flags are
obeyed — only `drive_cmd`, `steer_cmd` and an `ARRIVED` indicator request are
dropped. Rejecting it instead would turn a Jetson-side mux bug into a comm
timeout, stopping the rover and pointing the operator at the wrong fault. What
the operator gets instead is `CMD_STATUS_WRONG_SOURCE` in `TELEM_CMD_ECHO` and
`FAULT_CMD_SOURCE` in `fault_status`, both naming the actual problem. The
condition is self-clearing: the next authorised frame drops it.

**Safety flags are never gated by it.** `stop`, `autonomy_abort`,
`return_request` and `c2_lost` are honoured from either producer, because
anyone who can see the rover is in trouble must be able to stop it, and because
under rule 1.e.xi signalling a stop or a return is the *operator's* action
during an autonomous run. Only the two fields that assert what the rover is
*doing* — the motion command, and the arrival claim worth 25 points under
1.e.xvii — need the source to own the mode.

**The mux is on the MCU, and that is the point.** The Jetson could refuse to
emit an autonomy command while teleoperated, and it should. But then one
Jetson-side bug, one stale queue, or one mode change racing a command in
flight is enough to drive an autonomous rover from the teleop path. The
controller is the only place that sees the mode and the command *in the same
frame*, so it is the only place the two can be checked against each other.

**What this does not catch**, stated plainly: the check is between two fields
of one frame, so a Jetson that sends a *stale mode* alongside a stale command
is self-consistent and passes. That case is caught one layer up, by the mode
echo in `TELEM_STATE` (§3.5) disagreeing with what the operator set — which is
exactly why the echo exists. The gate narrows the window; it does not close it,
and nothing inside one trusted sender can.

**Why a bit and not a second CAN id.** A dedicated id per producer would bind
the source to the frame through the CRC seed, which is strictly stronger. It
would also put `mode`, `stop`, `c2_lost` and `indicator_request` in two frames
at once and need a precedence table for every one of them — two sources of
truth for every field, where today there is one. The producers are both inside
one trusted node, so the weaker binding costs nothing real and the simpler
state machine is worth a great deal.

#### 3.1.6 `return_request` — acknowledged, deliberately not acted on

`return_request` is in Task 4's command list, so the Jetson must be able to
send it and see that it arrived. It is also one of the two items §9 leaves as a
**team decision**, because changing what it *does* changes `effectiveStop()`
and therefore safety behaviour.

v0.6 does the part that is unambiguous and nothing more: while `mode ==
AUTONOMOUS` and the frame is not stale, the controller reports the request in
`TELEM_CMD_ECHO` bit 6. `effectiveStop()` is untouched, the indicator stays red
(a returning rover is still under autonomous operation), and the drive stage
sees no difference.

That split is not a dodge. Driving back to a previously visited location is
**route planning**, which Task 4 explicitly places on the autonomy side — so
the behaviour belongs in the autonomy stack, which executes the return by
sending ordinary drive and steer commands. What only the controller can do is
confirm the request arrived, and that is what it now does. The open question in
§9 — whether `return_request` should move the rover and whether
`autonomy_abort` should stop it — is unchanged and still Calb's call.

### 3.2 `TELEM_DRIVE_L` — `0x200`, DLC 8, 20 Hz

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0–3 | `enc_left` | int32 | encoder ticks | full int32, **wraps** | Left-side cumulative count. **Relative only** — difference consecutive readings and handle rollover. |
| 4–5 | `steer_fb` | int16 | 0.1 % of full steering range | −1000 … +1000 | Measured steering position, same scale as `steer_cmd`. |

### 3.3 `TELEM_DRIVE_R` — `0x201`, DLC 8, 20 Hz

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0–3 | `enc_right` | int32 | encoder ticks | full int32, **wraps** | Right-side cumulative count. |
| 4–5 | `cmd_age_ms` | uint16 | milliseconds | 0 … 65533, plus 2 sentinels | Time since the last **valid** CONTROL frame. See below. |

| `cmd_age_ms` value | Meaning |
|---|---|
| 0 … 65533 | A real measurement |
| `0xFFFE` (`CMD_AGE_MAX`) | Saturation ceiling — "at least 65.534 s" |
| `0xFFFF` (`CMD_AGE_UNKNOWN`) | **No valid CONTROL frame has ever arrived** |

Two distinct values because "you have never spoken to me" is a wiring or
bus-config problem while "you stopped speaking 65 s ago" is something that died
mid-mission. Different diagnoses.

### 3.4 `TELEM_POWER` — `0x202`, DLC 8, 20 Hz

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0–1 | `current_ca` | **int16** | centiamps (1 cA = 10 mA) | −327.68 … +327.67 A | Motor current. **Signed**: a braking motor genuinely produces negative current, and unsigned would wrap it to a huge positive value — the worst kind of bad telemetry, because it looks plausible. |
| 2–3 | `voltage_cv` | **int16** | centivolts (1 cV = 10 mV) | −327.68 … +327.67 V | Pack voltage. Signed for symmetry and to make a mis-wired sense line obvious rather than enormous. |
| 4–5 | `fault_status` | **uint16** | bitfield | see below | Active faults. |

**`fault_status`** — widened from `uint8` in v0.4, which had exactly one bit
left. Bits `0x0001`–`0x0040` keep their v0.3 values.

| Bit | Name | Meaning |
|---|---|---|
| `0x0001` | `COMM_TIMEOUT` | No valid CONTROL frame within the watchdog |
| `0x0002` | `OVER_CURRENT` | Current exceeded a threshold |
| `0x0004` | `ESTOP_ACTIVE` | A **soft** e-stop is engaged. Deliberately *not* the rules-mandated E-stop — see below |
| `0x0008` | `ENCODER_FAULT` | Encoder reading invalid or stalled |
| `0x0010` | `UNDERVOLTAGE` | Supply voltage below threshold |
| `0x0020` | `FIRMWARE_FAULT` | This boot followed a watchdog reset (5.2) |
| `0x0040` | `PROTOCOL_ERROR` | A frame on our id was uninterpretable: wrong DLC, undefined mode, bad flags, bad indicator, or `drive_cmd`/`steer_cmd` out of range |
| `0x0080` | `C2_LINK_LOST` | The last `CONTROL` frame reported the base-station link lost (`c2_lost = 1`). Raised in every mode; stops the rover only in `TELEOP` (§3.1.3). Cleared, not latched, once the value goes stale — see §3.5 |
| `0x0100` | `SEQ_GAP` | CONTROL frames were lost, duplicated or reordered |
| `0x0200` | `CRC_ERROR` | Application-layer CRC mismatch on our id |
| `0x0400` | `CMD_SOURCE` | A command arrived from the producer that does not own the current mode: an autonomy frame in `TELEOP`, or an operator frame in `AUTONOMOUS`. Drive and steering are refused; see §3.1.5 |
| `0x0800`–`0x8000` | *reserved* | |

`PROTOCOL_ERROR`, `CRC_ERROR` and `SEQ_GAP` all self-clear when a valid,
contiguous frame arrives, so fixing the sender clears the fault rather than
leaving it latched and misleading for the rest of the session. `C2_LINK_LOST`
clears the same way, and also clears when the value behind it goes stale (§3.5)
rather than asserting a link state the controller can no longer vouch for.

**`ESTOP_ACTIVE` cannot report the E-stop the rules require, and must not be
read as doing so.** Rule 2.a.v mandates a red push-button E-stop that *"shall
immediately stop the rover's movement and cease all power draw from any and all
batteries... All systems on the rover must have the power switched off by this
single E-stop."* When that button is pressed the Teensy loses power mid-cycle
and transmits nothing — there is no frame in which to set a bit. So the
mandated E-stop is visible to the Jetson only as telemetry stopping dead, and
this bit can only ever describe some *softer* stop: a commanded software stop,
or a secondary contactor that cuts the motors while leaving the logic powered.
Neither exists yet.

The same reasoning applies to the e-stop frame the `0x000`–`0x0FF` block is
reserved for (§2.1): any CAN frame is necessarily a **soft** stop, because the
hard one takes the bus down with everything else. That is not an argument
against the frame — a fast soft stop that leaves the rover diagnosable is worth
having — but it is not the rules requirement, and nothing here substitutes for
the button.

### 3.5 `TELEM_STATE` — `0x203`, DLC 8, 20 Hz

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0 | `mode` | uint8 | enum | 0–2 | **Echo** of the mode the controller believes it is in. Without it, a dropped or misread mode change is invisible from outside the rover. |
| 1 | `jetson_link` | uint8 | enum | 0–3 | Health of the **Jetson ↔ controller** link. |
| 2 | `c2_link` | uint8 | enum | 0–3 | Health of the **base-station C2** link, as *forwarded by the Jetson* (§3.1.3). Never derived from `jetson_link`. |
| 3 | `controller_health` | uint8 | enum | 0–3 | Aggregate low-level motor-driver / ESC health. |
| 4 | `indicator_state` | uint8 | enum | 0–4 | What the status indicator is **actually displaying**. |
| 5 | *reserved* | uint8 | — | transmit as 0 | |

**Link health enum** (`jetson_link`, `c2_link`):

| Value | Name | Meaning |
|---|---|---|
| 0 | `LINK_OK` | Healthy |
| 1 | `LINK_DEGRADED` | Intermittent: losses, gaps or CRC errors seen within the last 1000 ms |
| 2 | `LINK_LOST` | Down |
| 3 | `LINK_NOT_REPORTED` | **We do not know** — nobody has told us, or what we were told has gone stale |

**The two link fields are independent and must never be collapsed.** The 2027
autonomy course deliberately includes areas with no C2 line-of-sight while
onboard autonomy keeps working normally — rule 1.e.xvi places a scored target
*"behind the hill out of radio communications line of sight with the C2
station."* Treating C2 loss as equivalent to Jetson loss would stop the rover
exactly where the rules require it to keep going, and forfeit the 25 points for
that target.

`jetson_link` the controller measures for itself, from the command watchdog and
from sequence gaps or CRC errors seen within the last `LINK_DEGRADED_HOLD_MS`.

`c2_link` it cannot measure at all — it has no radio — so the field is only
ever the `c2_lost` bit the Jetson forwarded, mapped by `c2LinkState()`:

| Condition | `c2_link` | `FAULT_C2_LINK_LOST` |
|---|---|---|
| No valid `CONTROL` frame has ever arrived | `LINK_NOT_REPORTED` | clear |
| `JETSON_HEARTBEAT_LOST` — the forwarded value is **stale** | `LINK_NOT_REPORTED` | clear |
| Last frame said `c2_lost = 1` | `LINK_LOST` | **set** |
| Last frame said `c2_lost = 0` | `LINK_OK` | clear |

**The staleness rule lives in the sender, and that is the change from v0.3.1.**
The controller only knows C2 from the last `CONTROL` frame, so once the Jetson
goes silent that value is worthless: a clear bit from ten seconds ago does not
mean C2 is fine now, and a set bit does not mean it is still down. v0.3.1 put
that rule on the *receiver* — `describe_faults()` hid the stale bit and
substituted a `C2_UNKNOWN` label — which works only as long as every receiver
remembers to do it. A dashboard, a log parser or a second consumer that read
`fault_status` directly saw a stale claim as a live one. Now the controller
simply never transmits a value it cannot vouch for, so there is nothing for a
receiver to remember and nothing to get wrong. `LINK_NOT_REPORTED` is the
on-the-wire spelling of what `C2_UNKNOWN` used to mean.

#### 3.5.2 Placeholder: when the MCU gets its own radio

A radio on the controller is planned. **It changes no byte of either packet.**
`c2_link` is already a four-value enum in a byte of its own, and `jetson_link`
right beside it is already a *locally measured* link using those same four
values — so a `c2_link` that the controller measures for itself fits the field
exactly as a forwarded one does. `RoverController::c2LinkState()` is the single
swap point, and the table above is its current body. Nothing above it in §3.5
and nothing in §2.2 moves.

The Jetson-side seam already exists too: `host/c2_link.py` is written against
`is_connected()` / `recv()`, so a real radio drops in behind that interface
without touching `jetson_test.py`.

What the radio does change is three decisions nobody has made yet. They are
*policy*, not wire format, which is why this is a placeholder rather than a
field:

1. **Which observer decides the stop?** Two observers of one link disagree
   sooner or later. The controller's own measurement is both fresh and local,
   so it should win; the Jetson's forwarded `c2_lost` then becomes a
   cross-check rather than the source. Keep forwarding it — see (2).
2. **What does a disagreement mean?** It is information neither observer has
   alone. If the controller hears the base station and the Jetson does not, the
   *Jetson's* radio or software is broken, not the link. If the Jetson hears it
   and the controller does not, the controller's receiver is. Either is worth a
   fault bit from the reserved `0x0800`–`0x8000` range. **Do not pick the
   number until someone is building it** — v0.4 reserved `0x0080` for work that
   had not started, and reconciling that reservation is most of what v0.5 was.
3. **What is the debounce?** This is the one that will bite. Today the stop
   decision rests on a bit the Jetson has *already smoothed* — `C2Monitor`
   waits a full `C2_timeout_s` (1.0 s) before declaring loss. A raw local
   measurement polled at 20 Hz has no such smoothing, so marginal RF would stop
   and release a `TELEOP` rover over and over. Whatever feeds `c2LinkState()`
   locally needs its own hold-off; `LINK_DEGRADED_HOLD_MS` is the existing
   precedent in this file and 1.0 s is the number the Jetson already uses.

Note also that the **staleness rule's scope shrinks but does not vanish.** A
link the controller measures itself cannot go stale — it owns both the receiver
and the clock — so for that value `LINK_NOT_REPORTED` falls back to meaning
only "no reading yet", the boot case. But a forwarded bit kept as a cross-check
is still stale-able, and the rule above still governs it.

**First, establish which radio this is.** If it is a second path to the base
station, everything above applies. If it is a dedicated e-stop or RC-override
receiver, it is *not* `c2_link` at all: it is a third link needing its own
field, and almost certainly its own frame in the reserved `0x000`–`0x0FF`
block, which is empty for exactly this purpose (§2.1). The two readings lead to
different code, so the question is worth settling before anyone writes any.

**Controller health enum** (`controller_health`): `0=OK, 1=DEGRADED,
2=FAULT, 3=NOT_REPORTED`. **No longer a constant.** Since v0.6 it carries the
drive stage's own state (`DriveModule::health()`, §5.4), which is the layer
that actually knows:

| Value | When |
|---|---|
| `NOT_REPORTED` | the drive stage has never been updated — honestly "we do not know", not a misleading OK |
| `OK` | driving normally, or holding a stop at neutral |
| `DEGRADED` | the last mix **saturated** and was scaled down, so the rover is not delivering the commanded combination of speed and turn. Live, not latched, and a normal thing to see at full throttle in a tight turn |
| `FAULT` | **latched.** A command outside `CMD_MIN..CMD_MAX` reached the drive stage, which cannot happen while `decodeControl()` is doing its job — so something above it is broken |

This is still not real ESC telemetry. When motor drivers arrive on the bus with
their own messages, their aggregate health supersedes this and the field does
not change shape.

#### 3.5.1 The status indicator, and why request and state are two fields

**URC rule 1.e.ii** (2027): *"There must be an LED indicator on the back of the
rover, visible in bright daylight (e.g. LED array or high power LED), that will
signal: Red: Autonomous operation / Blue: Teleoperation (Manually driving) /
Flashing Green: Successful arrival at a target."*

The light is scored on what the rover is **actually doing**, which is why the
controller's own mode overrides the Jetson's request below. The enum is named
for the meanings rather than the colours, because the colour is a wiring
decision and the meaning is not:

| Value | Name | URC colour | Meaning |
|---|---|---|---|
| 0 | `INDICATOR_OFF` | off | Not operating. A legitimate idle state |
| 1 | `INDICATOR_TELEOP` | blue | Under operator control |
| 2 | `INDICATOR_AUTONOMOUS` | red | Driving itself |
| 3 | `INDICATOR_ARRIVED` | flashing green | Autonomous arrival at a target |
| 4 | `INDICATOR_FAULT` | **yellow** (placeholder) | Nothing is driving this rover |

**`INDICATOR_FAULT` is yellow, and the rules permit it.** 1.e.ii is a *minimum
signalling requirement* — it says what the indicator must signal, and attaches
no exclusivity: there is no "and no other colour" clause and no prohibition on
additional states. So a fourth indication is allowed. `INDICATOR_OFF` is the
same case. Yellow is the conventional "something is wrong" colour, cannot be
confused with any of the three mandated states, and on an RGB LED costs nothing
(red + green).

**Solid rather than flashing**, deliberately, and 1.e.xvii is why it matters:
arrival is scored on *"autonomously stopping within 1m of the target location
**and indicating its arrival**"*, so the flashing green is itself part of a
25-point criterion. A second flashing state would compete with a points-bearing
signal and leave the two distinguished by hue-while-blinking, which is the
hardest thing to read at distance or on competition video.

Two hardware constraints come from 1.e.ii directly and are easy to miss,
because neither is about colour:

- **On the back of the rover.** A placement requirement, not a suggestion.
- **Visible in bright daylight**, with *"LED array or high power LED"* given as
  the means. Every event runs in full daylight (3.d), so an indicator panel
  sized for an indoor bench test will not satisfy this.

Both are on the hardware item in §9, which is where the wiring work is tracked.

**The two fields differ on purpose.** The controller knows its own mode, so the
Jetson's request is honoured only where it agrees with that mode; otherwise the
controller overrides it. Checked top to bottom, first match wins
(`RoverController::indicatorState()`):

| # | Condition | Shown | Request |
|---|---|---|---|
| 1 | Command watchdog tripped (active mode `FAULT`) | `FAULT` | ignored |
| 2 | `TELEOP` with `c2_lost = 1` (active mode `FAULT`) | `FAULT` | ignored |
| 3 | Mode `SAFE` | `OFF` | ignored |
| 4 | Mode `TELEOP` | `TELEOP` | ignored — a human-driven rover never shows red or green |
| 5 | Mode `AUTONOMOUS` | `ARRIVED` if `ARRIVED` was requested **by the autonomy stack**, else `AUTONOMOUS` | only `ARRIVED`, and only from the source that owns the mode |

**Rule 5 gained the authority test in v0.6** (§3.1.5). "I have arrived" is an
assertion about what the *autonomy* did, and under rule 1.e.xvii it is worth 25
points, so a teleop-path frame claiming it during an autonomous run is the one
mix-up that could put flashing green on a rover that has arrived nowhere. Every
other indicator request is already decided by the mode alone, so the gate
changes nothing else.

Rules 1 and 2 are the two ways nothing can be driving the rover, and they show
`FAULT` rather than `OFF` because `OFF` reads as a legitimate idle state.

**A fault that does not stop the rover does not touch the light.** An
undervoltage warning, an over-current trip or one lost frame leaves an
autonomous rover showing red, which is what URC requires it to show; those
conditions are reported in `fault_status`, which is where an operator reads
them. v0.4 forced `FAULT` on *any* set bit, so a single `SEQ_GAP` mid-course
would have blanked a correctly-operating autonomous rover's light.

`stop` also does not change the light: a paused autonomous rover is still under
autonomous operation, so it stays red.

Comparing `indicator_request` against `indicator_state` tells an operator
immediately whether the controller honoured the request, and rule 4 means a
disagreement is normal rather than alarming.

### 3.6 `TELEM_CMD_ECHO` — `0x204`, DLC 8, 20 Hz

The acknowledgement. Everything in it is about the **command**, not the plant:
what the controller accepted, which frame it came from, and why it is or is not
being acted on.

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0–1 | `drive_applied` | int16 | 0.1 % of full drive effort | −1000 … +1000 | The drive handed to the drive stage this cycle. **0** under any stop or authority refusal. |
| 2–3 | `steer_applied` | int16 | 0.1 % of full steering range | −1000 … +1000 | The steering handed to the drive stage this cycle. |
| 4 | `ack_seq` | uint8 | frames | 0–255 wraps | The `seq` of the `CONTROL` frame those values came from. **Meaningless while `cmd_status` reads `NO_COMMAND`** — no frame has arrived to acknowledge, and 0 is an ordinary sequence number. |
| 5 | `status` | uint8 | bitfield | see below | Verdict, last decode result, return flag. |
| 6 | `seq` | uint8 | frames | 0–255 wraps | The telemetry cycle's own sequence number. See 2.2. |
| 7 | `crc8` | uint8 | — | 0–255 | See 4. |

**`status` byte (byte 5)**

| Bits | Field | Range | Meaning |
|---|---|---|---|
| 0–2 | `cmd_status` | 0–3 | What is being done with the last accepted command. |
| 3–5 | `last_decode` | 0–6 | `DecodeResult` for the **last frame seen on `0x100`**, or `DECODE_OK`. |
| 6 | `return_active` | 0/1 | The controller is holding a return request (§3.1.6). |
| 7 | *reserved* | **0** | The last reserved bit anywhere in this protocol. |

**`cmd_status`** — checked in this order, first match wins, so the value is
always the single most important reason the drive stage is seeing what it is:

| Value | Name | Meaning |
|---|---|---|
| 0 | `APPLIED` | the echoed drive and steer are what the drive stage was handed |
| 1 | `STOPPED` | a stop condition zeroed them: the command watchdog, the `stop` flag, a mode that does not permit motion, an autonomy abort, or C2 loss in `TELEOP`. *Which* one is in `fault_status` and the mode echo |
| 2 | `WRONG_SOURCE` | the frame was valid and the rover is not stopped, but its producer does not own the current mode, so its motion was refused (§3.1.5) |
| 3 | `NO_COMMAND` | no valid `CONTROL` frame has ever arrived |

**`last_decode` is a separate field from the verdict, on purpose.** A frame can
be rejected — a bad CRC, say — while an older accepted command is still being
applied; those are two different facts and collapsing them would lose one.
`fault_status` already says *that* something was rejected; this says *what*,
which is what an operator debugging a link at a competition actually needs.

**Why this is an acknowledgement and not a status.** `ack_seq` names the exact
frame the controller acted on, so the Jetson can match it against the `seq` it
sent and distinguish "my command got through" from "a command got through".
`host/autonomy_sim.py` is built on precisely that: every mission step ignores
any snapshot whose `ack_seq` predates the step's first frame, so a step can
never pass on telemetry caused by the previous one — the classic way an
integration test goes green while proving nothing.

**Why the accepted command rather than the drive stage's output.** They are the
same number today, because the drive stage applies no rate limiting (§9). If a
slew limiter is ever added, this field must keep reporting what was *accepted*:
the question it answers is whether the command survived validation, mode and
authority, and a ramp would blur that with how fast the drivetrain can respond.
The ramp would belong in `TELEM_DRIVE_L`'s feedback, beside `steer_fb`.

## 4. Integrity: two CRCs, covering different things

| Concern | Provided by |
|---|---|
| Message boundaries | CAN hardware |
| Wire corruption between the two CAN controllers | CAN hardware, 15-bit CRC |
| Corruption recovery | CAN hardware, automatic retransmit in µs |
| Priority / arbitration | CAN hardware, non-destructive |
| **Corruption in the software path** | **our CRC-8** |
| **Loss, duplication, reordering** | **our sequence number** |
| **Wrong-message-on-wrong-id** | **our CRC-8, via id seeding** |
| **Protocol version mismatch** | **our DLC, mode, flag and indicator validation** |

**Why an application-layer CRC when CAN already has one.** CAN's CRC is
computed by the transmitting controller and checked by the receiving
controller. It protects the **wire** between those two chips:

```
Jetson software → driver/DMA → [ CAN CRC covers this ] → driver/DMA → MCU software
   ^^^^^^^^^^^^^^^^^^^^^^^^^                              ^^^^^^^^^^^^^^^^^^^^^^^^
   covered only by our CRC-8                              covered only by our CRC-8
```

A bug assembling the struct, DMA corruption, or a driver copying into the wrong
buffer all produce a frame CAN considers perfectly valid and delivers with
wrong contents. Our CRC is computed by our code over our struct and checked by
our code on the far side, so it covers the whole path. This is the same
reasoning behind AUTOSAR's E2E protection, which also layers a CRC on top of
CAN's.

**Do not remove it as redundant.** It is not redundant; it covers a different
failure domain.

### 4.1 The algorithm

**CRC-8 / SAE-J1850** — what AUTOSAR E2E profiles 1 and 2 use.

| Parameter | Value |
|---|---|
| Polynomial | `0x1D` |
| Init | `0xFF` |
| Reflect in / out | No / No |
| Final XOR | `0xFF` |
| **Check value** (`"123456789"`) | **`0x4B`** |

Computed over **the CAN identifier (4 bytes, low byte first) followed by frame
bytes 0–6**, excluding the CRC byte itself.

**The id is in the seed on purpose** — AUTOSAR calls this a Data ID. Without
it, a correctly-CRC'd payload delivered on the wrong id would validate: a
telemetry frame mistakenly transmitted with `CONTROL`'s id would be accepted as
a command. With it, that fails.

**Porting note:** for a CRC-**8** the data byte is XORed into the whole
register (`crc ^= byte`); for a CRC-16 it goes into the high byte
(`crc ^= byte << 8`). Copying a CRC-16 loop and changing the width is the
classic way to produce a plausible-looking wrong answer. The check value is
pinned in `tests/cpp/test_protocol.cpp` and cross-checked against Python,
because this repo has already shipped a CRC documented as one variant and
implemented as another.

### 4.2 What a rejected frame does

A frame failing **any** validation — DLC, CRC, undefined `mode`, the reserved
flag bit set, an out-of-range `indicator_request`, or a `drive_cmd` or
`steer_cmd` outside ±1000 — is discarded and **does
not refresh the command watchdog**. Otherwise a node
on a mismatched protocol version could keep the rover alive while sending
commands it never understood. The matching fault bit is raised so the
resulting stop is diagnosable rather than silent.

## 5. Safety

### 5.1 The fail-safe rule

Five triggers — **comm timeout**, **explicit `stop`**, **any mode that does not
positively permit motion**, **`autonomy_abort` while `AUTONOMOUS`**, and
**`c2_lost` while `TELEOP`** — all force a stop through a single function,
`RoverController::effectiveStop()`. There is exactly one place in the codebase
where "should this rover be moving?" is answered. `commandedOutputs()` then
zeroes drive and steer *before* they reach the motor layer, so a future edit to
`setMotorOutputs()` cannot accidentally act on a stale command.

The first and fifth are deliberately different. A silent Jetson stops the rover
in **every** mode, because nothing is driving it. C2 loss stops it only in
`TELEOP`, because an autonomous Jetson drives perfectly well without the base
station — and the autonomy course is laid out so that it has to (§3.1.3).

The mode test is a **whitelist**. An earlier version asked `mode ==
MODE_SAFE` and stopped only then, so every undefined mode value read as
"not disabled" and permitted full throttle — a safety predicate that failed
*open*.

The Jetson separately considers the *link* down after 500 ms with no telemetry.
That is informational only; the rover's safety never depends on the Jetson
noticing anything.

**v0.6 added a second, independent reason for the outputs to be zero**, and it
is deliberately *not* folded into `effectiveStop()`: a command whose producer
does not own the current mode is refused by `commandAuthorized()` (§3.1.5).
The two are different facts and are reported differently — a stop means the
rover must not move, a refusal means *this command* is not the rover's to obey
— but both end in `commandedOutputs()`, so there is still exactly one place
where a command becomes an output.

One consequence is worth stating rather than discovering. A rover receiving
only unauthorised commands is **not stopped**: it sits in a motion-permitting
mode with the drive stage energized at neutral, ready to move the instant an
authorised command arrives, with `FAULT_CMD_SOURCE` and
`CMD_STATUS_WRONG_SOURCE` reported at 20 Hz. It is not moving, so it is not a
hazard, and treating it as a stop would need a timer and more state for no
behavioural gain. It would also make a mux bug look like a safety event, which
is the fault-diagnosis mistake this protocol has already fixed once.

### 5.2 Surviving a firmware fault

1. **Hardware watchdog** (`WDT_T4`, 1 s), fed once per loop. A hang resets the
   Teensy, driving outputs to a safe state.
2. **Boot-fault reporting.** The reset is otherwise invisible to the Jetson
   except as a telemetry gap — indistinguishable from a flaky bus. The next
   boot reads `SRC_SRSR` and, if the last reset was the watchdog, raises
   `FIRMWARE_FAULT` for that whole session and says so in the startup banner.
3. **Bounded counters.** Encoder accumulators pass through `wrapI32()`. In
   Python an unwrapped counter raised inside `struct.pack`; in C++ signed
   overflow is **undefined behaviour**, which is worse. CI runs the suite under
   UBSan to prove it.

> **Honest difference from the CircuitPython original:** there is no try/except
> guard, because Arduino builds run without exceptions. The hardware watchdog
> is the entire story for an unexpected fault, which is why arming it is not
> optional.

### 5.3 `millis()` wraps

Arduino's `millis()` is `uint32_t` and wraps every ~49.7 days. Unsigned
subtraction handles that correctly: `(now - then)` is computed modulo 2³².
Comparing timestamps directly (`now >= then + timeout`) does **not** — it
breaks the moment the counter wraps past the deadline. **Every time comparison
in `rover_controller.cpp` is written as an elapsed-time subtraction.** Sequence
numbers follow the same rule: `seqDelta()` subtracts, so `255 → 0` is a delta
of 1 rather than a 255-frame loss.

`test_controller.cpp` crosses both wrap boundaries explicitly, and the
`millis()` test was verified to *fail* against a naive timestamp-comparison
implementation. It is a real regression test, not decoration.

### 5.4 The drive stage

`firmware/src/rover_drive.*` is the last layer before a pin, and it is
deliberately **dumb about policy**. It knows nothing about modes, watchdogs,
faults, stop flags or who sent a command: every one of those decisions has
already been made by `RoverController`, which hands it a drive, a steer, and
one boolean saying whether the drives may be energized at all. Two layers
deciding when to stop is how a rover ends up with two answers.

What it does do:

1. **Refuses to energize.** `energized == false` puts every channel at neutral
   before anything else is considered, so no ordering mistake and no later edit
   can let a stale command through a stop. The caller has already zeroed the
   commands; this is the second, independent reason.
2. **Mixes.** `left = drive + steer`, `right = drive - steer`, scaled down
   **proportionally** if that saturates. Clamping each channel instead would
   change the *ratio* between them, which is the turn: a rover asked for a hard
   turn at full throttle would straighten out exactly when it was working
   hardest. Scaling gives up speed and keeps the turn, which is the right thing
   to lose.
3. **Converts to servo/ESC pulse widths** — 1500 µs neutral, ±500 µs full
   scale — which is the one output language nearly all small-rover drive
   hardware understands. Three channels are emitted: `left`/`right` for a
   skid-steer drivetrain and `steer` for a steering actuator. Which gets wired
   is a hardware decision, so the module emits both rather than guessing. **Do
   not wire both**, or the differential mix and the steering actuator each
   apply the same turn.
4. **Reports its own health**, which is what finally gives
   `TELEM_STATE.controller_health` a real value (§3.5).

Like `rover_protocol.*` and `rover_controller.*` it is pure C++ with no
Arduino dependency, so `tests/cpp/test_drive.cpp` exercises it natively and
`tools/rover_sim` links the same source the Teensy compiles.

## 6. Configuration constants

Every value in this section lives in **`firmware/src/rover_config.h`**,
mirrored by **`host/rover_config.py`** for the ones the Jetson needs.
`tests/host/test_config_sync.py` fails CI if the two ever disagree, so there is
one place to change a timeout rather than five.

What belongs there: values someone might tune, or that both ends must agree on.
What does not: values that describe a message's *contents* — fault bits, flag
masks, indicator and link codes, the command-age sentinels, the CRC polynomial.
Changing one of those changes the protocol rather than tuning it, so they stay
in `rover_protocol.h` beside the structs they describe.

### 6.1 Timing

| Constant | Value | Notes |
|---|---|---|
| `CONTROL` rate | 20 Hz | 1 frame per cycle (`CONTROL_RATE_HZ`, Jetson side) |
| `TELEMETRY` rate | 20 Hz | 5 frames per cycle |
| `WATCHDOG_TIMEOUT_MS` | 300 | 6× the control period. `DEFAULT_WATCHDOG_TIMEOUT_MS` in C++ |
| `TELEMETRY_PERIOD_MS` | 50 | `DEFAULT_TELEMETRY_PERIOD_MS` in C++ |
| `LINK_DEGRADED_HOLD_MS` | 1000 | How long a gap or CRC error keeps the link `DEGRADED` |
| `HW_WATCHDOG_MS` | 1000 | Teensy reset if the loop stalls (C++ only) |
| `LINK_TIMEOUT_S` (Jetson) | 0.5 | Informational only (Python only) |
| `C2_TIMEOUT_S` (Jetson, `C2Monitor`) | 1.0 | No base-station heartbeat for this long sets `c2_lost`. Longer than the 300 ms CAN watchdog because a radio drops packets far more often than a CAN bus. Tunable, and the base-station heartbeat rate is not yet defined (Python only) |

All placeholders, chosen to sit comfortably above one period plus margin. None
are derived from measured actuator response yet. Two of the relationships
between them are load-bearing rather than incidental, and are therefore
asserted in `test_config_sync.py`: the watchdog must tolerate a missed frame,
and `C2_TIMEOUT_S` must stay longer than the command watchdog — if those two
converged, a single radio hiccup would start stopping the rover in `TELEOP`.

### 6.2 Everything else

| Constant | Value | Notes |
|---|---|---|
| `PROTOCOL_VERSION` | 6 | Bump when a message's layout or meaning changes. Not sent on the wire; DLC and the CRC are the runtime mismatch signals, plus *which ids arrive* (§2.2) |
| `CAN_BITRATE_HZ` | 500000 | Every node on the bus must match |
| `CAN_ID_*` | `0x100`, `0x200`–`0x204` | §2.1 |
| `FRAME_DLC`, `FRAME_PAYLOAD` | 8, 6 | Plus `FRAME_SEQ_OFFSET` 6 and `FRAME_CRC_OFFSET` 7 |
| `MODE_*`, `MODE_MAX` | 0, 1, 2 | §3.1.1 |
| `CMD_MIN`, `CMD_MAX` | −1000, 1000 | Valid `drive_cmd`/`steer_cmd` range (§3.1.4) |
| `OVER_CURRENT_CA` | 4000 | 40.00 A, placeholder (C++ only) |
| `UNDERVOLTAGE_CV` | 2000 | 20.00 V, placeholder (C++ only) |
| `PWM_NEUTRAL_US` | 1500 | Servo/ESC neutral pulse width (§5.4) |
| `PWM_SPAN_US` | 500 | Full scale either side of neutral, so `CMD_MIN..CMD_MAX` maps onto 1000–2000 µs |

## 7. Demonstration and tests — no hardware required

```
make test      # 126 C++ tests + 98 host tests
make demo      # the Task 3 message-exchange demonstration
make autonomy  # the Task 4 mock-autonomy mission
```

`make autonomy` runs `host/autonomy_sim.py`: a fake autonomy stack driving the
real compiled controller through the real protocol — the same codec, the same
CAN ids, the same frames as `jetson_test.py`, with no side channel and no
shortcut. It runs the Task 4 command sequence and then the mode-separation
demonstration, verifying each step against telemetry before moving on:

```
PASS 1. start in SAFE                     PASS 8.  TELEOP refuses an autonomy command
PASS 2. receive AUTONOMOUS                PASS 9.  AUTONOMOUS accepts the same command
PASS 3. command forward movement          PASS 10. AUTONOMOUS refuses a teleop command
PASS 4. change steering                   PASS 11. an arrival claim needs authority too
PASS 5. stop                              PASS 12. stop overrides an authorised command
PASS 6. report TARGET_REACHED             PASS 13. abort overrides an authorised command
PASS 7. request autonomous return         PASS 14. teleop drives again afterwards

feedback loop: every required field reached the Jetson
  mode=TELEOP  applied=(+400,+0)  health=OK  age=38ms  faults=none
```

Steps 8 and 9 send an **identical** drive command and differ only in the mode;
step 14 is there because a safety gate that cannot be left is a rover that has
to be power-cycled at a competition.

A passing harness that cannot fail is not evidence, so
`tests/host/test_autonomy_mission.py` runs the **negative controls** too: a
step with an impossible expectation must fail and stop the run, a dead
controller must fail rather than hang, and no two steps may pass on the same
acknowledged frame — which is the check that a step is reading telemetry it
caused rather than the previous step's.

`make demo` runs a live exchange against `tools/rover_sim`, then exercises a
link cut, the stop flag, an autonomy abort, **C2 loss in each of the two modes
that differ**, a corrupted frame and a sequence gap. Captured output, trimmed
to one line per phase:

```
-- D2: C2 lost while AUTONOMOUS -- keeps driving, fault raised --
  t= 3.99s  seq= 80  enc=  +54580   +12.00A  23.80V  mode=AUTONOMOUS jet=OK   c2=LOST         ind=AUTONOMOUS age=20ms  faults=C2_LINK_LOST
-- D3: C2 lost while TELEOP -- stops, nobody can reach it --
  t= 4.50s  seq= 91  enc=  +63380    +0.00A  24.00V  mode=TELEOP     jet=OK   c2=LOST         ind=FAULT      age=20ms  faults=C2_LINK_LOST
-- E: corrupted payload (app-layer CRC must reject it) --
  t= 5.19s  seq=104  enc=  +63380    +0.00A  24.00V  mode=TELEOP     jet=LOST c2=NOT_REPORTED ind=FAULT      age=487ms faults=COMM_TIMEOUT,CRC_ERROR
-- F: sequence gap (frames 100 -> 140) --
  t= 5.38s  seq=108  enc=  +66420    +6.00A  23.90V  mode=TELEOP     jet=DEGRADED c2=OK       ind=TELEOP     age=98ms  faults=SEQ_GAP
```

Three things in that capture are the whole point of §3.1.3 and §3.5. In **D2**
an autonomous rover keeps drawing current with C2 lost, and the fault is
reported anyway. In **D3** the same loss in `TELEOP` zeroes the current. In
**E** the Jetson has gone silent, so `c2=NOT_REPORTED` and `C2_LINK_LOST` is
*absent* rather than latched: the controller stops asserting a C2 verdict it
can no longer vouch for.

Against real hardware the same host script runs unchanged:

```
python3 host/jetson_test.py --channel can0 --bitrate 500000
```

## 8. One implementation, pinned across two languages

The safety logic exists **once**, in `firmware/src/rover_controller.cpp`. The
simulator the host tests run against is *that same source compiled natively* —
not a Python mock. A hand-written mock would recreate the exact failure this
project already had: two copies of a safety rule, where the simulator passes
while the firmware misbehaves and the tests check the wrong copy.

The **codec** genuinely must exist twice, since the Teensy runs C++ and the
Jetson runs Python and neither can import the other. So it is pinned rather
than trusted. `tests/host/test_golden_vectors.py` compiles the C++ encoder,
runs it, and asserts Python produces identical output for:

- every encoded frame, byte for byte
- the CRC check value and the id-seeded frame CRC
- `seqDelta` across the wrap
- `isCommandableMode` and `modePermitsMotion` for **all 256** values
- `isKnownIndicator` for **all 256** values
- `c2_lost` and `from_autonomy` round-tripping in every mode, and bits 6 and 7
  decoding as fields rather than as reserved bits — the guard against a typo'd
  reserved mask quietly swallowing a live flag
- `sourceOwnsMode` for **both sources × all 256** mode values, so the command
  mux cannot mean one thing on the Jetson and another on the Teensy
- `packCmdStatus` for every verdict × decode result × return flag: two 3-bit
  fields and a flag sharing one byte is exactly the packing that goes wrong on
  one side only

Verified to fail on an injected endianness change.

## 9. Known placeholders

- **`drive_cmd`/`steer_cmd` scaling** is unitless ±1000; expect real units once
  motors and steering geometry are chosen.
- **Two encoders and one aggregate current reading.** A 6-wheel rover needs
  six, and one current figure cannot identify *which* wheel is stalling. Both
  change the payload when they grow — likely into per-corner frames, which CAN
  makes cheap.
- **`controller_health` now reports the drive stage, not the motor drivers.**
  v0.6 gave it `DriveModule::health()` (§3.5, §5.4), which is a real value from
  a real layer — but it is the health of a software output stage, not of an
  ESC. Real motor-driver telemetry arrives on a CAN bus as the drivers' own
  messages and supersedes it without changing the field.
- **The drive stage applies no rate limiting, deliberately.** A slew limiter
  would make the drive *applied* differ from the drive *accepted*, and
  `TELEM_CMD_ECHO` exists to report the latter (§3.6). Ramping belongs in the
  ESC or in a motor controller that knows the drivetrain's inertia; inventing a
  rate here would mean inventing a tuning constant with nothing behind it. If
  one is ever added, it goes below the echo, not above it.
- **The drive pins are placeholders and the PWM mode is unverified.**
  `PIN_DRIVE_L/R` and `PIN_STEER` in `rover_firmware.cpp` are not wired to
  anything and need checking against the Teensy 4.1 PWM-capable pin list.
  `analogWrite()` produces a continuous PWM waveform, which suits a brushed
  H-bridge or a PWM-input ESC; a hobby servo or a PPM-input ESC wants a 1–2 ms
  pulse in a 20 ms frame instead. **Establish which the drive hardware expects
  before connecting anything that moves.**
- **`INDICATOR_FAULT` is yellow** (§3.5.1). Checked against the 2027 rules:
  1.e.ii is a minimum signalling requirement with no exclusivity clause, so a
  fourth indication is permitted. This one is settled, not open.
- **Nothing drives the status light yet, and this is a scoring risk rather
  than a TODO.** The chain stops at telemetry: the protocol defines the
  states, `indicatorState()` decides which applies, `TELEM_STATE` reports it,
  and there is no pin assignment and no driver. The only LED in
  `rover_firmware.cpp` is `LED_PIN = 13`, the Teensy's onboard one, used as a
  telemetry heartbeat blink and unrelated to the indicator. So
  `indicator_state` is a value the Jetson can read and nobody can see.
  Rule 1.e.xvii scores a target as reached only on *"autonomously stopping
  within 1m of the target location **and indicating its arrival**"* — 25
  points per target — so the missing output costs points directly, not just
  compliance. It also has two constraints that are easy to miss because
  neither is about colour: the indicator must be **on the back of the rover**
  and **visible in bright daylight** (1.e.ii suggests an LED array or
  high-power LED; every event runs in full daylight). Its own hardware item,
  not part of the `setMotorOutputs()` stub.
- **`autonomy_abort` and `return_request` need a decision against the scoring
  rules, and the current behaviour is probably not it.** v0.4 flagged
  `autonomy_abort` forcing a stop as a judgement call the brief did not spell
  out. The 2027 rules do spell out the surrounding scoring, and reading 1.e.xi
  and 1.e.xviii together they define **three** operator actions, not two:

  | Operator action | Penalty | Who drives |
  |---|---|---|
  | Signal the rover to **stop** | 20% of that task | nobody |
  | Signal it to **autonomously return** (to the astronaut, or back the way it came) | 20% of that task | the rover, still autonomous |
  | **Teleoperate** back to a previously visited location | 50% of that task | the operator |

  Penalties for leaving autonomous mode cap at 50%, so once a team has taken
  the teleoperation hit on a task, further aborts are free.

  Two consequences worth a decision:

  1. **`return_request` doing nothing is the real gap.** The autonomous return
     is a 20% option; teleoperating back is 50%. A rover that cannot return
     under its own control leaves only the 50% route, so the field that
     currently has no behaviour is the one guarding 30 percentage points per
     task. **v0.6 narrowed this but did not close it** (§3.1.6): the request is
     now acknowledged in `TELEM_CMD_ECHO`, so the Jetson can prove it arrived,
     and the autonomy stack can execute the return with ordinary drive and
     steer commands. What is still undecided is whether the *controller* should
     do anything with it, and that remains this open question rather than a
     firmware decision.
  2. **`autonomy_abort` forcing a stop makes it close to a duplicate of
     `stop`**, differing only in being ignored outside `AUTONOMOUS`. If abort
     is meant to be the "stop autonomy but stay able to return" signal, then
     stopping the wheels is the wrong action and `return_request` should be
     what moves.

  Both are above the protocol: the fields, their bits and their meanings are
  fixed either way, and this is about what the controller and the autonomy
  stack *do* with them. Left as-is pending that call, because changing
  `effectiveStop()` changes safety behaviour.
- **Motor and sensor I/O is stubbed** behind `readSensors()` and
  `setMotorOutputs()` in `rover_firmware.cpp`.
- **`SRC_SRSR` watchdog bit mask is unverified on hardware.** It fails safe,
  but confirm it by deliberately hanging the loop once on the bench.
- **A controller-side radio is planned, and the packets are ready for it.**
  `c2_link` is a locally-measurable enum in its own byte, so the swap is
  `c2LinkState()` and nothing else — no wire change (§3.5.2). Three open
  policy questions go with it: which observer decides the stop, what a
  disagreement between the two observers means (and which reserved fault bit
  it gets, once someone is actually building it), and what the debounce is,
  since a raw local reading has none of the smoothing `C2Monitor` applies
  today. Settle first whether this radio is a second path to the base station
  or a dedicated e-stop receiver — the latter is a third link, not this field.
  **Note the reserved fault range has moved**: v0.6 took `0x0400` for
  `FAULT_CMD_SOURCE` under the same rule — someone was actually building the
  thing that needed it — so the free bits are now `0x0800`–`0x8000` and a
  C2-observer disagreement takes `0x0800`.
- **Not yet used:** CAN FD, the second and third CAN buses, and a dedicated
  high-priority e-stop frame in the reserved `0x000`–`0x0FF` block.
- **The authority gate compares two fields of one frame**, so a Jetson that
  sends a stale *mode* alongside a stale command is self-consistent and passes
  it (§3.1.5). The mode echo in `TELEM_STATE` is what catches that, one layer
  up. Nothing inside a single trusted sender can close the window entirely;
  this narrows it.
- **`TELEM_STATE.mode` still echoes the *commanded* mode.**
  [PR #18](https://github.com/schradivarius/URC-MCU-Jetson-Comm-Protocol/pull/18)
  (merged via #24) renamed the modes to `SAFE`/`TELEOP`/`AUTONOMOUS`, added a
  non-commandable `MODE_FAULT = 3` and introduced `activeMode()`. Echoing
  `activeMode()` instead would let the Jetson see `FAULT`, which is strictly
  more informative, but it changes what goes on the wire and is a team
  decision. `MODE_FAULT` has no owner under §3.1.5: `sourceOwnsMode()` falls
  through `modePermitsMotion()` to "no mismatch possible", which is pinned by
  `fault_has_no_owner`.

## 10. Repository layout and build

| Path | Runs on | Purpose |
|---|---|---|
| `firmware/rover_firmware.cpp` | Teensy 4.1 | Hardware wiring only |
| `firmware/src/rover_protocol.*` | **Both** | Message definitions, CRC, codec |
| `firmware/src/rover_controller.*` | **Both** | Command/safety state machine |
| `firmware/src/rover_drive.*` | **Both** | Drive/steering output stage (§5.4) |
| `host/rover_protocol.py` | Jetson | Python codec (pinned to the C++ one) |
| `host/protocol_root.py` | Jetson | Resolves which checkout `firmware/src` and `tools/` come from — the one place that decides |
| `host/can_link.py` | Jetson | Sim and python-can backends |
| `host/c2_link.py` | Jetson | C2 (base-station) link-loss detection, and a fake link for testing |
| `host/jetson_test.py` | Jetson | Live harness, snapshot reassembly |
| `host/demo.py` | Jetson | The Task 3 demonstration in section 7 |
| `host/autonomy_sim.py` | Jetson | Mock autonomy stack: the Task 4 mission (§7) |
| `tools/rover_sim.cpp` | dev machine | Simulator: real controller, fake plant |
| `tools/golden_vectors.cpp` | dev machine | Emits vectors for cross-language pinning |
| `tests/cpp/`, `tests/host/` | dev machine | 224 tests total |

### 10.1 Flashing the Teensy 4.1

1. PlatformIO, with `platformio.ini` from the repo root. It names the board
   (`teensy41`) and both libraries — **FlexCAN_T4** (`tonton81/FlexCAN_T4`) and
   **WDT_T4** (`tonton81/WDT_T4`) — so there is nothing to install by hand.
2. `pio run -t upload`. Sources under `src/` are compiled automatically.
3. `pio device monitor` at 115200 for the boot banner.

**Before powering anything:**

- Teensy 4.1 is **3.3 V and NOT 5 V tolerant.** A 5 V sensor wired straight to
  a pin destroys the board. Level-shift everything.
- Each CAN bus needs a **transceiver**; the MCU cannot drive a differential bus.
- The bus needs **120 Ω termination at both physical ends.** Exactly one
  resistor, or three, is the most common CAN bring-up failure.

### 10.2 Running the autonomy program from a separate repository

The autonomy program is a *consumer* of this protocol, not part of it, so it
**can** live in a repository of its own. It currently does not, and this section
describes an option rather than a plan: two files move and nothing else.

| Moves to the autonomy repository | Stays here |
|---|---|
| `host/autonomy_sim.py` | `host/rover_protocol.py`, `host/rover_config.py` |
| `tests/host/test_autonomy_mission.py` | `host/protocol_root.py`, `host/can_link.py`, `host/jetson_test.py` |
| | everything under `firmware/` and `tools/` |

Worth saying plainly before the mechanics: the ratio argues against doing this
yet. Task 4 added 5 files and modified 23 existing ones, because it was a wire
change (v0.5 → v0.6) as much as a new program. 23 of those 28 files are on the
protocol side of this seam, the two that move import four modules that stay, and
`ci.yml`'s autonomy step is the only place a real consumer exercises the
protocol end to end. The seam below is built and tested so that the move is
cheap *later* — when the autonomy stack is real perception code in its own
toolchain, or someone outside the team needs the protocol without it.

What must **not** move, and must not be copied either, is the Python codec. It
is a hand-written mirror of the C++ headers, and §8 keeps the two honest with
golden vectors emitted by the C++ and asserted by the Python **in one CI run**.
Split them across repositories and that check still passes on both sides while
the two drift apart — the single failure this whole design is built to prevent.

So the autonomy repository takes a checkout of *this* repository (a submodule,
or a CI checkout step) and points `ROVER_PROTOCOL_ROOT` at it:

```sh
ROVER_PROTOCOL_ROOT=../protocol python3 autonomy/autonomy_sim.py --mock
```

Everything that reads `firmware/src/` or `tools/` resolves the root through
`host/protocol_root.py`, which falls back to the enclosing repository when the
variable is unset, so a shared checkout needs nothing. The two files that have
to locate `host/` before they can import anything from it — `autonomy_sim.py`
and `test_autonomy_mission.py` — read the variable by name instead, and are the
only places that do. A root that does not contain `firmware/src` and
`tools/rover_sim.cpp` fails by name at `build_sim()`, before the staleness
check, so a binary left over from an earlier run cannot quietly stand in for
sources that are not there.

Two consequences worth knowing before relying on it:

- The simulator is compiled **into** the protocol checkout
  (`tests/cpp/build/`), so that checkout has to be writable. A read-only
  submodule cache is the one layout this does not serve.
- The autonomy repository pins a **commit** of this one, and moving that pin is
  how an autonomy change opts in to a new protocol version. Nothing on the wire
  reports the version (§6), so a pin nobody moves is caught the same way any
  other revision mismatch is: by *which ids arrive*. A v0.5 controller sends
  four telemetry frames, the Jetson's snapshot never completes, and
  `TelemetryAssembler.missing_ids()` names `0x204` (§2.2). The skew table in
  §3.1.2 covers the other direction. `PROTOCOL_VERSION` itself is compile-time
  only, pinned across the two languages by `test_config_sync.py`.
- That pinning is **only** honest if it reads the checkout being compiled.
  Every path into `firmware/src/` and `tools/` resolves through
  `host/protocol_root.py` for that reason: when only `can_link.py` honoured the
  variable, the integration suite drove the overridden firmware while the
  pinning suite checked the local pair, and the host suite passed over a
  `PROTOCOL_VERSION` mismatch.
  `test_the_pinning_follows_the_firmware_that_is_actually_compiled` pins it.

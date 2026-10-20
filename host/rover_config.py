"""
rover_config.py -- Every protocol and timing constant, in one place.

Mirrors firmware/src/rover_config.h. tests/host/test_config_sync.py fails if
the two ever disagree, so change both together.

Frame sizes are NOT repeated here: rover_protocol.py derives what it can from
its struct format strings, and the sync test checks those against the C++
values.

Jetson-only values (the send rate, the two host-side link timeouts) have no
C++ counterpart and so are not in the cross-language comparison. They are
here anyway, because "one place to change a timeout" is the point of the file.
"""

# --- protocol version: bump whenever a message's layout or meaning changes ---
# v0.5 is the first revision with the application-layer CRC and sequence
# number; v0.6 adds the command-source bit and TELEM_CMD_ECHO. Not carried on
# the wire.
PROTOCOL_VERSION = 6

# --- link ---
CAN_BITRATE_HZ = 500000       # 500 kbps; every node must match

# --- timing ---
WATCHDOG_TIMEOUT_MS = 300     # controller stops after this long without CONTROL
TELEMETRY_PERIOD_MS = 50      # 20 Hz
LINK_DEGRADED_HOLD_MS = 1000  # how long a gap or CRC error holds DEGRADED
CONTROL_RATE_HZ = 20          # Jetson-side send rate
LINK_TIMEOUT_S = 0.5          # Jetson-side, informational only
# No base-station heartbeat for this long sets c2_lost. Deliberately longer
# than WATCHDOG_TIMEOUT_MS: a radio drops packets far more often than a CAN
# bus, so the two must not share a threshold. Jetson-side only -- the rover's
# own safety never depends on it.
C2_TIMEOUT_S = 1.0

# --- CAN identifiers (also the bus priority: lowest id wins arbitration) ---
CAN_ID_CONTROL       = 0x100
CAN_ID_TELEM_DRIVE_L = 0x200
CAN_ID_TELEM_DRIVE_R = 0x201
CAN_ID_TELEM_POWER   = 0x202
CAN_ID_TELEM_STATE   = 0x203
CAN_ID_TELEM_CMD_ECHO = 0x204

# --- operating modes ---
# The Jetson commands SAFE, TELEOP or AUTONOMOUS; the controller enters FAULT
# on its own when a link the mode needs is gone.
MODE_SAFE       = 0
MODE_TELEOP     = 1
MODE_AUTONOMOUS = 2
MODE_FAULT      = 3
MODE_MAX        = MODE_AUTONOMOUS   # highest COMMANDABLE mode

# --- valid command range for drive_cmd / steer_cmd (tenths of a percent) ---
CMD_MIN = -1000
CMD_MAX = 1000

# --- drive / steering output stage ---
# Servo/ESC pulse widths: 1500 us neutral, +/-500 us full scale, which is what
# CMD_MIN..CMD_MAX maps onto. Mirrored here only so the Jetson side can check
# the numbers it reads back; nothing on the Jetson drives a pin.
PWM_NEUTRAL_US = 1500
PWM_SPAN_US    = 500

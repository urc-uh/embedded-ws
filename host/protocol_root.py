"""protocol_root.py -- where the firmware sources live, resolved in one place.

One function, in one file, because the alternative is what actually happened:
can_link.py honoured ROVER_PROTOCOL_ROOT while test_config_sync.py and
test_golden_vectors.py each derived the root from their own __file__. Point the
variable at a second checkout and the integration suite drove THAT firmware
while the pinning suite pinned the LOCAL C++/Python pair -- so the whole host
suite reported OK over a PROTOCOL_VERSION mismatch, which is the exact drift
section 8 of PROTOCOL.md exists to catch.

So: anything that reads firmware/src/ or tools/ at run time resolves the root
through here, and the pinning follows the firmware actually being compiled.

The one place that CANNOT use this module is code that has to find host/ before
it can import anything from it -- host/autonomy_sim.py and
tests/host/test_autonomy_mission.py bootstrap sys.path themselves, reading the
same variable by name.
"""

import os

#: The environment variable, named once so a grep for it finds every user.
ENV_VAR = "ROVER_PROTOCOL_ROOT"

# This file lives in host/, so the enclosing repository is one directory up.
DEFAULT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def protocol_root():
    """The root of the checkout holding firmware/src/ and tools/.

    Defaults to the enclosing repository, which is right whenever the Jetson
    code and the firmware share a checkout. ROVER_PROTOCOL_ROOT overrides it so
    the autonomy program can run from a repository of its own against a checkout
    of the protocol repository, instead of vendoring a copy of the firmware -- a
    copy being exactly how the C++ and the Python drift apart.
    """
    override = os.environ.get(ENV_VAR)
    if not override:
        return DEFAULT_ROOT
    return os.path.abspath(os.path.expanduser(override))


def firmware_src(*parts):
    """A path under the resolved checkout's firmware/src/."""
    return os.path.join(protocol_root(), "firmware", "src", *parts)


def tools(*parts):
    """A path under the resolved checkout's tools/."""
    return os.path.join(protocol_root(), "tools", *parts)


def build_dir():
    """Where natively-compiled binaries go.

    Inside the resolved checkout, so the binary sits with the sources it was
    built from and a stale one cannot be mistaken for a fresh build of a
    different tree. The cost is that the checkout must be writable; see
    PROTOCOL.md section 10.2.
    """
    return os.path.join(protocol_root(), "tests", "cpp", "build")


def is_overridden():
    """True when the root came from the environment rather than this file."""
    return bool(os.environ.get(ENV_VAR))

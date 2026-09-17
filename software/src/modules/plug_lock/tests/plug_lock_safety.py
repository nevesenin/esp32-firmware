#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["requests", "urllib3"]
# ///
#
# esp32-firmware
# Copyright (C) 2026 empunkt <empunkt@mailbox.org>
#
# Guided safety test for the plug lock, run by an operator standing at the wallbox
# with a Testboy TV 950 and four test switches fitted:
#
#   S1 RELAY OUT      in the Quad Relay channel 0 -> Mennekes OUT- leg
#   S2 LOCK FEEDBACK  in the Mennekes LOCK -> Dual Analog In channel 0 signal wire
#   S3 LOOP LINK      in the harness loop, Quad Relay channel 1 -> Dual Analog In
#                     channel 1
#   S4 LOCK SUPPLY    in the lock PSU's DC output, upstream of all three loads
#
# S1 and S2 break wiring that is normally continuous and come out again after the
# test. S3 is different: the harness loop it sits in IS production wiring, and it is
# what the firmware uses to tell a real plug lock from two unrelated Bricklets. S3
# makes a loop failure reproducible without cutting anything. The loop itself must
# stay.
#
# S3 is also the least consequential of the three: the loop carries only
# microamps into a high-impedance input and drives nothing, so opening it cannot
# leave an actuator
# energized the way cutting feedback with S2 can.
#
# NOTE, and it is the reason group 9 exists: S3 is NOT the removal procedure. Opening
# the loop faults the charger but does not allow the plug lock to be switched off,
# because a failed lock supply opens the loop just as effectively and must not hand a
# blown fuse a way past a guard that is meant to need someone at the enclosure. The
# documented removal is to unplug BOTH Bricklets, which group 10 exercises.
#
# S4 is the honest version of that supply failure, and S3 is not a substitute for it.
# The lock PSU feeds three things - the harness loop, the 12 V feedback into Digital
# In 4 channel 0, and the actuator itself. S3 kills only the first, so it reproduces a
# broken loop jumper with a healthy supply. S4 kills all three, which is what a blown
# fuse does. Group 9 uses S3 and group 11 uses S4; the disable gate must refuse in
# both, and for the gate the two are indistinguishable, but only S4 also drops the
# feedback and the motor.
#
# THE CONTROL BLOCK RELEASES THE PLUG WHEN ITS SUPPLY DISAPPEARS, and it does so under
# power. Observed 2026-09-16: opening S4 frees the plug at that instant, with the audible
# stroke of a driven actuator, while relay channel 0 is still commanded closed. Nothing
# commanded it. The module carries enough energy of its own to run one release stroke
# after its supply is gone, and spends it.
#
# So the actuator is neither of the two things this file has claimed. It is not the
# self-holding motor drive assumed until 2026-09-15, and it is not the spring-return
# assumed from then until 2026-09-16. It is driven in both directions, holds position
# while idle, and has a fail-safe release on supply loss.
#
# That is also why the socket has a mechanical emergency release: the fail-safe stroke
# covers a lost supply, and the mechanical release covers the case where even that stroke
# cannot run - drained storage, a jam, a failed driver. A genuine spring-return would not
# need one.
#
# Cutting S4 with a plug locked therefore frees the plug; it does not strand it. That is
# safe for a reason none of this changes: the contactor is blocked throughout (error
# state 6), so the socket is never live while the plug is loose.
#
# S4 is still SAFER than S2 for reaching the same firmware branch, and that argument is
# unaffected: S2 cuts feedback with a live motor, and LockFault then holds the relay
# energized into it indefinitely. S4 kills the motor and the feedback together, so the
# held relay drives nothing.
#
# Restore the supply BEFORE withdrawing anyway. Not because release needs the supply -
# the fail-safe stroke has already run - but because LockFault keeps relay channel 0
# commanded closed until lock_wanted goes false, so closing S4 re-locks the plug on the
# spot. Withdraw after that, holding Unlock, which drops lock_wanted and releases it
# properly.
#
# The TV 950's Unlock button is purely the CP switch - nothing mechanical happens when
# you press it, because a Type 2 infrastructure-side plug has no latch; that is on the
# vehicle connector at the far end of a cable. Holding it puts CP at A. Two consequences
# run through every prompt in this file:
#
#   Insertion ALWAYS holds it. Otherwise the charger sees CP B the moment the contacts
#   touch, locks against a plug that is not seated, and physically blocks the rest of
#   the insertion. Seven prompts were missing this until 2026-09-14.
#
#   Any step testing retention must NOT hold it - see 4.2. Holding it tells the firmware
#   the vehicle is gone, so the lock is released before force can reach the plug.
#
# The script drives and observes everything reachable over the network and prompts for
# the physical steps and visual observations it cannot perform. It samples continuously
# in the background, so assertions are made against the recorded timeline rather than
# against whatever happens to be true when the operator presses Enter.
#
# --host carries the scheme, because which one it is decides whether the run works at
# all: a charger with "HTTP disabled" answers plain http with 403 on every endpoint,
# and its TLS certificate is self-signed for its own hostname, so an https URL aimed
# at an IP address needs --insecure. Guessing either would fail deep inside step 0.1
# rather than at the argument.
#
# Usage:
#   ./plug_lock_safety.py --host https://warp4-abcd
#   ./plug_lock_safety.py --host https://192.168.101.238 --insecure
#   ./plug_lock_safety.py --host https://warp4-abcd --from-step 5.1
#   ./plug_lock_safety.py --list

import argparse
import json
import os
import signal
import sys
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable, Optional
from urllib.parse import urlsplit

import requests
import urllib3
from requests.auth import HTTPDigestAuth

# The regenerated bindings live in the sibling evse-v2-bricklet checkout; they are the
# only copy carrying the plug lock functions (FIDs 78-81).
_HERE = os.path.dirname(os.path.abspath(__file__))
_BINDINGS = os.path.normpath(os.path.join(_HERE, '../../../../../../evse-v2-bricklet/tests'))

if not os.path.isdir(os.path.join(_BINDINGS, 'tinkerforge')):
    sys.exit(f'Tinkerforge bindings not found at {_BINDINGS}/tinkerforge')

sys.path.insert(0, _BINDINGS)

from tinkerforge.ip_connection import IPConnection                              # noqa: E402
from tinkerforge.bricklet_evse_v2 import BrickletEVSEV2                         # noqa: E402
from tinkerforge.bricklet_industrial_quad_relay_v2 import BrickletIndustrialQuadRelayV2      # noqa: E402
from tinkerforge.bricklet_industrial_dual_analog_in_v2 import BrickletIndustrialDualAnalogInV2  # noqa: E402

# ---------------------------------------------------------------------------------
# Constants mirrored from the firmware. Kept in sync by hand.
# ---------------------------------------------------------------------------------

# esp32-firmware software/src/modules/plug_lock/plug_lock.cpp:40-50
UPDATE_INTERVAL_S = 0.250
ATTEMPT_TIMEOUT_S = 3.0
UNLOCK_TIMEOUT_S = 3.0
ATTEMPTS = 2
FEEDBACK_HIGH_READS = 2

# A failed lock is ATTEMPTS x ATTEMPT_TIMEOUT plus scheduling slack.
LOCK_FAULT_BUDGET_S = ATTEMPTS * ATTEMPT_TIMEOUT_S + 2.0

RELAY_CHANNEL = 0
INPUT_CHANNEL = 0

# The harness loop: relay channel 1 switches the lock supply onto input channel 1,
# whose other terminal sits on PSU-. This, not Bricklet discovery, is what the ESP32
# reports to the EVSE as "bricklet dedication verified", and it is what arms the
# feature in both directions.
RELAY_LOOP_CHANNEL = 1
INPUT_LOOP_CHANNEL = 1

# Millivolts. These MUST match modules/plug_lock_input/plug_lock_input.h - the script
# derives the same low/high/implausible verdict the firmware does, so that a timeline
# row shows what the firmware saw and not a second opinion. If the two ever disagree,
# the header wins and this is the copy that is stale.
INPUT_LOW_MIN = -1000
INPUT_FEEDBACK_LOW_MAX = 4600
INPUT_FEEDBACK_HIGH_MIN = 5000
INPUT_FEEDBACK_HIGH_MAX = 9500
INPUT_LOOP_LOW_MAX = 4600
INPUT_LOOP_HIGH_MIN = 8500
INPUT_LOOP_HIGH_MAX = 13500


def classify_input(mv, low_max, high_min, high_max):
    """Return (level, plausible), matching PlugLockInput::get_reading()."""
    if mv is None:
        return None, None
    if INPUT_LOW_MIN <= mv <= low_max:
        return False, True
    if high_min <= mv <= high_max:
        return True, True
    return False, False

# Each phase is bounded by PLUG_LOCK_LOOP_CHALLENGE_MAX_READS, and a challenge is two
# phases, so the worst case is twice that at one read per update interval, plus
# scheduling slack. The release phase used to be a fixed three reads; it now waits for
# the level it needs, which is what a single blended sample across the relay transition
# used to break.
LOOP_CHALLENGE_BUDGET_S = 12 * UPDATE_INTERVAL_S + 2.0

# Losing the loop takes the 4-read low debounce, deliberately slower than the feedback
# contact: a false low would cost a session plus the EVSE's error cooldown.
LOOP_LOSS_BUDGET_S = 4 * UPDATE_INTERVAL_S + 2.0

# Recovery is the high debounce plus a full challenge, which the firmware runs as soon
# as the level returns rather than waiting for the next interval.
LOOP_RECOVERY_BUDGET_S = LOOP_LOSS_BUDGET_S + LOOP_CHALLENGE_BUDGET_S

# evse-v2-bricklet plug_lock.h, PLUG_LOCK_REPORT_STALE_MS. The ESP32 keeps reporting
# once it has a verdict, so a lost loop reaches the EVSE as an asserted false rather
# than by going stale - but allow for the stale path too, since it is the same outcome.
PLUG_LOCK_REPORT_STALE_S = 5.0

# The EVSE latches a plug lock fault into IEC61851_STATE_EF and only leaves it after a
# cooldown, so a step that clears a fault has to allow for it.
EVSE_ERROR_COOLDOWN_S = 30.0

# plug_lock/Plug Lock Loop.uint8.enum
LOOP_UNKNOWN, LOOP_CHALLENGING, LOOP_VERIFIED = 0, 1, 2
LOOP_FAIL_OPEN, LOOP_FAIL_STUCK = 3, 4
LOOP_NAME = {
    LOOP_UNKNOWN: 'Unknown', LOOP_CHALLENGING: 'Challenging', LOOP_VERIFIED: 'Verified',
    LOOP_FAIL_OPEN: 'FailOpen', LOOP_FAIL_STUCK: 'FailStuck',
}

# generators/configs/bricklet_evse_v2_config.py, constant group 'Plug Lock State'
PLS_DISABLED, PLS_DEDICATION_NOT_VERIFIED, PLS_IDLE, PLS_WAITING = 0, 1, 2, 3
PLS_LOCKED, PLS_FAULT_TIMEOUT, PLS_FAULT_LOCK = 4, 5, 6
PLS_NAME = {
    PLS_DISABLED: 'DISABLED',
    PLS_DEDICATION_NOT_VERIFIED: 'BRICKLET_DEDICATION_NOT_VERIFIED',
    PLS_IDLE: 'IDLE', PLS_WAITING: 'WAITING', PLS_LOCKED: 'LOCKED',
    PLS_FAULT_TIMEOUT: 'FAULT_TIMEOUT', PLS_FAULT_LOCK: 'FAULT_LOCK',
}

# plug_lock/Plug Lock Actuation.uint8.enum
ACT_INACTIVE, ACT_UNLOCKED, ACT_LOCKING, ACT_LOCKED = 0, 1, 2, 3
ACT_UNLOCKING, ACT_LOCK_FAULT, ACT_UNLOCK_FAULT = 4, 5, 6
ACT_NAME = {
    ACT_INACTIVE: 'Inactive', ACT_UNLOCKED: 'Unlocked', ACT_LOCKING: 'Locking',
    ACT_LOCKED: 'Locked', ACT_UNLOCKING: 'Unlocking', ACT_LOCK_FAULT: 'LockFault',
    ACT_UNLOCK_FAULT: 'UnlockFault',
}

# evse_common: iec61851_state
CP_A, CP_B, CP_C, CP_D, CP_EF = 0, 1, 2, 3, 4
CP_NAME = {CP_A: 'A', CP_B: 'B', CP_C: 'C', CP_D: 'D', CP_EF: 'EF'}

ERROR_STATE_PLUG_LOCK = 6

# evse_common.h. Only needed to name whatever is holding the allowed current at zero:
# iec61851.c:579 keeps the EVSE in state B rather than C whenever
# charging_slot_get_max_current() is 0, so a charger that refuses to charge looks
# identical to a car that never asked. Naming the slot turns a 20 second timeout with no
# explanation into one line that says what to fix.
SLOT_NAMES = {
    0: 'incoming cable', 1: 'outgoing cable', 2: 'shutdown input', 3: 'GP input',
    4: 'autostart button', 5: 'global', 6: 'user (authorization)', 7: 'charge manager',
    8: 'external', 9: 'Modbus TCP', 10: 'Modbus TCP enable', 11: 'OCPP',
    12: 'charge limits', 13: 'require meter', 14: 'automation', 15: 'EEBus',
    16: 'P14a EnWG', 17: 'OVE R37',
}

DEV_EVSE = 2167
DEV_RELAY = 2102
DEV_INPUT = 2121

# Guard budgets (design section 2.3, G3). Exceeding one aborts into teardown.
MAX_ACTUATION_CYCLES = 20
MIN_ACTUATION_SPACING_S = 5.0
MIN_CP_CYCLE_SPACING_S = 10.0

SAMPLE_INTERVAL_S = 0.100

# SAMPLE_INTERVAL_S is a floor, not the period: one sample() is seven TFP round trips
# plus an HTTP GET, so rows actually land 1-2 s apart on a healthy charger and up to
# 3.2 s apart while one is still coming up after a reboot. This is how long settle()
# waits for a sample to land in the window the assertions are about to be made over.
SETTLE_SAMPLE_TIMEOUT_S = 6.0


class Abort(Exception):
    """Raised to unwind into teardown without running further steps."""


class StepFailed(Exception):
    pass


class GuardTripped(Abort):
    pass


# ---------------------------------------------------------------------------------
# Transport
# ---------------------------------------------------------------------------------

def split_host(raw: str) -> tuple[str, str]:
    """Split a --host URL into (base, hostname). Raises Abort on anything ambiguous.

    The two transports need different things out of it: the HTTP API needs the full
    origin including the scheme, the Tinkerforge proxy needs a bare hostname. Deriving
    both from one argument keeps them from ever disagreeing about which charger is
    under test.
    """
    parts = urlsplit(raw)

    if parts.scheme not in ('http', 'https'):
        raise Abort(f'--host must start with http:// or https:// (got {raw!r}). '
                    f'A charger with HTTP disabled answers plain http with 403, so '
                    f'the scheme cannot be guessed.')

    if not parts.hostname:
        raise Abort(f'--host has no hostname: {raw!r}')

    if parts.path.strip('/') or parts.query or parts.fragment:
        raise Abort(f'--host must be an origin only, no path: {raw!r}')

    # The Tinkerforge proxy port is --port, never the URL's. A URL port would be the
    # web interface's, and silently reusing it would aim the proxy at the web server.
    return f'{parts.scheme}://{parts.netloc}', parts.hostname


class Charger:
    """Both network surfaces: the Tinkerforge proxy on 4223 and the ESP32 HTTP API."""

    def __init__(self, host: str, port: int, user: Optional[str], password: Optional[str],
                 insecure: bool = False):
        self.base, self.host = split_host(host)
        self.port = port
        self.verify = not insecure
        self.auth = HTTPDigestAuth(user, password) if user else None
        self.ipcon = IPConnection()
        self.evse: Optional[BrickletEVSEV2] = None
        self.relay: Optional[BrickletIndustrialQuadRelayV2] = None
        self.input: Optional[BrickletIndustrialDualAnalogInV2] = None
        self.uids: dict[int, str] = {}
        self._lock = threading.Lock()

    # -- Tinkerforge -----------------------------------------------------------

    def enable_proxy(self, timeout: float = 20.0):
        """Start the Tinkerforge proxy, which is not running by default and does not
        survive a reboot.

        WARP4 builds the Hidden Proxy module, not Proxy: it has no persistent config
        and no frontend at all, just this endpoint (hidden_proxy.cpp:117-128). The net
        context lives in RAM, so every power cycle and every reboot drops it - which is
        why this sits in connect() rather than in main(), and therefore runs again on
        each reconnect().

        There is no LED to check on WARP4: pre_init() returns before assigning any LED
        pin when the co-bricklet module is present (esp32_ethernet_brick.cpp:151-158),
        so green_led_pin stays -1 and hidden_proxy's blinky task is never created.
        A successful call here is the only confirmation available.

        Transport errors are retried until `timeout`, because after a power cycle the
        web server comes up later than this call is made. A wrong scheme, a bad
        certificate or missing credentials are not retried - they cannot heal, and
        waiting out the timeout would only bury the reason.
        """
        deadline = time.monotonic() + timeout
        last_error: Optional[Exception] = None

        while True:
            try:
                r = requests.get(f'{self.base}/hidden_proxy/enable', auth=self.auth,
                                 verify=self.verify, timeout=5.0)
            except requests.exceptions.SSLError as e:
                raise Abort(f'TLS verification failed for {self.base}: {e}. The '
                            f'charger certificate is self-signed for its own '
                            f'hostname, so an IP address will never match it - pass '
                            f'--insecure, or use the hostname from the certificate.')
            except Exception as e:                                  # noqa: BLE001
                last_error = e

                if time.monotonic() >= deadline:
                    raise Abort(f'could not reach {self.base}: {last_error}')

                time.sleep(1.0)
                continue

            if r.status_code == 403:
                raise Abort(f'{self.base}/hidden_proxy/enable returned 403. This '
                            f'charger refuses plain HTTP - use an https:// --host.')

            if r.status_code == 401:
                raise Abort(f'{self.base}/hidden_proxy/enable returned 401. Pass '
                            f'--user and --password.')

            if not r.ok:
                raise Abort(f'{self.base}/hidden_proxy/enable returned {r.status_code}')

            return

    def connect(self, timeout: float = 20.0):
        """Connect and discover the three devices by device identifier.

        Discovery is by identifier rather than by hardcoded UID, following
        evse-v2-bricklet/tests/evse_v3_tester.py:57.
        """
        self.enable_proxy(timeout=timeout)

        deadline = time.monotonic() + timeout
        last_error: Optional[Exception] = None

        while time.monotonic() < deadline:
            try:
                self.ipcon.connect(self.host, self.port)
                break
            except Exception as e:                                  # noqa: BLE001
                last_error = e
                time.sleep(1.0)
        else:
            raise Abort(f'could not connect to {self.host}:{self.port}: {last_error}')

        found: dict[int, str] = {}
        done = threading.Event()

        def on_enumerate(uid, _connected_uid, _position, _hw, _fw, device_identifier, enum_type):
            if enum_type == IPConnection.ENUMERATION_TYPE_DISCONNECTED:
                return
            found[device_identifier] = uid
            if DEV_EVSE in found and DEV_RELAY in found and DEV_INPUT in found:
                done.set()

        self.ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, on_enumerate)
        self.ipcon.enumerate()
        done.wait(5.0)

        self.uids = dict(found)

        if DEV_EVSE not in found:
            raise Abort('no EVSE bricklet (2167) in the enumerate response')

        self.evse = BrickletEVSEV2(found[DEV_EVSE], self.ipcon)
        self.relay = (BrickletIndustrialQuadRelayV2(found[DEV_RELAY], self.ipcon)
                      if DEV_RELAY in found else None)
        self.input = (BrickletIndustrialDualAnalogInV2(found[DEV_INPUT], self.ipcon)
                      if DEV_INPUT in found else None)

    def reconnect(self, timeout: float = 120.0):
        """Re-establish both transports after a reboot or a power cycle."""
        try:
            self.ipcon.disconnect()
        except Exception:                                           # noqa: BLE001
            pass

        self.ipcon = IPConnection()
        self.connect(timeout=timeout)

    def close(self):
        try:
            self.ipcon.disconnect()
        except Exception:                                           # noqa: BLE001
            pass

    # -- HTTP ------------------------------------------------------------------

    def api_get(self, path: str, timeout: float = 2.0) -> Any:
        r = requests.get(f'{self.base}/{path}', auth=self.auth, verify=self.verify,
                         timeout=timeout)
        r.raise_for_status()
        return r.json()

    def api_put(self, path: str, payload: Any, timeout: float = 5.0) -> requests.Response:
        return requests.put(f'{self.base}/{path}', json=payload, auth=self.auth,
                            verify=self.verify, timeout=timeout)

    # -- Composite reads -------------------------------------------------------

    def sample(self) -> dict:
        """One timeline row. Never raises: a failed read becomes a None field."""
        row: dict[str, Any] = {'t': time.monotonic(), 'online': True}

        with self._lock:
            try:
                s = self.evse.get_plug_lock_state()
                row['pls'] = s.state
                row['lock_wanted'] = s.lock_wanted
            except Exception:                                       # noqa: BLE001
                row['pls'] = row['lock_wanted'] = None
                row['online'] = False

            try:
                row['enabled_bricklet'] = self.evse.get_plug_lock_configuration()
            except Exception:                                       # noqa: BLE001
                row['enabled_bricklet'] = None

            # One call for both channels, for the same reason get_all_voltages() is used
            # below: two calls a round trip apart can straddle a transition and report a
            # state the hardware was never in. That is not hypothetical - the 18:52 run
            # of 2026-09-13 recorded relay_loop=True next to input_loop_mv=17 in one row,
            # because the second read landed after the challenge had already restored the
            # relay. The relay is open for about 500 ms and a sample takes about 1.1 s.
            try:
                relay_value = self.relay.get_value() if self.relay else None
            except Exception:                                       # noqa: BLE001
                relay_value = None

            row['relay'] = relay_value[RELAY_CHANNEL] if relay_value else None
            row['relay_loop'] = relay_value[RELAY_LOOP_CHANNEL] if relay_value else None

            # Both channels come out of one call, so the feedback line and the loop are
            # sampled at the same instant rather than a round trip apart. The raw
            # millivolts are kept next to the derived level: the level is what the
            # firmware acts on, the millivolts are what makes a failure diagnosable.
            try:
                mv = self.input.get_all_voltages() if self.input else None
            except Exception:                                       # noqa: BLE001
                mv = None

            row['input_mv'] = mv[INPUT_CHANNEL] if mv else None
            row['input_loop_mv'] = mv[INPUT_LOOP_CHANNEL] if mv else None

            row['input'], row['input_plausible'] = classify_input(
                row['input_mv'],
                INPUT_FEEDBACK_LOW_MAX, INPUT_FEEDBACK_HIGH_MIN, INPUT_FEEDBACK_HIGH_MAX)

            # The loop, read directly off the Bricklets rather than through the
            # firmware's verdict. Recorded separately so a disagreement between the two
            # - a relay that closes but an input that never follows, say - is visible in
            # the timeline instead of being hidden behind a single Verified/FailOpen.
            row['input_loop'], row['input_loop_plausible'] = classify_input(
                row['input_loop_mv'],
                INPUT_LOOP_LOW_MAX, INPUT_LOOP_HIGH_MIN, INPUT_LOOP_HIGH_MAX)

        try:
            st = self.api_get('plug_lock/state', timeout=1.0)
            row['actuation'] = st.get('actuation')
            row['lock_closed'] = st.get('lock_closed')
            row['relay_found'] = st.get('relay_found')
            row['input_found'] = st.get('input_found')
            row['loop'] = st.get('loop')
            row['state_api'] = st.get('state')
        except Exception:                                           # noqa: BLE001
            row['actuation'] = row['lock_closed'] = None
            row['relay_found'] = row['input_found'] = row['state_api'] = None
            row['loop'] = None
            row['online'] = False

        try:
            ev = self.api_get('evse/state', timeout=1.0)
            row['cp'] = ev.get('iec61851_state')
            row['error_state'] = ev.get('error_state')
            row['contactor'] = ev.get('contactor_state')
        except Exception:                                           # noqa: BLE001
            row['cp'] = row['error_state'] = row['contactor'] = None

        return row


# ---------------------------------------------------------------------------------
# Timeline
# ---------------------------------------------------------------------------------

class Timeline:
    """Append-only sample log. Every assertion is a query over this."""

    def __init__(self):
        self.rows: list[dict] = []
        self._lock = threading.Lock()

    def append(self, row: dict):
        with self._lock:
            self.rows.append(row)

    def mark(self) -> float:
        return time.monotonic()

    def since(self, t0: float, t1: Optional[float] = None) -> list[dict]:
        t1 = t1 if t1 is not None else time.monotonic()
        with self._lock:
            return [r for r in self.rows if t0 <= r['t'] <= t1]

    def latest(self) -> dict:
        with self._lock:
            return dict(self.rows[-1]) if self.rows else {}

    def latest_with(self, key: str, max_age: float = 10.0) -> dict:
        """The most recent row in which `key` actually has a value.

        One sample is seven TFP round trips plus two HTTP GETs, and any of them can fail
        or time out on its own; the failing read is recorded as None and the rest of the
        row is still good. latest() therefore routinely returns a row where the one field
        an assertion cares about is None, and the assertion fails for no reason - step 0.4
        did exactly that on 2026-09-13, asserting cp == A against the first row of the run,
        which was taken before the first evse/state read had come back.

        Bounded rather than unbounded, because "the last value we ever saw" is not
        evidence about now: past max_age this returns {} and the caller fails on a missing
        value, which is the honest answer.
        """
        cutoff = time.monotonic() - max_age
        with self._lock:
            for r in reversed(self.rows):
                if r['t'] < cutoff:
                    break
                if r.get(key) is not None:
                    return dict(r)
        return {}

    # -- queries ---------------------------------------------------------------

    def saw(self, t0: float, key: str, value: Any) -> bool:
        return any(r.get(key) == value for r in self.since(t0))

    def never(self, t0: float, key: str, value: Any) -> bool:
        """True if the window holds evidence, and none of it is `value`.

        The evidence clause is not pedantry. Without it a window in which every sample of
        `key` is None - which is exactly what a reboot window looks like, since cp and
        error_state both arrive over the HTTP API that is down - would pass vacuously, and
        a check that cannot fail is worse than one that fails spuriously. Timeline.stable()
        has guarded its empty window since 2026-09-08 for the same reason; this did not.
        """
        rows = [r for r in self.since(t0) if r.get(key) is not None]
        return bool(rows) and all(r.get(key) != value for r in rows)

    def first_at(self, t0: float, key: str, value: Any) -> Optional[float]:
        for r in self.since(t0):
            if r.get(key) == value:
                return r['t'] - t0
        return None

    def sequence(self, t0: float, key: str, values: list[Any]) -> bool:
        """True if the given values appear in order (gaps allowed)."""
        it = iter(self.since(t0))
        for want in values:
            for r in it:
                if r.get(key) == want:
                    break
            else:
                return False
        return True

    def stable(self, t0: float, key: str, value: Any) -> bool:
        """True if every online sample in the window holds the value."""
        rows = [r for r in self.since(t0) if r.get('online')]
        return bool(rows) and all(r.get(key) == value for r in rows)

    def transitions(self, key: str, frm: Any, to: Any) -> int:
        with self._lock:
            rows = list(self.rows)
        n, prev = 0, None
        for r in rows:
            cur = r.get(key)
            if prev == frm and cur == to:
                n += 1
            if cur is not None:
                prev = cur
        return n


class Recorder(threading.Thread):
    def __init__(self, charger: Charger, timeline: Timeline):
        super().__init__(daemon=True, name='recorder')
        self.charger = charger
        self.timeline = timeline
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            started = time.monotonic()
            try:
                self.timeline.append(self.charger.sample())
            except Exception:                                       # noqa: BLE001
                self.timeline.append({'t': time.monotonic(), 'online': False})
            time.sleep(max(0.0, SAMPLE_INTERVAL_S - (time.monotonic() - started)))


# ---------------------------------------------------------------------------------
# Operator interaction
# ---------------------------------------------------------------------------------

class Operator:
    def __init__(self, report: list, auto_no: bool = False):
        self.report = report
        self.auto_no = auto_no

    def _emit(self, kind: str, text: str, answer: Any):
        self.report.append({'kind': kind, 'prompt': text, 'answer': answer,
                            'wall': time.time()})

    def action(self, text: str):
        print(f'\n  ACTION  {text}')
        try:
            input('          press Enter when done (Ctrl-C to abort) > ')
        except (EOFError, KeyboardInterrupt):
            raise Abort('operator aborted')
        self._emit('action', text, 'done')

    def observe(self, text: str) -> bool:
        """A yes/no observation. Recorded as operator-reported, not as measured data."""
        print(f'\n  OBSERVE {text}')
        while True:
            try:
                a = input('          [y/n] > ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                raise Abort('operator aborted')
            if a in ('y', 'yes'):
                self._emit('observe', text, True)
                return True
            if a in ('n', 'no'):
                self._emit('observe', text, False)
                return False

    def safe_gate(self, text: str):
        """A gate the run will not pass without an explicit affirmative."""
        print(f'\n  SAFETY  {text}')
        try:
            a = input('          type "yes" to continue > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            raise Abort('operator aborted')
        if a != 'yes':
            self._emit('safe_gate', text, False)
            raise Abort('safety gate not confirmed')
        self._emit('safe_gate', text, True)

    def note(self, text: str):
        print(f'          {text}')


# ---------------------------------------------------------------------------------
# Step framework
# ---------------------------------------------------------------------------------

@dataclass
class Step:
    id: str
    group: str
    title: str
    fn: Callable[['Context'], None]


STEPS: list[Step] = []

# The one group --from-step cannot skip. Everything in it establishes what the rest of
# the script assumes rather than testing anything: the three devices are there, the
# restore point is recorded, the socket is idle, the harness loop is verified and the
# charger can charge at all. Resuming past it does not save time worth having - it just
# gives every later failure one more possible cause, and leaves the teardown with no
# restore point to put the charger back to.
PREFLIGHT_GROUP = 'preflight'


def step(step_id: str, group: str, title: str):
    def deco(fn):
        STEPS.append(Step(step_id, group, title, fn))
        return fn
    return deco


@dataclass
class Context:
    charger: Charger
    timeline: Timeline
    op: Operator
    report: dict
    snapshot: dict = field(default_factory=dict)
    skip_mechanical: bool = False
    skip_actuator_stress: bool = False
    # Set only by 0.7, and only after the operator has said yes at a safe_gate. teardown()
    # keys the restore off this rather than off the snapshot, so a run that never got that
    # far cannot "restore" a setting it never touched.
    user_auth_disabled_by_run: bool = False
    _last_actuation: float = 0.0
    _last_cp_cycle: float = 0.0

    # -- assertions ------------------------------------------------------------

    def check(self, ok: bool, what: str):
        self.report['checks'].append({'what': what, 'ok': bool(ok), 'wall': time.time()})
        print(f'          {"PASS" if ok else "FAIL"}  {what}')
        if not ok:
            raise StepFailed(what)

    def soft(self, ok: bool, what: str):
        """Recorded, reported, but does not fail the run. Used where the expected
        behaviour is itself under investigation (review finding #3)."""
        self.report['checks'].append({'what': what, 'ok': bool(ok), 'soft': True,
                                      'wall': time.time()})
        print(f'          {"PASS" if ok else "NOTE"}  {what}')

    # -- guards ----------------------------------------------------------------

    def guard_actuation_budget(self):
        n = self.timeline.transitions('relay', False, True)
        if n > MAX_ACTUATION_CYCLES:
            raise GuardTripped(f'actuation budget exceeded: {n} > {MAX_ACTUATION_CYCLES}')

    def guard_actuation_spacing(self):
        wait = MIN_ACTUATION_SPACING_S - (time.monotonic() - self._last_actuation)
        if wait > 0:
            print(f'          (actuation spacing: waiting {wait:.1f}s)')
            time.sleep(wait)
        self._last_actuation = time.monotonic()

    def guard_cp_cycle_spacing(self):
        wait = MIN_CP_CYCLE_SPACING_S - (time.monotonic() - self._last_cp_cycle)
        if wait > 0:
            print(f'          (contactor spacing: waiting {wait:.1f}s)')
            time.sleep(wait)
        self._last_cp_cycle = time.monotonic()

    def require_cp_a(self):
        """G1. Nothing that bypasses the firmware happens unless the socket is idle."""
        cp = self.timeline.latest_with('cp').get('cp')
        if cp != CP_A:
            raise GuardTripped(f'refusing to proceed: CP state is {CP_NAME.get(cp, cp)}, not A')

    def require_socket_empty(self):
        """G1, for steps that run under a standing BRICKLET_DEDICATION_NOT_VERIFIED.

        Same intent as require_cp_a() - do not bypass the firmware with a plug in the
        socket - but CP carries no socket information here. A hardware-absent or loop-open
        fault holds EVSE error state 6 up unconditionally, which pins the CP report at EF
        whatever is in the socket: in 7.3 on 2026-09-14 cp read 4 twenty-four seconds
        before the plug went in. lock_wanted is the EVSE's own read of the socket and does
        keep tracking while the error is up, which is why 7.3 and 9.2 already assert
        withdrawal on it.

        Prefer require_cp_a() everywhere else. It is the stronger check, and it is correct
        for a fault raised by a lock *attempt* - that kind clears on withdrawal and CP does
        return to A (5.3 and 6.2, same run).
        """
        lw = self.timeline.latest_with('lock_wanted').get('lock_wanted')
        if lw is not False:
            raise GuardTripped(f'refusing to proceed: lock_wanted is {lw}, not False '
                               f'- the EVSE still sees a plug in the socket')

    # -- helpers ---------------------------------------------------------------

    def settle(self, seconds: float = 1.0):
        """Sleep, then make sure the assertion window that follows holds evidence.

        Every `mark() -> settle() -> stable()` site depends on a sample landing inside
        the window. Timeline.stable() reports False for an empty window, which is
        right - it must never pass on no data - but this used to be a bare sleep, and
        one sample() is seven TFP round trips plus an HTTP GET. A one second window is
        therefore routinely empty, and step 1.1 failed exactly that way on 2026-09-08
        with the firmware behaving correctly: the last sample landed 0.05 s before the
        mark and the next one 2.2 s after the check had already raised.

        Waiting for an *online* sample rather than for any sample keeps the existing
        failure mode: on a charger that is genuinely offline this times out, the window
        stays empty, and the check fails the way it does today.
        """
        entry = time.monotonic()
        time.sleep(seconds)

        deadline = time.monotonic() + SETTLE_SAMPLE_TIMEOUT_S

        while time.monotonic() < deadline:
            if any(r.get('online') for r in self.timeline.since(entry)):
                return
            time.sleep(SAMPLE_INTERVAL_S)

        print(f'          (settle: no online sample within {SETTLE_SAMPLE_TIMEOUT_S:.1f}s, '
              f'the assertion window is empty)')

    def wait_until(self, pred: Callable[[dict], bool], timeout: float, what: str) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pred(self.timeline.latest()):
                return True
            time.sleep(SAMPLE_INTERVAL_S)
        print(f'          (timed out after {timeout:.1f}s waiting for {what})')
        return False

    def charging_blockers(self) -> list[str]:
        """Active charging slots sitting at 0 mA, named.

        Anything on this list stops the EVSE reaching IEC 61851 state C no matter what
        the car asks for, and none of it is the plug lock's doing - the plug lock blocks
        through plug_lock_contactor_close_is_blocked(), which shows up as error state 6.
        A charger left needing authorization, or with the autostart button off, simply
        sits in state B and looks exactly like a Charge button that was not pressed.
        """
        try:
            slots = self.charger.api_get('evse/slots')
        except Exception as e:                                      # noqa: BLE001
            return [f'evse/slots unreadable ({e})']

        blockers = []
        for i, slot in enumerate(slots):
            if slot.get('active') and slot.get('max_current') == 0:
                blockers.append(f'slot {i} {SLOT_NAMES.get(i, "?")}')

        return blockers

    def require_charging_possible(self):
        """Say what is in the way BEFORE the operator is asked to hold a button for
        twenty seconds, rather than after."""
        blockers = self.charging_blockers()
        if not blockers:
            return

        self.op.note('charging is currently blocked by: ' + ', '.join(blockers))
        self.op.note('this is charger configuration, not the plug lock - the plug lock '
                     'refuses through error state 6, which is not set here')

        if any(b.startswith('slot 6 ') for b in blockers):
            self.op.note('slot 6 means the charger requires authorization, which a '
                         'Testboy cannot give. Either switch "Authorization required" '
                         'off for the duration of this test, or present an authorized '
                         'NFC tag before each charging step.')

        raise GuardTripped('refusing to ask for a charge that cannot start: '
                           + ', '.join(blockers))

    def check_never(self, t0: float, key: str, value: Any, what: str,
                    evidence_timeout: float = 5.0):
        """never() over a window that is guaranteed to hold evidence.

        Timeline.never() refuses to pass on a window containing no samples of `key`,
        because a check that cannot fail is worse than one that fails spuriously. But a
        short window can legitimately contain none, and for two compounding reasons: one
        sample is about 1.1 s while a loop challenge is over in about 1.5 s, and
        Charger.sample() stamps row['t'] when it *starts* while Timeline.append() happens
        when it finishes. A row timestamped inside the window therefore does not become
        visible until roughly a sample period after that - the same skew settle() was
        written for.

        So the wait polls since(t0), not latest(). Asserting that evidence exists
        somewhere while querying a window that does not yet contain it is how step 0.6
        reported a violation that had not happened, twice, on 2026-09-13.
        """
        deadline = time.monotonic() + evidence_timeout

        while time.monotonic() < deadline:
            if any(r.get(key) is not None for r in self.timeline.since(t0)):
                break

            time.sleep(SAMPLE_INTERVAL_S)
        else:
            self.check(False, f'{what} - NO {key.upper()} SAMPLE IN THE WINDOW, '
                              f'nothing was judged')
            return

        self.check(self.timeline.never(t0, key, value), what)

    def expect_state(self, **fields):
        """Print the precondition before the operator is asked to do anything, so a
        mismatch fails before a physical action is taken on a wrong assumption."""
        parts = []
        for k, v in fields.items():
            parts.append(f'{k}={v}')
        print(f'          expected now: {", ".join(parts)}')

        # Per field rather than one shared row: a single sample can carry a good pls and
        # a None cp, and a precondition that fails on the None would send the operator
        # away on a reading that was never taken.
        for k, v in fields.items():
            cur = self.timeline.latest_with(k).get(k)
            self.check(cur == v, f'precondition {k} == {v} (is {cur})')

    def set_enabled(self, enabled: bool) -> requests.Response:
        return self.charger.api_put('plug_lock/config_update', {'enabled': enabled})

    def challenge_loop(self) -> requests.Response:
        """Force a harness loop challenge now instead of at the next 15 minute mark.
        Refused while the actuator is being driven, which is deliberate."""
        return self.charger.api_put('plug_lock/loop_challenge', None)

    # -- the harness loop, and the escape hatch built on it --------------------

    def open_s3(self, why: str):
        """Break the harness loop. Flipped live like S1 and S2, not power cycled: the
        loop carries only microamps into a high-impedance input and drives nothing, which
        makes S3 the least consequential of the three switches. CP A is still required,
        because opening the loop while the feature is enabled faults charging."""
        self.require_cp_a()
        self.op.action(f'Open switch S3 LOOP LINK. ({why})')
        self.op.safe_gate('Confirm S3 LOOP LINK is OPEN.')

        if not self.wait_until(lambda x: x.get('loop') == LOOP_FAIL_OPEN,
                               LOOP_LOSS_BUDGET_S, 'FailOpen'):
            raise GuardTripped('the harness loop did not report FailOpen after S3 was '
                               'opened - check that S3 is really in the loop leg')

    def close_s3(self):
        """Restore the loop and wait for it to be re-proven, not merely re-connected.
        The firmware challenges as soon as the level returns, so this is a real
        verdict rather than a steady high."""
        self.op.action('Close switch S3 LOOP LINK.')
        self.op.safe_gate('Confirm S3 LOOP LINK is CLOSED.')

        if not self.wait_until(lambda x: x.get('loop') == LOOP_VERIFIED,
                               LOOP_RECOVERY_BUDGET_S, 'Verified'):
            raise GuardTripped('the harness loop did not return to Verified after S3 '
                               'was closed')

    def remove_both_bricklets(self, why: str):
        """The documented removal procedure, and the only thing that opens the disable
        guard. Both, not either: the EVSE is told "not found" only when neither
        Bricklet answers, so one left in place keeps the guard shut."""
        self.require_cp_a()
        self.power_cycle(f'Unplug BOTH plug lock Bricklets from their ports - the '
                         f'Industrial Quad Relay 2.1 and the Industrial Dual Analog In '
                         f'2.0. ({why})')
        self.settle(3.0)

        if (self.timeline.latest_with('relay_found').get('relay_found') is not False
                or self.timeline.latest_with('input_found').get('input_found') is not False):
            raise GuardTripped('one or both plug lock Bricklets are still detected after '
                               'being unplugged - check that both really came out')

    def refit_both_bricklets(self):
        """Put them back and wait for the loop to be re-proven, not merely for the
        devices to reappear."""
        self.power_cycle('Plug BOTH plug lock Bricklets back into their ports.')
        self.settle(3.0)

        if not self.wait_until(lambda x: x.get('loop') == LOOP_VERIFIED,
                               LOOP_RECOVERY_BUDGET_S, 'Verified'):
            raise GuardTripped('the harness loop did not return to Verified after the '
                               'Bricklets were refitted')

    def ensure_disabled(self):
        """Get to enabled == false, whatever it takes.

        The API on its own cannot do this: the EVSE refuses `enabled -> false` unless
        the last hardware report says neither Bricklet was found, because a socket
        wired for a lock but with the lock switched off would energize with a
        removable plug.

        THIS IS EXPENSIVE - two power cycles and four operator actions. Opening the
        loop is deliberately not enough (see the note at the top of this file), so
        there is no cheap route. Steps that need a disabled starting point are worth
        scheduling before anything enables the feature; on a charger that arrives
        disabled this returns immediately.

        Leaves both Bricklets fitted and the loop re-verified, so the next step starts
        from a healthy rig with the feature merely off.
        """
        if self.enabled_readback() is False:
            return

        self.remove_both_bricklets('the disable guard needs both Bricklets gone')

        r = self.set_enabled(False)
        self.settle(1.5)

        if not (r.ok and self.enabled_readback() is False):
            raise GuardTripped(f'disable was refused even with both Bricklets gone '
                               f'(HTTP {r.status_code}) - the escape hatch is not working')

        self.refit_both_bricklets()

    def ensure_enabled(self):
        """Get to enabled == true. Requires a verified loop, so it is a plain API call
        on a healthy rig - but fail loudly rather than leaving a later step to blame
        the firmware for a broken harness."""
        if self.enabled_readback() is True:
            return

        cur = self.timeline.latest_with('loop').get('loop')
        if cur != LOOP_VERIFIED:
            raise GuardTripped(f'cannot enable: harness loop is '
                               f'{LOOP_NAME.get(cur, cur)}, not Verified')

        r = self.set_enabled(True)
        self.settle(1.5)

        if not (r.ok and self.enabled_readback() is True):
            raise GuardTripped(f'enable was refused (HTTP {r.status_code})')

    def enabled_readback(self) -> Optional[bool]:
        """Read the flag back over 4223, i.e. from the EVSE rather than from the ESP32."""
        try:
            return self.charger.evse.get_plug_lock_configuration()
        except Exception:                                           # noqa: BLE001
            return None

    def power_cycle(self, instruction: str):
        """G2. Every cabling change is de-energized, and the power-down is confirmed
        before the change instruction is printed."""
        self.op.action('Switch the wallbox OFF at its supply.')
        self.op.safe_gate('Confirm the wallbox is de-energized and the socket is dead.')
        self.op.action(instruction)
        self.op.action('Switch the wallbox back ON.')
        print('          reconnecting ...')
        self.charger.reconnect()
        self.wait_until(lambda r: r.get('online'), 90.0, 'charger back online')


# ---------------------------------------------------------------------------------
# Group 0 - preflight
# ---------------------------------------------------------------------------------

@step('0.1', 'preflight', 'Discover the three devices')
def s01(c: Context):
    c.check(DEV_EVSE in c.charger.uids, 'EVSE bricklet 2167 present')
    c.check(DEV_RELAY in c.charger.uids, 'Industrial Quad Relay 2.1 (2102) present')
    c.check(DEV_INPUT in c.charger.uids, 'Industrial Dual Analog In 2.0 (2121) present')

    ident = c.charger.evse.get_identity()
    fw = '.'.join(str(x) for x in ident.firmware_version)
    # Read the version, never assert a constant: build-warp4 writes 200 + n into the
    # major byte on every build, so the number moves.
    c.report['evse_firmware'] = fw
    c.report['uids'] = {str(k): v for k, v in c.charger.uids.items()}
    c.op.note(f'EVSE firmware {fw}, UID {ident.uid}')


@step('0.2', 'preflight', 'Snapshot the restore point')
def s02(c: Context):
    c.snapshot['config'] = c.charger.api_get('plug_lock/config')
    c.snapshot['state'] = c.charger.api_get('plug_lock/state')
    c.snapshot['evse'] = c.charger.api_get('evse/state')

    # Captured here even though only 0.7 may change it, because the restore point has to
    # exist before anything is touched. evse/user_enabled is the only setting this run
    # can alter that is not the plug lock's own, and the only one whose loss would leave
    # the charger less secure rather than merely misconfigured.
    c.snapshot['user_enabled'] = c.charger.api_get('evse/user_enabled')

    c.report['snapshot'] = c.snapshot
    c.op.note(f'restore point: enabled={c.snapshot["config"].get("enabled")}, '
              f'user authorization={c.snapshot["user_enabled"].get("enabled")}')


@step('0.3', 'preflight', 'Security: is the proxy authenticated?')
def s03(c: Context):
    try:
        proxy = c.charger.api_get('proxy/config')
    except Exception as e:                                          # noqa: BLE001
        c.op.note(f'proxy/config unreadable ({e}); recording as unknown')
        c.report['proxy_authenticated'] = None
        return

    secret = proxy.get('authentication_secret', '')
    c.report['proxy_authenticated'] = bool(secret)
    # Report only. This script working at all is the proof: anyone on the LAN can
    # drive the relay bricklet directly, bypassing every firmware interlock.
    c.soft(bool(secret), 'proxy authentication_secret is set (report-only)')


@step('0.4', 'preflight', 'Socket is idle')
def s04(c: Context):
    c.check(c.timeline.latest_with('cp').get('cp') == CP_A, 'CP state is A (nothing plugged in)')
    c.check(c.timeline.latest_with('relay_found').get('relay_found') is True, 'relay bricklet found')
    c.check(c.timeline.latest_with('input_found').get('input_found') is True, 'input bricklet found')
    c.op.safe_gate('Confirm all four switches are CLOSED: S1 RELAY OUT, '
                   'S2 LOCK FEEDBACK, S3 LOOP LINK, S4 LOCK SUPPLY.')


@step('0.5', 'preflight', 'Harness loop is verified')
def s05(c: Context):
    # Discovery is not enough any more. Everything from here on assumes the loop is
    # good: without it the EVSE is being told there is no hardware, enabling is refused
    # and every later group would fail for the wrong reason.
    loop = c.timeline.latest_with('loop').get('loop')
    c.op.note(f'harness loop reports {LOOP_NAME.get(loop, loop)}')

    if loop == LOOP_FAIL_STUCK:
        c.op.note('FailStuck means the loop does not open when the relay is released. '
                  'The usual cause is relay channel 1 wired with reversed polarity: the '
                  'CPC1002N is unipolar and conducts through its body diode. DC+ must '
                  'face PSU+.')
    elif loop == LOOP_FAIL_OPEN:
        c.op.note('FailOpen means the loop never closes. Check the link PSU+ -> relay '
                  'channel 1 -> input channel 1 -> PSU-, the input polarity, and that '
                  'the lock PSU is actually live.')

    c.check(loop == LOOP_VERIFIED, 'plug_lock/state.loop is Verified')

    # The firmware holds the loop relay closed except during a challenge, so this is
    # what a healthy idle loop looks like straight off the Bricklets.
    c.check(c.timeline.latest_with('relay_loop').get('relay_loop') is True,
            'relay channel 1 is closed')
    c.check(c.timeline.latest_with('input_loop').get('input_loop') is True,
            'input channel 1 reads high')


@step('0.6', 'preflight', 'The loop challenge actually toggles the relay')
def s06(c: Context):
    # 0.5 only proves a steady high, which a 12 V strap across input channel 1 would
    # also produce. The release phase is the half a strap cannot survive, so watch one
    # happen rather than trusting the retained verdict.
    c.require_cp_a()

    t0 = c.timeline.mark()
    r = c.challenge_loop()
    c.check(r.ok, f'loop_challenge accepted (HTTP {r.status_code})')

    # Polled directly rather than read off the timeline. Challenging is a display value
    # that exists for about 1.25 s - release is two consecutive lows and restore is a
    # settle plus two highs, at 250 ms each - against a background sampler that produces
    # a row every ~1.1 s. The 18:58 run of 2026-09-13 missed it even though the release
    # itself was captured in the same row, and the one sample that did land inside the
    # window had loop = None because that HTTP read did not return in time.
    #
    # Ten polls a second against a 1.25 s window is not a race. Widening the assertion
    # instead would have left nothing checking that a *new* challenge ran at all: the
    # retained verdict stays Verified throughout, so the check below passes either way.
    saw_challenging = False
    release_mv = None
    deadline = time.monotonic() + LOOP_CHALLENGE_BUDGET_S

    while time.monotonic() < deadline:
        try:
            st = c.charger.api_get('plug_lock/state', timeout=1.0)
        except Exception:                                           # noqa: BLE001
            time.sleep(0.1)
            continue

        if st.get('loop') == LOOP_CHALLENGING:
            saw_challenging = True

        # The release itself, captured at the same rate and for the same reason. This is
        # the firmware's own reading of input channel 1 rather than the script's - one
        # more layer of indirection than the proxy read, but it is a published
        # measurement and not a conclusion, and it is the only one that can actually be
        # sampled inside a 500 ms window.
        mv = st.get('loop_voltage')
        if mv is not None and mv <= INPUT_LOOP_LOW_MAX and (release_mv is None or mv < release_mv):
            release_mv = mv

        if saw_challenging and release_mv is not None:
            break

        time.sleep(0.1)

    c.report['loop_release_mv'] = release_mv

    c.check(saw_challenging, 'a challenge starts')
    c.check(release_mv is not None,
            f'input channel 1 went low during the release (lowest {release_mv} mV, '
            f'not a strap)')

    c.check(c.wait_until(lambda x: x.get('loop') == LOOP_VERIFIED,
                         LOOP_CHALLENGE_BUDGET_S, 'Verified'),
            'the challenge ends in Verified')

    # The point of the whole step, and it rests on the input rather than the relay.
    #
    # relay.get_value() is the firmware's own bookkeeping read back, not a measurement of
    # a contact - it reports what was commanded, so it can never be independent evidence
    # that the loop opened. The independent evidence is the voltage on the other channel:
    # a strap across input channel 1 holds it at ~12 V through the release, and nothing
    # the firmware commands can make that reading fall.
    #
    # It is also the only one of the two that is reliably observable. The release lasts
    # about 500 ms and a sample takes about 1.1 s, so catching the relay mid-release is a
    # race; the input is captured in the same round trip as the voltage that explains it.
    c.check(c.timeline.latest_with('input_loop').get('input_loop') is True, 'input channel 1 high again')

    # Both report-only, and for the same reason: the background sampler produces a row
    # every ~1.1 s against a ~500 ms release, so landing inside it is luck. When it does
    # land, these are the stronger evidence - read straight off the Bricklets over the
    # proxy, independent of the ESP32's own ADC handling - so they are worth recording.
    # They are just not something a run may be failed on.
    c.soft(c.timeline.saw(t0, 'input_loop', False),
           'input channel 1 observed low by direct read (report-only, sampling race)')
    c.soft(c.timeline.saw(t0, 'relay_loop', False),
           'relay channel 1 observed released (report-only, sampling race)')
    c.check(c.timeline.latest_with('relay_loop').get('relay_loop') is True, 'relay channel 1 closed again')

    # The EVSE must not have seen a fault from a challenge it is not supposed to notice.
    c.check_never(t0, 'pls', PLS_DEDICATION_NOT_VERIFIED,
                  'the EVSE never saw BRICKLET_DEDICATION_NOT_VERIFIED during the challenge')


@step('0.7', 'preflight', 'Charging is possible at all')
def s07(c: Context):
    # Everything from group 2 onward needs a real charging session at some point, and a
    # charger that will not start one fails those steps as a twenty second timeout on
    # "CP state reaches C" - which reads exactly like a Charge button that was not
    # pressed. That cost a run on 2026-09-13, twenty minutes in, with the plug lock
    # reporting LOCKED and error_state 0 throughout because none of it was its doing.
    #
    # Checked here so it costs thirty seconds instead, before any operator time is spent.
    blockers = c.charging_blockers()

    for b in blockers:
        c.op.note(f'blocking: {b}')

    # Slot 6 is the one this run can clear itself, and the only setting it ever changes
    # outside the plug lock. Behind a safe_gate rather than done silently: unlike
    # everything else here it OUTLIVES THE PROCESS - the handler writes the EVSE's flash
    # through apply_slot_default(), so it survives the wallbox power cycles in group 1,
    # the ESP32 reboots in group 2, and a crash of this script. teardown() puts it back
    # on every path the script controls, but SIGKILL and a closed terminal are not among
    # them, and the state it would leave behind is a charger anyone can start.
    if blockers == [f'slot 6 {SLOT_NAMES[6]}']:
        c.op.note('Slot 6 means the charger requires authorization, which a Testboy '
                  'cannot give. This run can switch it off and put it back at the end.')
        c.op.safe_gate('Turn user authorization OFF for the duration of this test? It is '
                       'persistent in the EVSE, and is only restored if this script gets '
                       'to run its teardown.')

        c.charger.api_put('evse/user_enabled_update', {'enabled': False})
        c.user_auth_disabled_by_run = True
        c.settle(1.5)

        blockers = c.charging_blockers()
        for b in blockers:
            c.op.note(f'still blocking: {b}')

    c.check(not blockers,
            'no active charging slot is holding the allowed current at 0 mA')

    if blockers:
        c.op.note('The plug lock itself is not involved: it blocks through error state '
                  '6, which these steps assert on separately.')


# ---------------------------------------------------------------------------------
# Group 1 - inert behaviour, no plug
# ---------------------------------------------------------------------------------

@step('1.1', 'inert', 'Disabled is inert')
def s11(c: Context):
    # ensure_disabled() rather than set_enabled(False): if the charger arrives already
    # enabled, the API cannot turn it off while the loop is verified, so this goes
    # by removing both Bricklets. On an already-disabled charger it is a no-op, which
    # is worth arranging: the route is two power cycles and four operator actions.
    c.ensure_disabled()
    t0 = c.timeline.mark()
    c.settle(1.0)
    c.check(c.timeline.stable(t0, 'pls', PLS_DISABLED), 'FID 81 reports DISABLED')
    c.check(c.timeline.stable(t0, 'lock_wanted', False), 'lock_wanted stays false')
    c.check(c.timeline.stable(t0, 'relay', False), 'relay channel 0 stays open')
    # Disabled must not stop the loop being monitored - the verdict is what lets the
    # feature be enabled again, so it has to keep working while the feature is off.
    c.check(c.timeline.stable(t0, 'loop', LOOP_VERIFIED),
            'the harness loop stays Verified while disabled')


@step('1.2', 'inert', 'Enable is accepted at CP A')
def s12(c: Context):
    c.require_cp_a()
    r = c.set_enabled(True)
    c.check(r.ok, f'config_update accepted (HTTP {r.status_code})')
    c.settle(1.5)
    c.check(c.enabled_readback() is True, 'FID 79 reads back enabled')
    c.check(c.wait_until(lambda x: x.get('pls') == PLS_IDLE, 3.0, 'IDLE'),
            'FID 81 reports IDLE with hardware present and no plug')


@step('1.3', 'inert', 'Disable is refused while the Bricklets are fitted')
def s13(c: Context):
    # The inverse of what this step used to assert. Disabling at CP A used to be
    # accepted, the flag persisted, and the socket then energized with a removable
    # plug on the *next* session - a hazard separated in time from the decision, and
    # reachable by an unauthenticated request on the LAN. The gate is now the harness,
    # not the session, so nothing about an idle socket makes a disable acceptable.
    c.require_cp_a()
    c.expect_state(pls=PLS_IDLE, loop=LOOP_VERIFIED)

    t0 = c.timeline.mark()
    r = c.set_enabled(False)
    c.settle(1.5)

    c.check(not r.ok or c.enabled_readback() is True,
            f'disable refused at CP A with the loop verified (HTTP {r.status_code})')
    c.check(c.enabled_readback() is True, 'FID 79 still reads back enabled')
    c.check(c.timeline.stable(t0, 'pls', PLS_IDLE), 'FID 81 stays IDLE')
    c.report['cp_a_disable_http'] = r.status_code


# ---------------------------------------------------------------------------------
# Group 2 - a restart is not a failure
#
# A reboot must not drop a held lock, must not be reported as an error, and - once a
# session is running - must release the contactor deliberately rather than by faulting.
#
# The last of those cannot be observed from here while it happens: cp, error_state and
# the Bricklet readings all arrive through the ESP32, which is the thing that is away.
# 2.3 therefore asks the operator to watch the phase LEDs, which is the one channel that
# does not go through the processor under test.
# ---------------------------------------------------------------------------------

@step('2.1', 'reboot', 'Lock the plug, then reboot the ESP32')
def s21(c: Context):
    c.require_cp_a()
    c.set_enabled(True)
    c.settle(1.5)
    c.guard_actuation_spacing()

    c.expect_state(pls=PLS_IDLE, relay=False)
    c.op.action('Hold the Unlock button, plug the TV 950 in, then release it.')

    c.check(c.wait_until(lambda x: x.get('pls') == PLS_LOCKED, ATTEMPT_TIMEOUT_S + 3.0,
                         'LOCKED'), 'plug reaches LOCKED')
    c.check(c.timeline.latest_with('actuation').get('actuation') == ACT_LOCKED, 'actuation is Locked')
    c.guard_actuation_budget()

    t0 = c.timeline.mark()
    c.report['reboot_mark'] = t0
    c.charger.api_put('reboot', {})
    c.op.note('ESP32 rebooting ...')
    time.sleep(3.0)
    c.charger.reconnect()
    c.wait_until(lambda x: x.get('online'), 90.0, 'charger back online')
    c.settle(2.0)

    # The Inactive branch reconciles from the feedback contact and deliberately never
    # drops the relay to find out (plug_lock.cpp:181-196). Anything else would release
    # a plug that might be live.
    c.check_never(t0, 'relay', False, 'relay channel never opened across the reboot')
    c.check(c.timeline.saw(t0, 'actuation', ACT_LOCKED), 'actuation converged back to Locked')

    # The headline of the absence grace: a restart is not a failure and must not be
    # reported as one. Before it, every reboot spent its whole length in
    # IEC61851_STATE_EF with error state 6 and a blinking 6 on the LED, which said
    # nothing true. Samples taken while the API was unreachable are None and match
    # neither value, so this asserts about what was actually observed.
    c.check_never(t0, 'error_state', ERROR_STATE_PLUG_LOCK,
                  f'no error state {ERROR_STATE_PLUG_LOCK} across the reboot')
    c.check_never(t0, 'cp', CP_EF, 'never entered IEC 61851 state EF')

    # And it comes all the way back on its own.
    c.check(c.wait_until(lambda x: x.get('pls') == PLS_LOCKED, 30.0, 'LOCKED'),
            'FID 81 returns to LOCKED without intervention')


@step('2.2', 'reboot', 'Reboot while charging releases the contactor deliberately')
def s22(c: Context):
    """The shutdown gate. The ESP32 announces the restart in pre_reboot(), the EVSE
    selects IEC 61851 state B over state C, and the contactor is released the ordinary
    way instead of by faulting.

    It interrupts charging; it does not end the session. Nothing records that a session
    was stopped, so charging resumes by itself once the returning instance clears the
    flag. That is asserted below rather than treated as a defect - the guarantee is that
    the contactor is open while the ESP32 is away, not that the session is over.
    """
    c.check(c.timeline.latest_with('pls').get('pls') == PLS_LOCKED, 'still locked before charging')

    c.require_charging_possible()

    c.op.action('Press and HOLD the Charge button, and keep holding it until told to stop.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_C, 20.0, 'CP C'), 'CP state reaches C')
    c.check(c.op.observe('Are the phase LEDs on?'), 'operator sees phases (charging)')

    # BEFORE the reboot, and blocking. The phase LEDs are the only observation channel
    # that does not run through the processor being rebooted - cp, error_state and the
    # Bricklet readings are all unavailable for the whole window - and they go out within
    # a few seconds of the command. A note printed after api_put would be read too late
    # to see the thing it asks about.
    c.op.action('Keep holding Charge, and watch the phase LEDs. Confirm when you are '
                'watching - the reboot is triggered next.')

    t0 = c.timeline.mark()
    c.report['charging_reboot_mark'] = t0
    c.charger.api_put('reboot', {})
    c.op.note('ESP32 rebooting while charging ...')

    c.check(c.op.observe('Did the phase LEDs go OUT?'),
            'contactor released while the ESP32 was away')

    time.sleep(3.0)
    c.charger.reconnect()
    c.wait_until(lambda x: x.get('online'), 90.0, 'charger back online')
    c.settle(2.0)

    c.check_never(t0, 'error_state', ERROR_STATE_PLUG_LOCK,
                  f'no error state {ERROR_STATE_PLUG_LOCK}: the stop was deliberate, not a fault')
    c.check_never(t0, 'cp', CP_EF, 'never entered IEC 61851 state EF')
    c.check_never(t0, 'relay', False, 'plug stayed locked throughout')

    # Documented, not lamented: see the docstring.
    c.check(c.wait_until(lambda x: x.get('cp') == CP_C, 30.0, 'CP C'),
            'charging resumes on its own once the gate clears')

    c.op.action('Release the Charge button.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_B, 20.0, 'CP B'), 'back to CP state B')


@step('2.3', 'reboot', 'Unplug and return to idle')
def s23(c: Context):
    c.check(c.timeline.latest_with('pls').get('pls') == PLS_LOCKED, 'still locked before release')
    c.guard_cp_cycle_spacing()
    c.op.action('Hold the Unlock button and withdraw the TV 950.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_A, 15.0, 'CP A'), 'back to CP state A')
    c.check(c.wait_until(lambda x: x.get('relay') is False, UNLOCK_TIMEOUT_S + 3.0,
                         'relay open'), 'relay channel released')


# ---------------------------------------------------------------------------------
# Group 3 - the enable guard
# ---------------------------------------------------------------------------------

@step('3.1', 'enable-guard', 'Start charging with the feature disabled')
def s31(c: Context):
    # A disable is no longer available as a plain setup mechanism, so this goes through
    # the escape hatch. It leaves the loop closed and verified again, which case 3.2
    # depends on: the enable it attempts must be refused because the *contactor* is
    # closed, not because the harness is missing.
    c.require_cp_a()
    c.ensure_disabled()
    c.expect_state(pls=PLS_DISABLED, loop=LOOP_VERIFIED)

    c.guard_cp_cycle_spacing()
    c.require_charging_possible()
    c.op.action('Hold the Unlock button, plug the TV 950 in, release it, '
                'then press and HOLD the Charge button.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_C, 20.0, 'CP C'), 'CP state reaches C')
    c.check(c.op.observe('Are the phase LEDs on?'), 'operator sees phases (charging)')


@step('3.2', 'enable-guard', 'Enabling while charging must be refused')
def s32(c: Context):
    r = c.set_enabled(True)
    c.settle(1.5)
    # The guard found in review: enabling mid-session would arm an interlock against a
    # live contactor.
    c.check(not r.ok or c.enabled_readback() is False,
            f'enable refused while charging (HTTP {r.status_code})')
    c.check(c.timeline.latest_with('pls').get('pls') == PLS_DISABLED, 'still DISABLED')
    c.op.action('Release the Charge button, then withdraw the TV 950.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_A, 20.0, 'CP A'), 'back to CP state A')


# ---------------------------------------------------------------------------------
# Group 4 - the happy path
# ---------------------------------------------------------------------------------

@step('4.1', 'interlock', 'Plug in and lock')
def s41(c: Context):
    c.require_cp_a()
    c.set_enabled(True)
    c.settle(1.5)
    c.guard_actuation_spacing()
    c.expect_state(pls=PLS_IDLE, relay=False)

    t0 = c.timeline.mark()
    c.op.action('Hold the Unlock button, plug the TV 950 in, then release it.')

    c.check(c.wait_until(lambda x: x.get('pls') == PLS_LOCKED, ATTEMPT_TIMEOUT_S + 3.0,
                         'LOCKED'), 'reaches LOCKED within the attempt timeout')
    c.check(c.timeline.sequence(t0, 'pls', [PLS_IDLE, PLS_WAITING, PLS_LOCKED]),
            'timeline passes IDLE -> WAITING -> LOCKED')
    c.check(c.timeline.latest_with('lock_closed').get('lock_closed') is True, 'lock_closed set after debounce')
    c.check(c.timeline.latest_with('relay').get('relay') is True, 'relay channel 0 closed')
    c.check(c.timeline.latest_with('input').get('input') is True, 'analog in channel 0 reads a locked level')

    lock_ms = c.timeline.first_at(t0, 'pls', PLS_LOCKED)
    c.report['lock_time_s'] = lock_ms
    c.guard_actuation_budget()


@step('4.2', 'interlock', 'Mechanical retention (optional)')
def s42(c: Context):
    if c.skip_mechanical:
        c.op.note('skipped (--skip-mechanical): physical retention not verified')
        c.report['checks'].append({'what': 'mechanical retention', 'ok': None,
                                   'skipped': True})
        return

    c.expect_state(pls=PLS_LOCKED, cp=CP_B)
    c.op.safe_gate('Confirm the phase LEDs are OFF (not charging) before touching the plug.')
    # The only step that applies force to an engaged lock. A light pull only - do not
    # attempt to overcome the actuator.
    #
    # Do NOT hold the Unlock button. On the TV 950 it is purely the CP switch - nothing
    # mechanical happens when you press it - so holding it puts CP at A, the firmware reads
    # a disconnected vehicle and releases the lock before any force reaches the plug. The
    # prompt said to hold it until 2026-09-14, which made the step command the very thing it
    # was asserting to be impossible; it failed that way twice with the firmware behaving
    # correctly.
    #
    # With the button released the actuator is the ONLY thing retaining the plug: a Type 2
    # infrastructure-side plug has no latch, that is on the vehicle connector at the far end
    # of the cable. So a pull here does bear on the lock and not on some other retention.
    #
    # The instruction lives in the prompt rather than in a preceding note() so the operator
    # reads it at the moment they answer.
    held = c.op.observe('Apply a LIGHT pull only. Do not force it. '
                        'Does the plug stay retained?')
    c.check(held, 'plug is mechanically retained while LOCKED')


@step('4.3', 'interlock', 'Charging is permitted when locked')
def s43(c: Context):
    c.guard_cp_cycle_spacing()
    c.expect_state(pls=PLS_LOCKED)
    c.require_charging_possible()
    c.op.action('Press and HOLD the Charge button.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_C, 20.0, 'CP C'), 'CP state reaches C')
    c.check(c.op.observe('Are the phase LEDs on?'), 'operator sees phases')
    c.check(c.timeline.latest_with('error_state').get('error_state') != ERROR_STATE_PLUG_LOCK,
            'no plug lock error while properly locked')
    c.check(c.timeline.latest_with('pls').get('pls') == PLS_LOCKED, 'still LOCKED while charging')


@step('4.4', 'interlock', 'Release and unlock')
def s44(c: Context):
    c.op.action('Release the Charge button.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_B, 15.0, 'CP B'), 'back to CP state B')
    c.op.action('Hold the Unlock button and withdraw the TV 950.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_A, 20.0, 'CP A'), 'back to CP state A')
    c.check(c.wait_until(lambda x: x.get('relay') is False, UNLOCK_TIMEOUT_S + 3.0,
                         'relay open'), 'relay released after the plug is gone')


# ---------------------------------------------------------------------------------
# Group 5 - the veto proves itself
# ---------------------------------------------------------------------------------

@step('5.1', 'veto', 'Open S1 RELAY OUT')
def s51(c: Context):
    c.require_cp_a()
    # S1 is chosen over S2 deliberately: with the actuator disconnected the relay
    # closes into an open circuit, so LockFault holding relay_locked = true
    # (plug_lock.cpp:288-291) drives nothing and the plug is never physically locked.
    c.op.action('Open switch S1 RELAY OUT.')
    c.op.safe_gate('Confirm S1 RELAY OUT is OPEN and S2 LOCK FEEDBACK is CLOSED.')


@step('5.2', 'veto', 'A lock that never confirms becomes a fault')
def s52(c: Context):
    c.set_enabled(True)
    c.settle(1.5)
    c.guard_actuation_spacing()
    c.expect_state(pls=PLS_IDLE)

    t0 = c.timeline.mark()
    c.op.action('Hold the Unlock button, plug the TV 950 in, then release it.')

    c.check(c.wait_until(lambda x: x.get('pls') == PLS_FAULT_TIMEOUT,
                         LOCK_FAULT_BUDGET_S + 3.0, 'FAULT_TIMEOUT'),
            f'reaches FAULT_TIMEOUT after {ATTEMPTS} attempts')
    c.check(c.timeline.saw(t0, 'pls', PLS_WAITING), 'passed through WAITING first')
    c.check_never(t0, 'lock_closed', True, 'lock_closed never asserted')
    c.check(c.timeline.saw(t0, 'actuation', ACT_LOCK_FAULT), 'actuation reached LockFault')
    c.report['fault_time_s'] = c.timeline.first_at(t0, 'pls', PLS_FAULT_TIMEOUT)


@step('5.3', 'veto', 'The contactor must not close')
def s53(c: Context):
    c.guard_cp_cycle_spacing()
    c.expect_state(pls=PLS_FAULT_TIMEOUT)

    t0 = c.timeline.mark()
    c.require_charging_possible()
    c.op.action('Press and HOLD the Charge button.')
    c.settle(5.0)

    # Two independent paths to the same conclusion: what the operator sees at the
    # socket, and what the firmware reports.
    leds_off = not c.op.observe('Are the phase LEDs on?')
    c.check(leds_off, 'phase LEDs stay OFF - the contactor did not close')
    c.check(c.timeline.saw(t0, 'error_state', ERROR_STATE_PLUG_LOCK),
            f'EVSE reports error state {ERROR_STATE_PLUG_LOCK} (Plug Lock)')

    c.op.action('Release the Charge button, then withdraw the TV 950.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_A, 20.0, 'CP A'), 'back to CP state A')


@step('5.4', 'veto', 'Close S1 and recover')
def s54(c: Context):
    c.require_cp_a()
    c.op.action('Close switch S1 RELAY OUT.')
    c.op.safe_gate('Confirm both S1 and S2 are CLOSED.')
    c.settle(2.0)
    c.check(c.wait_until(lambda x: x.get('pls') in (PLS_IDLE, PLS_DISABLED), 10.0,
                         'IDLE'), 'recovers to IDLE with the actuator reconnected')


# ---------------------------------------------------------------------------------
# Group 6 - feedback lost mid-session (only reachable with the switches fitted)
# ---------------------------------------------------------------------------------

@step('6.1', 'feedback-loss', 'Lock, then open S2 while locked (optional)')
def s61(c: Context):
    # Skippable since S4 exists. This is the only step in the rig that leaves a live
    # motor being driven: LockFault holds relay channel 0 closed indefinitely, so with
    # feedback cut and the supply healthy the actuator really locks the plug and then
    # stays energized. Case 11.1 reaches the same firmware branch with the supply gone,
    # so nothing is being driven. Keep this one for the specific fault of a broken
    # feedback wire; skip it if you would rather not stress the actuator.
    if c.skip_actuator_stress:
        c.op.note('skipped (--skip-actuator-stress): use group 11 for the same branch')
        return

    c.require_cp_a()
    c.set_enabled(True)
    c.settle(1.5)
    c.guard_actuation_spacing()
    c.op.action('Hold the Unlock button, plug the TV 950 in, then release it.')
    c.check(c.wait_until(lambda x: x.get('pls') == PLS_LOCKED, ATTEMPT_TIMEOUT_S + 3.0,
                         'LOCKED'), 'locked before the feedback is cut')

    # Re-close S2 promptly: with a real actuator connected, LockFault holds relay channel 0
    # closed indefinitely, so the motor keeps being driven for as long as the switch is
    # open. That is an actuator-stress bound, NOT a firmware deadline - see below.
    t0 = c.timeline.mark()
    c.op.note('Re-close S2 as soon as you have opened it. The actuator is driven '
              'continuously while the feedback is out.')
    c.op.action('Open switch S2 LOCK FEEDBACK.')

    c.check(c.wait_until(lambda x: x.get('lock_closed') is False, 3.0, 'lock_closed false'),
            'feedback loss observed')
    c.check(c.timeline.saw(t0, 'actuation', ACT_LOCKING),
            'actuation re-entered Locking (plug_lock.cpp:243-251)')

    c.op.action('Close switch S2 LOCK FEEDBACK again NOW.')

    # This used to assert a recovery to LOCKED. That is only reachable if S2 closes within
    # PLUG_LOCK_ATTEMPTS x PLUG_LOCK_ATTEMPT_TIMEOUT = 6 s of the feedback dropping, and
    # the request to close it is printed only after the operator has walked back from the
    # switch to the keyboard - so the window has already expired by the time anyone can
    # read it. On 2026-09-14 the loss was at 17:10:52, LockFault latched at 17:10:59, and
    # the Enter for "Open S2" landed in that same second. One operator cannot do it, and no
    # wording change makes it possible.
    #
    # So assert what the rig can actually produce, which is also the safety-relevant half:
    # the attempts run out into LockFault and the EVSE raises its plug lock error. The
    # fault then holds until lock_wanted goes false, which 6.2's withdrawal delivers.
    c.check(c.timeline.saw(t0, 'actuation', ACT_LOCK_FAULT),
            'attempts run out into LockFault')
    c.check(c.timeline.saw(t0, 'error_state', ERROR_STATE_PLUG_LOCK),
            'EVSE raises plug lock error state 6')
    c.guard_actuation_budget()


@step('6.2', 'feedback-loss', 'Unplug')
def s62(c: Context):
    if c.skip_actuator_stress:
        c.op.note('skipped (--skip-actuator-stress): 6.1 left nothing plugged in')
        return

    c.op.safe_gate('Confirm S2 LOCK FEEDBACK is CLOSED.')
    c.op.action('Hold the Unlock button and withdraw the TV 950.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_A, 20.0, 'CP A'), 'back to CP state A')


# ---------------------------------------------------------------------------------
# Group 7 - hardware absent must fail safe
# ---------------------------------------------------------------------------------

@step('7.1', 'no-hardware', 'Remove the Dual Analog In bricklet')
def s71(c: Context):
    c.require_cp_a()
    c.set_enabled(True)
    c.settle(1.5)
    c.power_cycle('Unplug the Industrial Dual Analog In 2.0 bricklet from its port.')
    c.settle(3.0)

    c.check(c.timeline.latest_with('input_found').get('input_found') is False, 'input_found is false')
    c.check(c.wait_until(lambda x: x.get('pls') in (PLS_DEDICATION_NOT_VERIFIED,
                                                    PLS_DISABLED),
                         10.0, 'BRICKLET_DEDICATION_NOT_VERIFIED'),
            'FID 81 reports BRICKLET_DEDICATION_NOT_VERIFIED (or DISABLED)')


@step('7.2', 'no-hardware', 'Enabling is impossible without the bricklets')
def s72(c: Context):
    r = c.set_enabled(True)
    c.settle(1.5)
    c.check(not r.ok or c.enabled_readback() is False,
            f'enable refused with hardware absent (HTTP {r.status_code})')


@step('7.3', 'no-hardware', 'Fail safe, not fail open')
def s73(c: Context):
    c.guard_cp_cycle_spacing()
    t0 = c.timeline.mark()
    c.require_charging_possible()
    c.op.action('Hold the Unlock button, plug the TV 950 in, release it, '
                'then press and HOLD the Charge button.')
    c.settle(5.0)

    leds_off = not c.op.observe('Are the phase LEDs on?')
    c.check(leds_off, 'phase LEDs stay OFF with the feedback bricklet absent')
    c.check(c.timeline.saw(t0, 'error_state', ERROR_STATE_PLUG_LOCK),
            'EVSE reports the plug lock error rather than charging')

    c.op.action('Release the Charge button, then withdraw the TV 950.')

    # NOT cp == A. With the feedback Bricklet absent the EVSE holds plug lock error state 6,
    # which pins it in EF - cp read 4 from 20:24:10 on 2026-09-14, twenty-four seconds
    # before the plug went in, and stayed there. Asserting CP A here asserts the absence of
    # the very fail-safe this group exists to prove.
    #
    # lock_wanted is the EVSE's own read of the socket and does still track while the error
    # is up: true at insertion, false at withdrawal, in that same run.
    c.check(c.wait_until(lambda x: x.get('lock_wanted') is False, 20.0, 'lock_wanted false'),
            'the EVSE sees the socket empty again')


@step('7.4', 'no-hardware', 'Refit the Dual Analog In bricklet')
def s74(c: Context):
    c.power_cycle('Plug the Industrial Dual Analog In 2.0 bricklet back into its port.')
    c.settle(3.0)
    c.check(c.timeline.latest_with('input_found').get('input_found') is True, 'input_found is true again')


@step('7.5', 'no-hardware', 'Remove the Quad Relay bricklet')
def s75(c: Context):
    c.require_cp_a()
    c.power_cycle('Unplug the Industrial Quad Relay 2.1 bricklet from its port.')
    c.settle(3.0)
    c.check(c.timeline.latest_with('relay_found').get('relay_found') is False, 'relay_found is false')
    c.check(c.wait_until(lambda x: x.get('pls') in (PLS_DEDICATION_NOT_VERIFIED,
                                                    PLS_DISABLED),
                         10.0, 'BRICKLET_DEDICATION_NOT_VERIFIED'),
            'FID 81 reports BRICKLET_DEDICATION_NOT_VERIFIED (or DISABLED)')


@step('7.6', 'no-hardware', 'Refit the Quad Relay bricklet')
def s76(c: Context):
    c.power_cycle('Plug the Industrial Quad Relay 2.1 bricklet back into its port.')
    c.settle(3.0)
    c.check(c.timeline.latest_with('relay_found').get('relay_found') is True, 'relay_found is true again')


# ---------------------------------------------------------------------------------
# Group 8 - must stay locked
# ---------------------------------------------------------------------------------

@step('8.1', 'stay-locked', 'Disabling while charging must be refused')
def s81(c: Context):
    c.require_cp_a()
    c.set_enabled(True)
    c.settle(1.5)
    c.guard_actuation_spacing()
    c.op.action('Hold the Unlock button, plug the TV 950 in, then release it.')
    c.check(c.wait_until(lambda x: x.get('pls') == PLS_LOCKED, ATTEMPT_TIMEOUT_S + 3.0,
                         'LOCKED'), 'locked')

    c.guard_cp_cycle_spacing()
    c.require_charging_possible()
    c.op.action('Press and HOLD the Charge button.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_C, 20.0, 'CP C'), 'charging')

    t0 = c.timeline.mark()
    r = c.set_enabled(False)
    c.settle(1.5)
    c.check(not r.ok or c.enabled_readback() is True,
            f'disable refused while charging (HTTP {r.status_code})')
    c.check_never(t0, 'relay', False, 'relay never released during the session')


@step('8.2', 'stay-locked', 'Still plugged in at CP B')
def s82(c: Context):
    c.op.action('Release the Charge button but leave the TV 950 plugged in.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_B, 15.0, 'CP B'), 'CP state B')

    r = c.set_enabled(False)
    c.settle(1.5)

    # Promoted from a soft check. It used to be soft because review finding #3 lived
    # here: disable_is_blocked() consulted plug_lock_car_detected(), which reports "car
    # present" whenever the CP reading cannot be trusted - and a CP disconnect is
    # routine, so the API recovery path could close with only a power cycle to get out.
    # The guard no longer consults the CP at all, so the finding is dissolved and the
    # behaviour is assertable.
    c.check(not r.ok or c.enabled_readback() is True,
            f'disable refused while a vehicle is connected (HTTP {r.status_code})')
    c.report['cp_b_disable_http'] = r.status_code


@step('8.3', 'stay-locked', 'Disable is still refused once unplugged')
def s83(c: Context):
    c.op.action('Hold the Unlock button and withdraw the TV 950.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_A, 20.0, 'CP A'), 'back to CP state A')
    c.check(c.wait_until(lambda x: x.get('relay') is False, UNLOCK_TIMEOUT_S + 3.0,
                         'relay open'), 'relay released')

    # Inverted. Unplugging is no longer what makes a disable acceptable - the plug being
    # gone says nothing about the next session, and this is exactly the state in which
    # the old guard let the interlock be removed for good. Only removing both Bricklets
    # does it, which group 10 covers.
    r = c.set_enabled(False)
    c.settle(1.5)
    c.check(not r.ok or c.enabled_readback() is True,
            f'disable still refused with an idle socket (HTTP {r.status_code})')
    c.check(c.enabled_readback() is True, 'the feature is still armed')


# ---------------------------------------------------------------------------------
# Group 9 - a broken loop faults, but must NOT open the disable path
#
# This group is the regression test for the defect that motivated the whole guard.
# A failed lock supply de-asserts the harness loop while both Bricklets keep their
# own supply from the Bricklet port and stay discoverable. S3 reproduces exactly that
# state. If a disable is ever accepted in here, a blown fuse is enough to strip the
# interlock from a wallbox with the enclosure shut.
# ---------------------------------------------------------------------------------

@step('9.1', 'loop-fault', 'Opening S3 faults the charger')
def s91(c: Context):
    # The counterpart to group 8: having proved the feature cannot be switched off over
    # the network, prove there *is* a way out and that it is safe.
    c.require_cp_a()
    c.ensure_enabled()
    c.expect_state(loop=LOOP_VERIFIED, pls=PLS_IDLE)

    t0 = c.timeline.mark()
    c.op.action('Open switch S3 LOOP LINK.')
    c.op.safe_gate('Confirm S3 LOOP LINK is OPEN.')

    c.check(c.wait_until(lambda x: x.get('loop') == LOOP_FAIL_OPEN,
                         LOOP_LOSS_BUDGET_S, 'FailOpen'),
            'the harness loop reports FailOpen')
    c.check(c.wait_until(lambda x: x.get('pls') == PLS_DEDICATION_NOT_VERIFIED,
                         PLUG_LOCK_REPORT_STALE_S + 2.0, 'BRICKLET_DEDICATION_NOT_VERIFIED'),
            'the EVSE reports BRICKLET_DEDICATION_NOT_VERIFIED')

    # Both bricklets are still there and still answering. That is the whole point: a
    # broken loop is not a missing device, and discovery cannot tell the difference.
    c.check(c.timeline.latest_with('relay_found').get('relay_found') is True, 'relay bricklet still found')
    c.check(c.timeline.latest_with('input_found').get('input_found') is True, 'input bricklet still found')
    c.check_never(t0, 'relay', True, 'lock relay never asserted with no plug')


@step('9.2', 'loop-fault', 'Charging is blocked while the loop is open')
def s92(c: Context):
    # Fail safe, not fail open - the same assertion as 7.3 but reached without pulling
    # a bricklet, and therefore the one that matters: this is the state a user actually
    # creates when they follow the documented removal procedure.
    c.guard_cp_cycle_spacing()
    c.require_charging_possible()
    c.op.action('Hold the Unlock button, plug the TV 950 in, release it, '
                'then press and HOLD the Charge button.')
    c.settle(5.0)

    c.check(not c.op.observe('Are any of the phase LEDs on?'),
            'phase LEDs stay OFF - the contactor did not close')
    c.check(c.timeline.latest_with('error_state').get('error_state') == ERROR_STATE_PLUG_LOCK,
            'EVSE reports error state 6 (Plug Lock)')

    c.op.action('Release the Charge button, hold Unlock and withdraw the TV 950.')

    # NOT cp == A, for the same reason as 7.3: 9.1 has just asserted
    # BRICKLET_DEDICATION_NOT_VERIFIED, which holds error state 6 up for as long as S3 is
    # open - through 9.5 - regardless of what is in the socket. That pins the EVSE in EF.
    #
    # Unlike 7.3 this has not been observed on hardware; it is inferred from 9.1's own
    # assertion putting the module in the identical state. A fault raised by a lock
    # attempt does clear on withdrawal and CP does return to A - 5.3 and 6.2 both did that
    # on 2026-09-14 - but this is not that kind of fault.
    c.check(c.wait_until(lambda x: x.get('lock_wanted') is False, 20.0, 'lock_wanted false'),
            'the EVSE sees the socket empty again')


@step('9.3', 'loop-fault', 'Disable is REFUSED with the loop open')
def s93(c: Context):
    # THE REGRESSION TEST. This step used to assert the opposite, and that was the
    # defect: the loop is de-asserted by any fault on the lock supply, not only by
    # someone deliberately disconnecting it, so accepting a disable here let a blown
    # fuse open a path that is meant to require someone at the enclosure. Repair the
    # supply afterwards and the socket energizes with a removable plug.
    #
    # If this check ever fails, stop. It means the guard is reading the loop again
    # instead of Bricklet discovery.
    #
    # NOT require_cp_a(), for the same reason 9.2 checks withdrawal on lock_wanted: 9.1's
    # BRICKLET_DEDICATION_NOT_VERIFIED pins the EVSE in EF until S3 closes at 9.5. A run on
    # 2026-09-15 tripped the CP guard here with the socket genuinely empty, which is the
    # hardware confirmation of what 9.2's comment had only inferred.
    c.require_socket_empty()
    c.expect_state(loop=LOOP_FAIL_OPEN)

    c.check(c.timeline.latest_with('relay_found').get('relay_found') is True, 'relay bricklet still found')
    c.check(c.timeline.latest_with('input_found').get('input_found') is True, 'input bricklet still found')

    r = c.set_enabled(False)
    c.settle(1.5)
    c.check(not r.ok or c.enabled_readback() is True,
            f'disable refused with a broken loop and both Bricklets fitted '
            f'(HTTP {r.status_code})')
    c.check(c.enabled_readback() is True, 'the feature is still armed')
    c.report['loop_open_disable_http'] = r.status_code


@step('9.4', 'loop-fault', 'Enabling is impossible while the loop is open')
def s94(c: Context):
    # The other half of the one-way door: having got out, you cannot get back in until
    # the harness is genuinely reconnected. Otherwise the escape hatch would double as
    # a way to arm the feature against hardware that is not there.
    # Reached with the feature still ON now that 9.3 no longer switches it off, so
    # this asserts the guard rather than the transition: an enable request must not be
    # able to re-arm anything while the loop is unproven, and must not disturb the
    # state that is already there.
    #
    # Same guard substitution as 9.3 - S3 is still open, so the EVSE is still pinned in EF.
    c.require_socket_empty()
    c.expect_state(loop=LOOP_FAIL_OPEN)

    was = c.enabled_readback()
    r = c.set_enabled(True)
    c.settle(1.5)
    c.check(c.enabled_readback() == was,
            f'enable is a no-op with the loop open (HTTP {r.status_code})')


@step('9.5', 'loop-fault', 'Closing S3 re-proves the loop')
def s95(c: Context):
    c.close_s3()

    # Not merely reconnected - re-proven. The firmware runs a full challenge as soon as
    # the level returns rather than waiting for the next interval, so reaching Verified
    # here means the release phase passed too.
    c.check(c.timeline.latest_with('loop').get('loop') == LOOP_VERIFIED,
            'the harness loop is Verified again')

    c.ensure_enabled()
    c.check(c.wait_until(lambda x: x.get('pls') == PLS_IDLE, 3.0, 'IDLE'),
            'FID 81 back to IDLE')

    # The fault clears on its own once the loop is proven again - nothing had to be
    # switched off to recover from a supply failure, which is the point.
    c.check(c.wait_until(lambda x: x.get('error_state') != ERROR_STATE_PLUG_LOCK,
                         EVSE_ERROR_COOLDOWN_S + 5.0, 'error cleared'),
            'the plug lock error clears once the loop is verified again')


# ---------------------------------------------------------------------------------
# Group 10 - the escape hatch
#
# The counterpart to groups 8 and 9: having proved the feature cannot be switched off
# by a session ending, nor by a broken loop, prove there IS a way out and that taking
# it needs someone at the enclosure with both hands.
# ---------------------------------------------------------------------------------

@step('10.1', 'bricklet-escape', 'One Bricklet gone is not enough')
def s101(c: Context):
    # Both, not either. One remaining Bricklet is a partly disassembled lock rather
    # than a removed one, and a single loose 7-pin cable must not open this path.
    c.require_cp_a()
    c.ensure_enabled()
    c.power_cycle('Unplug ONLY the Industrial Dual Analog In 2.0 Bricklet.')
    c.settle(3.0)

    c.check(c.timeline.latest_with('input_found').get('input_found') is False, 'input_found is false')
    c.check(c.timeline.latest_with('relay_found').get('relay_found') is True, 'relay_found is still true')

    r = c.set_enabled(False)
    c.settle(1.5)
    c.check(not r.ok or c.enabled_readback() is True,
            f'disable refused with one Bricklet still fitted (HTTP {r.status_code})')


@step('10.2', 'bricklet-escape', 'Both gone opens the door')
def s102(c: Context):
    c.power_cycle('Now unplug the Industrial Quad Relay 2.1 Bricklet as well, so that '
                  'BOTH are out.')
    c.settle(3.0)

    c.check(c.timeline.latest_with('relay_found').get('relay_found') is False, 'relay_found is false')
    c.check(c.timeline.latest_with('input_found').get('input_found') is False, 'input_found is false')

    # Safe precisely because charging is already blocked in this state - 7.3 and 9.2
    # both showed that - so accepting it can only clear a standing fault, never switch
    # off a lock that still works.
    r = c.set_enabled(False)
    c.settle(1.5)
    c.check(r.ok and c.enabled_readback() is False,
            f'disable accepted with both Bricklets gone (HTTP {r.status_code})')
    c.check(c.wait_until(lambda x: x.get('pls') == PLS_DISABLED, 3.0, 'DISABLED'),
            'FID 81 reports DISABLED')
    c.check(c.wait_until(lambda x: x.get('error_state') != ERROR_STATE_PLUG_LOCK,
                         EVSE_ERROR_COOLDOWN_S + 5.0, 'error cleared'),
            'the plug lock error clears once the feature is off')


@step('10.3', 'bricklet-escape', 'Enabling is impossible with both gone')
def s103(c: Context):
    # The other half of the one-way door: having got out, you cannot get back in until
    # the hardware is genuinely there again. Otherwise the escape hatch would double as
    # a way to arm the feature against hardware that is not fitted.
    c.require_cp_a()

    r = c.set_enabled(True)
    c.settle(1.5)
    c.check(not r.ok or c.enabled_readback() is False,
            f'enable refused with both Bricklets gone (HTTP {r.status_code})')


@step('10.4', 'bricklet-escape', 'Refit both and re-arm')
def s104(c: Context):
    c.refit_both_bricklets()

    c.check(c.timeline.latest_with('loop').get('loop') == LOOP_VERIFIED,
            'the harness loop is Verified again')

    r = c.set_enabled(True)
    c.settle(1.5)
    c.check(r.ok and c.enabled_readback() is True,
            f'enable accepted again (HTTP {r.status_code})')
    c.check(c.wait_until(lambda x: x.get('pls') == PLS_IDLE, 3.0, 'IDLE'),
            'FID 81 back to IDLE')


# ---------------------------------------------------------------------------------
# Group 11 - the lock supply actually fails
#
# The honest version of group 9. S3 breaks the loop jumper with the supply healthy;
# S4 kills the supply, which is what a blown fuse does, and takes the 12 V feedback
# and the motor with it. The disable gate must refuse in both, and that is the point:
# from the gate's side the two are indistinguishable, which is precisely why it cannot
# be allowed to read the loop.
#
# Safe with a plug locked, but NOT for the reason this group was written with. The
# control block runs a fail-safe release stroke when its supply disappears, so the plug
# comes free the moment S4 opens - under power, not by spring. See the file header. The
# contactor is blocked throughout, so a loose plug never sits in a live socket, and
# nothing is stranded for the operator to force.
#
# Restore the supply before withdrawing all the same - see the file header. LockFault
# holds relay channel 0 closed until lock_wanted goes false, so closing S4 re-locks the
# plug immediately, and the withdrawal in 11.3 is what releases it properly.
# ---------------------------------------------------------------------------------

@step('11.1', 'supply-loss', 'Kill the lock supply with the plug locked')
def s111(c: Context):
    c.require_cp_a()
    c.ensure_enabled()
    c.guard_actuation_spacing()
    c.op.action('Hold the Unlock button, plug the TV 950 in, then release it.')
    c.check(c.wait_until(lambda x: x.get('pls') == PLS_LOCKED, ATTEMPT_TIMEOUT_S + 3.0,
                         'LOCKED'), 'locked before the supply is cut')

    t0 = c.timeline.mark()
    c.op.note('The plug comes FREE the instant the supply goes, with an audible stroke - '
              'the control block runs a fail-safe release on its own stored energy. That '
              'is expected. Charging stays blocked the whole time, so the socket is never '
              'live - leave the plug where it is until 11.3.')
    c.op.action('Open switch S4 LOCK SUPPLY.')
    c.op.safe_gate('Confirm S4 LOCK SUPPLY is OPEN.')

    # All three loads die together, which is the difference from S3.
    c.check(c.wait_until(lambda x: x.get('lock_closed') is False, 3.0, 'lock_closed false'),
            'the 12 V feedback drops with the supply')
    c.check(c.wait_until(lambda x: x.get('loop') == LOOP_FAIL_OPEN,
                         LOOP_LOSS_BUDGET_S, 'FailOpen'),
            'the harness loop reports FailOpen')
    c.check(c.timeline.saw(t0, 'actuation', ACT_LOCKING),
            'actuation re-entered Locking (plug_lock.cpp:243-251)')
    c.check(c.wait_until(lambda x: x.get('actuation') == ACT_LOCK_FAULT,
                         LOCK_FAULT_BUDGET_S + 2.0, 'LockFault'),
            'and gives up in LockFault once the retries are spent')
    c.check(c.wait_until(lambda x: x.get('error_state') == ERROR_STATE_PLUG_LOCK,
                         PLUG_LOCK_REPORT_STALE_S + 3.0, 'error state 6'),
            'EVSE reports error state 6 (Plug Lock)')

    # Both bricklets keep their own supply from the bricklet port. This is the whole
    # premise of the guard, asserted here against a real supply failure rather than
    # against S3 standing in for one.
    c.check(c.timeline.latest_with('relay_found').get('relay_found') is True, 'relay bricklet still found')
    c.check(c.timeline.latest_with('input_found').get('input_found') is True, 'input bricklet still found')

    # The firmware's view and the hardware agree here: it reports the plug unlocked and
    # the plug really is. Asserted rather than merely recorded, because an actuator that
    # held position through a lost supply would be a different safety case - it would
    # strand a plug that the firmware has already given up on, and the file header, this
    # group and 11.3 would all need rewriting again.
    #
    # The moment of release is NOT what this observes. The stroke runs when the supply
    # goes, which is 30 s or so before the operator is asked, so a plug found free here
    # is consistent with the fail-safe having run at S4 and with nothing else. Watching
    # for the sound at S4 is what establishes the timing; this establishes the outcome.
    #
    # Unlock is NOT held for this, per the retention rule in the file header: holding it
    # would drop lock_wanted and release the plug through the firmware, which is the one
    # thing that would make the observation meaningless.
    retained = c.op.observe('Is the TV 950 still held in the socket? '
                            '(pull gently, do NOT force it)')
    c.check(not retained,
            'the plug is released by the supply loss - fail-safe stroke ran')
    c.report['actuator_released_by_supply_loss'] = not retained


@step('11.2', 'supply-loss', 'Disable is REFUSED after a real supply failure')
def s112(c: Context):
    # The same assertion as 9.3, reached the honest way. If 9.3 passes and this fails,
    # the gate is reading something that a broken jumper and a blown fuse do not share.
    r = c.set_enabled(False)
    c.settle(1.5)
    c.check(not r.ok or c.enabled_readback() is True,
            f'disable refused with the lock supply dead (HTTP {r.status_code})')
    c.check(c.enabled_readback() is True, 'the feature is still armed')


@step('11.3', 'supply-loss', 'Restore the supply, then withdraw')
def s113(c: Context):
    # Order matters, though not for the reason this step was written with. Release has
    # already happened - the control block's fail-safe stroke ran when S4 opened and 11.1
    # left the plug free. The supply goes back first because the loop has to be seen
    # Verified again while the fault is still standing, which is what proves the recovery
    # is the supply returning rather than the session ending.
    #
    # Closing S4 re-locks the plug on the spot: LockFault holds relay channel 0 commanded
    # closed until lock_wanted goes false. The withdrawal below, with Unlock held, is what
    # drops it and releases the plug properly.
    c.op.action('Close switch S4 LOCK SUPPLY.')
    c.op.safe_gate('Confirm S4 LOCK SUPPLY is CLOSED.')

    c.check(c.wait_until(lambda x: x.get('loop') == LOOP_VERIFIED,
                         LOOP_RECOVERY_BUDGET_S, 'Verified'),
            'the harness loop is Verified again')

    # LockFault holds until lock_wanted goes false, so the plug has to come out before
    # the charger is idle again - the supply returning is not enough on its own.
    c.op.action('Hold the Unlock button and withdraw the TV 950.')
    c.check(c.wait_until(lambda x: x.get('cp') == CP_A, 20.0, 'CP A'), 'back to CP state A')
    c.check(c.wait_until(lambda x: x.get('error_state') != ERROR_STATE_PLUG_LOCK,
                         EVSE_ERROR_COOLDOWN_S + 5.0, 'error cleared'),
            'the plug lock error clears once the session ends')
    c.guard_actuation_budget()


# ---------------------------------------------------------------------------------
# Teardown
# ---------------------------------------------------------------------------------

def teardown(c: Context) -> bool:
    """Always runs. Returns True only if the charger is verifiably restored."""
    print('\n=== teardown ===')
    ok = True

    # Restoring `enabled` is no longer a single API call. Turning the feature ON is,
    # provided the loop is verified; turning it OFF is refused for exactly as long as
    # either Bricklet is still found, so it needs both of them out. Getting this wrong
    # would end every otherwise-clean run in exit 2, so both directions are handled
    # explicitly rather than by writing the flag and hoping.
    try:
        want = bool(c.snapshot.get('config', {}).get('enabled', False))
        got = c.enabled_readback()

        if got == want:
            print(f'          plug lock enabled already {want}, nothing to restore')
        elif want:
            c.ensure_enabled()
            got = c.enabled_readback()
            print(f'          plug lock enabled restored to {want} (read back {got})')
        else:
            print('          the run armed the plug lock and the restore point is off.')
            print('          The EVSE refuses that until neither Bricklet is found, so')
            print('          this needs both of them unplugged and refitted.')
            c.ensure_disabled()
            got = c.enabled_readback()
            print(f'          plug lock enabled restored to {want} (read back {got})')

        ok = ok and (got == want)
    except Exception as e:                                          # noqa: BLE001
        print(f'          FAILED to restore config: {e}')
        ok = False

    # Put user authorization back before anything else that can fail, and report the
    # outcome either way. This is the only setting the run changes that makes the charger
    # LESS secure, so it must never be left to a later step's success.
    if c.user_auth_disabled_by_run:
        want_auth = bool(c.snapshot.get('user_enabled', {}).get('enabled', True))
        try:
            c.charger.api_put('evse/user_enabled_update', {'enabled': want_auth})
            time.sleep(1.0)
            got_auth = bool(c.charger.api_get('evse/user_enabled').get('enabled'))

            if got_auth == want_auth:
                print(f'          user authorization restored to {want_auth}')
            else:
                print(f'          *** user authorization is {got_auth}, wanted {want_auth}')
                ok = False

            c.report['user_auth_restored'] = got_auth == want_auth
        except Exception as e:                                      # noqa: BLE001
            print(f'          *** FAILED to restore user authorization: {e}')
            print('          *** THE CHARGER MAY START A SESSION WITHOUT AUTHORIZATION.')
            print('          *** Switch "Authorization required" back on by hand.')
            c.report['user_auth_restored'] = False
            ok = False

    for key, label in (('relay_found', 'Quad Relay'), ('input_found', 'Dual Analog In')):
        if c.timeline.latest_with(key).get(key) is not True:
            print(f'          {label} is NOT detected')
            ok = False

    # The loop is production wiring, not a fixture: leaving it open would leave the
    # wallbox unable to arm its plug lock at all. Verified, not merely closed.
    restore_loop = c.timeline.latest_with('loop').get('loop')
    if restore_loop != LOOP_VERIFIED:
        print(f'          harness loop is {LOOP_NAME.get(restore_loop, restore_loop)}, '
              f'not Verified')
        ok = False

    # Printed on every run, restored or not, touched or not. A state this consequential
    # should not have to be inferred from the absence of an error message.
    try:
        auth_now = bool(c.charger.api_get('evse/user_enabled').get('enabled'))
        print(f'\n          user authorization is currently: '
              f'{"ON" if auth_now else "OFF"}')
        if not auth_now:
            print('          *** THIS CHARGER WILL START A SESSION WITHOUT A USER. ***')
    except Exception as e:                                          # noqa: BLE001
        print(f'\n          could not read user authorization state: {e}')
        ok = False

    print('\n  RESTORE CHECKLIST')
    print('    [ ] S1 RELAY OUT      closed')
    print('    [ ] S2 LOCK FEEDBACK  closed')
    print('    [ ] S3 LOOP LINK      closed')
    print('    [ ] S4 LOCK SUPPLY    closed')
    print('    [ ] Industrial Quad Relay 2.1   fitted')
    print('    [ ] Industrial Dual Analog In 2.0 fitted')
    print('    [ ] TV 950 removed from the socket')
    print('    [ ] wallbox energized and reachable')
    print('    [ ] "Authorization required" back ON, if this run switched it off')
    print('')
    print('    After the test is finished for good, remove S1, S2 and S4 and')
    print('    restore those wires to plain continuous connections. S3 sits in the')
    print('    harness loop, which IS production wiring - leave that loop intact,')
    print('    with or without the switch.')

    try:
        answer = input('\n  Confirm every line above is done [yes] > ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        answer = ''

    if answer != 'yes':
        print('          checklist NOT confirmed')
        ok = False

    return ok


# ---------------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(description='Guided plug lock safety test')
    p.add_argument('--host', help='charger origin including the scheme, '
                                  'e.g. https://warp4-abcd')
    p.add_argument('--insecure', action='store_true',
                   help='do not verify the TLS certificate; needed for an https '
                        '--host given as an IP address, because the charger '
                        'certificate is self-signed for its own hostname')
    p.add_argument('--port', type=int, default=4223, help='Tinkerforge proxy port')
    p.add_argument('--user', help='HTTP digest user, if authentication is on')
    p.add_argument('--password', help='HTTP digest password')
    p.add_argument('--from-step', dest='from_step',
                   help=f'resume at this step id. Group {PREFLIGHT_GROUP} runs first '
                        'either way; naming a step in it resumes nowhere and is ignored')
    p.add_argument('--only-group', dest='only_group', help='run one group only')
    p.add_argument('--skip-mechanical', action='store_true',
                   help='skip step 4.2, the only step applying force to the lock')
    p.add_argument('--skip-actuator-stress', dest='skip_actuator_stress',
                   action='store_true',
                   help='skip group 6, the only steps that drive a live actuator with '
                        'the feedback cut; group 11 covers the same firmware branch')
    p.add_argument('--report', help='path for the JSON report')
    p.add_argument('--list', action='store_true', help='list the steps and exit')
    args = p.parse_args()

    if args.list:
        for s in STEPS:
            print(f'  {s.id:<5} [{s.group}] {s.title}')
        return 0

    if not args.host:
        p.error('--host is required')

    if args.from_step:
        target = next((s for s in STEPS if s.id == args.from_step), None)

        # Previously an id that matches nothing skipped every step and reported a
        # completed run of zero steps. With preflight now exempt from the filter it
        # would report a completed run of seven passing ones, which reads like a
        # result. Say so instead.
        if target is None:
            p.error(f'--from-step {args.from_step} matches no step; '
                    'use --list to see the ids')

        # Nothing to resume at: preflight runs first whatever is asked for, so the
        # only thing honouring this could do is skip the earlier preflight steps -
        # exactly what the exemption exists to prevent.
        if target.group == PREFLIGHT_GROUP:
            print(f'\n  --from-step {args.from_step} names a {PREFLIGHT_GROUP} step. '
                  'Ignored; running the whole script.')
            args.from_step = None

    print('=' * 76)
    print('  Plug lock safety test')
    print('  Requires: Testboy TV 950, switches S1 RELAY OUT, S2 LOCK FEEDBACK,')
    print('            S3 LOOP LINK, S4 LOCK SUPPLY')
    print('  This exercises live mains. Phases are energized whenever Charge is held.')
    print('=' * 76)

    report: dict[str, Any] = {
        'host': args.host,
        'started': time.time(),
        'checks': [],
        'prompts': [],
        'steps': [],
    }

    if args.insecure:
        # Otherwise every one of the several thousand requests a run makes prints an
        # InsecureRequestWarning, which would bury the operator prompts.
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

    # Before the recorder exists and before teardown is armed, so a malformed --host or
    # an unreachable charger - by far the likeliest first mistakes - are reported here
    # rather than left to unwind as a traceback. Nothing has been touched yet, so there
    # is nothing to restore. Charger() itself validates --host, hence inside the try.
    try:
        charger = Charger(args.host, args.port, args.user, args.password,
                          insecure=args.insecure)
        charger.connect()
    except Abort as e:
        print(f'\n  CANNOT CONNECT: {e}')
        return 1

    timeline = Timeline()
    recorder = Recorder(charger, timeline)
    recorder.start()

    # Wait for a row, not for a duration. SAMPLE_INTERVAL_S is a floor and one sample() is
    # about 1.1 s, and Timeline.append() happens when the sample FINISHES - so a flat 1 s
    # sleep can leave the timeline empty. A step opening with require_cp_a() or
    # expect_state() then reads cp as None off latest_with() and trips the guard. Group 0
    # hides this because 0.1 spends its time on discovery without querying the timeline;
    # --from-step 4.1 goes straight at it and failed that way on 2026-09-14. Raising the
    # sample rate cannot help: the recorder already runs flat out.
    deadline = time.monotonic() + 10.0
    while not timeline.rows and time.monotonic() < deadline:
        time.sleep(0.1)

    if not timeline.rows:
        print('\n  CANNOT SAMPLE: the recorder produced no row in 10 s')
        recorder.stop_event.set()
        recorder.join(timeout=5.0)
        charger.close()
        return 1

    ctx = Context(charger=charger, timeline=timeline,
                  op=Operator(report['prompts']), report=report,
                  skip_mechanical=args.skip_mechanical,
                  skip_actuator_stress=args.skip_actuator_stress)

    interrupted = threading.Event()

    def on_sigint(_sig, _frm):
        interrupted.set()
        raise Abort('interrupted')

    signal.signal(signal.SIGINT, on_sigint)

    started = False
    outcome = 'completed'

    try:
        for s in STEPS:
            if args.only_group and s.group != args.only_group:
                continue
            # Exempt from --from-step, never from --only-group: asking for one group
            # is asking for that group, and preflight is itself one of them.
            #
            # `started` deliberately stays False here. Preflight comes first in STEPS,
            # so running it must not be taken for having reached the resume point -
            # the filter below still has to find the named step afterwards.
            if args.from_step and not started and s.group != PREFLIGHT_GROUP:
                if s.id != args.from_step:
                    continue
                started = True

            print(f'\n--- {s.id}  {s.title}  [{s.group}]')
            t0 = timeline.mark()
            try:
                s.fn(ctx)
                report['steps'].append({'id': s.id, 'result': 'pass'})
            except StepFailed as e:
                report['steps'].append({'id': s.id, 'result': 'fail', 'reason': str(e)})
                print(f'\n  STEP {s.id} FAILED: {e}')
                outcome = 'failed'
                break
            finally:
                report.setdefault('windows', {})[s.id] = [t0, timeline.mark()]

    except GuardTripped as e:
        print(f'\n  GUARD TRIPPED: {e}')
        outcome = 'guard'
    except Abort as e:
        print(f'\n  ABORTED: {e}')
        outcome = 'aborted'
    except Exception:                                               # noqa: BLE001
        traceback.print_exc()
        outcome = 'error'
    finally:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        restored = teardown(ctx)
        recorder.stop_event.set()
        recorder.join(timeout=5.0)
        charger.close()

        report['outcome'] = outcome
        report['restored'] = restored
        report['finished'] = time.time()
        report['timeline'] = timeline.rows

        path = args.report or f'plug_lock_safety_{int(report["started"])}.json'
        with open(path, 'w') as f:
            json.dump(report, f, indent=1)
        print(f'\n  report: {path}  ({len(timeline.rows)} samples)')

    failed = [c for c in report['checks'] if not c.get('soft') and c.get('ok') is False]
    print(f'\n  outcome: {outcome}, {len(failed)} failed checks')

    if not restored:
        print('\n  *** UNSAFE STATE - the charger was not verifiably restored. ***')
        return 2

    return 0 if outcome == 'completed' and not failed else 1


if __name__ == '__main__':
    sys.exit(main())

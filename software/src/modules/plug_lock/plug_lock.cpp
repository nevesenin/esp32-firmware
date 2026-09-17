/* esp32-firmware
 * Copyright (C) 2026 empunkt <empunkt@mailbox.org>
 *
 * plug_lock.cpp: Type 2 plug lock, ESP32 half
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "plug_lock.h"

#include "event_log_prefix.h"
#include "generated/module_dependencies.h"

#include "generated/plug_lock_state.enum.h"
#include "generated/plug_lock_loop.enum.h"

#include "gcc_warnings.h"

// The timing budget, all of it constrained by the EVSE's 10 s backstop:
//
//   250 ms poll latency + 3 s x 2 attempts + 250 ms report ~ 6.5 s
//
// against 10 s leaves 3.5 s of margin. The actuator's measured lock cycle is under
// 2 s, so 3 s per attempt is 50 % margin for an actuator stiffened by cold, wear or a
// tight plug. Do not raise the per-attempt timeout without raising the EVSE backstop:
// 5 s x 2 would exceed it, the EVSE would fault first and the specific cause would be
// lost.
static constexpr millis_t PLUG_LOCK_UPDATE_INTERVAL = 250_ms;
static constexpr micros_t PLUG_LOCK_ATTEMPT_TIMEOUT = 3_s;
static constexpr micros_t PLUG_LOCK_UNLOCK_TIMEOUT  = 3_s;
static constexpr uint8_t  PLUG_LOCK_ATTEMPTS        = 2;

// Two consecutive high reads is 500 ms, a quarter of the measured lock cycle and well
// inside the per-attempt timeout.
static constexpr uint8_t PLUG_LOCK_FEEDBACK_HIGH_READS = 2;

// Failed reads tolerated before the feedback is treated as gone.
static constexpr uint8_t PLUG_LOCK_FEEDBACK_READ_FAILURES = 1;

// Harness loop timing. See plug_lock.h for what the loop is and why it exists.
//
// The steady-state debounce is asymmetric the other way round from the feedback
// contact: 500 ms to call the loop good, 1 s to call it lost.
static constexpr uint8_t PLUG_LOCK_LOOP_HIGH_READS = 2;
static constexpr uint8_t PLUG_LOCK_LOOP_LOW_READS  = 4;

// One read discarded after the relay is restored. The CPC1002N switches in microseconds
// and a tick is 250 ms, so this is not about the relay - it is about not turning a
// single unlucky sample into a verdict. Only the Restore phase uses it; Release waits
// for the level it needs instead of discarding a fixed number of reads.
static constexpr uint8_t PLUG_LOCK_LOOP_SETTLE_READS = 1;

// Consecutive reads required in each challenge phase.
static constexpr uint8_t PLUG_LOCK_LOOP_RELEASE_READS = 2;
static constexpr uint8_t PLUG_LOCK_LOOP_RESTORE_READS = 2;

// Total reads *either* phase may take before it is called failed. Four spare reads over
// the minimum, so neither a glitched sample nor a slow one fails an intact loop. Worst
// case a challenge now runs 12 reads, i.e. 3 s.
static constexpr uint8_t PLUG_LOCK_LOOP_CHALLENGE_MAX_READS = 6;

// How often a verified loop is re-proven. The steady-state high is monitored
// continuously; this is only about periodically re-running the release phase, which is
// the half that a 12 V strap across the input cannot survive. Rare on purpose: it
// briefly opens the loop, and there is nothing to gain from doing that often.
static constexpr micros_t PLUG_LOCK_LOOP_CHALLENGE_INTERVAL = 900_s; // 15 minutes

// How long we may stay silent towards the EVSE while waiting for a first verdict.
// Silence is the right answer for a moment - it leaves the EVSE in its existing "no
// fresh assertion" state instead of actively claiming the hardware vanished, which
// matters when the charger boots with a plug already locked - but it must not last.
static constexpr micros_t PLUG_LOCK_LOOP_VERDICT_TIMEOUT = 5_s;

// How long this instance may keep telling the EVSE that it has not settled yet. The
// claim is only there to stop the first seconds of a start-up being reported as a
// failure, and "not settled" is supposed to end when a verdict exists - so a deadline
// catches the case where something goes wrong before one ever does. Measured start-ups
// reach a verdict about 2.2 s after the Bricklets are found; this is an outer bound, not
// a budget. The EVSE bounds the claim independently, which is what actually makes it
// safe; this only stops us making a claim we know to be stale.
//
// Must stay SHORTER than the EVSE's PLUG_LOCK_CLAIM_TIMEOUT_MS (30 s, plug_lock.h over
// there), so the side making the claim always gives up before the side believing it does.
// Inverted, the EVSE would fault on an instance that was about to stop claiming anyway.
static constexpr micros_t PLUG_LOCK_STARTUP_SETTLE_TIMEOUT = 20_s;

void PlugLock::pre_setup()
{
    config = Config::Object({
        {"enabled", Config::Bool(false)},
    });

    config_update = Config::Object({
        {"enabled", Config::Bool(false)},
    });

    state = Config::Object({
        {"state", Config::Enum(PlugLockState::Disabled)},
        {"lock_wanted", Config::Bool(false)},
        {"actuation", Config::Enum(PlugLockActuation::Inactive)},
        {"lock_closed", Config::Bool(false)},
        {"relay_found", Config::Bool(false)},
        {"input_found", Config::Bool(false)},
        // The harness loop verdict. relay_found and input_found stay, but they are now
        // diagnostics only: this is what the EVSE is told and what arms the feature.
        {"loop", Config::Enum(PlugLockLoop::Unknown)},
        // The millivolts behind lock_closed and the loop verdict, and whether each
        // landed inside a band this wiring should be able to produce. Diagnostics
        // only. They are here because the feedback line was debugged for weeks with
        // no way to see it, and two numbers would have shortened that a great deal.
        {"feedback_voltage", Config::Int32(0)},
        {"feedback_plausible", Config::Bool(false)},
        {"loop_voltage", Config::Int32(0)},
        {"loop_plausible", Config::Bool(false)},
    });
}

void PlugLock::setup()
{
    initialized = true;
}

void PlugLock::register_urls()
{
    // plug_lock/config is a read-only mirror: the EVSE owns the setting and persists
    // it, we only show what it reports back.
    api.addState("plug_lock/config", &config);
    api.addState("plug_lock/state", &state);

    api.addCommand("plug_lock/config_update", &config_update, {}, [this](Language /*language*/, String &errmsg) {
        const bool enabled = config_update.get("enabled")->asBool();

        if (!evse_supported) {
            errmsg = "The charge controller firmware does not support the plug lock";
            return;
        }

        // Defense in depth. The EVSE rejects this too, and its rejection is the one
        // that matters, because it holds the interlock and must not take our word for
        // its own preconditions. Checking here as well only buys a message the user
        // can act on - and the loop verdict is the one thing we know and it does not,
        // so the messages here can name the actual defect.
        if (enabled && loop_verdict != PlugLockLoop::Verified) {
            if (loop_verdict == PlugLockLoop::BrickletsNotFound)
                errmsg = "Plug lock hardware not found";
            else if (loop_verdict == PlugLockLoop::FailStuck)
                errmsg = "The plug lock harness loop does not open. Check the polarity of relay channel 1.";
            else
                errmsg = "The plug lock harness loop is not verified. Check the link between relay channel 1, input channel 1 and the lock supply.";

            return;
        }

        const int rc = evse_v2.set_plug_lock_configuration(enabled);

        if (rc == TF_E_INVALID_PARAMETER) {
            // The EVSE refuses to enable the plug lock while the contactor is closed,
            // and to disable it while it still has to hold the plug. The hardware
            // precondition for enabling is checked above, so a rejection here is
            // almost always about the contactor.
            errmsg = enabled ? "Stop the charging session before enabling the plug lock"
                             : "The plug lock cannot be switched off while its Bricklets are still connected. Unplug both of them inside the enclosure.";
            return;
        }

        if (rc != TF_E_OK) {
            errmsg = "Failed to write plug lock configuration to EVSE";
            return;
        }

        config_read = update_config_from_bricklet();
    }, false);

    // Re-proves the loop now instead of at the next 15 minute mark. Exists for
    // commissioning and for the guided hardware test: after correcting the wiring you
    // want an answer at once, not after a quarter of an hour.
    api.addCommand("plug_lock/loop_challenge", Config::Null(), {}, [this](Language /*language*/, String &errmsg) {
        if (!bricklets_found) {
            errmsg = "Plug lock hardware not found";
            return;
        }

        if (!challenge_allowed()) {
            errmsg = "The plug lock is actuating. Try again once it has settled.";
            return;
        }

        challenge_requested = true;
    }, true);

    // From here rather than from the first discovery: what the EVSE is being told is
    // "this instance has not had its chance yet", and the chance starts when we do.
    startup_deadline = now_us() + PLUG_LOCK_STARTUP_SETTLE_TIMEOUT;

    task_scheduler.scheduleUncancelable([this]() {this->update();}, PLUG_LOCK_UPDATE_INTERVAL);
}

void PlugLock::pre_reboot()
{
    if (!evse_supported)
        return;

    // Tell the EVSE we are going on purpose. It releases the contactor by selecting
    // IEC 61851 state B, with the plug still connected and the PWM still running -
    // instead of noticing the silence five seconds later and faulting over something
    // that is not a failure.
    //
    // INTERRUPTS CHARGING, DOES NOT END THE SESSION. Nothing records that a session was
    // stopped: the charging slots are untouched, and the returning instance clears the
    // flag within a few seconds. With the car still at state C, charging then resumes on
    // its own. That is ordinary charger behaviour and is not what this is for.
    //
    // What it is for is the invariant: "contactor open" becomes true of every deliberate
    // absence, which makes a contactor found closed with no assertion an unambiguous
    // anomaly rather than the normal outcome of a restart.
    //
    // This can only ever make the charger more restrictive, which is what makes it safe
    // to send at all: a crash, a panic, a power cut or /force_reboot (which unregisters
    // the whole pre-reboot hook, main.cpp) simply never gets here, and the EVSE falls
    // through to its ordinary staleness handling. There is nothing to be gained by
    // withholding it, so nothing has to be trusted.
    shutting_down = true;

    // ORDERING DEPENDENCY, and it fails silently if it ever breaks. IoScheduler::await()
    // refuses every request once its own pre_reboot() has run, and main.cpp walks the
    // modules in REVERSE registration order. This works only because PlugLock is
    // registered after IoScheduler in the generated modules.cpp, so it is torn down
    // first and the channel is still open here. If that order ever inverts, the call
    // below returns TF_E_TIMEOUT and the announcement quietly stops happening - hence
    // the log rather than a bare discarded return code.
    // The published values rather than a fresh is_found() pair: this has to report what
    // the module last concluded, the same thing update() reported, not a discovery answer
    // taken at a different instant on the way out.
    const int rc = evse_v2.set_plug_lock_hardware_state(loop_verdict == PlugLockLoop::Verified,
                                                        !state.get("relay_found")->asBool() && !state.get("input_found")->asBool(),
                                                        lock_closed,
                                                        actuation == PlugLockActuation::LockFault,
                                                        true,
                                                        !startup_settled);

    if (rc != TF_E_OK)
        logger.printfln("Could not tell the charge controller about the reboot (error %d). It will fall back to its staleness handling.", rc);
}

void PlugLock::read_feedback()
{
    PlugLockInputReading reading;

    if (plug_lock_input.get_reading(&reading) != TF_E_OK) {
        // The harness loop rides on this same call, so it has nothing to say either.
        // read_loop() holds its counters rather than counting a failure as a low.
        loop_raw_valid = false;

        // A failed read is not the same as a low read. A low read means the plug is
        // not locked and is acted on at once; a failed read means we do not know. A
        // single transient failure - a busy IO scheduler, a bricklet reset - must not
        // drop a charging session, so allow one before giving up. Two cycles is
        // 500 ms, still far inside the EVSE's 10 s backstop, and the plug cannot
        // physically come out in that time.
        if (feedback_read_failures < PLUG_LOCK_FEEDBACK_READ_FAILURES) {
            ++feedback_read_failures;
            return;
        }

        feedback_high_count = 0;
        lock_closed = false;
        return;
    }

    // Channel 1 comes free with the channel 0 read. Stashed before the early returns
    // below so that the loop is served on every successful call, whatever the feedback
    // contact happens to be doing.
    loop_raw = reading.level[PLUG_LOCK_INPUT_LOOP_CHANNEL];
    loop_raw_valid = true;

    // Diagnostics, published but never branched on. Left at their previous values on a
    // failed read, for the same reason loop_raw is: a failure says nothing, and stale
    // numbers next to a read failure are less misleading than zeroes.
    feedback_mv = reading.voltage[PLUG_LOCK_INPUT_CHANNEL];
    loop_mv = reading.voltage[PLUG_LOCK_INPUT_LOOP_CHANNEL];
    feedback_plausible = reading.plausible[PLUG_LOCK_INPUT_CHANNEL];
    loop_plausible = reading.plausible[PLUG_LOCK_INPUT_LOOP_CHANNEL];

    feedback_read_failures = 0;

    if (feedback_reads < PLUG_LOCK_FEEDBACK_HIGH_READS)
        ++feedback_reads;

    if (!reading.level[PLUG_LOCK_INPUT_CHANNEL]) {
        feedback_high_count = 0;
        lock_closed = false;
        return;
    }

    if (feedback_high_count < PLUG_LOCK_FEEDBACK_HIGH_READS)
        ++feedback_high_count;

    if (feedback_high_count >= PLUG_LOCK_FEEDBACK_HIGH_READS)
        lock_closed = true;
}

void PlugLock::set_actuation(PlugLockActuation next)
{
    if (actuation == next)
        return;

    actuation = next;
    state.get("actuation")->updateEnum(next);
}

void PlugLock::start_lock_attempt()
{
    relay_locked = true;
    retry_release = false;
    attempt_deadline = now_us() + PLUG_LOCK_ATTEMPT_TIMEOUT;
    ++attempts;
}

void PlugLock::advance(bool lock_wanted)
{
    switch (actuation) {
    case PlugLockActuation::Inactive:
        // Recovery after a reboot or after the bricklets came back. Reconcile against
        // what the feedback says, never against anything we remembered: the plug may
        // have been locked before we restarted, and dropping the relay to find out
        // would release a plug that might be live.
        //
        // Which is why this waits for the debounce to fill. lock_closed is false until
        // two high reads have arrived, so deciding on the first tick would always read
        // an already-locked plug as unlocked and command it open. update() leaves the
        // relay alone entirely while we are still in this state.
        if (feedback_reads < PLUG_LOCK_FEEDBACK_HIGH_READS)
            break;

        relay_locked = lock_closed;
        attempts = 0;
        set_actuation(lock_closed ? PlugLockActuation::Locked : PlugLockActuation::Unlocked);
        break;

    case PlugLockActuation::Unlocked:
        relay_locked = false;

        if (lock_wanted) {
            attempts = 0;
            start_lock_attempt();
            set_actuation(PlugLockActuation::Locking);
        }
        break;

    case PlugLockActuation::Locking:
        if (!lock_wanted) {
            relay_locked = false;
            attempt_deadline = now_us() + PLUG_LOCK_UNLOCK_TIMEOUT;
            set_actuation(PlugLockActuation::Unlocking);
            break;
        }

        if (lock_closed) {
            retry_release = false;
            set_actuation(PlugLockActuation::Locked);
            break;
        }

        if (retry_release) {
            start_lock_attempt();
            break;
        }

        if (deadline_elapsed(attempt_deadline)) {
            if (attempts < PLUG_LOCK_ATTEMPTS) {
                logger.printfln("Plug lock did not close on attempt %u, retrying", static_cast<unsigned>(attempts));
                relay_locked = false;
                retry_release = true;
                break;
            }

            logger.printfln("Plug lock did not close after %u attempts. Blocking charging.", static_cast<unsigned>(attempts));
            set_actuation(PlugLockActuation::LockFault);
        }
        break;

    case PlugLockActuation::Locked:
        relay_locked = true;

        if (!lock_wanted) {
            relay_locked = false;
            attempt_deadline = now_us() + PLUG_LOCK_UNLOCK_TIMEOUT;
            set_actuation(PlugLockActuation::Unlocking);
            break;
        }

        if (!lock_closed) {
            // The plug came unlocked while it should be held. Reported immediately -
            // the EVSE turns this into a fault if the contactor is closed.
            logger.printfln("Plug lock feedback lost while the plug should be locked");
            attempts = 0;
            start_lock_attempt();
            set_actuation(PlugLockActuation::Locking);
        }
        break;

    case PlugLockActuation::Unlocking:
        relay_locked = false;

        if (lock_wanted) {
            attempts = 0;
            start_lock_attempt();
            set_actuation(PlugLockActuation::Locking);
            break;
        }

        if (!lock_closed) {
            set_actuation(PlugLockActuation::Unlocked);
            break;
        }

        if (deadline_elapsed(attempt_deadline)) {
            // Stuck locked. That is the safe state, so this is logged and reported but
            // never sent to the EVSE as a lock fault: charging is over anyway, and
            // blocking the next session would punish the user twice.
            logger.printfln("Plug lock did not release. Use the mechanical emergency release.");
            set_actuation(PlugLockActuation::UnlockFault);
        }
        break;

    case PlugLockActuation::LockFault:
        // Hold the command. The plug is not confirmed locked, so the EVSE is refusing
        // to energize; releasing the relay here would only make a plug that might yet
        // engage certainly not engage. Cleared when the vehicle is disconnected.
        relay_locked = true;

        if (!lock_wanted) {
            relay_locked = false;
            attempts = 0;
            attempt_deadline = now_us() + PLUG_LOCK_UNLOCK_TIMEOUT;
            set_actuation(PlugLockActuation::Unlocking);
        }
        break;

    case PlugLockActuation::UnlockFault:
        relay_locked = false;

        if (lock_wanted) {
            attempts = 0;
            start_lock_attempt();
            set_actuation(PlugLockActuation::Locking);
            break;
        }

        if (!lock_closed) {
            // It let go eventually.
            logger.printfln("Plug lock released");
            set_actuation(PlugLockActuation::Unlocked);
        }
        break;

    default:
        // Every value above is handled; this only catches a corrupted state. Go back
        // through Inactive, which re-derives everything from the feedback contact
        // rather than from anything remembered.
        set_actuation(PlugLockActuation::Inactive);
        break;
    }
}

bool PlugLock::challenge_allowed() const
{
    // Everything except the two states in which the actuator is actually being driven.
    //
    // A challenge DOES disturb the feedback reading, and measurement 2026-09-12 settled
    // by how much: releasing the loop relay takes an engaged plug from 7217 mV to
    // 5596 mV. The two input channels share the bricklet's analog ground through its
    // bias network rather than being optocoupled apart, and differential rejection does
    // not save us here because LOCK is driven through such a large series resistance
    // that the channel follows that ground rather than its own source. The feedback
    // bands in plug_lock_input.h are set to hold the same verdict across both, which is
    // what makes a challenge safe to run while the plug is locked - not isolation.
    //
    // Excluding Locking and Unlocking is therefore not about electrical interference.
    // It is about keeping a failed lock attempt down to one possible cause.
    //
    // Inactive is deliberately allowed. A charger that boots with a plug already locked
    // stays in Inactive until the feedback debounce fills, and it still has to be able
    // to reach a verdict; excluding it would leave exactly that wallbox unable to prove
    // a perfectly good harness.
    return actuation != PlugLockActuation::Locking && actuation != PlugLockActuation::Unlocking;
}

void PlugLock::reset_loop()
{
    loop_high_count = 0;
    loop_low_count  = 0;
    loop_raw        = false;
    loop_raw_valid  = false;
    challenge_phase = ChallengePhase::None;
    challenge_reads = 0;
    challenge_highs = 0;
    challenge_lows  = 0;
    loop_relay_closed = true;
}

void PlugLock::start_challenge()
{
    challenge_phase     = ChallengePhase::Release;
    challenge_reads     = 0;
    challenge_highs     = 0;
    challenge_lows      = 0;
    challenge_requested = false;

    // Release first, then restore, so the loop spends the least possible time open.
    loop_relay_closed = false;

    next_challenge = now_us() + PLUG_LOCK_LOOP_CHALLENGE_INTERVAL;
}

void PlugLock::finish_challenge(PlugLockLoop verdict)
{
    challenge_phase = ChallengePhase::None;
    challenge_reads = 0;
    challenge_highs = 0;
    challenge_lows  = 0;
    loop_relay_closed = true;

    // The steady-state debounce was suspended for the duration of the challenge. Start
    // it from scratch rather than from counters that describe a deliberately toggled
    // loop.
    loop_high_count = 0;
    loop_low_count  = 0;

    if (loop_verdict == verdict)
        return;

    switch (verdict) {
    case PlugLockLoop::Verified:
        logger.printfln("Plug lock harness loop verified");
        break;

    case PlugLockLoop::FailStuck:
        logger.printfln("Plug lock harness loop stays closed with the relay released. Check the polarity of relay channel 1.");
        break;

    case PlugLockLoop::FailOpen:
        logger.printfln("Plug lock harness loop did not close. Check the link between relay channel 1, input channel 1 and the lock supply.");
        break;

    case PlugLockLoop::Unknown:
    case PlugLockLoop::Challenging:
    case PlugLockLoop::BrickletsNotFound:
    default:
        // Neither is a verdict. A challenge only ever ends in one of the three above,
        // so this is unreachable; it is listed so that adding a value to the enum is a
        // compile error here rather than a silent fall-through.
        return;
    }

    loop_verdict = verdict;
}

void PlugLock::read_loop()
{
    // A failed read is not a low read, and the loop is even less urgent than the
    // feedback contact: hold the counters and try again next cycle.
    if (!loop_raw_valid)
        return;

    if (loop_raw) {
        loop_low_count = 0;

        if (loop_high_count < PLUG_LOCK_LOOP_HIGH_READS)
            ++loop_high_count;
    } else {
        loop_high_count = 0;

        if (loop_low_count < PLUG_LOCK_LOOP_LOW_READS)
            ++loop_low_count;
    }
}

void PlugLock::advance_loop()
{
    switch (challenge_phase) {
    case ChallengePhase::Release:
        ++challenge_reads;

        // The verdict here is "the input went low within the budget", not "the input is
        // low right now". The earlier version failed on the first high read, on the
        // premise that no high reading in this phase can be benign. That premise was
        // wrong: a read taken across the relay opening returns a blend of both levels -
        // measured at 9272 mV on 2026-09-13, between ends of 12120 mV and -3 mV - and
        // one such read was enough to fail an intact harness. The Bricklet's sample rate
        // is set fast enough that this should no longer happen (plug_lock_input.cpp),
        // but a verdict that a single early sample can decide is fragile either way.
        if (loop_raw_valid) {
            if (loop_raw)
                challenge_lows = 0;
            else if (challenge_lows < PLUG_LOCK_LOOP_RELEASE_READS)
                ++challenge_lows;
        }

        if (challenge_lows >= PLUG_LOCK_LOOP_RELEASE_READS) {
            challenge_phase   = ChallengePhase::Restore;
            challenge_reads   = 0;
            challenge_lows    = 0;
            loop_relay_closed = true;
            break;
        }

        // Never got there, and that is the half a strap cannot survive: with the relay
        // released the input has to stop seeing the supply, and a welded contact, a relay
        // channel wired backwards and conducting through its body diode, or 12 V tied
        // straight across the input to fake the marker all stay high for the whole budget.
        // Restoring the relay one second later than before costs nothing - the loop is a
        // voltage sense carrying microamps and drives nothing.
        if (challenge_reads >= PLUG_LOCK_LOOP_CHALLENGE_MAX_READS)
            finish_challenge(PlugLockLoop::FailStuck);
        break;

    case ChallengePhase::Restore:
        ++challenge_reads;

        if (loop_raw_valid && challenge_reads > PLUG_LOCK_LOOP_SETTLE_READS) {
            if (loop_raw)
                ++challenge_highs;
            else
                challenge_highs = 0;
        }

        if (challenge_highs >= PLUG_LOCK_LOOP_RESTORE_READS) {
            finish_challenge(PlugLockLoop::Verified);
            break;
        }

        // Consecutive reads are required, but a couple of spare attempts are allowed so
        // that one glitched sample does not fail an intact loop.
        if (challenge_reads >= PLUG_LOCK_LOOP_CHALLENGE_MAX_READS)
            finish_challenge(PlugLockLoop::FailOpen);
        break;

    case ChallengePhase::None:
    default:
        read_loop();

        if (loop_low_count >= PLUG_LOCK_LOOP_LOW_READS) {
            if (loop_verdict != PlugLockLoop::FailOpen) {
                logger.printfln("Plug lock harness loop lost. Charging is blocked while the plug lock is enabled.");
                loop_verdict = PlugLockLoop::FailOpen;
            }

            // Re-prove it the moment it comes back instead of at the next interval.
            // Safe to keep re-arming: an open loop never reaches the high debounce, so
            // this cannot turn into relay chatter, and a loop that fails its challenge
            // ends up back here rather than retrying immediately.
            next_challenge = 0_us;
            break;
        }

        if (loop_high_count < PLUG_LOCK_LOOP_HIGH_READS || !challenge_allowed())
            break;

        if (challenge_requested || deadline_elapsed(next_challenge))
            start_challenge();
        break;
    }
}

void PlugLock::publish_loop()
{
    // Challenging is a display value only. The verdict itself is held across the
    // challenge, because the loop legitimately reads low while the relay is released
    // and a momentary FailOpen would fault a charging session every fifteen minutes.
    state.get("loop")->updateEnum(challenge_phase != ChallengePhase::None ? PlugLockLoop::Challenging
                                                                         : loop_verdict);
}

void PlugLock::update()
{
    // An EVSE older than these functions answers "unknown function". Stay inert and
    // silent rather than retrying four times a second.
    if (!evse_supported)
        return;

    const bool relay_found = plug_lock_relay.is_found();
    const bool input_found = plug_lock_input.is_found();
    const bool found = relay_found && input_found;

    state.get("relay_found")->updateBool(relay_found);
    state.get("input_found")->updateBool(input_found);

    if (found && !bricklets_found) {
        // Bound how long we may stay silent towards the EVSE waiting for a first
        // verdict, starting from the moment there is something to ask.
        reset_loop();
        loop_verdict     = PlugLockLoop::Unknown;
        next_challenge   = 0_us;
        verdict_deadline = now_us() + PLUG_LOCK_LOOP_VERDICT_TIMEOUT;
    }

    bricklets_found = found;

    if (bricklets_found) {
        // Read feedback, advance the state machine, then re-assert the relay. It is
        // re-asserted on every cycle rather than set once, so a bricklet reset or a
        // brief unplug heals itself. Never a monoflop.
        read_feedback();
        advance(state.get("lock_wanted")->asBool());

        // After advance(), so challenge_allowed() sees the actuation state this cycle
        // ended up in rather than the previous one.
        advance_loop();

        // Not while still in Inactive: there we have not yet established whether the
        // plug is locked, and writing the relay would command it open.
        if (actuation != PlugLockActuation::Inactive)
            plug_lock_relay.set_locked(relay_locked);

        // The loop channel has no such restriction and is written unconditionally: it
        // drives nothing, and a separate single-channel write can never disturb the
        // lock command. See plug_lock_relay.h.
        plug_lock_relay.set_loop_closed(loop_relay_closed);
    } else {
        set_actuation(PlugLockActuation::Inactive);
        feedback_high_count = 0;
        feedback_reads = 0;
        feedback_read_failures = 0;
        lock_closed = false;
        relay_locked = false;
        attempts = 0;

        // Not Unknown. Unknown means "ask me again in a moment" and withholds the
        // report below; with no Bricklets there is nothing left to ask, and staying
        // silent forever would also stop the config read and the firmware-support
        // probe that both hang off a successful report.
        //
        // Not FailOpen either: that means a loop which will not close while both
        // Bricklets are present, which is a different repair and a different message.
        reset_loop();
        loop_verdict = PlugLockLoop::BrickletsNotFound;
    }

    state.get("lock_closed")->updateBool(lock_closed);
    state.get("feedback_voltage")->updateInt(feedback_mv);
    state.get("feedback_plausible")->updateBool(feedback_plausible);
    state.get("loop_voltage")->updateInt(loop_mv);
    state.get("loop_plausible")->updateBool(loop_plausible);
    publish_loop();

    // No verdict yet, and still inside the grace window. Withholding the report leaves
    // the EVSE in its existing "no fresh assertion" state instead of actively claiming
    // the hardware vanished - which matters when the charger boots with a plug already
    // locked, where an asserted false would fault a session that is perfectly fine.
    if (loop_verdict == PlugLockLoop::Unknown && !deadline_elapsed(verdict_deadline))
        return;

    // Settled once there is a real verdict about the loop, or once we have run out of
    // time to produce one. Latched: a dedication of false after this point means checked
    // and broken, which is what entitles the EVSE to fault on it immediately.
    // BrickletsNotFound is deliberately not a verdict here - at start-up it is simply
    // what discovery says before it has finished, which is the case this exists for.
    if (!startup_settled
        && (loop_verdict == PlugLockLoop::Verified
            || loop_verdict == PlugLockLoop::FailOpen
            || loop_verdict == PlugLockLoop::FailStuck
            || deadline_elapsed(startup_deadline))) {
        startup_settled = true;
    }

    // Send the hardware state first: it doubles as the heartbeat that refreshes the
    // EVSE's staleness timer and it is the precondition for enabling the feature at
    // all. See the ordering note in plug_lock.h.
    //
    // lock_closed is the debounced feedback input, deliberately not derived from the
    // state machine, so that the EVSE applies its own logic to a measurement rather
    // than to our inference.
    //
    // The first argument is the harness loop verdict, NOT bricklets_found. Discovery
    // would let any unrelated pair of these Bricklets arm the feature, and it is
    // assertable by anything that can plug a Bricklet in; the loop is a wire. See the
    // note at the top of plug_lock.h.
    // Two independent facts, deliberately not one. The first is the loop verdict, which
    // arms *enabling*; the second is discovery, which is the only thing that can unarm
    // the feature. Discovery runs on the Bricklet port and survives a lock supply that
    // does not, which is exactly why the EVSE cannot infer one from the other.
    //
    // Both, not either: one remaining Bricklet is a partly disassembled lock, and one
    // loose 7-pin cable must not open the disable path.
    const int rc = evse_v2.set_plug_lock_hardware_state(loop_verdict == PlugLockLoop::Verified,
                                                        !relay_found && !input_found,
                                                        lock_closed,
                                                        actuation == PlugLockActuation::LockFault,
                                                        shutting_down,
                                                        !startup_settled);

    if (rc == TF_E_NOT_SUPPORTED) {
        logger.printfln("Charge controller firmware does not support the plug lock. Disabling plug lock support.");
        evse_supported = false;
        return;
    }

    if (rc != TF_E_OK)
        return;

    if (!config_read) {
        config_read = update_config_from_bricklet();

        if (config_read && config.get("enabled")->asBool()
            && loop_verdict != PlugLockLoop::Verified
            && !logged_enabled_with_unverified_dedication) {
            logged_enabled_with_unverified_dedication = true;

            if (!bricklets_found)
                logger.printfln("Plug lock is enabled in EVSE config but no bricklets were found. Charging is blocked until they are connected or the plug lock is disabled.");
            else
                logger.printfln("Plug lock is enabled in EVSE config but its harness loop is not verified. Charging is blocked until the loop is restored or the plug lock is disabled.");
        }
    }

    uint8_t evse_state;
    bool lock_wanted;

    if (evse_v2.get_plug_lock_state(&evse_state, &lock_wanted) != TF_E_OK)
        return;

    state.get("state")->updateEnum(static_cast<PlugLockState>(evse_state));
    state.get("lock_wanted")->updateBool(lock_wanted);
}

bool PlugLock::update_config_from_bricklet()
{
    bool enabled;

    if (evse_v2.get_plug_lock_configuration(&enabled) != TF_E_OK)
        return false;

    config.get("enabled")->updateBool(enabled);

    return true;
}

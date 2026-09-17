/* esp32-firmware
 * Copyright (C) 2026 empunkt <empunkt@mailbox.org>
 *
 * plug_lock.h: Type 2 plug lock, ESP32 half
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

#pragma once

#include "module.h"
#include "config.h"
#include "TFTools/Micros.h"

#include "generated/plug_lock_actuation.enum.h"
#include "generated/plug_lock_loop.enum.h"

// The Type 2 plug lock is driven by two bricklets on the ESP32, not by the EVSE: an
// Industrial Quad Relay 2.1 (modules/plug_lock_relay) energizes the lock motor and an
// Industrial Dual Analog In 2.0 (modules/plug_lock_input) reads the feedback line.
// The EVSE cannot see either of them.
//
// The split: the EVSE decides *whether* the plug has to be locked and *whether*
// charging may happen - it holds the interlock, refuses to energize until the plug is
// confirmed locked, and blocks charging with error state 6 when the lock is enabled
// but not working. This module decides *how* the lock is driven. No lock command ever
// crosses the wire; the EVSE publishes lock_wanted and this side executes it.
//
// ORDERING, load-bearing: the EVSE rejects "Set Plug Lock Configuration" with
// enabled = true unless a fresh "Set Plug Lock Hardware State" report exists. The
// periodic update therefore sends the hardware state before reading anything back.
// Do not reorder that so a legitimate enable starts getting rejected.
//
// The hardware report goes stale on the EVSE after 5 s, so the update interval has to
// stay well below that.
//
// WHAT IS REPORTED AS "bricklet dedication verified" IS THE HARNESS LOOP, NOT DEVICE
// DISCOVERY. Quad Relay channel 1 switches the lock supply onto Analog In channel 1,
// which reads it back. Toggling that relay and watching the input follow proves that
// these two specific Bricklets are wired to each other and to a lock supply; is_found()
// only proves that some Quad Relay and some Dual Analog In are plugged in, which any user
// with unrelated Bricklets satisfies. Because the loop is a wire, nothing reachable
// over the network can assert it, which is what makes it the gate for *enabling* the
// feature.
//
// It is not a sound gate for the reverse direction, and the EVSE currently uses it as
// one - see the KNOWN DEFECT note on plug_lock_disable_is_blocked() in the EVSE's
// plug_lock.h. A dead lock PSU de-asserts the loop with both Bricklets still fitted, so
// "not verified" cannot be read as "the lock was removed".
//
// The two notions must not be collapsed. bricklets_found still drives the actuation
// state machine and the channel 0 writes; only the *reported* bit is the loop verdict.
// Conflating them would drop the lock relay on a loop failure and release a plug that
// may be live.
class PlugLock final : public IModule
{
public:
    PlugLock(){}
    void pre_setup() override;
    void setup() override;
    void register_urls() override;

    // Announces the restart to the EVSE, which releases the contactor through IEC 61851
    // state B rather than leaving the EVSE to notice the silence and fault. See the
    // implementation for the ordering dependency this relies on, and for why this
    // interrupts a session rather than ending it.
    void pre_reboot() override;

private:
    void update();
    bool update_config_from_bricklet();

    void read_feedback();
    void advance(bool lock_wanted);
    void set_actuation(PlugLockActuation next);
    void start_lock_attempt();

    void read_loop();
    void advance_loop();
    void start_challenge();
    void finish_challenge(PlugLockLoop verdict);
    bool challenge_allowed() const;
    void reset_loop();
    void publish_loop();

    ConfigRoot config;
    ConfigRoot config_update;
    ConfigRoot state;

    bool config_read = false;

    // Cleared for good the first time the EVSE answers "unknown function", i.e. when
    // this firmware is newer than the EVSE's. Updates are not atomic across the two
    // processors, so that combination happens in the field. It has to read as "not
    // supported, stay inert" - no retry, no logging four times a second.
    bool evse_supported = true;

    // Enabled on the EVSE with no verified dedication refuses to charge and blinks 6,
    // with nothing to explain why. Logged once so it is diagnosable without counting
    // blinks. Covers both reasons the verdict can be unverified - the board-swap case,
    // where an EVSE carrying enabled = true moved into a charger with no lock bricklets,
    // and a loop that is broken or unpowered while both bricklets are fitted - so the
    // two are told apart by the message, not by this flag.
    bool logged_enabled_with_unverified_dedication = false;

    PlugLockActuation actuation = PlugLockActuation::Inactive;

    // Debounced feedback, asymmetric on purpose: two consecutive high reads to call
    // the plug locked, but a single low read to call it unlocked. Slow to claim the
    // safe state, quick to give it up. Mirrors lock.c's own asymmetry.
    bool lock_closed = false;
    uint8_t feedback_high_count = 0;
    // How many readings we have actually taken. Until the debounce has had a chance to
    // settle we do not know whether the plug is locked, and must not decide that it is
    // not: it may have been locked before this processor restarted.
    uint8_t feedback_reads = 0;
    // A failed read is not a low read. One transient failure must not drop a charging
    // session, so it costs one extra cycle before we act on it.
    uint8_t feedback_read_failures = 0;

    // Millivolts as last read, and whether each landed inside a band the wiring should
    // be able to produce. Published and nothing more - no branch anywhere reads these,
    // because folding an implausible reading into a low has already made the safe
    // choice by the time they get here. See modules/plug_lock_input/plug_lock_input.h.
    int32_t feedback_mv = 0;
    int32_t loop_mv = 0;
    bool feedback_plausible = false;
    bool loop_plausible = false;

    bool relay_locked = false;

    micros_t attempt_deadline = 0_us;
    uint8_t attempts = 0;
    // One tick with the relay released before re-asserting. The actuator module
    // actuates internally, so a fresh command edge is likelier to work than holding a
    // command it is already failing to satisfy.
    bool retry_release = false;

    // ---- harness loop ----

    // Both Bricklets answered. Drives everything above; deliberately NOT what gets
    // reported to the EVSE. See the note at the top of this file.
    bool bricklets_found = false;

    // Set once in pre_reboot() and never cleared: this instance is on its way out, and
    // every report it still manages to send should carry the announcement. The instance
    // that comes back starts false again, which is what releases the EVSE's gate.
    bool shutting_down = false;

    // Last raw reading of the loop input, taken by read_feedback() from the same
    // get_values() call that serves channel 0, so the loop costs no extra Bricklet
    // traffic at all.
    bool loop_raw = false;
    bool loop_raw_valid = false;

    // Steady-state debounce, asymmetric and biased the *opposite* way to the feedback
    // contact above: slow to call the loop lost, quick to call it good. Losing the
    // marker is never an urgent hazard - the lock itself is still holding and channel 0
    // is watched independently - while a false low costs a dropped session plus the
    // EVSE's 30 s error cooldown. So take a full second to be sure.
    uint8_t loop_high_count = 0;
    uint8_t loop_low_count = 0;

    // The retained verdict. Never Challenging: a challenge in progress holds the last
    // verdict, because the loop legitimately reads low while the relay is released.
    // Challenging is a display value only, produced by publish_loop().
    PlugLockLoop loop_verdict = PlugLockLoop::Unknown;

    // Commanded state of the loop relay. Closed by default and re-asserted every cycle,
    // same doctrine as channel 0, so the marker is continuously monitored rather than
    // sampled. Only the release phase of a challenge opens it.
    bool loop_relay_closed = true;

    enum class ChallengePhase : uint8_t {
        None = 0,
        Release, // relay open, the input must go low - this is the half a strap fails
        Restore, // relay closed again, the input must come back
    };

    ChallengePhase challenge_phase = ChallengePhase::None;
    uint8_t challenge_reads = 0;
    uint8_t challenge_highs = 0;
    // The mirror of challenge_highs, for the Release half. Both count *consecutive*
    // reads and reset on a read of the other level.
    uint8_t challenge_lows = 0;
    // Set by the API command. Runs the next challenge as soon as one is allowed,
    // ignoring the periodic interval.
    bool challenge_requested = false;
    micros_t next_challenge = 0_us;

    // Cap on how long we may report nothing at all while waiting for a first verdict.
    micros_t verdict_deadline = 0_us;

    // Latched the first time this instance has an actual verdict on the harness loop,
    // and never cleared: after that a dedication of false means "checked and broken"
    // rather than "not asked yet", and the EVSE is entitled to fault on it at once.
    // A restart makes a new instance, which starts unsettled again - which is the point.
    bool startup_settled = false;
    micros_t startup_deadline = 0_us;
};

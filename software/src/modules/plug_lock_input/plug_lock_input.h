/* esp32-firmware
 * Copyright (C) 2026 empunkt <empunkt@mailbox.org>
 *
 * plug_lock_input.h: Industrial Dual Analog In 2.0 reading the plug lock feedback
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

#include "device_module.h"
#include "bindings/bricklet_industrial_dual_analog_in_v2.h"

// Wiring convention, fixed by the plug lock harness and not configurable:
// channel 0 is the LOCK feedback line, channel 1 the harness loop return.
//
// Both are read as voltages. The Mennekes control block drives LOCK through a
// large series resistance, on the order of 50-100 kOhm, so it cannot deliver
// the ~0.4 mA an optocoupled input needs in order to switch. That is why the
// Industrial Digital In 4 2.0 which used to sit here never once read the line
// high. An ADC needs only a voltage it can digitise, so a weak source stops
// mattering. See modules/plug_lock/plug_lock.h.
//
// Each channel is a differential pair with 100 kOhm from either terminal to the
// bricklet's analog ground and 280 kOhm into the difference amplifier, so a
// single-ended signal sees roughly 74 kOhm.
#define PLUG_LOCK_INPUT_CHANNEL 0

// Channel 1 is the harness loop return, fed from Quad Relay channel 1 and
// returning to PSU-. Unlike the optocoupled input this replaced, the loop is no
// longer a series current circuit: an analog input is a shunt, not a series
// element, so the loop is sensed as the voltage passed by the closed relay.
// Closed sits at the supply rail. Open is near zero only while the plug is
// unlocked (-6 mV measured, the CPC1002N's off-state leakage into 74 kOhm); with
// the pin engaged the released channel floats onto the analog ground that the
// feedback line is lifting and reads about 4 V instead. Both are lows, and the
// band below is widened to say so.
//
// POLARITY IS MANDATORY here too: the input "+" has to face the relay output,
// i.e. the more positive node. See modules/plug_lock_relay/plug_lock_relay.h.
#define PLUG_LOCK_INPUT_LOOP_CHANNEL 1

// Millivolts; the bricklet's range is -35000 to +35000.
//
// Each channel is sorted into one of three outcomes rather than compared against
// a single threshold: low, high, or implausible. An optocoupled input could only
// ever answer "past the switching point or not"; an ADC can also say "that is not
// a value this wiring should be able to produce", and here that is worth more
// than the extra resolution.
//
// Implausible is folded into low by get_reading(), so a reading that makes no
// sense can never be taken for a locked plug or a verified loop. The millivolts
// and a per-channel plausible flag come back alongside, so the difference stays
// visible to a human without widening what the state machine acts on.
//
// The loop's high band starts well above half the rail on purpose. A break in the
// loop's return leg leaves the channel reading the supply through its own 100 kOhm
// bias resistor and the feedback channel's, which is a two-resistor divider at
// almost exactly half the rail. The series current loop this replaced caught a
// broken conductor structurally, by having no circuit at all; here that case is
// caught only because it lands outside the band, so the band is what does the
// work and it is not decoration.
//
// THE TWO CHANNELS ARE NOT INDEPENDENT, and the bands below exist mostly to cope
// with that. Both channels' bias networks meet at the same analog ground, so
// whichever channel is driven lifts that ground, and the other channel - which is
// only ever driven through a large series resistance, or not at all - reads the
// lift as a pedestal. Measured on the rig at 12.117 V, all four combinations:
//
//                          loop relay closed   loop relay released
//   feedback, unlocked            4010 mV                  0 mV
//   feedback, pin engaged         7217 mV               5596 mV
//   loop                         12117 mV               4007 mV (pin engaged)
//                                                         -6 mV (unlocked)
//
// So an unlocked plug reads 4010 mV, not the ~0 mV the first design assumed: the
// module is fed from the rail at +12V whatever the relay does, and with OUT-
// unreturned the only path back to PSU- is through the bricklet's own bias
// network. The feedback bands therefore have to separate 4010 from 5596, not
// 0 from 7217.
//
// MEASURED 2026-09-12 on the rig. Each threshold sits near the middle of the gap
// the table leaves, so each carries roughly 600 mV of margin.
#define PLUG_LOCK_INPUT_LOW_MIN            -1000

// Low covers an unlocked plug at 4010 mV, whether or not the loop relay is
// closed. High covers an engaged pin at 5596 mV during a loop challenge and at
// 7217 mV otherwise. The gap between them is implausible, which folds to low -
// the safe direction.
#define PLUG_LOCK_INPUT_FEEDBACK_LOW_MAX    4600
#define PLUG_LOCK_INPUT_FEEDBACK_HIGH_MIN   5000
// Nothing on this channel can legitimately exceed the rail. A reading up there
// means LOCK has been strapped to +12V rather than driven through the module's
// series resistance, and that would otherwise look exactly like a locked plug.
#define PLUG_LOCK_INPUT_FEEDBACK_HIGH_MAX   9500

// The released loop reads -6 mV with the plug unlocked but 4007 mV with it
// engaged, for the same reason the feedback channel has a pedestal. Both are
// genuine lows, so the band has to reach past the higher of them.
#define PLUG_LOCK_INPUT_LOOP_LOW_MAX        4600
#define PLUG_LOCK_INPUT_LOOP_HIGH_MIN       8500
#define PLUG_LOCK_INPUT_LOOP_HIGH_MAX      13500

// Everything one bricklet call can say, kept together because it is one call:
// the harness loop rides along with the feedback line and must not cost extra
// traffic. Index both arrays with PLUG_LOCK_INPUT_CHANNEL and
// PLUG_LOCK_INPUT_LOOP_CHANNEL.
struct PlugLockInputReading {
    // As read, in millivolts.
    int32_t voltage[2];

    // What the state machine acts on. Implausible counts as low.
    bool level[2];

    // False when the voltage fell into neither band. Diagnostic only - nothing
    // branches on it, because folding implausible into low has already made the
    // safe choice.
    bool plausible[2];
};

// Optional device: without it the plug lock stays inert, the charger works as
// usual. The plug_lock module reads is_found() and reports it to the EVSE,
// which refuses to arm the feature while it is false.
class PlugLockInput final : public DeviceModule<TF_IndustrialDualAnalogInV2,
                                                tf_industrial_dual_analog_in_v2_create,
                                                tf_industrial_dual_analog_in_v2_get_bootloader_mode,
                                                tf_industrial_dual_analog_in_v2_reset,
                                                tf_industrial_dual_analog_in_v2_destroy,
                                                false>
{
public:
    PlugLockInput();

    void setup() override;

    void setup_input();

    bool is_found() const {return device_found;}

    // Raw, undebounced reading of both channels. The bricklet this replaced
    // made the level comparison in hardware at 3 V, so making it here leaves the
    // division of labour where it was: plug_lock does the debouncing, so that
    // what it reports to the EVSE is a measurement rather than a conclusion.
    int get_reading(PlugLockInputReading *ret_reading);
};

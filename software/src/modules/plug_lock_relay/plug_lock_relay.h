/* esp32-firmware
 * Copyright (C) 2026 empunkt <empunkt@mailbox.org>
 *
 * plug_lock_relay.h: Industrial Quad Relay 2.1 driving the Type 2 plug lock
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
#include "bindings/bricklet_industrial_quad_relay_v2.h"

// Wiring convention, fixed by the plug lock harness and not configurable:
// channel 0 is the OUT-/PSU- dry contact that energizes the lock motor.
// See modules/plug_lock/plug_lock.h.
#define PLUG_LOCK_RELAY_CHANNEL 0

// Channel 1 is the harness loop: PSU+ -> this channel -> Dual Analog In channel 1,
// whose other terminal sits on PSU-. It drives nothing; it exists so the firmware
// can prove that these two specific Bricklets are wired to each other and to the
// lock supply, which mere device discovery cannot. See modules/plug_lock/plug_lock.h.
//
// The loop used to be a series current circuit through an optocoupled input, which
// was its own load. An analog input is a shunt, not a series element, so the loop is
// now sensed as a voltage instead and carries only the microamps the input's bias
// network draws.
//
// POLARITY IS MANDATORY. Since hardware 2.1 the channels are solid state
// (CPC1002N) and unipolar: DC+ has to face the more positive node, here PSU+.
// Wired backwards the body diode conducts and the channel reads permanently
// closed, which is the fail-dangerous orientation - the loop would appear
// verified without the relay ever switching.
#define PLUG_LOCK_RELAY_LOOP_CHANNEL 1

// Channels 2 and 3 are unused.

// Optional device: without it the plug lock stays inert, the charger works as
// usual. The plug_lock module reads is_found() and reports it to the EVSE,
// which refuses to arm the feature while it is false.
class PlugLockRelay final : public DeviceModule<TF_IndustrialQuadRelayV2,
                                                tf_industrial_quad_relay_v2_create,
                                                tf_industrial_quad_relay_v2_get_bootloader_mode,
                                                tf_industrial_quad_relay_v2_reset,
                                                tf_industrial_quad_relay_v2_destroy,
                                                false>
{
public:
    PlugLockRelay();

    void setup() override;

    void setup_relay();

    bool is_found() const {return device_found;}

    // Static, never a monoflop: a monoflop would release the lock on its own while
    // the plug may still be live. plug_lock re-asserts this on every cycle rather
    // than setting it once, so a bricklet reset or a brief unplug heals itself.
    int set_locked(bool locked);

    // The harness loop channel. Deliberately a second single-channel write rather
    // than one set_value() covering both: set_selected_value() touches exactly one
    // channel, which is what guarantees that a loop write can never disturb the lock
    // command - including in the Inactive state, where plug_lock must not write
    // channel 0 at all because it has not yet established whether the plug is locked.
    // One extra HAL call per cycle is a cheap price for leaving that path untouched.
    int set_loop_closed(bool closed);
};

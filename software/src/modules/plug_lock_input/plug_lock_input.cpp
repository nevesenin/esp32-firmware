/* esp32-firmware
 * Copyright (C) 2026 empunkt <empunkt@mailbox.org>
 *
 * plug_lock_input.cpp: Industrial Dual Analog In 2.0 reading the plug lock feedback
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

#include "plug_lock_input.h"

#include "event_log_prefix.h"
#include "generated/module_dependencies.h"
#include "generated/industrial_dual_analog_in_v2_bricklet_firmware_bin.embedded.h"

#include "gcc_warnings.h"

PlugLockInput::PlugLockInput(): DeviceModule(industrial_dual_analog_in_v2_bricklet_firmware_bin_data,
                                             industrial_dual_analog_in_v2_bricklet_firmware_bin_length,
                                             "plug_lock_input",
                                             "Industrial Dual Analog In 2.0",
                                             "plug lock input",
                                             [this](){this->setup_input();}) {}

void PlugLockInput::setup()
{
    setup_input();
}

void PlugLockInput::setup_input()
{
    if (!this->DeviceModule::setup_device())
        return;

    // The Bricklet powers up at 2 SPS (mcp3911.c:319 in its own firmware) and each value
    // is a boxcar average over the whole 500 ms conversion. Left at that, a module that
    // polls every 250 ms reads the same conversion twice and, worse, any read taken
    // across a real change returns a blend of before and after: a released harness loop
    // measured 9272 mV on 2026-09-13 while the two ends of the step were 12120 mV and
    // -3 mV. That is what made the boot loop challenge fail intermittently.
    //
    // 61 SPS is ~16 ms per conversion, so every poll gets a value that is both fresh and
    // fully settled, with room to spare. Still heavily oversampled, and none of the bands
    // in plug_lock_input.h is anywhere near tight enough for the extra noise to matter.
    const int rc = io_scheduler.hal_call([&]() {
        return tf_industrial_dual_analog_in_v2_set_sample_rate(&device,
                   TF_INDUSTRIAL_DUAL_ANALOG_IN_V2_SAMPLE_RATE_61_SPS);
    });

    // Not fatal, and deliberately not a reason to leave the module uninitialized: the
    // readings stay correct, they just go back to being slow and averaged, which is the
    // behaviour this module shipped with. Say so rather than leaving the timing
    // assumptions above silently wrong.
    if (rc != TF_E_OK)
        logger.printfln("Could not set the plug lock input sample rate (error %d). Readings stay at 2 SPS.", rc);

    initialized = true;
}

// A voltage inside the low band is a low, one inside the high band is a high,
// and anything else is neither. "Neither" is reported as a low, so that a wiring
// fault which happens to land between the bands can never be read as a locked
// plug or a verified loop; the plausible flag is what tells the two apart
// afterwards.
static void classify(int32_t mv, int32_t low_max, int32_t high_min, int32_t high_max,
                     bool *ret_level, bool *ret_plausible)
{
    if (mv >= PLUG_LOCK_INPUT_LOW_MIN && mv <= low_max) {
        *ret_level = false;
        *ret_plausible = true;
        return;
    }

    if (mv >= high_min && mv <= high_max) {
        *ret_level = true;
        *ret_plausible = true;
        return;
    }

    *ret_level = false;
    *ret_plausible = false;
}

int PlugLockInput::get_reading(PlugLockInputReading *ret_reading)
{
    *ret_reading = {};

    if (!device_found)
        return TF_E_NOT_INITIALIZED;

    // Both channels in one call, which is what keeps the harness loop free of
    // extra bricklet traffic. Needs bricklet firmware 2.0.6 or newer; on
    // anything older this returns an error rather than silently reading one
    // channel, so do not downgrade the embedded .zbin below that.
    int rc = io_scheduler.hal_call([&]() { return tf_industrial_dual_analog_in_v2_get_all_voltages(&device, ret_reading->voltage); });

    if (rc != TF_E_OK)
        return rc;

    classify(ret_reading->voltage[PLUG_LOCK_INPUT_CHANNEL],
             PLUG_LOCK_INPUT_FEEDBACK_LOW_MAX,
             PLUG_LOCK_INPUT_FEEDBACK_HIGH_MIN,
             PLUG_LOCK_INPUT_FEEDBACK_HIGH_MAX,
             &ret_reading->level[PLUG_LOCK_INPUT_CHANNEL],
             &ret_reading->plausible[PLUG_LOCK_INPUT_CHANNEL]);

    classify(ret_reading->voltage[PLUG_LOCK_INPUT_LOOP_CHANNEL],
             PLUG_LOCK_INPUT_LOOP_LOW_MAX,
             PLUG_LOCK_INPUT_LOOP_HIGH_MIN,
             PLUG_LOCK_INPUT_LOOP_HIGH_MAX,
             &ret_reading->level[PLUG_LOCK_INPUT_LOOP_CHANNEL],
             &ret_reading->plausible[PLUG_LOCK_INPUT_LOOP_CHANNEL]);

    return TF_E_OK;
}

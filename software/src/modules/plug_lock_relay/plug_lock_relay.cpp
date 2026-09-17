/* esp32-firmware
 * Copyright (C) 2026 empunkt <empunkt@mailbox.org>
 *
 * plug_lock_relay.cpp: Industrial Quad Relay 2.1 driving the Type 2 plug lock
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

#include "plug_lock_relay.h"

#include "event_log_prefix.h"
#include "generated/module_dependencies.h"
#include "generated/industrial_quad_relay_v2_bricklet_firmware_bin.embedded.h"

#include "gcc_warnings.h"

PlugLockRelay::PlugLockRelay(): DeviceModule(industrial_quad_relay_v2_bricklet_firmware_bin_data,
                                             industrial_quad_relay_v2_bricklet_firmware_bin_length,
                                             "plug_lock_relay",
                                             "Industrial Quad Relay 2.0",
                                             "plug lock relay",
                                             [this](){this->setup_relay();}) {}

void PlugLockRelay::setup()
{
    setup_relay();
}

void PlugLockRelay::setup_relay()
{
    if (!this->DeviceModule::setup_device())
        return;

    initialized = true;
}

int PlugLockRelay::set_locked(bool locked)
{
    if (!device_found)
        return TF_E_NOT_INITIALIZED;

    return io_scheduler.hal_call([&]() { return tf_industrial_quad_relay_v2_set_selected_value(&device, PLUG_LOCK_RELAY_CHANNEL, locked); });
}

int PlugLockRelay::set_loop_closed(bool closed)
{
    if (!device_found)
        return TF_E_NOT_INITIALIZED;

    return io_scheduler.hal_call([&]() { return tf_industrial_quad_relay_v2_set_selected_value(&device, PLUG_LOCK_RELAY_LOOP_CHANNEL, closed); });
}

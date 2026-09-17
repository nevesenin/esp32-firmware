import { PlugLockState } from "./generated/plug_lock_state.enum";
import { PlugLockActuation } from "./generated/plug_lock_actuation.enum";
import { PlugLockLoop } from "./generated/plug_lock_loop.enum";

export interface config {
    enabled: boolean;
}

export interface state {
    state: PlugLockState;
    lock_wanted: boolean;
    actuation: PlugLockActuation;
    lock_closed: boolean;
    relay_found: boolean;
    input_found: boolean;
    loop: PlugLockLoop;
    feedback_voltage: number;
    feedback_plausible: boolean;
    loop_voltage: number;
    loop_plausible: boolean;
}

export interface config_update {
    enabled: boolean;
}

export interface loop_challenge {
}

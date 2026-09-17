/** @jsxImportSource preact */
import { h } from "preact";
let x = {
    "plug_lock": {
        "navbar": {
            "plug_lock": "Type2 Plug"
        },
        "content": {
            "plug_lock": "Plug lock",
            "plug_lock_desc": "Lock the charging cable in the socket",
            "unlock_fault": "The plug lock did not release. Use the mechanical emergency release to free the cable.",
            "one_way": "Once switched on, the plug lock can only be switched off again after both Bricklets have been unplugged inside the enclosure. A broken test loop is deliberately not enough, because a failed lock power supply breaks it too.",
            "loop_no_bricklets": "The plug lock Bricklets were not found. Check that the Industrial Quad Relay Bricklet 2.1 and the Industrial Dual Analog In Bricklet 2.1 are connected.",
            "loop_open": "The plug lock harness loop is not verified. Check the link between relay channel 1, input channel 1 and the lock supply.",
            "loop_stuck": "The plug lock harness loop does not open when the relay is released. Check the polarity of relay channel 1.",
            "plug_lock_help": <>
                <p>Locks the Type 2 plug in the charging socket, so that it cannot be pulled out while charging.</p>
                <p>The lock is driven by an Industrial Quad Relay Bricklet 2.1 and read back by an Industrial Dual Analog In Bricklet 2.1. Channel 1 of both is wired together into a test loop, so that the charger can tell a real plug lock from two unrelated Bricklets that happen to be connected.</p>
                <p><b>Once switched on, the plug lock can only be switched off again after both Bricklets have been unplugged.</b> A socket that is wired for a lock but has it switched off would energize with a plug that can be pulled out. To remove the plug lock, unplug both Bricklets inside the enclosure while the wallbox is de-energized; switching the setting off is then accepted. A broken test loop is deliberately not enough on its own, because a failed lock power supply breaks the loop while both Bricklets are still fitted.</p>
                <p><b>While the plug lock is enabled and the harness loop is missing, charging is blocked.</b> The charge controller then reports error state 6 and the status LED blinks six times.</p>
            </>
        },
        "script": {
            "save_failed": "Failed to save the plug lock setting"
        }
    }
}

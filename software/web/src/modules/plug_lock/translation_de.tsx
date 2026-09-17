/** @jsxImportSource preact */
import { h } from "preact";
let x = {
    "plug_lock": {
        "navbar": {
            "plug_lock": "Steckerverriegelung"
        },
        "content": {
            "plug_lock": "Steckerverriegelung",
            "plug_lock_desc": "Ladekabel in der Buchse verriegeln",
            "unlock_fault": "Die Steckerverriegelung hat nicht freigegeben. Nutze die mechanische Notentriegelung, um das Kabel zu lösen.",
            "one_way": "Einmal eingeschaltet, kann die Steckerverriegelung erst wieder abgeschaltet werden, wenn beide Bricklets im Gehäuse abgezogen wurden. Eine unterbrochene Prüfschleife reicht bewusst nicht aus, denn ein Ausfall der Versorgung der Verriegelung unterbricht sie ebenfalls.",
            "loop_no_bricklets": "Die Bricklets der Steckerverriegelung wurden nicht gefunden. Prüfe, ob das Industrial Quad Relay Bricklet 2.1 und das Industrial Dual Analog In Bricklet 2.1 angeschlossen sind.",
            "loop_open": "Die Prüfschleife der Steckerverriegelung ist nicht verifiziert. Prüfe die Verbindungen zwischen Kanal 1 des Dual Analog In Bricklets, Kanal 1 des Relay Bricklet und der Spannungsversorgung der Verriegelung.",
            "loop_stuck": "Die Prüfschleife der Steckerverriegelung öffnet nicht, wenn das Relais abfällt. Prüfe die Polarität von Relaiskanal 1.",
            "plug_lock_help": <>
                <p>Verriegelt den Typ-2-Stecker in der Ladebuchse, sodass er während des Ladevorgangs nicht abgezogen werden kann.</p>
                <p>Die Verriegelung wird von einem Industrial Quad Relay Bricklet 2.1 angesteuert und von einem Industrial Dual Analog In Bricklet 2.1 zurückgelesen. Kanal 1 beider Bricklets ist zu einer Prüfschleife verdrahtet, damit die Wallbox eine echte Verriegelung von zwei zufällig angeschlossenen, nicht zugehörigen Bricklets unterscheiden kann.</p>
                <p><b>Einmal eingeschaltet, kann die Steckerverriegelung erst wieder abgeschaltet werden, wenn beide Bricklets abgezogen wurden.</b> Eine Buchse, die für eine Verriegelung verdrahtet ist, diese aber abgeschaltet hat, würde mit einem abziehbaren Stecker Spannung führen. Zum Entfernen der Steckerverriegelung beide Bricklets im spannungsfreien Gehäuse abziehen, danach wird das Abschalten der Einstellung akzeptiert. Eine unterbrochene Prüfschleife reicht dafür bewusst nicht aus, denn ein Ausfall der Versorgung der Verriegelung unterbricht die Schleife, während beide Bricklets weiterhin verbaut sind.</p>
                <p><b>Solange die Steckerverriegelung aktiviert ist und die Prüfschleife fehlt, wird das Laden blockiert.</b> Der Ladecontroller meldet dann Fehlerzustand 6 und die Status-LED blinkt sechsmal.</p>
            </>
        },
        "script": {
            "save_failed": "Speichern der Steckerverriegelung fehlgeschlagen"
        }
    }
}

#!/usr/bin/env python3
from pynput.keyboard import Controller, Key, KeyCode

UKLAD_NAWIGACJA = [
    ["menu", None,   None,    None],
    ["home", "up",   "pgup",  None],
    ["left", None,   "right", None],
    ["end",  "down", "pgdn",  None],
    ["ins",  "ins",  "del",   None],
]

UKLAD_MENU = [
    ["exit"],
    ["NUM LOCK"],
    ["SUSPEND"],
    ["CALIBRATE"],
]

_VK = {
    "0": 0x60, "1": 0x61, "2": 0x62, "3": 0x63, "4": 0x64,
    "5": 0x65, "6": 0x66, "7": 0x67, "8": 0x68, "9": 0x69,
    "*": 0x6A,
    "+": 0x6B,
    "-": 0x6D,
    ".": 0x6E,
    "/": 0x6F,
}

KLAWISZE = {etykieta: KeyCode.from_vk(vk) for etykieta, vk in _VK.items()}
KLAWISZE["enter"] = Key.enter
KLAWISZE.update({
    "home": Key.home, "up": Key.up, "pgup": Key.page_up,
    "left": Key.left, "right": Key.right,
    "end": Key.end, "down": Key.down, "pgdn": Key.page_down,
    "ins": Key.insert, "del": Key.delete,
})

_klawiatura = Controller()

_tryb = "numpad"
_numlock = True
_zlecona_kalibracja = False


def aktywny_uklad(uklad_numpad):
    if _tryb == "menu":
        return UKLAD_MENU
    return uklad_numpad if _numlock else UKLAD_NAWIGACJA


def zawieszony():
    return _tryb == "suspend"


def kalibracja_zadana():
    global _zlecona_kalibracja
    if _zlecona_kalibracja:
        _zlecona_kalibracja = False
        return True
    return False


def run(rig=None, position=None):
    global _tryb, _numlock, _zlecona_kalibracja

    if _tryb == "suspend":
        _tryb = "numpad"
        if rig is not None:
            rig.displayText("Working...")
        return

    etykieta = (rig.displayed_text if rig is not None else "").strip()

    if _tryb == "menu":
        if etykieta == "exit":
            _tryb = "numpad"
        elif etykieta == "NUM LOCK":
            _numlock = not _numlock
            _tryb = "numpad
        elif etykieta == "SUSPEND":
            _tryb = "suspend"
            if rig is not None:
                rig.displayText("Suspended")
        elif etykieta == "CALIBRATE":
            _zlecona_kalibracja = True
            _tryb = "numpad"
        return

    if etykieta == "menu":
        _tryb = "menu"
        return

    klawisz = KLAWISZE.get(etykieta)
    if klawisz is None:
        return

    _klawiatura.press(klawisz)
    _klawiatura.release(klawisz)

if __name__ == "__main__":
    import time

    class _Atrapa:
        displayed_text = ""

        def displayText(self, text):
            self.displayed_text = text

    atrapa = _Atrapa()
    time.sleep(5)
    for etykieta in ["7", "8", "9", "+", "1", ".", "0", "enter"]:
        atrapa.displayed_text = etykieta
        run(atrapa, (0, 0))
        time.sleep(0.3)

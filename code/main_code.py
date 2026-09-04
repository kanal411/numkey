import time
import cv2
import numpy as np
from finger_api import FingerRig
import button_click
import kalibracja

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

numpadKeys = [
    ["menu", "/", "*", "-"],
    ["7", "8", "9", "+"],
    ["4", "5", "6", "+"],
    ["1", "2", "3", "enter"],
    ["0", "0", ".", "enter"]
]


def pytaj_o_kalibracje(rig, sekundy=2.0):
    rig.displayText("recalibrate?")
    rig.clear_button_presses()
    koniec = time.monotonic() + sekundy
    while time.monotonic() < koniec:
        if rig.button_pressed():
            return True
        rig.wait(0.05)
    return False


def wykonaj_kalibracje(rig):
    rogi = [[[-1, -1], [-1, -1]], [[-1, -1], [-1, -1]]]

    rig.displayText("Keep your finger above the key.")
    rig.move_to(0, 0)
    time.sleep(10)
    print("smile")
    rog = rig.read_finger(True, 3.0, True, True)
    rog.found = False
    while not rog.found:
        rog = rig.read_finger(True, 3.0, True, True)
        if rog.found:
            rogi[0][1][0], rogi[0][1][1] = rog.x, rog.y
        else:
            rig.move_to(3875, 6350)
            time.sleep(1)
            rig.move_to(0, 0)
            time.sleep(4)
    rig.move_to(7750, 0)
    time.sleep(4)
    print("smile")
    rog = rig.read_finger(True, 3.0, True, True)
    rog.found = False
    while not rog.found:
        rog = rig.read_finger(True, 3.0, True, True)
        if rog.found:
            rogi[1][1][0], rogi[1][1][1] = rog.x, rog.y
        else:
            rig.move_to(3875, 6350)
            time.sleep(1)
            rig.move_to(7750, 0)
            time.sleep(4)
    rig.move_to(7750, 12700)
    time.sleep(4)
    print("smile")
    rog = rig.read_finger(True, 3.0, True, True)
    rog.found = False
    while not rog.found:
        rog = rig.read_finger(True, 3.0, True, True)
        if rog.found:
            rogi[1][0][0], rogi[1][0][1] = rog.x, rog.y
        else:
            rig.move_to(3875, 6350)
            time.sleep(1)
            rig.move_to(7750, 12700)
            time.sleep(4)
    rig.move_to(0, 12700)
    time.sleep(4)
    print("smile")
    rog = rig.read_finger(True, 3.0, True, True)
    rog.found = False
    while not rog.found:
        rog = rig.read_finger(True, 3.0, True, True)
        if rog.found:
            rogi[0][0][0], rogi[0][0][1] = rog.x, rog.y
        else:
            rig.move_to(3875, 6350)
            time.sleep(1)
            rig.move_to(0, 12700)
            time.sleep(4)
    print("Calculating. Please wait...")                                                                   
    rig.displayText("Please wait...")

    finger_borders = np.float32([
    rogi[0][0],
    rogi[1][0],
    rogi[1][1],
    rogi[0][1]
    ])

    plane_points = np.float32([
        [0, 12700],
        [7750, 12700],
        [7750, 0],
        [0, 0]
    ])

    H, _ = cv2.findHomography(finger_borders, plane_points)
    return H


with FingerRig(on_demand=True) as rig:
    print("Starting homing sequence.")
    rig.displayText("Homing...")
    rig.home()

    H = kalibracja.wczytaj()
    if H is not None and pytaj_o_kalibracje(rig):
        print("Zlecono ponowna kalibracje.")
        H = None
    if H is None:
        H = wykonaj_kalibracje(rig)
        kalibracja.zapisz(H)

    print("Working.")
    rig.displayText("")

    rig.clear_button_presses()

    rig.on_button(lambda x, y: button_click.run(rig, (x, y)))

    aktualnyTekst = "placeholder"
    rig.displayText("Working...")
    while True:
        while rig.busy:
            time.sleep(0.1)

        if button_click.kalibracja_zadana():
            rig.on_button(None)
            H = wykonaj_kalibracje(rig)
            kalibracja.zapisz(H)
            rig.clear_button_presses()
            rig.on_button(lambda x, y: button_click.run(rig, (x, y)))
            print("Working.")
            rig.displayText("Working...")
            continue

        if button_click.zawieszony():
            rig.wait(0.2)
            continue

        palec = rig.read_finger()

        if button_click.zawieszony():
            continue

        if palec.found:
            pixel = np.array([
                [[palec.x, palec.y]]
            ], dtype=np.float32)

            plane_point = cv2.perspectiveTransform(pixel, H)

            finX, finY = plane_point[0][0]

            finX = clamp(finX, 0, 7750)
            finY = clamp(finY, 0, 12700)

            uklad = button_click.aktywny_uklad(numpadKeys)
            wierszy = len(uklad)
            kolumn = len(uklad[0])
            szerKol = 7750 / kolumn
            wysWier = 12700 / wierszy

            kol = clamp(int(finX // szerKol), 0, kolumn - 1)
            wier = clamp(int(finY // wysWier), 0, wierszy - 1)

            wierszTablicy = (wierszy - 1) - wier
            klawisz = uklad[wierszTablicy][kol]

            if klawisz is None:
                continue
            if klawisz != rig.displayed_text:
                rig.displayText(klawisz)

            kolOd = kolDo = kol
            while kolOd > 0 and uklad[wierszTablicy][kolOd - 1] == klawisz:
                kolOd -= 1
            while kolDo < kolumn - 1 and uklad[wierszTablicy][kolDo + 1] == klawisz:
                kolDo += 1

            wierOd = wierDo = wierszTablicy
            while wierOd > 0 and uklad[wierOd - 1][kol] == klawisz:
                wierOd -= 1
            while wierDo < wierszy - 1 and uklad[wierDo + 1][kol] == klawisz:
                wierDo += 1

            celX = (kolOd + kolDo + 1) / 2 * szerKol
            celY = ((wierszy - 1 - wierDo) + (wierszy - 1 - wierOd) + 1) / 2 * wysWier

            rig.move_to(celX, celY)
        else:
            rig.wait(0.1)

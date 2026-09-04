#!/usr/bin/env python3

import json
import os
import time

import numpy as np

SCIEZKA = "kalibracja.json"

OBSZAR = (7750, 12700)

WERSJA = 1


def zapisz(H, obszar=OBSZAR, sciezka=SCIEZKA) -> bool:
    if H is None:
        print("Kalibracja: brak macierzy do zapisania.")
        return False
    try:
        dane = {
            "wersja": WERSJA,
            "zapisano": time.strftime("%Y-%m-%d %H:%M:%S"),
            "obszar": list(obszar),
            "H": np.asarray(H, dtype=float).tolist(),
        }
        with open(sciezka, "w", encoding="utf-8") as f:
            json.dump(dane, f, indent=2)
        print(f"Kalibracja zapisana w {os.path.abspath(sciezka)}")
        return True
    except OSError as e:
        print(f"Kalibracja: nie udalo sie zapisac ({e}).")
        return False


def wczytaj(obszar=OBSZAR, sciezka=SCIEZKA):
    if not os.path.exists(sciezka):
        return None

    try:
        with open(sciezka, "r", encoding="utf-8") as f:
            dane = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        print(f"Kalibracja: plik {sciezka} jest nieczytelny ({e}).")
        return None

    zapisany_obszar = tuple(dane.get("obszar", ()))
    if zapisany_obszar and zapisany_obszar != tuple(obszar):
        print(f"Kalibracja: zapisana dla obszaru {zapisany_obszar}, a teraz jest "
              f"{tuple(obszar)}. Wymaga powtorzenia.")
        return None

    try:
        H = np.asarray(dane["H"], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        print("Kalibracja: plik nie zawiera poprawnej macierzy.")
        return None

    if H.shape != (3, 3):
        print(f"Kalibracja: macierz ma ksztalt {H.shape}, oczekiwano (3, 3).")
        return None
    if not np.all(np.isfinite(H)):
        print("Kalibracja: macierz zawiera wartosci nieskonczone lub NaN.")
        return None
    if np.isclose(np.linalg.det(H), 0.0):
        print("Kalibracja: macierz jest osobliwa (rogi byly zdegenerowane).")
        return None

    print(f"Kalibracja wczytana z {sciezka} (zapisano: {dane.get('zapisano', '?')})")
    return H


def istnieje(sciezka=SCIEZKA) -> bool:
    return os.path.exists(sciezka)


def usun(sciezka=SCIEZKA) -> bool:
    if not os.path.exists(sciezka):
        return False
    os.remove(sciezka)
    print(f"Kalibracja: usunieto {sciezka}.")
    return True

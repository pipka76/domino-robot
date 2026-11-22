"""Sdílené drobnosti pro robot i mock (bez závislosti na brian.*)."""
import time

COLOURS = ("NONE", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN")
ROTS = ("NONE", "CW", "CCW")
SLOTS = ("LEFT", "RIGHT")


class TimeUp(Exception):
    """Vyhozeno, když vyprší časový limit jízdy."""


def now():
    # MicroPython má ticks_ms, CPython ne
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms() / 1000.0
    return time.time()


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def sgn(v):
    return 1 if v > 0 else -1 if v < 0 else 0


def other_rot(rot):
    return "CCW" if rot == "CW" else "CW"

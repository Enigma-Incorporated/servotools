"""Changing servo IDs. See docs/protocol.md for why the re-lock matters."""

from __future__ import annotations

from . import registers as reg
from .bus import Bus

TEMP_BASE = 100  # scratch ID space that keeps a renumber collision-free


def set_id(bus: Bus, old: int, new: int) -> bool:
    """Torque off, unlock, write the ID, re-lock. The re-lock commits it to EEPROM."""
    bus.write_byte(old, reg.TORQUE_ENABLE, 0, settle=0.03)
    bus.write_byte(old, reg.LOCK, 0, settle=0.03)
    bus.write_byte(old, reg.ID, new, settle=0.05)
    bus.write_byte(new, reg.LOCK, 1, settle=0.03)
    return bus.ping(new)


def renumber(bus: Bus, mapping: dict[int, int]) -> bool:
    """Apply an old->new ID map. Everyone goes via temp IDs so no two ever collide."""
    ok = True
    for cur, fin in mapping.items():
        ok &= set_id(bus, cur, TEMP_BASE + fin)
    for fin in mapping.values():
        ok &= set_id(bus, TEMP_BASE + fin, fin)
    return ok

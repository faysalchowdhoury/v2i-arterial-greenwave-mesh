"""Signals, SPaT messages, GLOSA and the fuel model.

GLOSA
-----
Given distance ``d`` to the stop line, the current time and the SPaT-derived green windows
``[g_s, g_e)`` of the next signal, pick the earliest window that can be met with a speed in
``[v_min, v_max]``, arriving ``margin`` seconds inside it:

``t_arrive = clip(g_s + margin, d / v_max, d / v_min)``, feasible if ``t_arrive <= g_e - margin``,
and ``v_adv = d / t_arrive``. If no window up to the horizon is feasible, no advice is given and the
driver behaves normally (they will stop).

Fuel
----
Power-based instantaneous fuel model (Akcelik-type, light vehicle):

``P = m v (a + g C_r) + 0.5 rho C_d A v^3``  (W), ``f = alpha + beta P / 1000`` mL/s for ``P > 0``,
``f = alpha`` otherwise, with alpha = 0.375 mL/s (idle), beta = 0.09 mL/kJ, m = 1,500 kg, C_r = 0.012,
C_d A = 0.70 m^2. CO2 = 2.35 g per mL of gasoline. These are representative published values for the
model's form, not calibrated MOVES rates: they rank alternatives; they are not inventory-grade.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

M, G, CR, RHO, CDA = 1500.0, 9.81, 0.012, 1.2, 0.70
ALPHA, BETA, CO2_PER_ML = 0.375, 0.09, 2.35


def fuel_rate_ml_s(v: np.ndarray, a: np.ndarray) -> np.ndarray:
    p = M * v * (a + G * CR) + 0.5 * RHO * CDA * v ** 3
    return np.where(p > 0, ALPHA + BETA * p / 1000.0, ALPHA)


@dataclass(frozen=True)
class Signal:
    x: float
    cycle: float = 90.0
    green: float = 45.0
    yellow: float = 4.0
    offset: float = 0.0

    def state(self, t: float) -> str:
        tc = (t - self.offset) % self.cycle
        if tc < self.green:
            return "G"
        if tc < self.green + self.yellow:
            return "Y"
        return "R"

    def spat(self, t: float) -> tuple[str, float]:
        """(current state, seconds until it changes) - the content of a J2735 SPaT movement event."""
        tc = (t - self.offset) % self.cycle
        if tc < self.green:
            return "G", self.green - tc
        if tc < self.green + self.yellow:
            return "Y", self.green + self.yellow - tc
        return "R", self.cycle - tc


def green_windows_from_spat(state: str, t_change: float, now: float, cycle: float, green: float, yellow: float,
                            horizon: float = 180.0) -> list[tuple[float, float]]:
    """Reconstruct upcoming green windows (absolute time) from one SPaT message plus MAP/timing knowledge."""
    out: list[tuple[float, float]] = []
    if state == "G":
        first_end = now + t_change
        out.append((now, first_end))
        start = first_end + yellow + (cycle - green - yellow)
    elif state == "Y":
        start = now + t_change + (cycle - green - yellow)
    else:
        start = now + t_change
    while start < now + horizon:
        out.append((start, start + green))
        start += cycle
    return out


def glosa(d: float, now: float, windows: list[tuple[float, float]], v_min: float, v_max: float,
          margin: float = 2.0, queue_clear_s: float = 0.0) -> float | None:
    """Advisory speed, or ``None`` if no green window is reachable.

    ``queue_clear_s`` (queue-aware GLOSA) shifts the earliest usable arrival past the time the
    vehicles already ahead need to discharge after green onset; naive GLOSA uses 0.
    """
    if d <= 0:
        return None
    t_lo, t_hi = now + d / v_max, now + d / max(v_min, 0.1)
    for gs, ge in windows:
        earliest = gs + margin + queue_clear_s
        t_arr = min(max(earliest, t_lo), t_hi)
        if earliest <= t_arr <= ge - margin:
            return d / (t_arr - now)
    return None

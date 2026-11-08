"""Single-lane arterial microsimulation with SPaT broadcast and GLOSA.

* Vehicles: IDM (v0 = 15.6 m/s / 35 mph, T = 1.2 s, a = 1.5, b = 2.0, s0 = 2 m), Poisson arrivals.
* Signals: fixed-time, broadcasting SPaT every simulation step (5 Hz at dt = 0.2 s). Messages reach equipped vehicles
  within ``comm_range_m`` with probability ``1 - loss`` and ``latency_s`` delay. A vehicle uses the most
  recent message if it is younger than 1 s.
* Equipped vehicles: when advice exists for the next signal, the IDM desired speed is replaced by the
  advisory speed (smoothly, never above the limit). Car following and red-light stopping always apply,
  so the advice can never make a vehicle unsafe.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np
import pandas as pd

from .core import Signal, fuel_rate_ml_s, glosa, green_windows_from_spat

V_LIMIT = 15.6
V_MIN_ADV = 6.7  # 15 mph: lowest advisory speed shown


@dataclass
class ArterialConfig:
    signals: tuple[Signal, ...] = (Signal(400, offset=0), Signal(800, offset=20), Signal(1300, offset=50), Signal(1700, offset=15))
    length_m: float = 2000.0
    demand_vph: float = 600.0
    equipped_share: float = 0.0
    loss: float = 0.0
    latency_s: float = 0.1
    comm_range_m: float = 300.0
    duration_s: float = 2400.0
    warmup_s: float = 300.0
    dt: float = 0.2
    queue_aware: bool = False
    discharge_headway_s: float = 2.0
    advisory_decel: float = 0.6  # max deceleration used to follow advice (~coast-down; braking wastes the energy saved)
    seed: int = 0


@dataclass
class _Msg:
    t: float
    state: str
    t_change: float


@dataclass
class RunResult:
    vehicles: pd.DataFrame
    messages: dict[str, int] = field(default_factory=dict)


def _idm(v: np.ndarray, v0: np.ndarray, gap: np.ndarray, dv: np.ndarray) -> np.ndarray:
    a_max, b, t_h, s0 = 1.5, 2.0, 1.2, 2.0
    s_star = s0 + np.maximum(0.0, v * t_h + v * dv / (2 * math.sqrt(a_max * b)))
    free = 1 - (v / np.maximum(v0, 0.1)) ** 4
    return a_max * (free - (s_star / np.maximum(gap, 0.1)) ** 2)


def simulate(cfg: ArterialConfig) -> RunResult:
    rng = np.random.default_rng(cfg.seed)
    dt = cfg.dt
    n_steps = int(cfg.duration_s / dt)
    arrivals = np.cumsum(rng.exponential(3600.0 / cfg.demand_vph, int(cfg.demand_vph * cfg.duration_s / 3600 * 1.4) + 10))
    arrivals = arrivals[arrivals < cfg.duration_s]
    equipped_draw = rng.random(arrivals.size) < cfg.equipped_share
    chan = np.random.default_rng(cfg.seed + 1000)
    sig_x = np.array([s.x for s in cfg.signals])

    # state arrays (front of platoon first)
    x = np.empty(0)
    v = np.empty(0)
    ids = np.empty(0, int)
    eq = np.empty(0, bool)
    t_enter = np.empty(0)
    stats: dict[int, dict[str, float]] = {}
    last_msg: dict[tuple[int, int], _Msg] = {}  # (vehicle, signal) -> latest delivered SPaT
    pending: list[tuple[float, int, int, str, float]] = []
    published = delivered = dropped = 0
    a_prev = np.empty(0)
    next_arr = 0
    for k in range(n_steps):
        t = k * dt
        # spawn
        while next_arr < arrivals.size and arrivals[next_arr] <= t:
            if x.size == 0 or x[-1] > 8.0:
                x = np.r_[x, 0.0]
                v = np.r_[v, V_LIMIT * 0.9]
                ids = np.r_[ids, next_arr]
                eq = np.r_[eq, equipped_draw[next_arr]]
                t_enter = np.r_[t_enter, arrivals[next_arr]]
                a_prev = np.r_[a_prev, 0.0]
                stats[next_arr] = {"fuel_ml": 0.0, "stops": 0, "idle_s": 0.0, "stopped": 0.0}
                next_arr += 1
            else:
                break
        # SPaT broadcast once per simulation step (dt = 0.2 s -> 5 Hz; J2735 deployments typically use 10 Hz)
        if x.size:
            for kk, s in enumerate(cfg.signals):
                st, tch = s.spat(t)
                in_range = eq & (x < s.x) & (s.x - x <= cfg.comm_range_m)
                for i in np.flatnonzero(in_range):
                    published += 1
                    if chan.random() < cfg.loss:
                        dropped += 1
                        continue
                    heapq.heappush(pending, (t + cfg.latency_s, int(ids[i]), kk, st, tch))
            while pending and pending[0][0] <= t + 1e-9:
                ta, vid, kk, st, tch = heapq.heappop(pending)
                # t_change was measured at send time; by arrival, latency_s of it has elapsed
                last_msg[(vid, kk)] = _Msg(ta, st, tch - cfg.latency_s)
                delivered += 1
        if x.size == 0:
            continue
        # next signal ahead of each vehicle
        nxt = np.searchsorted(sig_x, x + 1e-6)
        has_sig = nxt < sig_x.size
        d_sig = np.where(has_sig, sig_x[np.minimum(nxt, sig_x.size - 1)] - x, np.inf)
        v_adv = np.full(x.size, np.nan)
        for i in np.flatnonzero(eq & has_sig & (d_sig <= cfg.comm_range_m)):
            kk = int(nxt[i])
            msg = last_msg.get((int(ids[i]), kk))
            if msg is None or t - msg.t > 1.0:
                continue
            s = cfg.signals[kk]
            age = t - msg.t
            wins = green_windows_from_spat(msg.state, max(msg.t_change - age, 0.0), t, s.cycle, s.green, s.yellow)
            q_clear = 0.0
            if cfg.queue_aware:  # vehicles already queued before this stop line (known from their BSMs)
                n_queued = int(np.sum((x[:i] < s.x) & (x[:i] > x[i]) & (v[:i] < 2.0)))
                q_clear = n_queued * cfg.discharge_headway_s
            adv = glosa(float(d_sig[i]), t, wins, V_MIN_ADV, V_LIMIT, queue_clear_s=q_clear)
            if adv is not None:
                v_adv[i] = adv
        # car following + red-light stopping (always active), then comfortable tracking of any advice
        v0 = np.full(x.size, V_LIMIT)
        gap = np.r_[np.inf, x[:-1] - x[1:] - 4.5]
        dv = np.r_[0.0, v[1:] - v[:-1]]
        a = _idm(v, v0, gap, dv)
        advised = ~np.isnan(v_adv)
        a = np.where(advised, np.minimum(a, np.clip((np.nan_to_num(v_adv) - v) / 3.0, -cfg.advisory_decel, 1.0)), a)
        for kk, s in enumerate(cfg.signals):
            st = s.state(t)
            if st == "G":
                continue
            ahead = (nxt == kk) & has_sig
            dist = s.x - x
            must_stop = ahead & ((st == "R") | (dist > v * v / (2 * 3.0) + v * 1.0))
            if must_stop.any():
                a_stop = _idm(v, v0, np.maximum(dist - 1.0, 0.1), v)
                a = np.where(must_stop, np.minimum(a, a_stop), a)
        a = np.clip(a, -6.0, 1.5)
        v_new = np.maximum(v + a * dt, 0.0)
        a_real = (v_new - v) / dt
        x = x + 0.5 * (v + v_new) * dt
        fuel = fuel_rate_ml_s(v_new, a_real) * dt
        for j, vid in enumerate(ids):
            s_ = stats[int(vid)]
            s_["fuel_ml"] += float(fuel[j])
            if v_new[j] < 0.5:
                s_["idle_s"] += dt
                if not s_["stopped"]:
                    s_["stops"] += 1
                    s_["stopped"] = 1.0
            elif v_new[j] > 3.0:
                s_["stopped"] = 0.0
        v = v_new
        a_prev = a_real
        done = x >= cfg.length_m
        if done.any():
            for j in np.flatnonzero(done):
                stats[int(ids[j])]["t_exit"] = t + dt
            keep = ~done
            x, v, ids, eq, t_enter, a_prev = x[keep], v[keep], ids[keep], eq[keep], t_enter[keep], a_prev[keep]
    rows = []
    for vid, s_ in stats.items():
        if "t_exit" not in s_ or arrivals[vid] < cfg.warmup_s:
            continue
        rows.append({"vehicle": vid, "equipped": bool(equipped_draw[vid]), "travel_time_s": s_["t_exit"] - arrivals[vid],
                     "stops": s_["stops"], "idle_s": s_["idle_s"], "fuel_ml": s_["fuel_ml"],
                     "co2_g": s_["fuel_ml"] * 2.35})
    return RunResult(pd.DataFrame(rows), {"published": published, "delivered": delivered, "dropped": dropped})

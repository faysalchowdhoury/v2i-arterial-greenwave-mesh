"""Equipped share x communication range x packet loss sweep (10 seeds), report and figure."""

from __future__ import annotations

import json
import math
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from scipy import stats

from .sim import ArterialConfig, simulate

SHARES = (0.0, 0.25, 0.5, 0.75, 1.0)
RANGES = (300.0, 600.0)
LOSSES = (0.0, 0.3)


def _one(args: tuple[float, float, float, bool, int]) -> dict[str, Any]:
    share, rng_m, loss, qa, seed = args
    r = simulate(ArterialConfig(equipped_share=share, comm_range_m=rng_m, loss=loss, queue_aware=qa, seed=seed))
    b = r.vehicles
    return {"share": share, "range_m": rng_m, "loss": loss, "queue_aware": qa, "seed": seed, "vehicles": len(b),
            "stops": b["stops"].mean(), "travel_time_s": b["travel_time_s"].mean(), "idle_s": b["idle_s"].mean(),
            "fuel_ml": b["fuel_ml"].mean(), "co2_g": b["co2_g"].mean(),
            "stops_unequipped": b.loc[~b["equipped"], "stops"].mean() if (~b["equipped"]).any() else math.nan,
            "stops_equipped": b.loc[b["equipped"], "stops"].mean() if b["equipped"].any() else math.nan,
            "msgs_published": r.messages["published"], "msgs_dropped": r.messages["dropped"]}


def run(out_dir: str | Path, seeds: int = 10, workers: int = 2) -> dict[str, Any]:
    out = Path(out_dir)
    (out / "figures").mkdir(parents=True, exist_ok=True)
    jobs = [(0.0, 300.0, 0.0, False, s) for s in range(seeds)]
    jobs += [(sh, r, l, False, s) for sh in SHARES[1:] for r in RANGES for l in LOSSES for s in range(seeds)]
    jobs += [(1.0, r, 0.0, True, s) for r in RANGES for s in range(seeds)]  # queue-aware variant
    t0 = time.perf_counter()
    with ProcessPoolExecutor(workers) as pool:
        rows = list(pool.map(_one, jobs))
    df = pd.DataFrame(rows)
    df.to_csv(out / "runs.csv", index=False)
    base = df[(df["share"] == 0)].sort_values("seed")
    groups = []
    for key, sub in df.groupby(["share", "range_m", "loss", "queue_aware"]):
        sub = sub.sort_values("seed")
        e = {"share": key[0], "range_m": key[1], "loss": key[2], "queue_aware": key[3]}
        for m in ("stops", "travel_time_s", "idle_s", "fuel_ml", "co2_g", "stops_unequipped", "stops_equipped"):
            x = sub[m].to_numpy(float)
            e[m] = float(np.nanmean(x))
            if m in ("stops", "travel_time_s", "fuel_ml") and key[0] > 0:
                b = base[m].to_numpy(float)
                e[m + "_pct"] = 100 * (x.mean() - b.mean()) / b.mean()
                e[m + "_p"] = float(stats.ttest_rel(b, x).pvalue)
        groups.append(e)
    summary = {"runs": len(df), "wall_s": time.perf_counter() - t0, "groups": groups}
    (out / "summary.json").write_text(json.dumps(summary, indent=2, default=float), encoding="utf-8")
    _plot(pd.DataFrame(groups), out / "figures")
    return summary


def _plot(g: pd.DataFrame, fig_dir: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    g = g[~g["queue_aware"]]
    b = g[g["share"] == 0].iloc[0]
    fig, axes = plt.subplots(1, 3, figsize=(15, 3.8))
    for (r, l), col, ls in (((300.0, 0.0), "#1c7ed6", "-"), ((300.0, 0.3), "#1c7ed6", "--"), ((600.0, 0.0), "#2b8a3e", "-"), ((600.0, 0.3), "#2b8a3e", "--")):
        sub = pd.concat([pd.DataFrame([b]), g[(g["range_m"] == r) & (g["loss"] == l)]]).sort_values("share")
        lab = f"{r:.0f} m, loss {l:.0%}"
        axes[0].plot(100 * sub["share"], sub["stops"], ls, marker="o", color=col, label=lab)
        axes[1].plot(100 * sub["share"], sub["fuel_ml"], ls, marker="o", color=col)
        axes[2].plot(100 * sub["share"], sub["travel_time_s"], ls, marker="o", color=col)
    axes[0].set_ylabel("stops per vehicle")
    axes[1].set_ylabel("fuel per vehicle (mL, 2 km)")
    axes[2].set_ylabel("travel time (s)")
    for ax in axes:
        ax.set_xlabel("GLOSA-equipped share (%)")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(fig_dir / "glosa_sweep.png", dpi=130)
    plt.close(fig)


def write_report(s: dict[str, Any], out: Path) -> Path:
    g = pd.DataFrame(s["groups"]).sort_values(["queue_aware", "range_m", "loss", "share"])
    L = ["# GLOSA study", "", f"{s['runs']} runs (10 seeds per cell, 40 min each after 5-min warm-up) in {s['wall_s']:.0f} s.", "",
         "| Equipped | Range | Loss | Queue-aware | Stops/veh | Stops: unequipped / equipped | Travel time (s) | Idle (s) | Fuel (mL) | CO2 (g) |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for r in g.itertuples():
        def pct(m: str) -> str:
            v = getattr(r, m + "_pct", None)
            p = getattr(r, m + "_p", None)
            if v is None or (isinstance(v, float) and math.isnan(v)):
                return ""
            return f" ({v:+.0f}%{'' if p is not None and p < 0.05 else ', n.s.'})"
        L.append(f"| {100 * r.share:.0f}% | {r.range_m:.0f} m | {100 * r.loss:.0f}% | {'yes' if r.queue_aware else 'no'} | "
                 f"{r.stops:.2f}{pct('stops')} | {r.stops_unequipped:.2f} / {r.stops_equipped:.2f} | {r.travel_time_s:.1f}{pct('travel_time_s')} | "
                 f"{r.idle_s:.1f} | {r.fuel_ml:.1f}{pct('fuel_ml')} | {r.co2_g:.0f} |")
    L += ["", "![GLOSA sweep](figures/glosa_sweep.png)", ""]
    md = out / "RESULTS.md"
    md.write_text("\n".join(L), encoding="utf-8")
    return md

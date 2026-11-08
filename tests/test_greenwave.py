from __future__ import annotations

import numpy as np
import pytest
from fastapi.testclient import TestClient

from greenwave.api.app import create_app
from greenwave.core import Signal, fuel_rate_ml_s, glosa, green_windows_from_spat
from greenwave.sim import ArterialConfig, simulate


def test_signal_spat() -> None:
    s = Signal(100, cycle=90, green=45, yellow=4, offset=10)
    assert s.spat(10.0) == ("G", 45.0)
    assert s.spat(56.0)[0] == "Y" and s.spat(60.0) == ("R", pytest.approx(40.0))


def test_windows_reconstruction_matches_signal() -> None:
    s = Signal(0, offset=7)
    for t in (3.0, 30.0, 52.5, 80.0):
        st, tc = s.spat(t)
        wins = green_windows_from_spat(st, tc, t, s.cycle, s.green, s.yellow)
        for gs, ge in wins:
            mid = 0.5 * (gs + ge)
            assert s.state(mid) == "G"


def test_glosa_cases() -> None:
    wins = [(20.0, 65.0), (110.0, 155.0)]
    # 300 m away at t=0: max speed arrives at 19.2 s (red) -> advice to arrive at 22 s
    assert glosa(300.0, 0.0, wins, 6.7, 15.6) == pytest.approx(300 / 22.0)
    # too far to make the first window slowly enough? then the second one
    assert glosa(50.0, 0.0, [(30.0, 40.0)], 6.7, 15.6) is None
    # queue-aware delays the target arrival
    assert glosa(300.0, 0.0, wins, 6.7, 15.6, queue_clear_s=10.0) == pytest.approx(300 / 32.0)


def test_fuel_model() -> None:
    assert float(fuel_rate_ml_s(np.array(0.0), np.array(0.0))) == pytest.approx(0.375)
    assert float(fuel_rate_ml_s(np.array(15.0), np.array(1.0))) > float(fuel_rate_ml_s(np.array(15.0), np.array(0.0)))
    assert float(fuel_rate_ml_s(np.array(15.0), np.array(-2.0))) == pytest.approx(0.375)  # braking: idle rate, no credit


def test_glosa_reduces_stops_and_loss_is_counted() -> None:
    base = simulate(ArterialConfig(duration_s=1500, warmup_s=200, seed=3)).vehicles
    full = simulate(ArterialConfig(duration_s=1500, warmup_s=200, equipped_share=1.0, comm_range_m=600, seed=3))
    assert full.vehicles["stops"].mean() < 0.7 * base["stops"].mean()
    # same arrivals; a couple more may finish inside the horizon when they stop less
    assert abs(len(full.vehicles) - len(base)) <= 0.03 * len(base)
    lossy = simulate(ArterialConfig(duration_s=600, warmup_s=100, equipped_share=1.0, loss=0.5, seed=3))
    assert lossy.messages["dropped"] / lossy.messages["published"] == pytest.approx(0.5, abs=0.02)


def test_api() -> None:
    c = TestClient(create_app())
    r = c.post("/v1/advisory", json={"distance_m": 300, "state": "R", "time_to_change_s": 20})
    assert r.status_code == 200 and r.json()["advisory_speed_mps"] == pytest.approx(300 / 22.0, abs=0.01)
    assert c.post("/v1/advisory", json={"distance_m": -1, "state": "R", "time_to_change_s": 1}).status_code == 422

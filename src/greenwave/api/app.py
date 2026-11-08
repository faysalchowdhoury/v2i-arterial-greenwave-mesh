"""``POST /v1/advisory``: GLOSA speed for one vehicle from one SPaT message (what an RSU or app would serve)."""

from __future__ import annotations

from typing import Literal

from fastapi import FastAPI
from pydantic import BaseModel, Field

from .. import __version__
from ..core import glosa, green_windows_from_spat


class AdvisoryRequest(BaseModel):
    distance_m: float = Field(gt=0, le=2000)
    state: Literal["G", "Y", "R"]
    time_to_change_s: float = Field(ge=0, le=300)
    cycle_s: float = Field(90, gt=20, le=300)
    green_s: float = Field(45, gt=1)
    yellow_s: float = Field(4, ge=3, le=6)
    v_min_mps: float = Field(6.7, gt=0)
    v_max_mps: float = Field(15.6, gt=0)
    queued_vehicles: int = Field(0, ge=0, le=200)


def create_app() -> FastAPI:
    app = FastAPI(title="GLOSA advisory", version=__version__)

    @app.get("/health")
    def health() -> dict[str, str]:
        return {"status": "ok", "version": __version__}

    @app.post("/v1/advisory")
    def advisory(r: AdvisoryRequest) -> dict[str, float | None | str]:
        wins = green_windows_from_spat(r.state, r.time_to_change_s, 0.0, r.cycle_s, r.green_s, r.yellow_s)
        v = glosa(r.distance_m, 0.0, wins, r.v_min_mps, r.v_max_mps, queue_clear_s=2.0 * r.queued_vehicles)
        return {"advisory_speed_mps": None if v is None else round(v, 2),
                "message": "no green reachable within speed range: prepare to stop" if v is None else "ok"}

    return app

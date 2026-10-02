"""Command-line fusion of one IMU burst and a GPS fix."""

from __future__ import annotations

import asyncio
import logging

from .engine import KinematicSensorFusionSafeguard
from .wire import KIND_GPS, KIND_IMU


def _imu(seq: int, t_ns: int) -> dict[str, object]:
    return {
        "kind": KIND_IMU,
        "seq": seq,
        "t_ns": t_ns,
        "ax": 1.5,
        "ay": 0.02,
        "az": -0.01,
    }


def main() -> int:
    """Fuse a short trace and log the state dict."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S%z",
    )
    fusion = KinematicSensorFusionSafeguard()
    t_ns = 1_000_000_000
    records: list[dict[str, object]] = []
    seq = 1
    for _ in range(10):
        records.append(_imu(seq, t_ns))
        seq += 1
        t_ns += 10_000_000
    seq += 2
    t_ns += 20_000_000
    records.append(_imu(seq, t_ns))
    records.append(
        {
            "kind": KIND_GPS,
            "seq": 1,
            "t_ns": t_ns + 20_000_000,
            "px": 0.05,
            "py": 0.0,
            "pz": 0.0,
            "vx": 0.30,
            "vy": 0.0,
            "vz": 0.0,
        }
    )
    report = asyncio.run(fusion.run(records))
    logging.getLogger(__name__).info("%s", report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

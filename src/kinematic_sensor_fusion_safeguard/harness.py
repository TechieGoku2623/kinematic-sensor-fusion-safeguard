"""Deterministic checks and a 5000-iteration latency benchmark."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import random
import sys
import time
import tracemalloc

from .engine import (
    GRAVITY_MPS2,
    NOMINAL_DT_S,
    SKEW_BUDGET_NS,
    KinematicSensorFusionSafeguard,
)
from .exceptions import EngineKernelException
from .wire import KIND_GPS, KIND_IMU, pack_frame, unpack_frame

ITERATIONS: int = 5_000
SEED: int = 178


def _percentile(samples: list[float], fraction: float) -> float:
    ordered = sorted(samples)
    if not ordered:
        return 0.0
    index = math.ceil(fraction * len(ordered)) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def _imu(
    seq: int, t_ns: int, ax: float, ay: float = 0.0, az: float = 0.0
) -> dict[str, object]:
    return {
        "kind": KIND_IMU,
        "seq": seq,
        "t_ns": t_ns,
        "ax": ax,
        "ay": ay,
        "az": az,
    }


def _gps(
    seq: int,
    t_ns: int,
    px: float,
    py: float,
    pz: float,
    vx: float,
    vy: float,
    vz: float,
) -> dict[str, object]:
    return {
        "kind": KIND_GPS,
        "seq": seq,
        "t_ns": t_ns,
        "px": px,
        "py": py,
        "pz": pz,
        "vx": vx,
        "vy": vy,
        "vz": vz,
    }


def _integrate(samples: int, accel: float, dt: float) -> tuple[float, float]:
    position = 0.0
    velocity = 0.0
    for _ in range(samples):
        position = position + velocity * dt + 0.5 * accel * dt * dt
        velocity = velocity + accel * dt
    return position, velocity


async def _checks(failures: list[str]) -> None:
    payload = pack_frame(KIND_IMU, 4, 1_000_000_000, 0.5, -0.25, 0.0, 0.0, 0.0, 0.0)
    frame = unpack_frame(payload)
    if frame["kind"] != KIND_IMU or frame["seq"] != 4 or frame["c0"] != 0.5:
        failures.append("IMU frame did not round-trip")
    if frame["t_ns"] != 1_000_000_000 or frame["c1"] != -0.25:
        failures.append("IMU channels did not round-trip")
    gps_payload = pack_frame(KIND_GPS, 2, 5, 1.5, 2.5, 3.5, 0.5, 0.0, -0.5)
    gps_frame = unpack_frame(gps_payload)
    if gps_frame["c0"] != 1.5 or gps_frame["c3"] != 0.5 or gps_frame["c5"] != -0.5:
        failures.append("GPS frame did not round-trip")
    try:
        unpack_frame(b"\x00\x01")
        failures.append("short sensor frame did not raise")
    except EngineKernelException as exc:
        if not str(exc):
            failures.append("short sensor frame exception had no message")

    stamp = 1_000_000_000
    rows = [_imu(step + 1, stamp + step * 10_000_000, 2.0) for step in range(10)]
    report = await KinematicSensorFusionSafeguard().run(rows)
    expected_p, expected_v = _integrate(10, 2.0, NOMINAL_DT_S)
    if not math.isclose(float(report["position"][0]), expected_p, abs_tol=1e-12):
        failures.append(f"position was {report['position']}")
    if not math.isclose(float(report["velocity"][0]), expected_v, abs_tol=1e-12):
        failures.append(f"velocity was {report['velocity']}")
    if report["rejected_updates"] or report["dropouts"] or report["skew_rejects"]:
        failures.append("clean IMU trace counted a fault")
    if not all(math.isfinite(float(value)) for value in report["position"]):
        failures.append("position was not finite")

    gated = await KinematicSensorFusionSafeguard().run(
        [
            _gps(1, 2_000_000_000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            _gps(2, 2_010_000_000, 5_000.0, 5_000.0, 5_000.0, 0.0, 0.0, 0.0),
        ]
    )
    if int(gated["rejected_updates"]) != 3:
        failures.append(f"chi-square rejects were {gated['rejected_updates']}")
    if any(abs(float(value)) > 1.0 for value in gated["position"]):
        failures.append(f"gated position moved: {gated['position']}")

    gapped = await KinematicSensorFusionSafeguard().run(
        [
            _imu(1, 3_000_000_000, 2.0),
            _imu(4, 3_030_000_000, 2.0),
        ]
    )
    if gapped["dropouts"] != 2:
        failures.append(f"dropout count was {gapped['dropouts']}")
    if not math.isfinite(float(gapped["velocity"][0])):
        failures.append("dropout predict was not finite")

    try:
        await KinematicSensorFusionSafeguard().run(
            [
                _imu(1, 5_000_000_000, 0.1),
                _gps(
                    1,
                    5_000_000_000 + SKEW_BUDGET_NS + 1,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ),
            ]
        )
        failures.append("IMU/GPS skew over budget did not raise")
    except EngineKernelException as exc:
        if "skew" not in str(exc):
            failures.append("skew exception text missing")

    try:
        await KinematicSensorFusionSafeguard().run(
            [_imu(1, 6_000_000_000, 13.0 * GRAVITY_MPS2)]
        )
        failures.append("acceleration above 12 g did not raise")
    except EngineKernelException as exc:
        if "12 g" not in str(exc):
            failures.append("12 g exception text missing")

    held = await KinematicSensorFusionSafeguard().run(
        [_imu(1, 7_000_000_000, 12.0 * GRAVITY_MPS2, 0.0, 0.0)]
    )
    if not math.isfinite(float(held["velocity"][0])):
        failures.append("exactly 12 g was rejected")

    try:
        await KinematicSensorFusionSafeguard().run(
            [_imu(1, 8_000_000_000, float("nan"))]
        )
        failures.append("NaN acceleration did not raise")
    except EngineKernelException as exc:
        if not str(exc):
            failures.append("NaN acceleration exception had no message")


def _batches(rng: random.Random) -> list[list[dict[str, object]]]:
    batches: list[list[dict[str, object]]] = []
    stamp = 10_000_000_000
    for seq in range(1, ITERATIONS + 1):
        ax = 0.2 + rng.random() * 0.05
        batches.append([_imu(seq, stamp, ax, 0.0, 0.01)])
        stamp += 10_000_000
    return batches


async def _benchmark(batches: list[list[dict[str, object]]]) -> list[float]:
    fusion = KinematicSensorFusionSafeguard()
    samples: list[float] = []
    for batch in batches:
        started = time.perf_counter_ns()
        await fusion.run(batch)
        samples.append((time.perf_counter_ns() - started) / 1_000.0)
    return samples


def main() -> int:
    """Print one status dict and exit 0 only when every check passes."""
    logging.getLogger("kinematic_sensor_fusion_safeguard").setLevel(logging.ERROR)
    failures: list[str] = []
    asyncio.run(_checks(failures))
    probe = KinematicSensorFusionSafeguard()
    started = time.perf_counter_ns()
    asyncio.run(probe.run([_imu(1, 1_000, 0.1)]))
    latency_us = (time.perf_counter_ns() - started) / 1_000.0
    batches = _batches(random.Random(SEED))
    tracemalloc.start()
    samples = asyncio.run(_benchmark(batches))
    _current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    status = {
        "status": "PASS" if not failures else "FAIL",
        "failures": failures,
        "latency_us": round(latency_us, 3),
        "memory_peak_bytes": peak,
        "benchmark_iterations": len(samples),
        "benchmark_avg_us": round(sum(samples) / len(samples), 3),
        "benchmark_p99_us": round(_percentile(samples, 0.99), 3),
    }
    sys.stdout.write(json.dumps(status) + "\n")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())

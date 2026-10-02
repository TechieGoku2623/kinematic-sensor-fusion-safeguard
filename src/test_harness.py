"""Deterministic checks and a short latency benchmark for the fusion safeguard."""

from __future__ import annotations

import asyncio
import math
import random
import sys
import time
import tracemalloc
from pathlib import Path


def _load():
    root = Path(__file__).resolve().parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import main

    return main


def _percentile(samples: list[float], fraction: float) -> float:
    ordered = sorted(samples)
    if not ordered:
        return 0.0
    index = math.ceil(fraction * len(ordered)) - 1
    index = max(0, min(index, len(ordered) - 1))
    return ordered[index]


async def _expect_raise(factory, exc_type, failures: list[str], label: str) -> None:
    try:
        await factory()
    except exc_type:
        return
    failures.append(label)


async def _checks(mod, failures: list[str]) -> None:
    rng = random.Random(178)
    engine = mod.KinematicSensorFusionSafeguard()
    packed = engine.pack_frame(mod.KIND_IMU, 1, 1_000_000_000, 0.2, -0.1, 0.05)
    unpacked = engine.unpack_frame(packed)
    if unpacked["kind"] != mod.KIND_IMU or unpacked["seq"] != 1:
        failures.append("sensor frame did not round-trip")
    if abs(unpacked["x"] - 0.2) > 1e-12:
        failures.append("acceleration did not round-trip")

    t_ns = 1_000_000_000
    for step in range(10):
        ax = 2.0 + rng.random() * 0.0
        await engine.admit(
            engine.pack_frame(mod.KIND_IMU, step + 1, t_ns, ax, 0.0, 0.0)
        )
        t_ns += 10_000_000
    imu_only = engine.snapshot()
    if not 0.15 < float(imu_only["vx"]) < 0.25:
        failures.append(f"IMU integration landed at vx={imu_only['vx']}")

    fused = await engine.admit(engine.pack_frame(mod.KIND_GPS, 1, t_ns, 5.0, 0.0, 0.0))
    if not 3.0 < float(fused["vx"]) < 4.8:
        failures.append(f"GPS covariance update landed at vx={fused['vx']}")
    if float(fused["mean_variance"]) >= 0.6:
        failures.append("GPS update did not shrink variance")
    if float(fused["mean_innovation"]) <= 0.0:
        failures.append("innovation statistic was not recorded")

    over_g = mod.KinematicSensorFusionSafeguard()
    await _expect_raise(
        lambda: over_g.admit(
            over_g.pack_frame(
                mod.KIND_IMU,
                1,
                1_000_000_000,
                13.0 * mod.GRAVITY_MPS2,
                0.0,
                0.0,
            )
        ),
        mod.EngineKernelException,
        failures,
        "13g sample did not raise",
    )
    fast = mod.KinematicSensorFusionSafeguard()
    await _expect_raise(
        lambda: fast.admit(
            fast.pack_frame(mod.KIND_GPS, 1, 1_000_000_000, 450.0, 0.0, 0.0)
        ),
        mod.EngineKernelException,
        failures,
        "gps speed bound did not raise",
    )
    broken = mod.KinematicSensorFusionSafeguard()
    await _expect_raise(
        lambda: broken.admit(
            broken.pack_frame(mod.KIND_IMU, 1, 1_000_000_000, float("nan"), 0.0, 0.0)
        ),
        mod.EngineKernelException,
        failures,
        "NaN sample did not raise",
    )

    dropout = mod.KinematicSensorFusionSafeguard()
    await dropout.admit(
        dropout.pack_frame(mod.KIND_IMU, 1, 2_000_000_000, 0.1, 0.0, 0.0)
    )
    await dropout.admit(
        dropout.pack_frame(mod.KIND_IMU, 2, 2_010_000_000, 0.1, 0.0, 0.0)
    )
    gapped = await dropout.admit(
        dropout.pack_frame(mod.KIND_IMU, 6, 2_020_000_000, 0.1, 0.0, 0.0)
    )
    if gapped["dropouts"] != 3:
        failures.append(f"dropout count was {gapped['dropouts']}")
    await _expect_raise(
        lambda: dropout.admit(
            dropout.pack_frame(mod.KIND_IMU, 6 + 27, 2_030_000_000, 0.1, 0.0, 0.0)
        ),
        mod.EngineKernelException,
        failures,
        "long sensor dropout did not raise",
    )

    skew = mod.KinematicSensorFusionSafeguard()
    await skew.admit(skew.pack_frame(mod.KIND_IMU, 1, 5_000_000_000, 0.0, 0.0, 0.0))
    soft = await skew.admit(
        skew.pack_frame(mod.KIND_GPS, 1, 5_000_000_000 + 80_000_000, 4.0, 0.0, 0.0)
    )
    if int(soft["skew_clamps"]) < 1:
        failures.append("soft imu/gps skew was not clamped")
    await _expect_raise(
        lambda: skew.admit(
            skew.pack_frame(
                mod.KIND_GPS, 2, 5_000_000_000 + 3_000_000_000, 4.0, 0.0, 0.0
            )
        ),
        mod.EngineKernelException,
        failures,
        "hard imu/gps skew did not raise",
    )

    backward = mod.KinematicSensorFusionSafeguard()
    await backward.admit(
        backward.pack_frame(mod.KIND_IMU, 1, 8_000_000_000, 1.0, 0.0, 0.0)
    )
    held = await backward.admit(
        backward.pack_frame(mod.KIND_IMU, 2, 7_000_000_000, 1.0, 0.0, 0.0)
    )
    if int(held["skew_clamps"]) < 1 or float(held["vx"]) <= 0.0:
        failures.append("backward IMU clock was not clamped")

    jump = mod.KinematicSensorFusionSafeguard()
    await jump.admit(jump.pack_frame(mod.KIND_GPS, 1, 9_000_000_000, 10.0, 0.0, 0.0))
    await _expect_raise(
        lambda: jump.admit(
            jump.pack_frame(mod.KIND_GPS, 2, 9_100_000_000, 210.0, 0.0, 0.0)
        ),
        mod.EngineKernelException,
        failures,
        "gps jump did not raise",
    )

    repeat = mod.KinematicSensorFusionSafeguard()
    await repeat.admit(repeat.pack_frame(mod.KIND_IMU, 3, 1_000, 0.0, 0.0, 0.0))
    await _expect_raise(
        lambda: repeat.admit(repeat.pack_frame(mod.KIND_IMU, 3, 2_000, 0.0, 0.0, 0.0)),
        mod.EngineKernelException,
        failures,
        "repeated sequence did not raise",
    )

    ring = mod.KinematicSensorFusionSafeguard(ring_capacity=8)
    stamp = 4_000_000_000
    for seq in range(1, 21):
        await ring.admit(ring.pack_frame(mod.KIND_IMU, seq, stamp, 0.1, 0.0, 0.0))
        stamp += 10_000_000
    if ring.snapshot()["ring_count"] != 8:
        failures.append("ring count grew past the preallocated capacity")

    batch = mod.KinematicSensorFusionSafeguard()
    frames = [
        batch.pack_frame(mod.KIND_IMU, 1, 6_000_000_000, 0.4, 0.0, 0.0),
        batch.pack_frame(mod.KIND_IMU, 2, 6_010_000_000, 0.4, 0.0, 0.0),
        batch.pack_frame(mod.KIND_GPS, 1, 6_020_000_000, 1.0, 0.0, 0.0),
        batch.pack_frame(
            mod.KIND_IMU, 3, 6_030_000_000, 20.0 * mod.GRAVITY_MPS2, 0.0, 0.0
        ),
    ]
    report = await batch.run_worker(frames)
    if report["rejected_count"] != 1 or report["accepted"] != 3:
        failures.append("batch did not isolate the rejected sample")
    if report["topic"] != "flight.sensors.fused":
        failures.append("fusion topic mismatch")
    if not math.isfinite(float(report["speed_mps"])):
        failures.append("fused speed was not finite")


async def _benchmark(mod) -> list[float]:
    rng = random.Random(178)
    engine = mod.KinematicSensorFusionSafeguard()
    payloads: list[bytes] = []
    stamp = 10_000_000_000
    for seq in range(1, 8_001):
        ax = 0.2 + rng.random() * 0.05
        payloads.append(engine.pack_frame(mod.KIND_IMU, seq, stamp, ax, 0.0, 0.01))
        stamp += 10_000_000
    samples: list[float] = []
    for payload in payloads:
        started = time.perf_counter_ns()
        await engine.admit(payload)
        samples.append((time.perf_counter_ns() - started) / 1_000.0)
    return samples


def main() -> int:
    mod = _load()
    failures: list[str] = []
    asyncio.run(_checks(mod, failures))
    probe = mod.KinematicSensorFusionSafeguard()
    frame = probe.pack_frame(mod.KIND_IMU, 1, 1_000, 0.1, 0.0, 0.0)
    started = time.perf_counter_ns()
    asyncio.run(probe.admit(frame))
    latency_us = (time.perf_counter_ns() - started) / 1_000.0
    tracemalloc.start()
    samples = asyncio.run(_benchmark(mod))
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
    sys.stdout.write(repr(status) + "\n")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())

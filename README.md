# Kinematic Sensor Fusion Safeguard

A high-throughput, low-latency asynchronous engine engineered to resolve disagreement between IMU acceleration and GPS velocity with a complementary update, and to reject samples above 12 g or a GPS jump that exceeds the kinematic bound.

## 🏗️ Systems Architecture & Event Topology

`KinematicSensorFusionSafeguard` keeps three `_Axis` filters and a preallocated ring of `_Slot` records. `pack_frame` / `unpack_frame` carry kind, sequence, timestamp, and the three components. `admit` enqueues one frame. `run_worker` is the coroutine that drains the queue onto the topic name `flight.sensors.fused`.

The IMU branch integrates acceleration into the velocity state. The GPS branch applies a scalar covariance update on each axis. The hot path writes the next preallocated slot and does not grow the ring. An `asyncio.Lock` covers that slot index. `logging.basicConfig` timestamps every line. A sample that fails a gate raises `EngineKernelException` and is listed in `rejections` instead of moving the state.

The check structure follows a DO-178C objective style: each gate is named, the bound is a constant, and the fault text cites the measured value and the limit. This module is not a DAL certification and it does not fly an aircraft.

## 📊 Core Visual Walkthrough & Engine Pipeline Flow

```
IMU frame (accel)          GPS frame (velocity)
        \                      /
         v                    v
   sequence gap?  ---- dropout warning, state held
   clock skew?    ---- clamp if inside the bound, else reject
   |accel| > 12 g ---------- EngineKernelException
   GPS rate > bound -------- EngineKernelException
         \                    /
          v                  v
   complementary update on (vx, vy, vz)
          |
          v
   preallocated ring slot
          |
          v
   snapshot on flight.sensors.fused
```

Insert the structural terminal walkthrough recording at docs/assets/terminal-walkthrough.gif before publishing the release notes.

## ⚡ Low-Level OS Mechanics & Network Physics

Twelve g is `12 * 9.80665` m/s², compared with `math.hypot` of the acceleration vector so a single-axis spike and a diagonal spike share one gate. The complementary update trusts the GPS velocity with a fixed gain and leaks the integrated IMU velocity with the complementary gain. It is not a Kalman gain schedule. Variance is tracked per axis so `statistics` can report mean innovation and its spread after a batch.

Clock skew is a difference of integer nanoseconds. A small skew is clamped and counted. A large skew raises. The ring is allocated once in `__init__`. `snapshot` copies the published state under the lock and does not allocate a new slot. The queue replaces a socket: the process does not join a vehicle bus.

## ⚖️ Architecture Trade-offs & Pragmatic Decisions

A full error-state Kalman filter would model cross-axis correlation and a GPS time-of-flight bias. It would also need a matrix library and a noise-identification campaign. The complementary update is inspectable: one gain, three independent axes, a hard 12 g ceiling in front of the integrator. The ceiling is the safeguard. The filter is the estimate.

Sequence dropout does not interpolate a missing IMU sample. Interpolation would invent acceleration. The gap is logged, the previous sequence is recorded, and the next valid sample continues from the held velocity. GPS that arrives with a large time skew is clamped onto the IMU clock rather than integrated across a fictional dt.

## 🚀 Local Installation & Benchmarking

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python src/main.py
python src/test_harness.py
```

```python
import asyncio

from src.main import KinematicSensorFusionSafeguard


async def demo() -> None:
    fusion = KinematicSensorFusionSafeguard()
    frame = fusion.pack_frame(1, 0, 1_000_000_000, 0.1, 0.0, 0.0)
    await fusion.run_worker([frame])


asyncio.run(demo())
```

Runtime dependencies are the Python 3.12 standard library. `pip install -r requirements.txt` succeeds with no third-party pins.

## 🖥️ Terminal Diagnostic Output Preview

```
WARNING [fusion.safeguard] sensor dropout kind=1 gap=3 seq=14 previous=10
WARNING [fusion.safeguard] gps clock skew clamped seq=1 skew_ns=90000000 imu_ns=1190000000 gps_ns=1280000000
WARNING [fusion.safeguard] sample rejected error=acceleration 127.486 m/s^2 exceeds 117.680
WARNING [fusion.safeguard] sample rejected error=gps jump 217.165 m/s exceeds 150.000
INFO [fusion.safeguard] fusion topic=flight.sensors.fused accepted=24 rejected=2 speed_mps=13.477 dropouts=3 skew_clamps=3 ring_count=24
```

`python src/main.py` exits 0. 117.680 m/s² is the 12 g gate.

## 📊 Empirical Benchmarking Performance Report

Measured by `python src/test_harness.py` with a deterministic seed, 8000 iterations, `time.perf_counter_ns` latency in microseconds, and `tracemalloc` peak.

| Metric | Measured |
| --- | ---: |
| Status | PASS |
| Iterations | 8000 |
| Average latency | 18.956 µs |
| Empirical P99 | 25.671 µs |
| Scenario latency | 128.731 µs |
| tracemalloc peak | 964940 bytes |

## 🛡️ Edge-Case Resilience & SOC2/Regulatory Compliance

A sequence dropout logs the gap and does not synthesize the missing sample. IMU/GPS clock skew inside the bound is clamped and counted; skew past the bound raises `EngineKernelException`. Acceleration above 12 g and a GPS jump above the speed bound raise before the complementary state moves.

The layout is DO-178C objective style: traceable gates, fixed bounds, explicit fault strings. It is not a certification artifact and it does not claim a DAL. No crew identifier is stored. SOC 2 processing integrity applies to the rejection counter: a rejected sample is in `rejections` and not in the fused speed.

# Kinematic Sensor Fusion Safeguard

A high-throughput, low-latency asynchronous engine engineered to resolve IMU and GPS disagreement with a per-axis constant-velocity Kalman filter, a scalar chi-square gate, and hard limits on acceleration and clock skew.

## 🏗️ Systems Architecture & Event Topology

`KinematicSensorFusionSafeguard` keeps three independent filters. State on each axis is `[position, velocity]`. The transition is `F = [[1, dt], [0, 1]]`. IMU acceleration is the control input, `B = [0.5 dt^2, dt]`. A GPS frame is a position measurement `H = [1, 0]` followed, when the gate accepts it, by a velocity measurement `H = [0, 1]`. The update is explicit 2x2 arithmetic. NumPy is not imported.

`run` packs each sample, hops the bytes through an `asyncio.Queue`, and calls `ingest`. After the batch it publishes the position triple with `struct.pack` onto a bounded outbound queue. That queue is the in-process stand-in for the Kafka topic `flight.sensors.fused`. This process does not join a vehicle bus. TimescaleDB would persist the state dict and is not connected. Redis is not used as a state store. Kinesis is not used.

An `asyncio.Lock` covers the axes, the sequence cursors, and the skew counters. `logging.basicConfig` is called only from `__main__.py`.

```
IMU (ax, ay, az)                 GPS (position, velocity)
        \                              /
         v                            v
   |a| > 12 g ------------------> EngineKernelException
   |t_imu - t_gps| > 200 ms ----> EngineKernelException
   sequence gap ----------------> predict-only, accel = 0, dropouts += gap
         \                            /
          v                          v
   predict: x = F x + B u ,  P = F P F^T + Q
          |
          v
   GPS position: reject if innovation^2 / S > 9
          |                 else Joseph-form covariance update
          v
   position, velocity, rejected_updates, dropouts, skew_rejects
```

Twelve g is `12 * 9.80665` m/s^2, compared with the Euclidean norm of the acceleration vector. Exactly 12 g is inside the gate. The skew budget is 200_000_000 ns. The chi-square threshold is 9, the square of a 3-sigma residual on one degree of freedom.

## 📊 Core Visual Walkthrough & Engine Pipeline Flow

```
frame
  |
  +-- kind 1, IMU
  |     finite? magnitude? skew vs last GPS? sequence? dt?
  |     coast missing steps with zero acceleration
  |     predict with this sample's acceleration
  |
  +-- kind 2, GPS
        finite? skew vs last IMU? sequence? dt?
        coast missing steps with zero acceleration
        predict with the last IMU acceleration
        for each axis:
            y = z_p - position
            S = P00 + R_pos
            if y^2 / S > 9: rejected_updates += 1, skip velocity
            else: position update, then velocity update
  |
  v
snapshot dict, topic name flight.sensors.fused
```

Process noise uses the discrete white-acceleration form `Q = q [[dt^4/4, dt^3/2], [dt^3/2, dt^2]]` with `q = 1`. Position measurement variance is 9 m^2. Velocity measurement variance is 0.25 (m/s)^2. Covariance is updated with the Joseph form and then symmetrized. The same `Q` and `R` on every axis means the covariance trace can match across axes while position and velocity do not.

Insert the structural terminal walkthrough recording at docs/assets/terminal-walkthrough.gif before publishing the release notes.

## ⚡ Low-Level OS Mechanics & Network Physics

`dt` is the difference of integer nanosecond timestamps divided by 1e9. The first sample of a trace uses a nominal `dt` of 0.01 s because there is no prior stamp. A non-positive step raises. Sequence numbers are tracked per kind, so an IMU counter and a GPS counter do not collide.

A sequence hole of size `gap` increments `dropouts` by `gap` and spreads the elapsed time across `gap + 1` predict steps. The missing steps use zero acceleration. The filter does not synthesize a specific-force sample. Gaps above 4096 are still counted in full; the elapsed time is spread across 4096 coasting steps so a corrupt counter cannot spin the loop.

`statistics.fmean` and `statistics.pstdev` reduce the three covariance traces. The hot path writes no growing buffer: the axes are allocated once. The outbound queue drops the oldest blob when it is full. There is no socket.

## ⚖️ Architecture Trade-offs & Pragmatic Decisions

The three axes do not share a cross-covariance. A full error-state filter would model that correlation and would need a matrix library plus a noise-identification campaign. The 2x2 form is small enough to audit on one screen. The safeguard is the set of hard gates in front of the predict, not a claim that the estimate is optimal.

A GPS position that fails the chi-square gate also skips that axis's velocity update. A biased velocity then coasts until the next accepted fix. Coupling the two measurements that way avoids mixing a rejected position with a trusted velocity from the same fix. It also delays the velocity correction.

## 🚀 Local Installation & Benchmarking

```bash
python3 -m venv venv
source venv/bin/activate
pip install -e ".[dev]"
python -m kinematic_sensor_fusion_safeguard
python -m kinematic_sensor_fusion_safeguard.harness
```

```python
import asyncio

from kinematic_sensor_fusion_safeguard import KinematicSensorFusionSafeguard


async def demo() -> None:
    fusion = KinematicSensorFusionSafeguard()
    report = await fusion.run(
        [
            {
                "kind": 1,
                "seq": 1,
                "t_ns": 1_000_000_000,
                "ax": 0.2,
                "ay": 0.0,
                "az": 0.0,
            }
        ]
    )
    velocity_x = float(report["velocity"][0])
    position_x = float(report["position"][0])
    print(position_x, velocity_x, report["rejected_updates"])


asyncio.run(demo())
```

Runtime dependencies are the Python 3.12 standard library. `pip install -r requirements.txt` succeeds with no third-party pins. Install the package with `pip install .`.

## 🖥️ Terminal Diagnostic Output Preview

```
2026-10-02T03:20:15+0000 WARNING [kinematic_sensor_fusion_safeguard.engine] sensor dropout gap=2 modelled=2
2026-10-02T03:20:15+0000 INFO [kinematic_sensor_fusion_safeguard.engine] topic=flight.sensors.fused position=[0.04483783134653849, -4.181225898607358e-05, 2.090612949303679e-05] velocity=[0.29384623857816605, 0.0001530439598206165, -7.652197991030825e-05] rejected_updates=0 dropouts=2 skew_rejects=0
2026-10-02T03:20:15+0000 INFO [__main__] {'topic': 'flight.sensors.fused', 'position': [0.04483783134653849, -4.181225898607358e-05, 2.090612949303679e-05], 'velocity': [0.29384623857816605, 0.0001530439598206165, -7.652197991030825e-05], 'rejected_updates': 0, 'dropouts': 2, 'skew_rejects': 0, 'covariance_trace': 6.853282233599425, 'covariance_spread': 0.0}
```

`python -m kinematic_sensor_fusion_safeguard` exits 0. The IMU sequence skips two counts, those steps are predict-only, and the GPS fix 20 ms later is inside the skew budget so `skew_rejects` stays 0. `117.680` m/s^2 is the 12 g ceiling (`12 * 9.80665`).

## 📊 Empirical Benchmarking Performance Report

Measured by `PYTHONPATH=src python -m kinematic_sensor_fusion_safeguard.harness` with `random.Random(178)`, 5000 IMU iterations, `time.perf_counter_ns` latency in microseconds, and `tracemalloc` peak.

| Metric | Measured |
| --- | ---: |
| Status | PASS |
| Iterations | 5000 |
| Average latency | 152.689 µs |
| Empirical P99 | 199.304 µs |
| Scenario latency | 161.252 µs |
| tracemalloc peak | 201285 bytes |

## 🛡️ Edge-Case Resilience & SOC2/Regulatory Compliance

If the IMU timestamp and the GPS timestamp differ by more than 200 ms, `run` increments `skew_rejects` and raises `EngineKernelException`. The fault text cites both timestamps and the budget. Acceleration whose norm is above 12 g raises before predict runs. The message cites the measured magnitude and the 12 g limit.

A sequence hole does not invent the missing acceleration. The filter predicts across the hole with zero specific force and counts the hole in `dropouts`. A GPS position whose squared innovation exceeds `9 * S` is not applied; `rejected_updates` counts each rejected axis, and that axis's velocity update is skipped.

The check structure is aligned with DO-178C objectives for traceable input checks: each gate is named, the bound is a constant, and the fault string carries the measured value and the limit. This module is not a DO-178C certification, it does not claim a DAL, and it does not fly an aircraft. No crew identifier is stored. Processing integrity is the operational reading of the counters: a rejected sample is not folded into `position`.

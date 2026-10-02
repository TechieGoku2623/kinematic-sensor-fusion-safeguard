"""IMU and GPS fusion with explicit physical gates.

The IMU branch is a complementary integrator. The GPS branch is a scalar
covariance update on each axis. Samples that exceed 12 g, a speed bound, a
sequence gap, or the hard clock-skew bound raise ``EngineKernelException``.
The hot path writes a preallocated ring slot and does not grow that buffer.
The in-process queue stands in for the Kafka topic ``flight.sensors.fused``.
Persistence of the state dict would be a TimescaleDB insert performed by the
downstream consumer. This layout follows a DO-178C objective style of
traceable checks. It is not a DAL certification.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import statistics
import struct
import sys
from typing import Final

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S%z",
)

LOGGER = logging.getLogger("fusion.safeguard")

TOPIC: Final[str] = "flight.sensors.fused"
KIND_IMU: Final[int] = 1
KIND_GPS: Final[int] = 2
FRAME: Final[struct.Struct] = struct.Struct("<BIQddd")
GRAVITY_MPS2: Final[float] = 9.80665
MAX_ACCEL_MPS2: Final[float] = 12.0 * GRAVITY_MPS2
MAX_SPEED_MPS: Final[float] = 400.0
JUMP_MPS: Final[float] = 150.0
MAX_SEQ_GAP: Final[int] = 25
SOFT_SKEW_NS: Final[int] = 50_000_000
HARD_SKEW_NS: Final[int] = 2_000_000_000
NOMINAL_DT_S: Final[float] = 0.01
PROCESS_VAR: Final[float] = 0.08
MEAS_VAR: Final[float] = 0.4
COMP_TAU_S: Final[float] = 0.5
DROPOUT_VAR: Final[float] = 0.5
RING_CAPACITY: Final[int] = 256


class EngineKernelException(Exception):
    """Raised when a sensor frame fails a traceable safeguard check."""


class _Axis:
    """Mutable per-axis velocity and variance. Allocated once at start."""

    __slots__ = ("velocity", "variance")

    def __init__(self) -> None:
        self.velocity = 0.0
        self.variance = 1.0


class _Slot:
    """One preallocated ring element."""

    __slots__ = ("kind", "seq", "t_ns", "vx", "vy", "vz", "variance")

    def __init__(self) -> None:
        self.kind = 0
        self.seq = 0
        self.t_ns = 0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.variance = 0.0


def _mean_stdev(samples: list[float]) -> tuple[float, float]:
    if not samples:
        return 0.0, 0.0
    center = statistics.fmean(samples)
    if len(samples) == 1:
        return center, 0.0
    return center, statistics.pstdev(samples)


class KinematicSensorFusionSafeguard:
    """Fuse IMU acceleration with GPS velocity behind physical bounds."""

    def __init__(self, ring_capacity: int = RING_CAPACITY) -> None:
        if ring_capacity < 1:
            raise EngineKernelException("ring capacity must be positive")
        self._capacity = ring_capacity
        self._lock = asyncio.Lock()
        self._inbound: asyncio.Queue[bytes] = asyncio.Queue()
        self._axes = (_Axis(), _Axis(), _Axis())
        self._slots = tuple(_Slot() for _ in range(ring_capacity))
        self._slot_i = 0
        self._slot_n = 0
        self._innov = [0.0] * ring_capacity
        self._innov_i = 0
        self._innov_n = 0
        self._last_seq: dict[int, int | None] = {KIND_IMU: None, KIND_GPS: None}
        self._last_imu_ns: int | None = None
        self._last_gps_ns: int | None = None
        self._gps_updates = 0
        self._accepted = 0
        self._dropouts = 0
        self._skew_clamps = 0

    def pack_frame(
        self,
        kind: int,
        seq: int,
        t_ns: int,
        x: float,
        y: float,
        z: float,
    ) -> bytes:
        """Pack one sensor frame. ``x, y, z`` are accel or velocity by kind."""
        if kind not in (KIND_IMU, KIND_GPS):
            raise EngineKernelException(f"unknown sensor kind {kind}")
        if seq < 0 or t_ns < 0:
            raise EngineKernelException("sequence and timestamp must be non-negative")
        try:
            return FRAME.pack(kind, seq, t_ns, float(x), float(y), float(z))
        except (struct.error, OverflowError) as exc:
            raise EngineKernelException("sensor frame pack failed") from exc

    def unpack_frame(self, payload: bytes) -> dict[str, object]:
        """Unpack one sensor frame."""
        if len(payload) != FRAME.size:
            raise EngineKernelException(
                f"sensor frame length {len(payload)} != {FRAME.size}"
            )
        try:
            kind, seq, t_ns, x_val, y_val, z_val = FRAME.unpack(payload)
        except struct.error as exc:
            raise EngineKernelException("sensor frame unpack failed") from exc
        return {
            "kind": int(kind),
            "seq": int(seq),
            "t_ns": int(t_ns),
            "x": float(x_val),
            "y": float(y_val),
            "z": float(z_val),
        }

    def _reject_non_finite(self, x_val: float, y_val: float, z_val: float) -> None:
        if (
            not math.isfinite(x_val)
            or not math.isfinite(y_val)
            or not math.isfinite(z_val)
        ):
            raise EngineKernelException(
                f"non-finite sample x={x_val} y={y_val} z={z_val}"
            )

    def _note_sequence(self, kind: int, seq: int) -> None:
        previous = self._last_seq[kind]
        if previous is not None:
            if seq <= previous:
                raise EngineKernelException(
                    f"non-monotonic sequence kind={kind} seq={seq} previous={previous}"
                )
            gap = seq - previous - 1
            if gap > 0:
                if gap > MAX_SEQ_GAP:
                    raise EngineKernelException(
                        f"sensor dropout gap={gap} exceeds {MAX_SEQ_GAP}"
                    )
                self._dropouts += gap
                inflate = DROPOUT_VAR * float(gap)
                for axis in self._axes:
                    axis.variance += inflate
                LOGGER.warning(
                    "sensor dropout kind=%d gap=%d seq=%d previous=%d",
                    kind,
                    gap,
                    seq,
                    previous,
                )
        self._last_seq[kind] = seq

    def _push_state(self, kind: int, seq: int, t_ns: int) -> None:
        slot = self._slots[self._slot_i]
        slot.kind = kind
        slot.seq = seq
        slot.t_ns = t_ns
        slot.vx = self._axes[0].velocity
        slot.vy = self._axes[1].velocity
        slot.vz = self._axes[2].velocity
        slot.variance = (
            self._axes[0].variance + self._axes[1].variance + self._axes[2].variance
        ) / 3.0
        self._slot_i = (self._slot_i + 1) % self._capacity
        if self._slot_n < self._capacity:
            self._slot_n += 1

    def _push_innovation(self, value: float) -> None:
        self._innov[self._innov_i] = value
        self._innov_i = (self._innov_i + 1) % self._capacity
        if self._innov_n < self._capacity:
            self._innov_n += 1

    def _innovation_samples(self) -> list[float]:
        if self._innov_n == 0:
            return []
        if self._innov_n < self._capacity:
            return self._innov[: self._innov_n]
        return self._innov[self._innov_i :] + self._innov[: self._innov_i]

    def _apply_imu(
        self,
        seq: int,
        t_ns: int,
        ax: float,
        ay: float,
        az: float,
    ) -> None:
        self._reject_non_finite(ax, ay, az)
        mag_sq = ax * ax + ay * ay + az * az
        if mag_sq > MAX_ACCEL_MPS2 * MAX_ACCEL_MPS2:
            magnitude = math.sqrt(mag_sq)
            raise EngineKernelException(
                f"acceleration {magnitude:.3f} m/s^2 exceeds {MAX_ACCEL_MPS2:.3f}"
            )
        dt = NOMINAL_DT_S
        clamped = False
        if self._last_imu_ns is not None:
            delta_ns = t_ns - self._last_imu_ns
            if delta_ns <= 0:
                clamped = True
                dt = NOMINAL_DT_S
            elif delta_ns > HARD_SKEW_NS:
                raise EngineKernelException(f"imu clock step {delta_ns} ns")
            else:
                dt = delta_ns / 1_000_000_000.0
        self._note_sequence(KIND_IMU, seq)
        if clamped:
            self._skew_clamps += 1
            LOGGER.warning(
                "imu clock skew clamped seq=%d t_ns=%d held_dt_s=%.4f",
                seq,
                t_ns,
                dt,
            )
            if self._last_imu_ns is not None:
                self._last_imu_ns += int(NOMINAL_DT_S * 1_000_000_000.0)
        else:
            self._last_imu_ns = t_ns
        alpha = COMP_TAU_S / (COMP_TAU_S + dt)
        for axis, accel in zip(self._axes, (ax, ay, az), strict=True):
            predicted = axis.velocity + accel * dt
            axis.velocity = alpha * predicted + (1.0 - alpha) * axis.velocity
            axis.variance += PROCESS_VAR * dt
        self._push_state(KIND_IMU, seq, t_ns)
        self._accepted += 1

    def _apply_gps(
        self,
        seq: int,
        t_ns: int,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        self._reject_non_finite(vx, vy, vz)
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > MAX_SPEED_MPS:
            raise EngineKernelException(
                f"gps speed {speed:.3f} m/s exceeds {MAX_SPEED_MPS:.3f}"
            )
        hard_skew = False
        soft_skew = False
        if self._last_imu_ns is not None:
            skew_ns = abs(t_ns - self._last_imu_ns)
            if skew_ns > HARD_SKEW_NS:
                hard_skew = True
                raise EngineKernelException(f"imu/gps clock skew {skew_ns} ns")
            if skew_ns > SOFT_SKEW_NS:
                soft_skew = True
        predicted = (
            self._axes[0].velocity,
            self._axes[1].velocity,
            self._axes[2].velocity,
        )
        innovation = math.sqrt(
            (vx - predicted[0]) ** 2
            + (vy - predicted[1]) ** 2
            + (vz - predicted[2]) ** 2
        )
        if self._gps_updates > 0 and innovation > JUMP_MPS:
            raise EngineKernelException(
                f"gps jump {innovation:.3f} m/s exceeds {JUMP_MPS:.3f}"
            )
        self._note_sequence(KIND_GPS, seq)
        if soft_skew and not hard_skew:
            self._skew_clamps += 1
            LOGGER.warning(
                "gps clock skew clamped seq=%d skew_ns=%d imu_ns=%d gps_ns=%d",
                seq,
                abs(t_ns - (self._last_imu_ns or t_ns)),
                self._last_imu_ns,
                t_ns,
            )
        for axis, measured in zip(self._axes, (vx, vy, vz), strict=True):
            gain = axis.variance / (axis.variance + MEAS_VAR)
            residual = measured - axis.velocity
            axis.velocity = axis.velocity + gain * residual
            axis.variance = (1.0 - gain) * axis.variance
        self._last_gps_ns = t_ns
        self._gps_updates += 1
        self._push_innovation(innovation)
        self._push_state(KIND_GPS, seq, t_ns)
        self._accepted += 1

    def _ingest_locked(self, payload: bytes) -> None:
        frame = self.unpack_frame(payload)
        kind = int(frame["kind"])
        seq = int(frame["seq"])
        t_ns = int(frame["t_ns"])
        x_val = float(frame["x"])
        y_val = float(frame["y"])
        z_val = float(frame["z"])
        if kind == KIND_IMU:
            self._apply_imu(seq, t_ns, x_val, y_val, z_val)
            return
        if kind == KIND_GPS:
            self._apply_gps(seq, t_ns, x_val, y_val, z_val)
            return
        raise EngineKernelException(f"unknown sensor kind {kind}")

    def snapshot(self) -> dict[str, object]:
        """Return the safeguarded state. Safe to call after the worker returns."""
        speed = math.sqrt(
            self._axes[0].velocity ** 2
            + self._axes[1].velocity ** 2
            + self._axes[2].velocity ** 2
        )
        mean_var = (
            self._axes[0].variance + self._axes[1].variance + self._axes[2].variance
        ) / 3.0
        innov_mean, innov_stdev = _mean_stdev(self._innovation_samples())
        return {
            "topic": TOPIC,
            "vx": self._axes[0].velocity,
            "vy": self._axes[1].velocity,
            "vz": self._axes[2].velocity,
            "speed_mps": speed,
            "mean_variance": mean_var,
            "accepted": self._accepted,
            "dropouts": self._dropouts,
            "skew_clamps": self._skew_clamps,
            "gps_updates": self._gps_updates,
            "mean_innovation": innov_mean,
            "stdev_innovation": innov_stdev,
            "ring_count": self._slot_n,
        }

    async def admit(self, payload: bytes) -> dict[str, object]:
        """Ingest one frame. Physical and sequence faults raise."""
        async with self._lock:
            self._ingest_locked(payload)
            return self.snapshot()

    async def run_worker(self, payloads: list[bytes]) -> dict[str, object]:
        """Fuse a batch from the ``flight.sensors.fused`` stand-in queue."""
        if not payloads:
            raise EngineKernelException("empty sensor batch")
        for payload in payloads:
            await self._inbound.put(payload)
        buffered: list[bytes] = []
        for _ in range(len(payloads)):
            buffered.append(await self._inbound.get())
        rejections: list[str] = []
        async with self._lock:
            for payload in buffered:
                try:
                    self._ingest_locked(payload)
                except EngineKernelException as exc:
                    rejections.append(str(exc))
                    LOGGER.warning("sample rejected error=%s", exc)
            state = self.snapshot()
        state["rejected_count"] = len(rejections)
        state["rejections"] = rejections
        decoded: dict[str, object] = json.loads(json.dumps(state))
        LOGGER.info(
            "fusion topic=%s accepted=%d rejected=%d speed_mps=%.3f "
            "dropouts=%d skew_clamps=%d ring_count=%d",
            TOPIC,
            decoded["accepted"],
            decoded["rejected_count"],
            decoded["speed_mps"],
            decoded["dropouts"],
            decoded["skew_clamps"],
            decoded["ring_count"],
        )
        return decoded


async def _scenario() -> None:
    engine = KinematicSensorFusionSafeguard()
    frames: list[bytes] = []
    t_ns = 1_000_000_000
    seq = 1
    for step in range(20):
        if step == 10:
            seq += 3
        frames.append(engine.pack_frame(KIND_IMU, seq, t_ns, 1.5, 0.05, -0.02))
        seq += 1
        t_ns += 10_000_000
    for gps_seq in range(1, 4):
        frames.append(
            engine.pack_frame(KIND_GPS, gps_seq, t_ns + 80_000_000, 12.0, 0.2, 0.0)
        )
    frames.append(engine.pack_frame(KIND_IMU, seq, t_ns, 13.0 * GRAVITY_MPS2, 0.0, 0.0))
    frames.append(engine.pack_frame(KIND_GPS, 4, t_ns, 20.0, 0.0, 0.0))
    frames.append(engine.pack_frame(KIND_GPS, 6, t_ns, -200.0, 40.0, 0.0))
    report = await engine.run_worker(frames)
    sys.stdout.write(json.dumps(report) + "\n")


if __name__ == "__main__":
    asyncio.run(_scenario())

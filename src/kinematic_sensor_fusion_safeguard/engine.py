"""Per-axis constant-velocity Kalman filter with explicit 2x2 updates."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import statistics
import struct
from typing import Mapping, Sequence

from .exceptions import EngineKernelException
from .wire import FORMAT, KIND_GPS, KIND_IMU, pack_frame, unpack_frame

LOGGER = logging.getLogger(__name__)

TOPIC: str = "flight.sensors.fused"
GRAVITY_MPS2: float = 9.80665
MAX_G: float = 12.0
MAX_ACCEL_MPS2: float = MAX_G * GRAVITY_MPS2
SKEW_BUDGET_NS: int = 200_000_000
NOMINAL_DT_S: float = 0.01
R_POS: float = 9.0
R_VEL: float = 0.25
Q_ACC: float = 1.0
CHI_GATE: float = 9.0
MAX_MODELLED_GAP: int = 4096


class _Axis:
    """Position, velocity, and the upper triangle of a 2x2 covariance."""

    __slots__ = ("p", "v", "p00", "p01", "p11")

    def __init__(self) -> None:
        self.p = 0.0
        self.v = 0.0
        self.p00 = 25.0
        self.p01 = 0.0
        self.p11 = 4.0


def _as_int(record: Mapping[str, object], key: str) -> int:
    raw = record[key]
    if isinstance(raw, bool) or not isinstance(raw, int):
        raise EngineKernelException(f"{key} must be an integer")
    return raw


def _as_float(record: Mapping[str, object], key: str) -> float:
    raw = record[key]
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise EngineKernelException(f"{key} must be numeric")
    return float(raw)


def _check_size(payload: bytes) -> None:
    expected = struct.calcsize(FORMAT)
    if len(payload) != expected:
        raise EngineKernelException(f"sensor frame length {len(payload)} != {expected}")


def _json_object(report: dict[str, object]) -> dict[str, object]:
    decoded = json.loads(json.dumps(report, allow_nan=False))
    if not isinstance(decoded, dict):
        raise EngineKernelException("report was not a JSON object")
    return decoded


def _predict(axis: _Axis, accel: float, dt: float) -> None:
    """x = F x + B u, P = F P F^T + Q, with F = [[1, dt], [0, 1]]."""
    position = axis.p
    velocity = axis.v
    p00 = axis.p00
    p01 = axis.p01
    p11 = axis.p11
    axis.p = position + velocity * dt + 0.5 * accel * dt * dt
    axis.v = velocity + accel * dt
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt2 * dt2
    axis.p00 = p00 + 2.0 * dt * p01 + dt2 * p11 + Q_ACC * (dt4 / 4.0)
    axis.p01 = p01 + dt * p11 + Q_ACC * (dt3 / 2.0)
    axis.p11 = p11 + Q_ACC * dt2


def _update_position(axis: _Axis, measured: float) -> bool:
    """Scalar position update. H = [1, 0]. False when the chi-square gate fails."""
    innovation = measured - axis.p
    variance = axis.p00 + R_POS
    if variance <= 0.0:
        raise EngineKernelException("non-positive position innovation variance")
    if (innovation * innovation) / variance > CHI_GATE:
        return False
    gain_p = axis.p00 / variance
    gain_v = axis.p01 / variance
    axis.p = axis.p + gain_p * innovation
    axis.v = axis.v + gain_v * innovation
    p00 = axis.p00
    p01 = axis.p01
    p11 = axis.p11
    m00 = (1.0 - gain_p) * p00
    m01 = (1.0 - gain_p) * p01
    m10 = p01 - gain_v * p00
    m11 = p11 - gain_v * p01
    n00 = m00 * (1.0 - gain_p) + R_POS * gain_p * gain_p
    n01 = m00 * (-gain_v) + m01 + R_POS * gain_p * gain_v
    n10 = m10 * (1.0 - gain_p) + R_POS * gain_p * gain_v
    n11 = m10 * (-gain_v) + m11 + R_POS * gain_v * gain_v
    axis.p00 = n00
    axis.p01 = 0.5 * (n01 + n10)
    axis.p11 = n11
    return True


def _update_velocity(axis: _Axis, measured: float) -> None:
    """Scalar velocity update. H = [0, 1]."""
    innovation = measured - axis.v
    variance = axis.p11 + R_VEL
    if variance <= 0.0:
        raise EngineKernelException("non-positive velocity innovation variance")
    gain_p = axis.p01 / variance
    gain_v = axis.p11 / variance
    axis.p = axis.p + gain_p * innovation
    axis.v = axis.v + gain_v * innovation
    p00 = axis.p00
    p01 = axis.p01
    p11 = axis.p11
    m00 = p00 - gain_p * p01
    m01 = p01 - gain_p * p11
    m10 = (1.0 - gain_v) * p01
    m11 = (1.0 - gain_v) * p11
    n00 = m00 - m01 * gain_p + R_VEL * gain_p * gain_p
    n01 = m01 * (1.0 - gain_v) + R_VEL * gain_p * gain_v
    n10 = m10 - m11 * gain_p + R_VEL * gain_p * gain_v
    n11 = m11 * (1.0 - gain_v) + R_VEL * gain_v * gain_v
    axis.p00 = n00
    axis.p01 = 0.5 * (n01 + n10)
    axis.p11 = n11


class KinematicSensorFusionSafeguard:
    """Fuse IMU acceleration and GPS position behind traceable input checks."""

    def __init__(self) -> None:
        self._lock = asyncio.Lock()
        self._inbound: asyncio.Queue[bytes] = asyncio.Queue()
        self._outbound: asyncio.Queue[bytes] = asyncio.Queue(maxsize=256)
        self._axes = (_Axis(), _Axis(), _Axis())
        self._accel = (0.0, 0.0, 0.0)
        self._last_seq: dict[int, int | None] = {KIND_IMU: None, KIND_GPS: None}
        self._last_t_ns: int | None = None
        self._last_imu_ns: int | None = None
        self._last_gps_ns: int | None = None
        self._rejected_updates = 0
        self._dropouts = 0
        self._skew_rejects = 0
        self._frame_size = struct.calcsize(FORMAT)

    async def run(self, records: Sequence[Mapping[str, object]]) -> dict[str, object]:
        """Fuse a batch and return a JSON-serializable state dict."""
        batch = list(records)
        if not batch:
            raise EngineKernelException("empty sensor batch")
        async with self._lock:
            for record in batch:
                payload = _pack_record(record)
                _check_size(payload)
                if len(payload) != self._frame_size:
                    raise EngineKernelException("frame size mismatch")
                await self._inbound.put(payload)
                frame = unpack_frame(await self._inbound.get())
                self.ingest(frame)
            blob = struct.pack(
                "<ddd",
                self._axes[0].p,
                self._axes[1].p,
                self._axes[2].p,
            )
            await self._publish(blob)
            report = self._report()
        LOGGER.info(
            "topic=%s position=%s velocity=%s rejected_updates=%d "
            "dropouts=%d skew_rejects=%d",
            TOPIC,
            report["position"],
            report["velocity"],
            report["rejected_updates"],
            report["dropouts"],
            report["skew_rejects"],
        )
        return _json_object(report)

    def ingest(self, frame: Mapping[str, object]) -> None:
        """Apply one unpacked frame. ``run`` holds the engine lock."""
        kind = int(frame["kind"])
        seq = int(frame["seq"])
        t_ns = int(frame["t_ns"])
        channels = (
            float(frame["c0"]),
            float(frame["c1"]),
            float(frame["c2"]),
            float(frame["c3"]),
            float(frame["c4"]),
            float(frame["c5"]),
        )
        if kind == KIND_IMU:
            self._ingest_imu(seq, t_ns, channels[0], channels[1], channels[2])
            return
        if kind == KIND_GPS:
            self._ingest_gps(seq, t_ns, channels)
            return
        raise EngineKernelException(f"unknown sensor kind {kind}")

    def _finite(self, values: Sequence[float]) -> None:
        for value in values:
            if math.isnan(value) or not math.isfinite(value):
                raise EngineKernelException(f"non-finite sample value={value}")

    def _check_skew(self, kind: int, t_ns: int) -> None:
        other: int | None
        if kind == KIND_GPS:
            other = self._last_imu_ns
        else:
            other = self._last_gps_ns
        if other is None:
            return
        skew = abs(t_ns - other)
        if skew > SKEW_BUDGET_NS:
            self._skew_rejects += 1
            raise EngineKernelException(
                f"imu/gps clock skew {skew} ns exceeds {SKEW_BUDGET_NS} "
                f"t_ns={t_ns} other_ns={other}"
            )

    def _elapsed(self, t_ns: int) -> float:
        if self._last_t_ns is None:
            return NOMINAL_DT_S
        delta = t_ns - self._last_t_ns
        if delta <= 0:
            raise EngineKernelException(f"non-monotonic timestamp t_ns={t_ns}")
        return delta / 1_000_000_000.0

    def _sequence_gap(self, kind: int, seq: int) -> int:
        previous = self._last_seq[kind]
        if previous is None:
            return 0
        if seq <= previous:
            raise EngineKernelException(
                f"non-monotonic sequence kind={kind} seq={seq} previous={previous}"
            )
        return seq - previous - 1

    def _commit(self, kind: int, seq: int, t_ns: int) -> None:
        self._last_seq[kind] = seq
        self._last_t_ns = t_ns
        if kind == KIND_IMU:
            self._last_imu_ns = t_ns
        else:
            self._last_gps_ns = t_ns

    def _predict_all(self, ax: float, ay: float, az: float, dt: float) -> None:
        for axis, accel in zip(self._axes, (ax, ay, az), strict=True):
            _predict(axis, accel, dt)

    def _coast_gap(self, gap: int, total_dt: float) -> float:
        """Predict-only across missing sequence numbers. Do not invent accel."""
        if gap <= 0:
            return total_dt
        self._dropouts += gap
        modelled = gap if gap <= MAX_MODELLED_GAP else MAX_MODELLED_GAP
        step = total_dt / float(modelled + 1)
        for _ in range(modelled):
            self._predict_all(0.0, 0.0, 0.0, step)
        LOGGER.warning("sensor dropout gap=%d modelled=%d", gap, modelled)
        return step

    def _ingest_imu(self, seq: int, t_ns: int, ax: float, ay: float, az: float) -> None:
        self._finite((ax, ay, az))
        magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        if magnitude > MAX_ACCEL_MPS2:
            raise EngineKernelException(
                f"acceleration {magnitude:.3f} m/s^2 exceeds 12 g "
                f"({MAX_ACCEL_MPS2:.3f})"
            )
        self._check_skew(KIND_IMU, t_ns)
        total_dt = self._elapsed(t_ns)
        gap = self._sequence_gap(KIND_IMU, seq)
        step = self._coast_gap(gap, total_dt)
        self._predict_all(ax, ay, az, step)
        self._accel = (ax, ay, az)
        self._commit(KIND_IMU, seq, t_ns)

    def _ingest_gps(
        self,
        seq: int,
        t_ns: int,
        channels: tuple[float, float, float, float, float, float],
    ) -> None:
        self._finite(channels)
        self._check_skew(KIND_GPS, t_ns)
        total_dt = self._elapsed(t_ns)
        gap = self._sequence_gap(KIND_GPS, seq)
        step = self._coast_gap(gap, total_dt)
        self._predict_all(self._accel[0], self._accel[1], self._accel[2], step)
        position = channels[:3]
        velocity = channels[3:]
        for index, axis in enumerate(self._axes):
            if not _update_position(axis, position[index]):
                self._rejected_updates += 1
                LOGGER.warning("position update rejected axis=%d", index)
                continue
            _update_velocity(axis, velocity[index])
        self._commit(KIND_GPS, seq, t_ns)

    def _report(self) -> dict[str, object]:
        traces = [axis.p00 + axis.p11 for axis in self._axes]
        return {
            "topic": TOPIC,
            "position": [axis.p for axis in self._axes],
            "velocity": [axis.v for axis in self._axes],
            "rejected_updates": self._rejected_updates,
            "dropouts": self._dropouts,
            "skew_rejects": self._skew_rejects,
            "covariance_trace": statistics.fmean(traces),
            "covariance_spread": statistics.pstdev(traces),
        }

    async def _publish(self, blob: bytes) -> None:
        if self._outbound.full():
            self._outbound.get_nowait()
        await self._outbound.put(blob)


def _pack_record(record: Mapping[str, object]) -> bytes:
    kind = _as_int(record, "kind")
    seq = _as_int(record, "seq")
    t_ns = _as_int(record, "t_ns")
    if kind == KIND_IMU:
        return pack_frame(
            kind,
            seq,
            t_ns,
            _as_float(record, "ax"),
            _as_float(record, "ay"),
            _as_float(record, "az"),
            0.0,
            0.0,
            0.0,
        )
    if kind == KIND_GPS:
        return pack_frame(
            kind,
            seq,
            t_ns,
            _as_float(record, "px"),
            _as_float(record, "py"),
            _as_float(record, "pz"),
            _as_float(record, "vx"),
            _as_float(record, "vy"),
            _as_float(record, "vz"),
        )
    raise EngineKernelException(f"unknown sensor kind {kind}")

"""Little-endian IMU and GPS frames for the flight.sensors.fused stand-in."""

from __future__ import annotations

import struct

from .exceptions import EngineKernelException

FORMAT: str = "<BIQ6d"
FRAME: struct.Struct = struct.Struct(FORMAT)
KIND_IMU: int = 1
KIND_GPS: int = 2


def pack_frame(
    kind: int,
    seq: int,
    t_ns: int,
    c0: float,
    c1: float,
    c2: float,
    c3: float,
    c4: float,
    c5: float,
) -> bytes:
    """Pack one sensor frame.

    Layout, little-endian: kind ``uint8``, sequence ``uint32``, time ``uint64``,
    then six ``float64`` channels. IMU uses the first three as acceleration.
    GPS uses the first three as position and the last three as velocity.
    """
    if kind not in (KIND_IMU, KIND_GPS):
        raise EngineKernelException(f"unknown sensor kind {kind}")
    if seq < 0 or seq > 0xFFFFFFFF:
        raise EngineKernelException(f"sequence out of uint32 range: {seq}")
    if t_ns < 0 or t_ns > 0xFFFFFFFFFFFFFFFF:
        raise EngineKernelException(f"timestamp out of uint64 range: {t_ns}")
    try:
        return FRAME.pack(
            kind,
            seq,
            t_ns,
            float(c0),
            float(c1),
            float(c2),
            float(c3),
            float(c4),
            float(c5),
        )
    except (struct.error, OverflowError, ValueError) as exc:
        raise EngineKernelException("sensor frame pack failed") from exc


def unpack_frame(payload: bytes) -> dict[str, object]:
    """Unpack one sensor frame. Finite values round-trip exactly."""
    if len(payload) != FRAME.size:
        raise EngineKernelException(
            f"sensor frame length {len(payload)} != {FRAME.size}"
        )
    try:
        kind, seq, t_ns, c0, c1, c2, c3, c4, c5 = FRAME.unpack(payload)
    except struct.error as exc:
        raise EngineKernelException("sensor frame unpack failed") from exc
    return {
        "kind": int(kind),
        "seq": int(seq),
        "t_ns": int(t_ns),
        "c0": float(c0),
        "c1": float(c1),
        "c2": float(c2),
        "c3": float(c3),
        "c4": float(c4),
        "c5": float(c5),
    }

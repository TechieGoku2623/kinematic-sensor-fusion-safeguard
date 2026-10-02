"""Wire, Kalman, and edge-case tests. No network."""

from __future__ import annotations

import asyncio
import logging
import math
import unittest

from kinematic_sensor_fusion_safeguard import (
    EngineKernelException,
    KinematicSensorFusionSafeguard,
)
from kinematic_sensor_fusion_safeguard.engine import (
    GRAVITY_MPS2,
    NOMINAL_DT_S,
    SKEW_BUDGET_NS,
)
from kinematic_sensor_fusion_safeguard.wire import (
    KIND_GPS,
    KIND_IMU,
    pack_frame,
    unpack_frame,
)

logging.getLogger("kinematic_sensor_fusion_safeguard").setLevel(logging.CRITICAL)


def _imu(seq: int, t_ns: int, ax: float) -> dict[str, object]:
    return {
        "kind": KIND_IMU,
        "seq": seq,
        "t_ns": t_ns,
        "ax": ax,
        "ay": 0.0,
        "az": 0.0,
    }


def _gps(
    seq: int,
    t_ns: int,
    px: float,
    vx: float,
) -> dict[str, object]:
    return {
        "kind": KIND_GPS,
        "seq": seq,
        "t_ns": t_ns,
        "px": px,
        "py": 0.0,
        "pz": 0.0,
        "vx": vx,
        "vy": 0.0,
        "vz": 0.0,
    }


class KinematicEngineTest(unittest.TestCase):
    def test_wire_roundtrip(self) -> None:
        payload = pack_frame(
            KIND_IMU, 4, 1_000_000_000, 0.5, -0.25, 0.125, 0.0, 0.0, 0.0
        )
        frame = unpack_frame(payload)
        self.assertEqual(frame["kind"], KIND_IMU)
        self.assertEqual(frame["seq"], 4)
        self.assertEqual(frame["t_ns"], 1_000_000_000)
        self.assertEqual(frame["c0"], 0.5)
        self.assertEqual(frame["c1"], -0.25)
        self.assertEqual(frame["c2"], 0.125)
        gps = unpack_frame(pack_frame(KIND_GPS, 2, 8, 1.5, 0.0, 0.0, 0.25, 0.0, 0.0))
        self.assertEqual(gps["kind"], KIND_GPS)
        self.assertEqual(gps["c0"], 1.5)
        self.assertEqual(gps["c3"], 0.25)
        with self.assertRaises(EngineKernelException):
            unpack_frame(b"\x00\x01")

    def test_happy_path_constant_accel(self) -> None:
        stamp = 1_000_000_000
        report = asyncio.run(
            KinematicSensorFusionSafeguard().run(
                [_imu(step + 1, stamp + step * 10_000_000, 2.0) for step in range(10)]
            )
        )
        position = 0.0
        velocity = 0.0
        dt = NOMINAL_DT_S
        for _ in range(10):
            position = position + velocity * dt + 0.5 * 2.0 * dt * dt
            velocity = velocity + 2.0 * dt
        self.assertTrue(
            math.isclose(float(report["position"][0]), position, abs_tol=1e-12)
        )
        self.assertTrue(
            math.isclose(float(report["velocity"][0]), velocity, abs_tol=1e-12)
        )
        self.assertEqual(report["position"][1], 0.0)
        self.assertEqual(report["rejected_updates"], 0)
        self.assertEqual(report["dropouts"], 0)
        self.assertEqual(report["skew_rejects"], 0)
        self.assertEqual(report["topic"], "flight.sensors.fused")

    def test_skew_over_budget_raises(self) -> None:
        with self.assertRaises(EngineKernelException) as caught:
            asyncio.run(
                KinematicSensorFusionSafeguard().run(
                    [
                        _imu(1, 5_000_000_000, 0.1),
                        _gps(1, 5_000_000_000 + SKEW_BUDGET_NS + 1, 0.0, 0.0),
                    ]
                )
            )
        self.assertIn("skew", str(caught.exception))

    def test_acceleration_above_12g_raises(self) -> None:
        with self.assertRaises(EngineKernelException) as caught:
            asyncio.run(
                KinematicSensorFusionSafeguard().run(
                    [_imu(1, 6_000_000_000, 13.0 * GRAVITY_MPS2)]
                )
            )
        self.assertIn("12 g", str(caught.exception))

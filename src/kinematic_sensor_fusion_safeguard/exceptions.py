"""Kernel faults for the Kinematic Sensor Fusion Safeguard."""

from __future__ import annotations


class EngineKernelException(Exception):
    """Raised when a record fails a traceable kernel check."""

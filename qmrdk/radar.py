"""Acquisition interfaces. The simulated implementations are in qmrdk.sim;
hardware implementations sit behind the same interfaces."""

from typing import Protocol

import numpy as np

from qmrdk.config import Sweep


class Radar(Protocol):
    """A radar that returns frames of raw ADC codes."""

    sweep: Sweep

    def capture(self, n: int) -> np.ndarray:
        """Acquire one frame of `n` samples, uint16 codes."""


class Sled(Protocol):
    """A linear positioner carrying the radar along the rail."""

    def home(self) -> None:
        """Move to the reference position (x = 0)."""

    def move_to(self, x: float) -> None:
        """Move to rail position `x` (m) and settle."""

    def position(self) -> float:
        """Reported rail position, m."""


class UsbRadar:
    """QM-RDK over USBTMC (docs/protocol.md). Device layer is phase 1 work."""

    def __init__(self, resource: str | None = None):
        raise NotImplementedError(
            f"USB device layer not implemented (resource {resource!r}); use --sim"
        )


class HardwareSled:
    """Driver for the kit's sled controller, pending docs/sled.md."""

    def __init__(self, port: str | None = None):
        raise NotImplementedError(
            f"sled controller interface not documented (port {port!r}); use --sim"
        )

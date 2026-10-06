"""Acquisition interfaces. The simulated implementations are in qmrdk.sim;
hardware implementations sit behind the same interfaces."""

from typing import Protocol

import numpy as np

from qmrdk.config import Sweep
from qmrdk.device import Device, DeviceError, Idn, decode_chunk, ref_divider

__all__ = [
    "DeviceError",
    "HardwareSled",
    "Radar",
    "Sled",
    "UsbRadar",
    "decode_chunk",
    "ref_divider",
]


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
    """The Radar protocol over a QM-RDK on USB (`qmrdk.device.Device`):
    configures `sweep`, keeps the read-back values in `self.sweep`, and stops
    the sweep on `close`."""

    def __init__(
        self,
        resource: str | None = None,
        sweep: Sweep = Sweep(),
        serial: str | None = None,
        manager=None,
    ):
        self.device = Device(resource, serial, manager)
        try:
            self.sweep = self.device.configure(sweep)
        except BaseException:
            self.device.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    @property
    def idn(self) -> Idn:
        return self.device.idn

    def capture(self, n: int) -> np.ndarray:
        return self.device.capture(n)

    def close(self) -> None:
        self.device.close()


class HardwareSled:
    """Driver for the kit's sled controller, pending docs/sled.md."""

    def __init__(self, port: str | None = None):
        raise NotImplementedError(
            f"sled controller interface not documented (port {port!r}); use --sim"
        )

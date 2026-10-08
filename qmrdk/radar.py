"""Acquisition interfaces. The simulated implementations are in qmrdk.sim;
hardware implementations sit behind the same interfaces."""

import sys
from typing import Protocol

import numpy as np

from qmrdk.config import Sweep
from qmrdk.device import Device, DeviceError, Idn, decode_chunk, ref_divider
from qmrdk.sled import HardwareSled

__all__ = [
    "DeviceError",
    "HardwareSled",
    "ManualSled",
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


class ManualSled:
    """The Sled protocol by hand: `move_to` asks the operator (on `out`) to
    place the sled reference at the rail position along the tape from home and
    waits for Enter (`prompt`, default `input`); a repeated position is not
    asked again."""

    def __init__(self, prompt=input, out=None):
        self._prompt = prompt
        self._out = sys.stderr if out is None else out
        self._x = None

    def home(self) -> None:
        self.move_to(0.0)

    def move_to(self, x: float) -> None:
        x = float(x)
        if x == self._x:
            return
        step = "" if self._x is None else f", {(x - self._x) * 1e3:+.1f} mm from here"
        print(
            f"place the radar reference at {x:.4f} m = {x * 1e2:.2f} cm = "
            f"{x * 1e3:.1f} mm from home{step}; Enter when placed",
            file=self._out,
            flush=True,
        )
        self._prompt("")
        self._x = x

    def position(self) -> float:
        return 0.0 if self._x is None else self._x

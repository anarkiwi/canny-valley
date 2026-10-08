"""Serial driver for the rail sled controller (docs/sled.md): one stepper
axis moving the radar side to side, homed on the far-end limit switch."""

import os
import time

import numpy as np
import serial
from serial.tools import list_ports

from qmrdk.device import DeviceError

BAUD = 9600
USB_ID = (0x2341, 0x8036)
STEP = np.float32(np.float32(49.5) * np.float32(np.pi)) / np.float32(200)
MAX_MM = 950
BOOT_TIMEOUT = 2.0
ECHO_TIMEOUT = 5.0
HOME_TIMEOUT = 60.0
MOVE_TIMEOUT = 15.0


class SledError(DeviceError):
    """The controller refused a position or did not answer."""


def find_port() -> str:
    """Serial URL of the controller: `$QMRDK_SLED`, else the USB port with
    its vendor and product id."""
    url = os.environ.get("QMRDK_SLED")
    if url:
        return url
    for p in list_ports.comports():
        if (p.vid, p.pid) == USB_ID:
            return p.device
    raise SledError("sled controller not found; set QMRDK_SLED to its serial URL")


def reached_steps(mm: int) -> int:
    """Step count the controller moves to for an integer `mm` command: AVR
    single-precision division by the step length, truncated."""
    return int(np.float32(mm) / STEP)


class HardwareSled:
    """The Sled protocol over the controller's line protocol (docs/sled.md).
    Rail position 0 is `origin` (m) from home; `position` is the step reached.
    `home` moves to the origin; `rehome` runs the homing sequence."""

    def __init__(self, url=None, origin=0.05, settle=0.3, port=None):
        try:
            self.port = port or serial.serial_for_url(
                url or find_port(), BAUD, timeout=0.2
            )
        except serial.SerialException as exc:
            raise SledError(f"sled controller: {exc}") from exc
        self.origin, self.settle = float(origin), float(settle)
        self._steps = None
        if self._wait(BOOT_TIMEOUT, ("homing",)) and not self._wait(
            HOME_TIMEOUT, ("ready",)
        ):
            raise SledError("sled controller did not finish homing")

    def _wait(self, timeout, until):
        """First line in `until` within `timeout` s, else None."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.port.readline().decode(errors="replace").strip()
            if line in until:
                return line
        return None

    def _command(self, mm: int) -> None:
        self.port.reset_input_buffer()
        self.port.write(f"{mm}\n".encode())
        if not self._wait(ECHO_TIMEOUT, (f"moving to position {mm}",)):
            raise SledError(f"sled controller did not accept {mm}")
        timeout = HOME_TIMEOUT if mm == 0 else MOVE_TIMEOUT
        answer = self._wait(timeout, ("ready", "error"))
        if answer != "ready":
            raise SledError(f"sled controller: {answer or 'no ready'} at {mm} mm")
        time.sleep(self.settle)

    def command_mm(self, x: float) -> int:
        """Integer mm command nearest rail position `x` (m)."""
        mm = int(round(1e3 * (self.origin + x)))
        if not 1 <= mm <= MAX_MM:
            raise SledError(f"rail position {x:.4f} m outside the travel")
        return mm

    def move_to(self, x: float) -> None:
        mm = self.command_mm(x)
        if reached_steps(mm) != self._steps:
            self._command(mm)
            self._steps = reached_steps(mm)

    def home(self) -> None:
        self.move_to(0.0)

    def rehome(self) -> None:
        self._command(0)
        self._steps = 0

    def position(self) -> float:
        if self._steps is None:
            raise SledError("sled position unknown before the first move")
        return self._steps * float(STEP) * 1e-3 - self.origin

    def close(self) -> None:
        self.port.close()

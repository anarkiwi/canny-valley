"""Serial driver for the rail sled controller (docs/sled.md): one stepper
axis moving the radar side to side, homed on the far-end limit switch. Speaks
the firmware/sled protocol when the controller announces it on connect, else
the legacy sketch's protocol."""

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
BANNER = "id qmrdk-sled"


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
        self.step, self.max_steps = float(STEP) * 1e-3, None
        first = self._wait(BOOT_TIMEOUT, BANNER, "homing")
        self.firmware = first is not None and first.startswith(BANNER)
        if self.firmware:
            self._attach(int(first.split()[3]))
        elif first and not self._wait(HOME_TIMEOUT, "ready"):
            raise SledError("sled controller did not finish homing")

    def _wait(self, timeout, *keys):
        """First line that is, or starts with a word sequence, in `keys`
        within `timeout` s, else None."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.port.readline().decode(errors="replace").strip()
            if any(line == k or line.startswith(k + " ") for k in keys):
                return line
        return None

    def _send(self, line: str) -> None:
        self.port.reset_input_buffer()
        self.port.write(f"{line}\n".encode())

    def _request(self, line: str, key: str, timeout=ECHO_TIMEOUT) -> list:
        """Reply words to firmware command `line`; raises on `err` or none."""
        self._send(line)
        reply = self._wait(timeout, key, "err")
        if reply is None or reply.startswith("err"):
            raise SledError(f"sled controller: {reply or 'no reply'} to {line!r}")
        return reply.split()[1:]

    def status(self) -> dict:
        """Firmware `status?` fields as integers."""
        words = self._request("status?", "status")
        return {k: int(v) for k, v in (w.split("=") for w in words)}

    def _attach(self, step_nm: int) -> None:
        """Adopt the firmware's step length (nm per full step) and soft limit;
        home unless already homed."""
        state = self.status()
        self.step = step_nm * 1e-9 / state["microstep"]
        self.max_steps = state["max"]
        if state["homed"]:
            self._steps = state["pos"]
        else:
            self.rehome()

    def _command(self, mm: int) -> None:
        self._send(str(mm))
        if not self._wait(ECHO_TIMEOUT, f"moving to position {mm}"):
            raise SledError(f"sled controller did not accept {mm}")
        timeout = HOME_TIMEOUT if mm == 0 else MOVE_TIMEOUT
        answer = self._wait(timeout, "ready", "error")
        if answer != "ready":
            raise SledError(f"sled controller: {answer or 'no ready'} at {mm} mm")
        time.sleep(self.settle)

    def _motion(self, line: str, timeout: float) -> None:
        self._steps = None
        self._steps = int(self._request(line, "ok", timeout)[0])
        time.sleep(self.settle)

    def command_mm(self, x: float) -> int:
        """Integer mm command nearest rail position `x` (m)."""
        mm = int(round(1e3 * (self.origin + x)))
        if not 1 <= mm <= MAX_MM:
            raise SledError(f"rail position {x:.4f} m outside the travel")
        return mm

    def command_steps(self, x: float) -> int:
        """Firmware step nearest rail position `x` (m)."""
        steps = int(round((self.origin + x) / self.step))
        if not 0 <= steps <= self.max_steps:
            raise SledError(f"rail position {x:.4f} m outside the travel")
        return steps

    def move_to(self, x: float) -> None:
        if self.firmware:
            steps = self.command_steps(x)
            if steps != self._steps:
                self._motion(f"move {steps}", MOVE_TIMEOUT)
            return
        mm = self.command_mm(x)
        if reached_steps(mm) != self._steps:
            self._command(mm)
            self._steps = reached_steps(mm)

    def home(self) -> None:
        self.move_to(0.0)

    def rehome(self) -> None:
        if self.firmware:
            self._motion("home", HOME_TIMEOUT)
            return
        self._command(0)
        self._steps = 0

    def position(self) -> float:
        if self._steps is None:
            raise SledError("sled position unknown before the first move")
        return self._steps * self.step - self.origin

    def close(self) -> None:
        self.port.close()

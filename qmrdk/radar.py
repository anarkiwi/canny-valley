"""Acquisition interfaces. The simulated implementations are in qmrdk.sim;
hardware implementations sit behind the same interfaces."""

import math
import time
from typing import Protocol

import numpy as np

from qmrdk.config import Sweep
from qmrdk.constants import FS_NOMINAL


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


class DeviceError(RuntimeError):
    """The board is absent, reported an error or returned a malformed frame."""


VID = 0x2012
CHUNK = 31
F_REF = 20e6
SWEEP_TYPES = {"triangle": 2, "cw": 3}
LOCK_TIMEOUT = 2.0
RETRIES = 3


def _manager():
    import pyvisa  # pylint: disable=import-outside-toplevel

    return pyvisa.ResourceManager("@py")


def ref_divider(sweep: Sweep) -> int:
    """Smallest reference divider whose minimum slope allows the sweep
    (protocol §3.3)."""
    if sweep.kind == "cw":
        return 1
    return max(1, math.ceil(F_REF**2 / (sweep.slope * 2**25)))


def decode_chunk(resp: str) -> np.ndarray:
    """One `CAPT:FRAM?` response to codes; DeviceError if malformed."""
    if not resp or len(resp) % 4 or len(resp) > 4 * CHUNK:
        raise DeviceError(f"malformed frame chunk {resp[:16]!r}")
    try:
        raw = bytes.fromhex(resp)
    except ValueError as err:
        raise DeviceError(f"malformed frame chunk {resp[:16]!r}") from err
    return np.frombuffer(raw, dtype=">u2").astype(np.uint16)


class UsbRadar:
    """QM-RDK over USBTMC (docs/protocol.md): configures `sweep`, keeps the
    read-back values in `self.sweep`, and stops the sweep on `close`."""

    def __init__(self, resource: str | None = None, sweep: Sweep = Sweep()):
        manager = _manager()
        names = [r for r in manager.list_resources() if f"::{VID}::" in r]
        if resource is None and not names:
            raise DeviceError("no QM-RDK found")
        self.dev = manager.open_resource(resource or names[0])
        self.dev.read_termination = self.dev.write_termination = "\n"
        self.dev.timeout = 2000
        try:
            self.flush()
            self.idn = self.dev.query("*IDN?").split(",")
            if self.idn[:2] != ["Quonset Microwave", "QM4004"]:
                raise DeviceError(f"not a QM-RDK: {self.idn}")
            self.write("*CLS")
            self.sweep = self.configure(sweep)
        except Exception:
            self.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def query(self, cmd: str) -> str:
        return self.dev.query(cmd).strip()

    def errors(self) -> list[str]:
        """Drain the error queue."""
        out = []
        while len(out) < 10:
            resp = self.query("SYST:ERR?")
            if int(resp.split(",")[0]) in (0, -500):
                break
            out.append(resp)
        return out

    def write(self, cmd: str) -> None:
        """Send a setter and raise on any queued error (protocol §3.7)."""
        self.dev.write(cmd)
        errs = self.errors()
        if errs:
            raise DeviceError(f"{cmd}: {errs}")

    def configure(self, sweep: Sweep) -> Sweep:
        """Protocol §4 configure sequence; returns the read-back sweep."""
        kind = SWEEP_TYPES[sweep.kind]
        ramp = round(sweep.ramp_time * 1e3)
        if ramp < 1:
            raise DeviceError(f"ramp time {sweep.ramp_time} s below 1 ms")
        self.write(f"SWEEP:TYPE {kind}")
        self.write(f"SWEEP:FREQSTAR {sweep.f0 / 1e9:f}")
        self.write(f"SWEEP:FREQSTOP {sweep.f1 / 1e9:f}")
        div = ref_divider(sweep)
        if int(self.query("FREQ:REF:DIV?")) != div:
            self.write(f"FREQ:REF:DIV {div}")
        self.write(f"SWEEP:RAMPTIME {ramp}")
        self.write("SWEEP:START")
        deadline = time.monotonic() + LOCK_TIMEOUT
        while self.query("FREQ:LOCK?") != "1":
            if time.monotonic() > deadline:
                raise DeviceError("PLL not locked")
            time.sleep(0.05)
        kinds = {v: k for k, v in SWEEP_TYPES.items()}
        return Sweep(
            f0=float(self.query("SWEEP:FREQSTAR?")) * 1e9,
            f1=float(self.query("SWEEP:FREQSTOP?")) * 1e9,
            ramp_time=float(self.query("SWEEP:RAMPTIME?")) * 1e-3,
            kind=kinds[int(self.query("SWEEP:TYPE?"))],
        )

    def _frame(self, n: int) -> np.ndarray:
        self.dev.write(f"CAPT:FRAM {n}")
        codes = np.empty(n, dtype=np.uint16)
        got = 0
        timeout = self.dev.timeout
        self.dev.timeout = int(1e3 * (0.5 + n / FS_NOMINAL)) + timeout
        try:
            while got < n:
                chunk = decode_chunk(self.query("CAPT:FRAM?"))
                if got + chunk.size > n:
                    raise DeviceError("frame longer than requested")
                codes[got : got + chunk.size] = chunk
                got += chunk.size
        finally:
            self.dev.timeout = timeout
        return codes

    def capture(self, n: int) -> np.ndarray:
        """One frame of `n` codes; a failed frame is cleared and retried."""
        if not 1 <= n <= 4096:
            raise DeviceError(f"frame length {n} outside 1..4096")
        failure = None
        for _ in range(RETRIES):
            try:
                return self._frame(n)
            except Exception as err:  # pylint: disable=broad-exception-caught
                failure = err
                self.flush()
                self.errors()
        raise DeviceError(f"capture failed: {failure}") from failure

    def flush(self) -> None:
        """Discard pending frame data (pyvisa-py has no USBTMC device clear)."""
        for _ in range(4096 // CHUNK + 2):
            try:
                if self.query("CAPT:FRAM?") == "Not Ready":
                    return
            except Exception:  # pylint: disable=broad-exception-caught
                pass
        raise DeviceError("frame data does not drain")

    def close(self) -> None:
        """Stop the sweep (RF off) and release the device."""
        try:
            self.dev.write("SWEEP:STOP")
        finally:
            self.dev.close()


class HardwareSled:
    """Driver for the kit's sled controller, pending docs/sled.md."""

    def __init__(self, port: str | None = None):
        raise NotImplementedError(
            f"sled controller interface not documented (port {port!r}); use --sim"
        )

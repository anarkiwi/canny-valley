"""Typed QM-RDK command layer over `qmrdk.transport` (docs/protocol.md §3, §4):
validated configuration, lock wait, frame capture with recovery, and RF off
on every exit path."""

import atexit
import contextlib
import dataclasses
import math
import re
import signal
import threading
import time

import numpy as np
from tqdm import tqdm

from qmrdk.config import Sweep
from qmrdk.constants import FS_NOMINAL
from qmrdk.transport import (
    CHUNK,
    MAX_SAMPLES,
    NOT_READY,
    TIMEOUT_MS,
    DeviceError,
    DeviceTimeout,
    Disconnected,
    ForbiddenCommand,
    Idn,
    ScpiError,
    Transport,
    parse_entry,
)

__all__ = [
    "Device",
    "DeviceError",
    "Disconnected",
    "ForbiddenCommand",
    "Idn",
    "ScpiError",
    "Settings",
    "Status",
    "decode_chunk",
    "ref_divider",
]

F_REF = 20e6
F_MIN, F_MAX = 2.4e9, 2.5e9
RAMP_MS_MAX = 65536
REF_DIV_MAX = 256
SWEEP_TYPES = {"ramp": 0, "tri": 1, "triangle": 2, "cw": 3}
KINDS = {v: k for k, v in SWEEP_TYPES.items()}
LOCK_TIMEOUT = 2.0
LOCK_POLL = 0.05
ARM_DELAY = 0.1
CAPTURE_MARGIN = 1.0
RETRIES = 3
_HEX = re.compile(r"[0-9A-Fa-f]*")


class ConfigError(DeviceError):
    """A sweep the board cannot run, refused before anything is sent."""


class LockError(DeviceError):
    """`FREQ:LOCK?` did not read 1 after `SWEEP:START`."""


class FrameError(DeviceError):
    """A `CAPT:FRAM?` response failed validation (protocol §3.4)."""


@dataclasses.dataclass(frozen=True)
class Settings:
    """Read-back board state."""

    sweep: Sweep
    ref_div: int
    rf: bool
    locked: bool


@dataclasses.dataclass(frozen=True)
class Status:
    """`SYST:STAT?`: 0 operational, 1 reset, 2 awaiting input, 100/101
    recoverable/non-recoverable error, 110 over temperature."""

    code: int
    text: str


def ref_divider(sweep: Sweep) -> int:
    """Smallest reference divider whose minimum slope allows the sweep
    (protocol §3.3)."""
    if sweep.kind == "cw":
        return 1
    return max(1, math.ceil(F_REF**2 / (sweep.slope * 2**25)))


def validate(sweep: Sweep) -> tuple[int, int, int]:
    """Sweep type number, integer ramp time (ms) and reference divider of
    `sweep`; ConfigError if the board cannot run it."""
    if sweep.kind not in SWEEP_TYPES:
        raise ConfigError(f"sweep kind {sweep.kind!r} not in {list(SWEEP_TYPES)}")
    for f in (sweep.f0, sweep.f1):
        if not F_MIN <= f <= F_MAX:
            raise ConfigError(f"frequency {f / 1e9:g} GHz outside 2.4..2.5 GHz")
    ramp = sweep.ramp_time * 1e3
    if not math.isclose(ramp, round(ramp), abs_tol=1e-6):
        raise ConfigError(f"ramp time {ramp:g} ms is not an integer number of ms")
    ramp = round(ramp)
    if not 1 <= ramp <= RAMP_MS_MAX:
        raise ConfigError(f"ramp time {ramp} ms outside 1..{RAMP_MS_MAX} ms")
    if sweep.kind != "cw" and sweep.f1 <= sweep.f0:
        raise ConfigError("stop frequency must be above start frequency")
    div = ref_divider(sweep)
    if div > REF_DIV_MAX:
        t_max = sweep.bandwidth * REF_DIV_MAX * 2**25 / F_REF**2
        raise ConfigError(
            f"ramp time {ramp} ms above T_max {t_max * 1e3:.0f} ms for "
            f"{sweep.bandwidth / 1e6:g} MHz at reference divider {REF_DIV_MAX}"
        )
    return SWEEP_TYPES[sweep.kind], ramp, div


def decode_chunk(resp: str, expect: int | None = None) -> np.ndarray:
    """One `CAPT:FRAM?` response to codes; FrameError if malformed or, with
    `expect`, not exactly `expect` samples."""
    size = len(resp) // 4
    if (
        len(resp) % 4
        or not 0 < size <= CHUNK
        or expect not in (None, size)
        or not _HEX.fullmatch(resp)
    ):
        raise FrameError(f"malformed frame chunk {resp[:16]!r} ({len(resp)} chars)")
    return np.frombuffer(bytes.fromhex(resp), dtype=">u2").astype(np.uint16)


_OPEN = set()
_PREVIOUS = {}
_SIGNALS = (signal.SIGINT, signal.SIGTERM)


def _on_signal(signum, frame):
    """Stop RF on every open board, then defer to the previous handler."""
    for dev in list(_OPEN):
        dev.rf_off()
    prev = _PREVIOUS.get(signum, signal.SIG_DFL)
    if callable(prev):
        return prev(signum, frame)
    if prev == signal.SIG_IGN:
        return None
    raise SystemExit(128 + signum)


def _register(dev) -> None:
    if not _OPEN and threading.current_thread() is threading.main_thread():
        for sig in _SIGNALS:
            _PREVIOUS[sig] = signal.signal(sig, _on_signal)
    _OPEN.add(dev)


def _unregister(dev) -> None:
    _OPEN.discard(dev)
    if not _OPEN and threading.current_thread() is threading.main_thread():
        for sig, prev in list(_PREVIOUS.items()):
            if signal.getsignal(sig) is _on_signal:
                signal.signal(sig, prev)
            del _PREVIOUS[sig]


@atexit.register
def _close_all() -> None:
    for dev in list(_OPEN):
        with contextlib.suppress(DeviceError):
            dev.close()


class Device:
    """One QM-RDK. Opening runs the protocol §4 connect sequence; `close`
    (also on context exit and interpreter exit) sends `SWEEP:STOP` unless
    `leave_rf_on` is set; SIGINT and SIGTERM always stop the RF."""

    def __init__(
        self,
        resource: str | None = None,
        serial: str | None = None,
        manager=None,
        timeout: int = TIMEOUT_MS,
    ):
        self.transport = Transport(resource, serial, manager, timeout)
        self.leave_rf_on = False
        self.running = False
        _register(self)
        try:
            self.boot_errors = self._connect()
            self.sweep = self.settings().sweep
        except BaseException:
            self.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    @property
    def idn(self) -> Idn:
        return self.transport.idn

    def _connect(self) -> list[tuple[int, str]]:
        """Discard pending frame data and clear status; returns the errors
        queued before, `Power on` included."""
        self.transport.flush()
        queued = self.transport.errors(power_on=True)
        self.transport.command("*CLS")
        return queued

    def identify(self) -> Idn:
        return Idn.parse(self.transport.query("*IDN?"))

    def settings(self) -> Settings:
        q = self.transport.query
        sweep = Sweep(
            f0=float(round(float(q("SWEEP:FREQSTAR?")) * 1e9)),
            f1=float(round(float(q("SWEEP:FREQSTOP?")) * 1e9)),
            ramp_time=round(float(q("SWEEP:RAMPTIME?"))) / 1e3,
            kind=KINDS[int(q("SWEEP:TYPE?"))],
        )
        return Settings(
            sweep,
            int(q("FREQ:REF:DIV?")),
            q("POWE:RF?") == "1",
            q("FREQ:LOCK?") == "1",
        )

    def temperature(self) -> float:
        """Maximum board temperature, °C."""
        return float(self.transport.query("SYST:TEMP?"))

    def status(self) -> Status:
        return Status(*parse_entry(self.transport.query("SYST:STAT?")))

    def errors(self) -> list[tuple[int, str]]:
        return self.transport.errors()

    def configure(self, sweep: Sweep) -> Sweep:
        """Protocol §4 configure sequence and lock wait; returns and keeps the
        read-back sweep. RF is off if it fails."""
        kind, ramp, div = validate(sweep)
        t = self.transport
        try:
            t.command(f"SWEEP:TYPE {kind}")
            t.command(f"SWEEP:FREQSTAR {sweep.f0 / 1e9:f}")
            t.command(f"SWEEP:FREQSTOP {sweep.f1 / 1e9:f}")
            if int(t.query("FREQ:REF:DIV?")) != div:
                t.command(f"FREQ:REF:DIV {div}")
            t.command(f"SWEEP:RAMPTIME {ramp}")
            self.start()
            self.sweep = self.settings().sweep
        except DeviceError:
            self.rf_off()
            raise
        return self.sweep

    def start(self) -> None:
        """`SWEEP:START` and wait for PLL lock; RF off and LockError on timeout."""
        self.transport.command("SWEEP:START")
        deadline = time.monotonic() + LOCK_TIMEOUT
        while self.transport.query("FREQ:LOCK?") != "1":
            if time.monotonic() > deadline:
                self.rf_off()
                raise LockError("PLL not locked after SWEEP:START")
            time.sleep(LOCK_POLL)
        self.running = True

    def stop(self) -> None:
        """`SWEEP:STOP` and confirm the RF is off."""
        self.running = False
        self.transport.command("SWEEP:STOP")
        if self.transport.query("POWE:RF?") != "0":
            raise DeviceError("RF still on after SWEEP:STOP")

    def rf(self, on: bool) -> None:
        """RF on (sweep started and locked) or off."""
        if on:
            self.start()
        else:
            self.stop()

    def rf_off(self) -> None:
        """Best-effort `SWEEP:STOP` for error paths and signal handlers."""
        self.running = False
        with contextlib.suppress(DeviceError):
            self.transport.write("SWEEP:STOP")

    def reset(self) -> None:
        """`*RST` reboots the board; reopen it after re-enumeration and stop
        the RF it powers up with."""
        self.running = False
        self.transport.write("*RST")
        self.transport.wait_gone()
        self.transport.reopen()
        self.boot_errors = self._connect()
        self.stop()
        self.sweep = self.settings().sweep

    def save(self, slot: int, force: bool = False) -> None:
        """`*SAV`; location 0 (the power-up state) only with `force`."""
        self.scpi(f"*SAV {int(slot)}", force)

    def recall(self, slot: int) -> None:
        self.scpi(f"*RCL {int(slot)}")

    def restore_factory(self, force: bool = False) -> None:
        """`SYST:REST` overwrites the power-up state; only with `force`."""
        self.scpi("SYST:REST", force)

    def scpi(self, message: str, force: bool = False) -> str | None:
        """Raw command or query. Refuses undocumented commands and Bluetooth
        streaming always, `*SAV 0` and `SYST:REST` unless `force`, and `*RST`
        (use `reset`)."""
        for cmd in message.split(";"):
            head, _, arg = cmd.strip().upper().lstrip(":").partition(" ")
            if head == "*RST":
                raise ForbiddenCommand("use reset() for *RST")
            protected = (head == "*SAV" and arg.strip() in ("0", "+0")) or (
                head.startswith("SYST") and head.split(":")[-1].startswith("REST")
            )
            if protected and not force:
                raise ForbiddenCommand(f"{cmd.strip()!r} overwrites the power-up state")
        if message.rstrip().endswith("?"):
            try:
                return self.transport.query(message)
            except DeviceTimeout:
                self.transport.check(message)
                raise
        self.transport.command(message)
        return None

    def _frame(self, n: int) -> tuple[float, np.ndarray]:
        """Protocol §3.4 steps 1–3: host time at `CAPT:FRAM` and the codes."""
        t_host = time.time()
        self.transport.write(f"CAPT:FRAM {n}")
        codes = np.empty(n, dtype=np.uint16)
        first = round(1e3 * (ARM_DELAY + n / FS_NOMINAL + CAPTURE_MARGIN))
        for got in range(0, n, CHUNK):
            resp = self.transport.query(
                "CAPT:FRAM?",
                timeout=None if got else max(first, self.transport.timeout),
            )
            if resp == NOT_READY:
                raise FrameError(f"Not Ready after {got} of {n} samples")
            codes[got : got + CHUNK] = decode_chunk(resp, min(CHUNK, n - got))
        return t_host, codes

    def _recover(self) -> None:
        """Reopen after a lost session; the board may have rebooted, so its
        configuration is re-applied."""
        self.transport.reopen()
        self.boot_errors = self._connect()
        self.configure(self.sweep)

    def capture_timed(self, n: int) -> tuple[float, np.ndarray]:
        """One frame of `n` codes and its host time; a failed frame is
        flushed and retried, a lost session reopened. RF off on failure."""
        if not 1 <= n <= MAX_SAMPLES:
            raise ConfigError(f"frame length {n} outside 1..{MAX_SAMPLES}")
        if not self.running:
            raise DeviceError("sweep not started: configure or rf on first")
        failure = None
        for _ in range(RETRIES):
            try:
                return self._frame(n)
            except Disconnected as err:
                failure = err
                try:
                    self._recover()
                except DeviceError as lost:
                    raise DeviceError(f"capture failed: {lost}") from err
            except (FrameError, DeviceTimeout) as err:
                failure = err
                with contextlib.suppress(DeviceError):
                    self.transport.flush()
                    self.transport.errors()
        self.rf_off()
        raise DeviceError(f"capture failed: {failure}") from failure

    def capture(self, n: int) -> np.ndarray:
        """One frame of `n` uint16 codes."""
        return self.capture_timed(n)[1]

    def capture_many(
        self,
        n: int,
        count: int,
        desc: str = "capture",
        interval: float = 0.0,
        temperature: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        """`count` frames, [count, n] codes, their host UNIX times and, with
        `temperature`, the board temperature read before each frame (else
        None). Frame starts are at least `interval` seconds apart."""
        codes = np.empty((count, n), dtype=np.uint16)
        t_host = np.empty(count)
        temp = np.empty(count) if temperature else None
        start = -math.inf
        for i in tqdm(range(count), desc=desc, unit="frame"):
            time.sleep(max(0.0, start + interval - time.monotonic()))
            start = time.monotonic()
            if temp is not None:
                temp[i] = self.temperature()
            t_host[i], codes[i] = self.capture_timed(n)
        return codes, t_host, temp

    def _reopen_and_stop(self) -> None:
        """Stop the RF of a board that rebooted; an absent board is unpowered."""
        try:
            self.transport.reopen()
        except DeviceError:
            return
        self.stop()

    def close(self) -> None:
        """Stop the sweep (RF off) unless `leave_rf_on`, and release the board.
        A lost session is reopened to stop the RF of a rebooted board."""
        if self.transport.res is None and self not in _OPEN:
            return
        try:
            if not self.leave_rf_on:
                try:
                    self.stop()
                except Disconnected:
                    self._reopen_and_stop()
        finally:
            self.transport.close()
            _unregister(self)

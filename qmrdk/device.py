"""Typed QM-RDK command layer over `qmrdk.transport` (docs/protocol.md §3, §4,
§6): validated configuration, lock wait, frame or streamed capture with
recovery, and RF off on every exit path. Stock firmware and the CV
replacement firmware are told apart by `*IDN?`."""

import atexit
import contextlib
import dataclasses
import math
import re
import signal
import struct
import threading
import time

import numpy as np
from tqdm import tqdm

from qmrdk.config import Sweep, Timing
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
    FrameError,
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
    "FrameError",
    "Idn",
    "Limits",
    "ScpiError",
    "Settings",
    "Status",
    "decode_block",
    "decode_chunk",
    "ref_divider",
]

F_REF = 20e6
F_MIN, F_MAX = 2.4e9, 2.5e9
RAMP_MS_MAX = 65536
REF_DIV_MAX = 256
SWEEP_TYPES = {"ramp": 0, "tri": 1, "triangle": 2, "cw": 3}
KINDS = {v: k for k, v in SWEEP_TYPES.items()}
CV_TYPES = SWEEP_TYPES | {"saw": 4}
CV_KINDS = {v: k for k, v in CV_TYPES.items()}
MIN_SAMPLES = 4
BLOCK = 1024
BLOCK_FIELDS = struct.Struct("<IIHH")
OVERFLOW, END = 1, 2
CONT_MAX = 2**31 - 1
REPARK = 50e-3
PROGRESS_BLOCKS = 8
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


@dataclasses.dataclass(frozen=True)
class Settings:
    """Read-back board state; `fs`, `n0` and `first_up` are the device timing
    of CV firmware (None on stock firmware)."""

    sweep: Sweep
    ref_div: int
    rf: bool
    locked: bool
    fs: float | None = None
    n0: float | None = None
    first_up: bool | None = None

    @property
    def timing(self) -> Timing | None:
        return None if self.fs is None else Timing(self.fs, self.n0, self.first_up)


@dataclasses.dataclass(frozen=True)
class Limits:
    """`SWEEP:LIM?` of CV firmware (protocol §6): frequency range (Hz),
    minimum slope (Hz/s), ramp time quantum and minimum (s)."""

    f_min: float
    f_max: float
    slope_min: float
    t_quantum: float
    t_min: float

    @classmethod
    def parse(cls, resp: str) -> "Limits":
        try:
            f_min, f_max, slope, quantum, t_min = map(float, resp.split(","))
        except ValueError as err:
            raise DeviceError(f"malformed SWEEP:LIM? {resp!r}") from err
        return cls(
            float(round(f_min * 1e9)), float(round(f_max * 1e9)),
            slope, quantum / 1e3, t_min / 1e3,
        )  # fmt: skip


@dataclasses.dataclass(frozen=True)
class Status:
    """`SYST:STAT?`: 0 operational, 1 reset, 2 awaiting input, 100/101
    recoverable/non-recoverable error, 110 over temperature."""

    code: int
    text: str


def nearest(x: float) -> int:
    """`x` rounded to the nearest integer, halves away from zero (C `lround`)."""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def ref_divider(sweep: Sweep) -> int:
    """Smallest reference divider whose minimum slope allows the sweep
    (protocol §3.3)."""
    if sweep.kind == "cw":
        return 1
    return max(1, math.ceil(F_REF**2 / (sweep.slope * 2**25)))


def _validate_cv(sweep: Sweep, lim: Limits, fs: float) -> tuple[int, float, None]:
    if sweep.kind not in CV_TYPES:
        raise ConfigError(f"sweep kind {sweep.kind!r} not in {list(CV_TYPES)}")
    for f in (sweep.f0, sweep.f1):
        if not lim.f_min <= f <= lim.f_max:
            raise ConfigError(
                f"frequency {f / 1e9:g} GHz outside "
                f"{lim.f_min / 1e9:g}..{lim.f_max / 1e9:g} GHz"
            )
    nr = nearest(sweep.ramp_time * fs)
    if not 0 < sweep.ramp_time <= RAMP_MS_MAX / 1e3 or nr < MIN_SAMPLES:
        raise ConfigError(
            f"ramp time {sweep.ramp_time * 1e3:g} ms outside "
            f"{lim.t_min * 1e3:g}..{RAMP_MS_MAX} ms"
        )
    if sweep.kind != "cw":
        if sweep.f1 <= sweep.f0:
            raise ConfigError("stop frequency must be above start frequency")
        slope = sweep.bandwidth * fs / nr
        if slope < lim.slope_min:
            raise ConfigError(
                f"slope {slope:.4g} Hz/s below the minimum {lim.slope_min:.4g} "
                "Hz/s: shorten the ramp or widen the band"
            )
    if sweep.kind == "saw":
        if sweep.dwell and nearest(sweep.dwell * fs) < MIN_SAMPLES:
            raise ConfigError(
                f"dwell {sweep.dwell * 1e3:g} ms below {lim.t_min * 1e3:g} ms"
            )
    elif sweep.dwell:
        raise ConfigError("dwell applies to sawtooth sweeps only")
    return CV_TYPES[sweep.kind], sweep.ramp_time * 1e3, None


def validate(
    sweep: Sweep, limits: Limits | None = None, fs: float | None = None
) -> tuple[int, int | float, int | None]:
    """Sweep type number, ramp time (ms) and reference divider of `sweep`;
    ConfigError if the board cannot run it. Stock firmware (`limits` None):
    integer ms and the divider model of §3.3. CV firmware (its `limits`, `fs`):
    at least `MIN_SAMPLES` samples per ramp, `limits.slope_min`, no divider."""
    if limits is not None:
        return _validate_cv(sweep, limits, fs)
    if sweep.kind == "saw":
        raise ConfigError("sawtooth sweeps need CV firmware (protocol §6)")
    if sweep.dwell:
        raise ConfigError("dwell applies to sawtooth sweeps only")
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


def decode_block(payload: bytes, seq: int, first: int, n: int) -> np.ndarray:
    """Samples of one `CAPT:BLOC?` payload (protocol §6) expected as block
    `seq` starting at sample `first` of an `n`-sample acquisition;
    FrameError if it is empty, malformed, out of sequence, follows dropped
    samples (OVERFLOW) or disagrees with `n` (count, END)."""
    if len(payload) < BLOCK_FIELDS.size:
        raise FrameError(f"no block after {first} of {n} samples")
    got_seq, got_first, count, flags = BLOCK_FIELDS.unpack_from(payload)
    if len(payload) != BLOCK_FIELDS.size + 2 * count:
        raise FrameError(f"block {got_seq}: {len(payload)} bytes for {count} samples")
    if flags & OVERFLOW:
        raise FrameError(f"block {got_seq}: samples dropped before {got_first}")
    if (got_seq, got_first) != (seq, first):
        raise FrameError(
            f"block {got_seq} at sample {got_first}, expected {seq} at {first}"
        )
    last = first + count == n
    if not 0 < count <= n - first or bool(flags & END) != last:
        raise FrameError(
            f"block {seq}: {count} samples, END {bool(flags & END)}, "
            f"at {first} of {n}"
        )
    return np.frombuffer(payload, "<u2", count, BLOCK_FIELDS.size).astype(np.uint16)


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


class Device:  # pylint: disable=too-many-public-methods
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
        self.dark = False
        self.running = False
        self.limits, self.fs = None, None
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

    @property
    def cv(self) -> bool:
        """The board runs CV firmware (protocol §6)."""
        return self.idn.cv

    def _discard(self) -> None:
        """Drop pending capture data: `CAPT:ABOR` on CV firmware, else read
        `CAPT:FRAM?` until `Not Ready`."""
        if self.cv:
            self.transport.write("CAPT:ABOR")
        else:
            self.transport.flush()

    def _connect(self) -> list[tuple[int, str]]:
        """Stop a stream left running (CV firmware), discard pending frame data
        and clear status; returns the errors queued before, `Power on` included."""
        if self.cv:
            self.transport.write("CAPT:ABOR")
        self.transport.flush()
        queued = self.transport.errors(power_on=True)
        self.transport.command("*CLS")
        return queued

    def identify(self) -> Idn:
        return Idn.parse(self.transport.query("*IDN?"))

    def settings(self) -> Settings:
        """Read-back state. On CV firmware also refreshes `fs` (exactly
        `F_REF / R`) and `limits`, and reads the sweep timing."""
        q = self.transport.query
        if self.cv:
            self.fs = F_REF / nearest(F_REF / float(q("CAPT:RATE?")))
            self.limits = Limits.parse(q("SWEEP:LIM?"))
        f0 = float(round(float(q("SWEEP:FREQSTAR?")) * 1e9))
        f1 = float(round(float(q("SWEEP:FREQSTOP?")) * 1e9))
        ramp = float(q("SWEEP:RAMPTIME?"))
        kind = (CV_KINDS if self.cv else KINDS)[int(q("SWEEP:TYPE?"))]
        sweep = Sweep(
            f0=f0,
            f1=f1,
            ramp_time=ramp / 1e3 if self.cv else round(ramp) / 1e3,
            kind=kind,
            dwell=float(q("SWEEP:DWEL?")) / 1e3 if kind == "saw" else 0.0,
        )
        if not self.cv:
            return Settings(
                sweep,
                int(q("FREQ:REF:DIV?")),
                q("POWE:RF?") == "1",
                q("FREQ:LOCK?") == "1",
            )
        rf, locked = q("POWE:RF?") == "1", q("FREQ:LOCK?") == "1"
        return Settings(sweep, 1, rf, locked, self.fs, *self._origin())

    def _origin(self) -> tuple[float, bool]:
        """`SWEEP:ORIG?`: commanded first turnaround (samples) and direction."""
        resp = self.transport.query("SWEEP:ORIG?")
        try:
            n0, up = resp.split(",")
            return float(n0), int(up) == 1
        except ValueError as err:
            raise DeviceError(f"malformed SWEEP:ORIG? {resp!r}") from err

    def timing(self) -> Timing | None:
        """Device timing of the current sweep on CV firmware, else None."""
        return Timing(self.fs, *self._origin()) if self.cv else None

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
        cv = (self.limits, self.fs) if self.cv else ()
        kind, ramp, div = validate(sweep, *cv)
        t = self.transport
        fmt = ".9f" if self.cv else "f"
        try:
            t.command(f"SWEEP:TYPE {kind}")
            t.command(f"SWEEP:FREQSTAR {sweep.f0 / 1e9:{fmt}}")
            t.command(f"SWEEP:FREQSTOP {sweep.f1 / 1e9:{fmt}}")
            if self.cv:
                t.command(f"SWEEP:RAMPTIME {ramp:.6f}")
                if sweep.kind == "saw" and sweep.dwell > 0:
                    t.command(f"SWEEP:DWEL {sweep.dwell * 1e3:.6f}")
            else:
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
        """`SWEEP:START` and wait for PLL lock; RF off and LockError on timeout.
        With `dark` the same message switches the RF off again, so the sweep
        and captures run with the transmitter muted (bench use without an
        antenna or load)."""
        self.transport.command("SWEEP:START;:POWE:RF 0" if self.dark else "SWEEP:START")
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
        streaming always, `*SAV 0` and `SYST:REST` unless `force`, `*RST`
        (use `reset`) and the binary `CAPT:BLOC?` (use `capture`)."""
        for cmd in message.split(";"):
            head, _, arg = cmd.strip().upper().lstrip(":").partition(" ")
            if head == "*RST":
                raise ForbiddenCommand("use reset() for *RST")
            if head.startswith("CAPT:BLOC"):
                raise ForbiddenCommand("use capture() for binary blocks")
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

    def _latency(self) -> float:
        """Longest wait (s) from `CAPT:CONT` to the first sample (protocol
        §6): one period of a running triangle or sawtooth, the re-park of a
        single ramp or triangle."""
        s = self.sweep
        return {
            "triangle": 2.0 * s.ramp_time,
            "saw": s.ramp_time + s.dwell,
            "ramp": REPARK,
            "tri": REPARK,
        }.get(s.kind, 0.0)

    def _stream(self, n: int) -> tuple[float, np.ndarray]:
        """Protocol §6 streamed capture: host time at `CAPT:CONT n` and the
        codes of the validated blocks; a progress bar beyond
        `PROGRESS_BLOCKS` blocks."""
        t_host = time.time()
        self.transport.write(f"CAPT:CONT {n}")
        codes = np.empty(n, dtype=np.uint16)
        wait = self._latency() + min(n, BLOCK) / self.fs + CAPTURE_MARGIN
        got = seq = 0
        with tqdm(
            total=n,
            desc="stream",
            unit="sample",
            unit_scale=True,
            leave=False,
            disable=n <= PROGRESS_BLOCKS * BLOCK,
        ) as progress:
            while got < n:
                payload = self.transport.query_block(
                    "CAPT:BLOC?", timeout=max(round(1e3 * wait), self.transport.timeout)
                )
                block = decode_block(payload, seq, got, n)
                codes[got : got + block.size] = block
                got, seq = got + block.size, seq + 1
                progress.update(block.size)
                wait = BLOCK / self.fs + CAPTURE_MARGIN
        return t_host, codes

    def _recover(self) -> None:
        """Reopen after a lost session; the board may have rebooted, so its
        configuration is re-applied."""
        self.transport.reopen()
        self.boot_errors = self._connect()
        if self.cv:
            self.settings()
        self.configure(self.sweep)

    def capture_timed(self, n: int) -> tuple[float, np.ndarray]:
        """One frame of `n` codes and its host time: `CAPT:FRAM` pages on
        stock firmware (n <= 4096), a `CAPT:CONT` stream of any length on CV
        firmware. A failed frame is discarded and retried, a lost session
        reopened. RF off on failure."""
        top = CONT_MAX if self.cv else MAX_SAMPLES
        if not 1 <= n <= top:
            raise ConfigError(f"frame length {n} outside 1..{top}")
        if not self.running:
            raise DeviceError("sweep not started: configure or rf on first")
        read = self._stream if self.cv else self._frame
        failure = None
        for _ in range(RETRIES):
            try:
                return read(n)
            except Disconnected as err:
                failure = err
                try:
                    self._recover()
                except DeviceError as lost:
                    raise DeviceError(f"capture failed: {lost}") from err
            except (FrameError, DeviceTimeout) as err:
                failure = err
                with contextlib.suppress(DeviceError):
                    self._discard()
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
            if temp is not None:
                temp[i] = self.temperature()
            time.sleep(max(0.0, start + interval - time.monotonic()))
            start = time.monotonic()
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

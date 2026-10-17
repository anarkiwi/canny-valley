"""Simulated QM-RDK behind the PyVISA resource interface (docs/protocol.md):
the SCPI state machine of firmware V1.1.0 or of the CV firmware (§6), with
their formats, errors and capture paths, a resource manager that enumerates
boards, and fault injection for the host's recovery paths."""

import collections
import dataclasses
import functools
import inspect
import re
import time

import numpy as np
from pyvisa import errors
from pyvisa.constants import StatusCode

from qmrdk.config import ScanGeometry, Sweep
from qmrdk.device import (
    ARM_DELAY,
    BLOCK,
    BLOCK_FIELDS,
    CONT_MAX,
    CV_KINDS,
    END,
    F_REF,
    KINDS,
    MIN_SAMPLES,
    OVERFLOW,
    REPARK,
    nearest,
)
from qmrdk.sim.devices import SimRadar, SimSled
from qmrdk.sim.hardware import Hardware
from qmrdk.sim.scene import builtin_scene, compile_scene
from qmrdk.transport import CHUNK, MAKER, MAX_SAMPLES, MODEL, NOT_READY, TIMEOUT_MS, VID

PID = 0x13
MID_SCALE = 32768
FAULTS = (
    "nonhex",
    "truncate",
    "not_ready",
    "stall",
    "disconnect",
    "unplug",
    "overflow",
)
MESSAGES = {
    -102: "Syntax error",
    -108: "Parameter not allowed",
    -109: "Missing parameter",
    -112: "Program mnemonic too long",
    -113: "Undefined header",
    -211: "Trigger ignored",
    -222: "Data out of range",
    -224: "Illegal parameter value",
    -350: "Queue overflow",
    -500: "Power on",
    201: "Out of range",
}
# header: (type, minimum, maximum, read-back format); RAMPTIME 0 is accepted (H)
SETTINGS = {
    "SWEEP:FREQSTAR": (float, 2.4, 2.5, "{:.3f}"),
    "SWEEP:FREQSTOP": (float, 2.4, 2.5, "{:.3f}"),
    "SWEEP:RAMPTIME": (int, 0, 65536, "{:.2f}"),
    "SWEEP:TYPE": (int, 0, 3, "{:d}"),
    "FREQ:REF:DIV": (int, 1, 256, "{:d}"),
}
DEFAULTS = {
    "SWEEP:FREQSTAR": 2.4,
    "SWEEP:FREQSTOP": 2.5,
    "SWEEP:RAMPTIME": 16,
    "SWEEP:TYPE": 2,
    "FREQ:REF:DIV": 1,
}
# CV firmware header: (type, minimum, maximum); read-backs are the realised values
CV_SETTINGS = {
    "SWEEP:FREQSTAR": (float, 2.25, 2.5),
    "SWEEP:FREQSTOP": (float, 2.25, 2.5),
    "SWEEP:RAMPTIME": (float, 0.0, 65536.0),
    "SWEEP:TYPE": (int, 0, 4),
    "SWEEP:DWEL": (float, 0.0, np.inf),
    "FREQ:REF:DIV": (int, 1, 256),
    "CAPT:RATE": (float, 0.0, np.inf),
    "POWE:RF:TIME": (int, 0, 3600),
}
CV_DEFAULTS = DEFAULTS | {
    "SWEEP:RAMPTIME": 16.0,
    "SWEEP:DWEL": 1.0,
    "CAPT:RATE": 25_000.0,
    "POWE:RF:TIME": 30,
}
CV_R = (500, 16383)
CV_ORIGIN = 14.5e6
Q24 = 2**24
EMPTY_BLOCK = b"#6000000"
_INT = re.compile(r"[+-]?\d+")
_REAL = re.compile(r"[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?")
_BOOL = {"ON": True, "1": True, "OFF": False, "0": False}


def _timeout():
    return errors.VisaIOError(StatusCode.error_timeout)


def scene_source(
    scene: str = "single", hardware: Hardware | None = None, seed=0, temperature=None
):
    """Frame source `(n, sweep, timing=None) -> codes` of a built-in scene on
    the board model, at the temperature `temperature()` if given; types 0 and
    1 hold the frequency after one ramp or triangle (protocol §3.1). With CV
    device `timing` `(fs, n0, first_up)` the model runs at that clock (§6)."""
    hw = Hardware() if hardware is None else hardware
    rng = np.random.default_rng(seed)
    radars = {}

    def source(n: int, sweep: Sweep, timing=None) -> np.ndarray:
        board = hw
        if timing is not None:
            fs, n0, first_up = timing
            f0, f1 = sorted((sweep.f0, sweep.f1))
            sweep = dataclasses.replace(sweep, f0=f0, f1=f1)
            board = dataclasses.replace(
                hw,
                fs=fs,
                t_start=n0 / fs,
                t_reset=0.0,
                first_up=bool(first_up),
                f_prev=f0 if first_up else f1,
                continuous=sweep.kind in ("triangle", "saw"),
            )
        kind = sweep.kind if sweep.kind in ("cw", "saw") else "triangle"
        auto = dataclasses.replace(sweep, kind=kind)
        if auto not in radars:
            geom = compile_scene(builtin_scene(scene), auto.lam)
            radars[auto] = SimRadar(geom, hw, auto, SimSled(), ScanGeometry(), rng)
        radars[auto].hardware = (
            board
            if temperature is None
            else dataclasses.replace(board, temperature=temperature())
        )
        codes = radars[auto].capture(n)
        if sweep.kind in ("ramp", "tri"):
            ramps = 1 if sweep.kind == "ramp" else 2
            end = board.fs * (board.t_start + ramps * sweep.ramp_time)
            codes[int(np.ceil(end)) :] = MID_SCALE
        return codes

    return source


class SimBoard:
    """One simulated board: `source(n, sweep)` supplies the codes of each
    capture, `faults` (FAULTS) apply to successive captures, `log` holds every
    message; `*RST` and `disconnect` re-enumerate after `reboot_s`. A
    `firmware` "CV..." emulates protocol §6, calling `source(n, sweep, timing)`."""

    def __init__(
        self,
        serial: str = "0042",
        firmware: str = "V1.1.0",
        source=None,
        reboot_s: float = 0.0,
        temperature=31.25,
        fs: float = Hardware.fs,
        hardware: Hardware | None = None,
    ):
        self.serial, self.firmware = serial, firmware
        self.cv = firmware.startswith("CV")
        self.captures = 0
        self.source = (
            scene_source(hardware=hardware, temperature=self.read_temperature)
            if source is None
            else source
        )
        self.reboot_s, self.temperature, self.fs = reboot_s, temperature, fs
        self.memory = {0: dict(CV_DEFAULTS if self.cv else DEFAULTS)}
        self.faults = collections.deque()
        self.stuck_unlocked = False
        self.powered, self.boot, self.back_at = True, 0, 0.0
        self.log = []
        self._power_up()

    def _power_up(self):
        self.state = dict(self.memory[0])
        self.rf = self.sweeping = not self.cv
        self.errs = [(-500, MESSAGES[-500])]
        self.esr, self.ese, self.sre = 128, 0, 0
        self.frame, self.cursor, self.fault, self.wait_s = "", 0, None, None
        self.acq = None
        if self.cv:
            self.fs = F_REF / self._rate_div()

    def read_temperature(self) -> float:
        """Board temperature, degC: `temperature`, or its value at the number
        of frames captured so far if it is a function."""
        t = self.temperature
        return float(t(self.captures) if callable(t) else t)

    @property
    def online(self) -> bool:
        return self.powered and time.monotonic() >= self.back_at

    def reboot(self) -> None:
        """Drop off the bus and come back in the power-up state."""
        self.boot += 1
        self.back_at = time.monotonic() + self.reboot_s
        self._power_up()

    def unplug(self) -> None:
        self.boot += 1
        self.powered = self.rf = self.sweeping = False

    def inject(self, *faults: str) -> None:
        unknown = set(faults) - set(FAULTS)
        if unknown:
            raise ValueError(f"unknown faults {sorted(unknown)}")
        self.faults.extend(faults)

    def _rate_div(self) -> int:
        """CV sample clock divider R, `fs = F_REF / R`."""
        return nearest(F_REF / self.state["CAPT:RATE"])

    def _ramp(self):
        """CV quantisation (§6): `(Nr, N0q, N0q + step K, step, Nd)` with
        frequencies in Q24 units of `f / F_REF` and `K = Nr R`."""
        s, r = self.state, self._rate_div()
        nr = nearest(s["SWEEP:RAMPTIME"] * 1e-3 * self.fs)
        n0q, n1q = (
            nearest(s[k] * 1e9 * Q24 / F_REF)
            for k in ("SWEEP:FREQSTAR", "SWEEP:FREQSTOP")
        )
        step = nearest((n1q - n0q) / (nr * r))
        nd = nearest(s["SWEEP:DWEL"] * 1e-3 * self.fs)
        return nr, n0q, n0q + step * nr * r, step, nd

    @property
    def sweep(self) -> Sweep:
        s = self.state
        if self.cv:
            nr, n0q, n1q, _, nd = self._ramp()
            kind = CV_KINDS[s["SWEEP:TYPE"]]
            return Sweep(
                f0=n0q * F_REF / Q24,
                f1=n1q * F_REF / Q24,
                ramp_time=nr / self.fs,
                kind=kind,
                dwell=nd / self.fs if kind == "saw" else 0.0,
            )
        return Sweep(
            f0=s["SWEEP:FREQSTAR"] * 1e9,
            f1=s["SWEEP:FREQSTOP"] * 1e9,
            ramp_time=s["SWEEP:RAMPTIME"] * 1e-3,
            kind=KINDS[s["SWEEP:TYPE"]],
        )

    @property
    def timing(self) -> tuple[float, float, bool]:
        """CV device timing `(fs, n0, first_up)` of the current sweep."""
        if self.state["SWEEP:TYPE"] == 3:
            return self.fs, 0.0, True
        n0 = nearest(CV_ORIGIN / self._rate_div()) / 1e6
        return self.fs, n0, self._ramp()[3] > 0

    def locked(self) -> bool:
        """Protocol §3.3: the slope must reach f_ref^2 / (div * 2^25); CV
        firmware: a non-zero step per PD cycle."""
        s = self.state
        if not self.sweeping or self.stuck_unlocked:
            return False
        if s["SWEEP:TYPE"] == 3:
            return True
        if self.cv:
            return self._ramp()[3] != 0
        span = abs(s["SWEEP:FREQSTOP"] - s["SWEEP:FREQSTAR"]) * 1e9
        ramp = s["SWEEP:RAMPTIME"] * 1e-3
        return ramp > 0 and span / ramp >= F_REF**2 / (s["FREQ:REF:DIV"] * 2**25)

    def _error(self, code: int) -> None:
        if len(self.errs) >= 10:
            self.errs[-1] = (-350, MESSAGES[-350])
        else:
            self.errs.append((code, MESSAGES[code]))
        self.esr |= {1: 32, 2: 16, 4: 4}.get(-code // 100, 8)

    def _number(self, arg, kind, lo, hi):
        """Parsed parameter, or None with -109 / -102 / -222 (CV: 201) queued."""
        if not arg:
            return self._error(-109)
        if not (_INT if kind is int else _REAL).fullmatch(arg):
            return self._error(-102)
        value = kind(arg)
        if not lo <= value <= hi:
            return self._error(201 if self.cv else -222)
        return value

    def _cv_valid(self, head, value) -> bool:
        """CV range rules beyond the bounds: at least `MIN_SAMPLES` per ramp
        and dwell, the sample clock divider within `CV_R`."""
        if head == "SWEEP:RAMPTIME":
            return value > 0 and nearest(value * 1e-3 * self.fs) >= MIN_SAMPLES
        if head == "SWEEP:DWEL":
            return nearest(value * 1e-3 * self.fs) >= MIN_SAMPLES
        if head == "CAPT:RATE":
            return value > 0 and CV_R[0] <= nearest(F_REF / value) <= CV_R[1]
        return True

    def message(self, msg: str, timeout_s: float) -> str | bytes | None:
        """Execute one program message; the response of its last query."""
        self.log.append(msg.strip())
        out = None
        for cmd in msg.split(";"):
            parts = cmd.split(None, 1)
            if not parts:
                continue
            head, arg = parts[0].upper().lstrip(":"), "".join(parts[1:]).strip()
            if any(len(node) > 12 for node in head.rstrip("?").split(":")):
                self._error(-112)
                continue
            resp = self._execute(head, arg, timeout_s)
            out = out if resp is None else resp
        return out

    def _settings(self):
        return CV_SETTINGS if self.cv else SETTINGS

    def _execute(self, head, arg, timeout_s):
        if head.endswith("?"):
            return self._query(head, arg, timeout_s)
        if head in self._settings():
            value = self._number(arg, *self._settings()[head][:3])
            if value is not None and self.cv and not self._cv_valid(head, value):
                self._error(201)
            elif value is not None:
                self.state[head] = value
                self.sweeping &= head != "SWEEP:TYPE"
                if head == "CAPT:RATE":
                    self.fs = F_REF / self._rate_div()
            return None
        command = self._commands().get(head)
        if command is None:
            return self._error(-113)
        if inspect.signature(command).parameters:
            return command(arg) if arg else self._error(-109)
        return self._error(-108) if arg else command()

    def _query(self, head, arg, timeout_s):
        if arg:
            return self._error(-108)
        if head[:-1] in self._settings():
            if self.cv:
                return self._cv_readback(head[:-1])
            return SETTINGS[head[:-1]][3].format(self.state[head[:-1]])
        query = self._queries().get(head)
        return self._error(-113) if query is None else query(timeout_s)

    def _cv_readback(self, head):
        """Realised CV setting in its §6 format."""
        nr, n0q, n1q, _, nd = self._ramp()
        value = {
            "SWEEP:FREQSTAR": f"{n0q * F_REF / Q24 / 1e9:.9f}",
            "SWEEP:FREQSTOP": f"{n1q * F_REF / Q24 / 1e9:.9f}",
            "SWEEP:RAMPTIME": f"{nr / self.fs * 1e3:.6f}",
            "SWEEP:DWEL": f"{nd / self.fs * 1e3:.6f}",
            "CAPT:RATE": f"{self.fs:.6f}",
        }
        return value[head] if head in value else f"{self.state[head]:d}"

    def _queries(self):
        stb = (4 if self.errs else 0) | (32 if self.esr & self.ese else 0)
        stb |= 64 if stb & self.sre else 0
        common = {
            "*IDN?": lambda t: f"{MAKER},{MODEL},{self.serial},{self.firmware}",
            "SYST:IDEN?": lambda t: MODEL,
            "SYST:SERNUM?": lambda t: self.serial,
            "SYST:MODNUM?": lambda t: MODEL,
            "SYST:FIRM?": lambda t: self.firmware,
            "SYST:VERS?": lambda t: "1999.0",
            "SYST:TEMP?": lambda t: f"{self.read_temperature():.2f}",
            "SYST:STAT?": lambda t: '0,"Operational"',
            "SYST:ERR?": self._pop_error,
            "*ESR?": self._read_esr,
            "*ESE?": lambda t: str(self.ese),
            "*SRE?": lambda t: str(self.sre),
            "*STB?": lambda t: str(stb),
            "*OPC?": lambda t: "1",
            "*TST?": lambda t: "0",
            "POWE:RF?": lambda t: str(int(self.rf)),
            "FREQ:LOCK?": lambda t: str(int(self.locked())),
        }
        if self.cv:
            return common | {
                "CAPT:FRAM?": self._cv_chunk,
                "CAPT:BLOC?": self._block,
                "SWEEP:ORIG?": lambda t: f"{self.timing[1]:.6f},{self.timing[2]:d}",
                "SWEEP:LIM?": self._limits,
            }
        return common | {
            "*OPT?": lambda t: "0",
            "CAPT:FRAM?": self._chunk,
            "CAPT:STRE?": lambda t: "0",
        }

    def _commands(self):
        common = {
            "*CLS": self._clear,
            "*ESE": lambda a: self._register("ese", a),
            "*SRE": lambda a: self._register("sre", a),
            "*OPC": self._opc,
            "*WAI": lambda: None,
            "*RST": self.reboot,
            "SWEEP:START": lambda: self._run(True),
            "SWEEP:STOP": lambda: self._run(False),
            "POWE:RF": self._power,
        }
        if self.cv:
            return common | {
                "CAPT:FRAM": lambda a: self._acquire(a, 1, MAX_SAMPLES, True),
                "CAPT:CONT": lambda a: self._acquire(a, 0, CONT_MAX, False),
                "CAPT:ABOR": self._abort,
            }
        return common | {
            "*TRG": self._trigger,
            "SYST:PRES": self._preset,
            "*SAV": lambda a: self._memory(a, 0, True),
            "*RCL": lambda a: self._memory(a, 0, False),
            "SYST:CLRM": lambda a: self._memory(a, 1, None),
            "SYST:REST": self._restore,
            "CAPT:FRAM": self._arm,
        }

    def _pop_error(self, _):
        code, text = self.errs.pop(0) if self.errs else (0, "No error")
        return f'{code},"{text}"'

    def _read_esr(self, _):
        esr, self.esr = self.esr, 0
        return str(esr)

    def _clear(self):
        self.errs, self.esr = [], 0

    def _opc(self):
        self.esr |= 1

    def _register(self, name, arg):
        value = self._number(arg, int, 0, 255)
        if value is not None:
            setattr(self, name, value)

    def _trigger(self):
        if self.state["SWEEP:TYPE"] in (0, 1):
            self.sweeping = True
        else:
            self._error(-211)

    def _preset(self):
        self.state = dict(self.memory[0])

    def _restore(self):
        self.memory[0] = dict(DEFAULTS)

    def _memory(self, arg, lo, save):
        slot = self._number(arg, int, lo, 9)
        if slot is None:
            return
        if save is None:
            self.memory.pop(slot, None)
        elif save:
            self.memory[slot] = dict(self.state)
        elif slot in self.memory:
            self.state = dict(self.memory[slot])
        else:
            self._error(-222)

    def _run(self, on):
        if on and self.cv and self.state["SWEEP:TYPE"] != 3 and not self._ramp()[3]:
            return self._error(201)
        self.rf = self.sweeping = on
        return None

    def _power(self, arg):
        if arg.upper() not in _BOOL:
            return self._error(-224)
        self.rf = _BOOL[arg.upper()]
        return None

    def _limits(self, _):
        lo, hi = CV_SETTINGS["SWEEP:FREQSTAR"][1:]
        quantum = 1e3 / self.fs
        return f"{lo:.9f},{hi:.9f},{F_REF**2 / Q24:.6f},{quantum:.6f},{MIN_SAMPLES * quantum:.6f}"

    def _start_fault(self) -> bool:
        """Take the next fault; True if it took the board off the bus."""
        self.fault = self.faults.popleft() if self.faults else None
        if self.fault == "unplug":
            self.unplug()
        elif self.fault == "disconnect":
            self.reboot()
        return self.fault in ("unplug", "disconnect")

    def _arm(self, arg):
        n = self._number(arg, int, 1, MAX_SAMPLES)
        if n is None or self._start_fault():
            return
        sweep = self.sweep
        live = self.rf and self.sweeping and (sweep.kind == "cw" or sweep.ramp_time)
        codes = self.source(n, sweep) if live else np.full(n, MID_SCALE)
        self.captures += 1
        self.frame = np.asarray(codes, ">u2").tobytes().hex().upper()
        self.cursor, self.wait_s = 0, ARM_DELAY + n / self.fs

    def _chunk(self, timeout_s):
        if self.wait_s is not None:
            stalled = self.fault == "stall" or timeout_s < self.wait_s
            self.wait_s = None
            if stalled:
                raise _timeout()
            if self.fault == "not_ready":
                self.frame = ""
        if self.cursor >= len(self.frame):
            return NOT_READY
        chunk = self.frame[self.cursor : self.cursor + 4 * CHUNK]
        first, self.cursor = self.cursor == 0, self.cursor + 4 * CHUNK
        if first and self.fault == "nonhex":
            return "G" + chunk[1:]
        if first and self.fault == "truncate":
            return chunk[:-4]
        return chunk

    def _acquire(self, arg, lo, hi, frame):
        """CV `CAPT:FRAM n` (`frame`) or `CAPT:CONT n` (0: until `CAPT:ABOR`):
        samples from sample 0 of the sweep, first ready after the alignment
        latency (one period, or the re-park of types 0 and 1)."""
        n = self._number(arg, int, lo, hi)
        if n is None or self._start_fault():
            return
        sweep, live = self.sweep, self.rf and self.sweeping
        period = {
            "triangle": 2.0 * sweep.ramp_time,
            "saw": sweep.ramp_time + sweep.dwell,
        }
        latency = period.get(sweep.kind, REPARK if sweep.kind != "cw" else 0.0) * live
        period = round(period.get(sweep.kind, 0.0) * self.fs)
        self.acq = {
            "n": n,
            "live": live,
            "sweep": sweep,
            "timing": self.timing,
            "chunk": period * -(-4 * BLOCK // period) if period else 4 * BLOCK,
            "codes": np.empty(0, np.uint16),
            "cursor": 0,
            "seq": 0,
            "wait": latency + min(n or BLOCK, CHUNK if frame else BLOCK) / self.fs,
        }
        if n:
            self._extend(n)
        self.captures += 1

    def _extend(self, m):
        """Append `m` samples to the acquisition; types 0 and 1 sweep once."""
        a = self.acq
        once = a["sweep"].kind in ("ramp", "tri") and a["codes"].size
        if a["live"] and not once:
            more = self.source(m, a["sweep"], a["timing"])
        else:
            more = np.full(m, MID_SCALE)
        a["codes"] = np.concatenate([a["codes"], np.asarray(more, np.uint16)])

    def _abort(self):
        self.acq = None

    def _take(self, count, timeout_s):
        """`(first, samples)`: up to `count` next samples, None when nothing
        is pending; a timeout if they are not acquired within `timeout_s`."""
        a = self.acq
        if a is None or (a["n"] and a["cursor"] >= a["n"]):
            return None
        count = min(count, a["n"] - a["cursor"]) if a["n"] else count
        wait = a.pop("wait", count / self.fs)
        if timeout_s < wait or (a["seq"] == 0 and self.fault == "stall"):
            a["wait"] = 0.0
            raise _timeout()
        if a["seq"] == 0 and self.fault == "not_ready":
            self.acq = None
            return None
        while a["codes"].size < a["cursor"] + count:
            self._extend(a["chunk"])
        first = a["cursor"]
        a["cursor"] += count
        return first, a["codes"][first : first + count]

    def _cv_chunk(self, timeout_s):
        taken = self._take(CHUNK, timeout_s)
        if taken is None:
            return NOT_READY
        self.acq["seq"] += 1
        return taken[1].astype(">u2").tobytes().hex().upper()

    def _block(self, timeout_s):
        """`CAPT:BLOC?`: the next IEEE 488.2 block of §6, `#6000000` if none;
        faults: `overflow` drops one block before the second, `truncate`
        miscounts and `nonhex` corrupts the header of the first."""
        a = self.acq
        seq = None if a is None else a["seq"]
        overflow = seq == 1 and self.fault == "overflow"
        if overflow:
            a["cursor"] += BLOCK
        taken = self._take(BLOCK, timeout_s)
        if taken is None:
            return EMPTY_BLOCK
        first, data = taken
        a["seq"] += 1
        flags = (END if a["n"] and a["cursor"] >= a["n"] else 0) | (
            OVERFLOW if overflow else 0
        )
        count = data.size + int(seq == 0 and self.fault == "truncate")
        payload = (
            BLOCK_FIELDS.pack(seq, first, count, flags) + data.astype("<u2").tobytes()
        )
        head = b"G6" if seq == 0 and self.fault == "nonhex" else b"#6"
        return head + b"%06d" % len(payload) + payload


class SimResource:
    """The subset of a PyVISA message-based resource the transport uses:
    `read` strips the termination, `read_raw` returns up to `chunk_size` bytes
    and `read_bytes` exactly `count`, unmodified, as pyvisa-py over USBTMC."""

    def __init__(self, board: SimBoard, name: str):
        self.board, self.resource_name, self.boot = board, name, board.boot
        self.timeout = TIMEOUT_MS
        self.read_termination = self.write_termination = "\n"
        self.chunk_size = 20 * 1024
        self.closed = False
        self._out = None

    def _check(self):
        if self.closed:
            raise errors.InvalidSession()
        if not self.board.online or self.board.boot != self.boot:
            raise errors.VisaIOError(StatusCode.error_connection_lost)

    def write(self, message: str) -> int:
        self._check()
        out = self.board.message(message, self.timeout / 1e3)
        if "?" in message:
            out = out.encode() if isinstance(out, str) else out
            self._out = None if out is None else out + b"\n"
        return len(message) + len(self.write_termination or "")

    def _take(self, count: int) -> bytes:
        self._check()
        if not self._out:
            raise _timeout()
        out, self._out = self._out[:count], self._out[count:] or None
        return out

    def read(self) -> str:
        return self._take(len(self._out or b"")).decode().removesuffix("\n")

    def read_raw(self, size: int | None = None) -> bytes:
        return self._take(self.chunk_size if size is None else size)

    def read_bytes(self, count: int, chunk_size=None, break_on_termchar=False):
        del chunk_size, break_on_termchar
        if len(self._out or b"") < count:
            self._out = None
            raise _timeout()
        return self._take(count)

    def query(self, message: str) -> str:
        self.write(message)
        return self.read()

    def close(self) -> None:
        self.closed = True


class SimManager:
    """PyVISA resource manager enumerating `boards` (one default board if None)."""

    def __init__(self, boards=None):
        self.boards = [SimBoard()] if boards is None else list(boards)

    @staticmethod
    def name(board: SimBoard) -> str:
        """Resource name in the form pyvisa-py gives it."""
        return f"USB0::{VID}::{PID}::{board.serial}::0::INSTR"

    def list_resources(self, query: str = "?*::INSTR") -> tuple[str, ...]:
        del query
        online = [self.name(b) for b in self.boards if b.online]
        return ("ASRL1::INSTR", *online)

    def open_resource(self, name: str, **kwargs) -> SimResource:
        del kwargs
        for board in self.boards:
            if board.online and self.name(board) == name:
                return SimResource(board, name)
        raise errors.VisaIOError(StatusCode.error_resource_not_found)

    def close(self) -> None:
        """Nothing to release."""


@functools.cache
def default_manager() -> SimManager:
    """Process-wide simulated bus with one board, used by `--sim`."""
    return SimManager()

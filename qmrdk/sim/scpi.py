"""Simulated QM-RDK behind the PyVISA resource interface (docs/protocol.md):
the SCPI state machine of firmware V1.1.0 with its observed formats, errors
and capture paging, a resource manager that enumerates boards, and fault
injection for the host's recovery paths."""

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
from qmrdk.device import ARM_DELAY, F_REF, KINDS
from qmrdk.sim.devices import SimRadar, SimSled
from qmrdk.sim.hardware import Hardware
from qmrdk.sim.scene import builtin_scene, compile_scene
from qmrdk.transport import CHUNK, MAKER, MAX_SAMPLES, MODEL, NOT_READY, TIMEOUT_MS, VID

PID = 0x13
MID_SCALE = 32768
FAULTS = ("nonhex", "truncate", "not_ready", "stall", "disconnect", "unplug")
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
_INT = re.compile(r"[+-]?\d+")
_REAL = re.compile(r"[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?")
_BOOL = {"ON": True, "1": True, "OFF": False, "0": False}


def _timeout():
    return errors.VisaIOError(StatusCode.error_timeout)


def scene_source(scene: str = "single", hardware: Hardware | None = None, seed=0):
    """Frame source `(n, sweep) -> codes` synthesising a built-in scene with
    the board model. Types 0 and 1 run one ramp or triangle at the start of
    the frame and hold the frequency after it (protocol §3.1)."""
    hw = Hardware() if hardware is None else hardware
    rng = np.random.default_rng(seed)
    radars = {}

    def source(n: int, sweep: Sweep) -> np.ndarray:
        auto = dataclasses.replace(
            sweep, kind="cw" if sweep.kind == "cw" else "triangle"
        )
        if auto not in radars:
            geom = compile_scene(builtin_scene(scene), auto.lam)
            radars[auto] = SimRadar(geom, hw, auto, SimSled(), ScanGeometry(), rng)
        codes = radars[auto].capture(n)
        if sweep.kind in ("ramp", "tri"):
            ramps = 1 if sweep.kind == "ramp" else 2
            codes[int(np.ceil(hw.fs * (hw.t_start + ramps * sweep.ramp_time))) :] = (
                MID_SCALE
            )
        return codes

    return source


class SimBoard:
    """One simulated board. `source(n, sweep)` supplies the codes of each
    `CAPT:FRAM n`; `faults` (from FAULTS) apply to successive captures;
    `stuck_unlocked` keeps `FREQ:LOCK?` at 0; `*RST` and a `disconnect` fault
    re-enumerate after `reboot_s`; `log` holds every message received."""

    def __init__(
        self,
        serial: str = "0042",
        firmware: str = "V1.1.0",
        source=None,
        reboot_s: float = 0.0,
        temperature: float = 31.25,
        fs: float = Hardware.fs,
    ):
        self.serial, self.firmware = serial, firmware
        self.source = scene_source() if source is None else source
        self.reboot_s, self.temperature, self.fs = reboot_s, temperature, fs
        self.memory = {0: dict(DEFAULTS)}
        self.faults = collections.deque()
        self.stuck_unlocked = False
        self.powered, self.boot, self.back_at = True, 0, 0.0
        self.log = []
        self._power_up()

    def _power_up(self):
        self.state = dict(self.memory[0])
        self.rf = self.sweeping = True
        self.errs = [(-500, MESSAGES[-500])]
        self.esr, self.ese, self.sre = 128, 0, 0
        self.frame, self.cursor, self.fault, self.wait_s = "", 0, None, None

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

    @property
    def sweep(self) -> Sweep:
        s = self.state
        return Sweep(
            f0=s["SWEEP:FREQSTAR"] * 1e9,
            f1=s["SWEEP:FREQSTOP"] * 1e9,
            ramp_time=s["SWEEP:RAMPTIME"] * 1e-3,
            kind=KINDS[s["SWEEP:TYPE"]],
        )

    def locked(self) -> bool:
        """Protocol §3.3: the slope must reach f_ref^2 / (div * 2^25)."""
        s = self.state
        if not self.sweeping or self.stuck_unlocked:
            return False
        if s["SWEEP:TYPE"] == 3:
            return True
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
        """Parsed parameter, or None with -109 / -102 / -222 queued."""
        if not arg:
            return self._error(-109)
        if not (_INT if kind is int else _REAL).fullmatch(arg):
            return self._error(-102)
        value = kind(arg)
        if not lo <= value <= hi:
            return self._error(-222)
        return value

    def message(self, msg: str, timeout_s: float) -> str | None:
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

    def _execute(self, head, arg, timeout_s):
        if head.endswith("?"):
            return self._query(head, arg, timeout_s)
        if head in SETTINGS:
            value = self._number(arg, *SETTINGS[head][:3])
            if value is not None:
                self.state[head] = value
                self.sweeping &= head != "SWEEP:TYPE"
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
        if head[:-1] in SETTINGS:
            return SETTINGS[head[:-1]][3].format(self.state[head[:-1]])
        query = self._queries().get(head)
        return self._error(-113) if query is None else query(timeout_s)

    def _queries(self):
        stb = (4 if self.errs else 0) | (32 if self.esr & self.ese else 0)
        stb |= 64 if stb & self.sre else 0
        return {
            "*IDN?": lambda t: f"{MAKER},{MODEL},{self.serial},{self.firmware}",
            "SYST:IDEN?": lambda t: MODEL,
            "SYST:SERNUM?": lambda t: self.serial,
            "SYST:MODNUM?": lambda t: MODEL,
            "SYST:FIRM?": lambda t: self.firmware,
            "SYST:VERS?": lambda t: "1999.0",
            "SYST:TEMP?": lambda t: f"{self.temperature:.2f}",
            "SYST:STAT?": lambda t: '0,"Operational"',
            "SYST:ERR?": self._pop_error,
            "*ESR?": self._read_esr,
            "*ESE?": lambda t: str(self.ese),
            "*SRE?": lambda t: str(self.sre),
            "*STB?": lambda t: str(stb),
            "*OPC?": lambda t: "1",
            "*OPT?": lambda t: "0",
            "*TST?": lambda t: "0",
            "POWE:RF?": lambda t: str(int(self.rf)),
            "FREQ:LOCK?": lambda t: str(int(self.locked())),
            "CAPT:FRAM?": self._chunk,
            "CAPT:STRE?": lambda t: "0",
        }

    def _commands(self):
        return {
            "*CLS": self._clear,
            "*ESE": lambda a: self._register("ese", a),
            "*SRE": lambda a: self._register("sre", a),
            "*OPC": self._opc,
            "*WAI": lambda: None,
            "*TRG": self._trigger,
            "*RST": self.reboot,
            "SYST:PRES": self._preset,
            "*SAV": lambda a: self._memory(a, 0, True),
            "*RCL": lambda a: self._memory(a, 0, False),
            "SYST:CLRM": lambda a: self._memory(a, 1, None),
            "SYST:REST": self._restore,
            "SWEEP:START": lambda: self._run(True),
            "SWEEP:STOP": lambda: self._run(False),
            "POWE:RF": self._power,
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
        self.rf = self.sweeping = on

    def _power(self, arg):
        if arg.upper() not in _BOOL:
            return self._error(-224)
        self.rf = _BOOL[arg.upper()]
        return None

    def _arm(self, arg):
        n = self._number(arg, int, 1, MAX_SAMPLES)
        if n is None:
            return
        self.fault = self.faults.popleft() if self.faults else None
        if self.fault == "unplug":
            self.unplug()
            return
        if self.fault == "disconnect":
            self.reboot()
            return
        sweep = self.sweep
        live = self.rf and self.sweeping and (sweep.kind == "cw" or sweep.ramp_time)
        codes = self.source(n, sweep) if live else np.full(n, MID_SCALE)
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


class SimResource:
    """The subset of a PyVISA message-based resource the transport uses."""

    def __init__(self, board: SimBoard, name: str):
        self.board, self.resource_name, self.boot = board, name, board.boot
        self.timeout = TIMEOUT_MS
        self.read_termination = self.write_termination = "\n"
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
            self._out = out
        return len(message) + len(self.write_termination or "")

    def read(self) -> str:
        self._check()
        if self._out is None:
            raise _timeout()
        out, self._out = self._out, None
        return out

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

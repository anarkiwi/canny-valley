"""Fake QM-RDK pyvisa resource: the SCPI subset `qmrdk.radar.UsbRadar` uses,
with the firmware's observed formats and errors (docs/protocol.md)."""

import numpy as np

from qmrdk import radar

NAME = f"USB0::{radar.VID}::19::0042::INSTR"
IDN = "Quonset Microwave,QM4004,0042,V1.1.0"
NO_ERROR = '0, "No error"'


class FakeBoard:
    """`source(n)` supplies the codes of each `CAPT:FRAM n`. Faults: the
    first chunk of the next `bad` frames malformed, `lock` False never locks, `idn` reply.
    """

    def __init__(self, source=None, bad=0, lock=True, idn=IDN):
        self.source = source or (lambda n: np.arange(n, dtype=np.uint16))
        self.bad, self.lock, self.idn = bad, lock, idn
        self.state = {"TYPE": 2, "FREQSTAR": 2.4, "FREQSTOP": 2.5, "RAMPTIME": 16}
        self.div, self.sweeping, self.closed, self.corrupt = 1, True, False, False
        self.errs = ['-500,"Power on"']
        self.pending = np.empty(0, np.uint16)
        self.log = []
        self.timeout = None
        self.read_termination = self.write_termination = None

    def _error(self, code, text):
        self.errs = (self.errs + [f'{code},"{text}"'])[-10:]

    def _set(self, key, arg):
        if key == "RAMPTIME" and not arg.isdigit():
            return self._error(-102, "Syntax error")
        value = int(arg) if key in ("TYPE", "RAMPTIME") else float(arg)
        bounds = {"TYPE": (0, 3), "RAMPTIME": (1, 65536)}.get(key, (2.4, 2.5))
        if not bounds[0] <= value <= bounds[1]:
            return self._error(-222, "Data out of range")
        self.state[key] = value
        return None

    def _locked(self):
        s = self.state
        slope = (s["FREQSTOP"] - s["FREQSTAR"]) * 1e9 / (s["RAMPTIME"] * 1e-3)
        return self.lock and (
            s["TYPE"] == 3 or slope >= radar.F_REF**2 / (self.div * 2**25)
        )

    def write(self, cmd):
        self.log.append(cmd)
        head, _, arg = cmd.partition(" ")
        if head == "*CLS":
            self.errs = []
        elif head.startswith("SWEEP:") and head[6:] in self.state:
            self._set(head[6:], arg)
        elif head in ("SWEEP:START", "SWEEP:STOP"):
            self.sweeping = head == "SWEEP:START"
        elif head == "FREQ:REF:DIV" and 1 <= int(arg) <= 256:
            self.div = int(arg)
        elif head == "CAPT:FRAM":
            self.pending = np.asarray(self.source(int(arg)), np.uint16)
            self.corrupt, self.bad = self.bad > 0, max(self.bad - 1, 0)
        else:
            self._error(-113, "Undefined header")

    def query(self, cmd):
        self.log.append(cmd)
        s = self.state
        replies = {
            "*IDN?": lambda: self.idn,
            "SYST:ERR?": lambda: self.errs.pop(0) if self.errs else NO_ERROR,
            "FREQ:REF:DIV?": lambda: str(self.div),
            "FREQ:LOCK?": lambda: str(int(self.sweeping and self._locked())),
            "SWEEP:TYPE?": lambda: str(s["TYPE"]),
            "SWEEP:FREQSTAR?": lambda: f"{s['FREQSTAR']:.3f}",
            "SWEEP:FREQSTOP?": lambda: f"{s['FREQSTOP']:.3f}",
            "SWEEP:RAMPTIME?": lambda: f"{s['RAMPTIME']:.2f}",
            "CAPT:FRAM?": self._chunk,
        }
        if cmd not in replies:
            self._error(-113, "Undefined header")
            raise TimeoutError(cmd)
        return replies[cmd]() + "\n"

    def _chunk(self):
        if not self.pending.size:
            return "Not Ready"
        if self.corrupt:
            self.corrupt = False
            return "12G4"
        chunk, self.pending = self.pending[: radar.CHUNK], self.pending[radar.CHUNK :]
        return chunk.astype(">u2").tobytes().hex().upper()

    def close(self):
        self.closed = True


class FakeManager:
    """Resource manager listing `board` (none if None)."""

    def __init__(self, board=None):
        self.board = board

    def list_resources(self):
        return () if self.board is None else ("ASRL1::INSTR", NAME)

    def open_resource(self, name):
        assert name == NAME
        return self.board


def install(monkeypatch, board=None):
    """Route `UsbRadar` to `board`; returns it."""
    monkeypatch.setattr(radar, "_manager", lambda: FakeManager(board))
    return board

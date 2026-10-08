"""Sled controller driver against emulations of the legacy sketch and of
firmware/sled, the latter checked against the firmware's protocol transcript."""

import collections
import math
import pathlib
import re
import types

import numpy as np
import pytest
import serial
from serial.tools import list_ports

from qmrdk import sled as sled_mod
from qmrdk.sled import HardwareSled, SledError

DISPLACEMENT = np.float32(np.float32(49.5) * np.float32(math.pi)) / np.float32(200)


TRANSCRIPT = pathlib.Path(__file__).parents[1] / "firmware/sled/test/protocol.txt"


class FakePort:
    """Serial port whose pending input is the `lines` queue."""

    def __init__(self, lines=()):
        self.lines = collections.deque(lines)
        self.written, self.closed = [], False

    def readline(self):
        return f"{self.lines.popleft()}\r\n".encode() if self.lines else b""

    def reset_input_buffer(self):
        self.lines.clear()

    def close(self):
        self.closed = True


class FakeController(FakePort):
    """The sketch's serial behaviour: optional boot and homing lines, then per
    command line an echo, `error` above the travel, and `ready`."""

    def __init__(self, boot=True, homes=True, echo=True, travel=950):
        super().__init__(["boot", "homing"] + ["ready"] * homes if boot else [])
        self.echo, self.travel, self.steps = echo, travel, 0

    def write(self, data):
        self.written.append(data)
        for text in data.decode().splitlines():
            pos = abs(int(text)) if text.strip().lstrip("-").isdigit() else 0
            if self.echo:
                self.lines.append(f"moving to position {pos}")
            if pos > self.travel:
                self.lines.append("error")
            else:
                self.steps = int(np.float32(pos) / DISPLACEMENT) if pos else 0
            self.lines.append("ready")


def scale_round(v, num, den):
    n = v * num
    return (n + den // 2) // den if n >= 0 else -((-n + den // 2) // den)


class FakeFirmware(FakePort):
    """firmware/sled's line protocol with instantaneous motion; `switch=False`
    has no home switch, `limit=True` trips a limit switch on every move."""

    TRAVEL, MARGIN, LIMITS = 1221, 100, {"speed": 4000, "accel": 100000}

    def __init__(self, banner=True, homed=False, switch=True, limit=False):
        super().__init__([self.banner()] if banner else [])
        self.homed, self.switch, self.limit = homed, switch, limit
        self.pos, self.microstep, self.speed, self.accel = 0, 1, 500, 1000

    def banner(self):
        return f"id qmrdk-sled 1 777544 {getattr(self, 'microstep', 1)}"

    def status(self):
        fields = [int(self.homed), 0, 0, self.pos, self.pos, 0, 0]
        fields += [self.microstep, self.speed, self.accel, self.TRAVEL * self.microstep]
        keys = "homed moving homing pos target home end microstep speed accel max"
        return "status " + " ".join(f"{k}={v}" for k, v in zip(keys.split(), fields))

    def write(self, data):
        self.written.append(data)
        for line in data.decode().replace("\r", "").split("\n")[:-1]:
            if line:
                self.lines.append(self.reply(line))

    def reply(self, line):
        cmd, _, arg = line.partition(" ")
        if len(line) > 24 or (arg and not re.fullmatch(r"[+-]?\d{1,9}", arg)):
            return "err unknown"
        queries = {
            "id?": self.banner,
            "pos?": lambda: f"pos {self.pos}",
            "status?": self.status,
            "stop": lambda: f"ok {self.pos}",
            "home": self.home,
        }
        if not arg:
            return queries.get(cmd, lambda: "err unknown")()
        setters = {"move": self.move, "microstep": self.set_microstep}
        setters.update(dict.fromkeys(self.LIMITS, lambda v: self.set_limit(cmd, v)))
        return setters.get(cmd, lambda v: "err unknown")(int(arg))

    def set_limit(self, cmd, v):
        if not 1 <= v <= self.LIMITS[cmd]:
            return "err range"
        setattr(self, cmd, v)
        return f"{cmd} {v}"

    def set_microstep(self, v):
        if v not in (1, 2, 4, 8, 16):
            return "err range"
        self.pos, self.microstep = scale_round(self.pos, v, self.microstep), v
        return f"microstep {v}"

    def home(self):
        if not self.switch:
            self.pos = -(self.TRAVEL + self.MARGIN) * self.microstep
            return "err home timeout"
        self.homed, self.pos = True, 0
        return "ok 0"

    def move(self, v):
        if not self.homed:
            return "err not homed"
        if not 0 <= v <= self.TRAVEL * self.microstep:
            return "err range"
        if self.limit:
            self.homed = False
            return "err limit"
        self.pos = v
        return f"ok {v}"


@pytest.fixture(autouse=True)
def _short_timeouts(monkeypatch):
    for name in ("BOOT_TIMEOUT", "ECHO_TIMEOUT", "HOME_TIMEOUT", "MOVE_TIMEOUT"):
        monkeypatch.setattr(sled_mod, name, 0.02)


def make(ctrl=None, origin=0.05):
    ctrl = ctrl or FakeController()
    return ctrl, HardwareSled(origin=origin, settle=0, port=ctrl)


@pytest.mark.parametrize("boot", [True, False])
def test_startup(boot):
    ctrl, sled = make(FakeController(boot=boot))
    assert not ctrl.lines and not ctrl.written
    sled.close()
    assert ctrl.closed


def test_homing_timeout():
    with pytest.raises(SledError, match="homing"):
        make(FakeController(homes=False))


def test_reached_steps_match_controller():
    mm = np.arange(1, sled_mod.MAX_MM + 1)
    expect = np.trunc(mm.astype(np.float32) / DISPLACEMENT).astype(int)
    assert [sled_mod.reached_steps(int(m)) for m in mm] == expect.tolist()


@pytest.mark.parametrize("x", [0.0, 0.1234, 0.5, 0.899])
def test_move_to_records_reached_step(x):
    ctrl, sled = make()
    sled.move_to(x)
    mm = round(1e3 * (0.05 + x))
    assert ctrl.written == [f"{mm}\n".encode()]
    assert ctrl.steps == int(np.float32(mm) / DISPLACEMENT)
    assert sled.position() == pytest.approx(
        ctrl.steps * float(DISPLACEMENT) * 1e-3 - 0.05, abs=1e-9
    )
    assert abs(sled.position() - x) < float(DISPLACEMENT) * 1e-3 + 5e-4


def test_same_step_not_resent():
    ctrl, sled = make()
    sled.move_to(0.1)
    sled.move_to(0.1004)
    assert ctrl.written == [b"150\n"]
    sled.home()
    assert ctrl.written[-1] == b"50\n" and sled.position() == pytest.approx(
        ctrl.steps * float(DISPLACEMENT) * 1e-3 - 0.05
    )


@pytest.mark.parametrize("x", [-0.05, 0.901, 2.0])
def test_out_of_travel(x):
    ctrl, sled = make()
    with pytest.raises(SledError, match="outside the travel"):
        sled.move_to(x)
    assert not ctrl.written


def test_error_reply():
    _, sled = make(FakeController(travel=500), origin=0.0)
    with pytest.raises(SledError, match="error at 600 mm"):
        sled.move_to(0.6)


def test_no_echo():
    _, sled = make(FakeController(echo=False))
    with pytest.raises(SledError, match="did not accept"):
        sled.move_to(0.1)


def test_no_ready(monkeypatch):
    ctrl, sled = make()
    monkeypatch.setattr(
        ctrl, "write", lambda data: ctrl.lines.append("moving to position 150")
    )
    with pytest.raises(SledError, match="no ready at 150 mm"):
        sled.move_to(0.1)


def test_rehome():
    ctrl, sled = make()
    sled.move_to(0.3)
    sled.rehome()
    assert ctrl.written[-1] == b"0\n" and ctrl.steps == 0
    assert sled.position() == pytest.approx(-0.05)


def test_position_before_move():
    _, sled = make()
    with pytest.raises(SledError, match="unknown"):
        sled.position()


def test_find_port(monkeypatch):
    with pytest.raises(SledError, match="not found"):
        sled_mod.find_port()
    ports = [
        types.SimpleNamespace(vid=0x2341, pid=0x0043, device="/dev/ttyACM0"),
        types.SimpleNamespace(vid=0x2341, pid=0x8036, device="/dev/ttyACM1"),
    ]
    monkeypatch.setattr(list_ports, "comports", lambda: ports)
    assert sled_mod.find_port() == "/dev/ttyACM1"
    monkeypatch.setenv("QMRDK_SLED", "loop://")
    assert sled_mod.find_port() == "loop://"


def test_open_failure(monkeypatch):
    def fail(*_args, **_kwargs):
        raise serial.SerialException("no such port")

    monkeypatch.setattr(serial, "serial_for_url", fail)
    with pytest.raises(SledError, match="no such port"):
        HardwareSled("/dev/null-sled")
    with pytest.raises(SledError, match="not found"):
        HardwareSled()


def test_opens_url():
    sled = HardwareSled("loop://", settle=0)
    assert isinstance(sled.port, serial.SerialBase)
    sled.close()


def transcript():
    """(sent line, expected reply lines) pairs from the firmware transcript."""
    pairs = []
    for line in TRANSCRIPT.read_text().splitlines():
        if line.startswith("> "):
            pairs.append((line[2:], []))
        elif line.startswith("< "):
            pairs[-1][1].append(line[2:])
    return pairs


def test_fake_firmware_matches_transcript():
    ctrl = FakeFirmware(banner=False)
    pairs = transcript()
    assert len(pairs) > 20
    for sent, expect in pairs:
        ctrl.write(f"{sent}\r\n".encode())
        assert list(ctrl.lines) == expect, sent
        ctrl.lines.clear()
    ctrl.write(b"\n" + b"x" * 25 + b"\n")
    assert list(ctrl.lines) == ["err unknown"]


def firmware_sled(ctrl=None, origin=0.05):
    ctrl = ctrl or FakeFirmware()
    return ctrl, HardwareSled(origin=origin, settle=0, port=ctrl)


@pytest.mark.parametrize("homed", [False, True])
def test_firmware_attach(homed):
    ctrl, sled = firmware_sled(FakeFirmware(homed=homed))
    assert sled.firmware and ctrl.homed
    assert ctrl.written == [b"status?\n"] + [b"home\n"] * (not homed)
    assert sled.step == pytest.approx(777544e-9) and sled.max_steps == 1221
    assert sled.position() == pytest.approx(-0.05)


def test_firmware_microstep_scale():
    ctrl = FakeFirmware(banner=False, homed=True)
    ctrl.reply("move 100")
    ctrl.reply("microstep 4")
    ctrl.lines.append(ctrl.banner())
    _, sled = firmware_sled(ctrl)
    assert sled.step == pytest.approx(777544e-9 / 4) and sled.max_steps == 4884
    assert sled.position() == pytest.approx(400 * 777544e-9 / 4 - 0.05)


@pytest.mark.parametrize("x", [-0.0496, 0.0, 0.1234, 0.5, 0.899])
def test_firmware_move_exact_step(x):
    ctrl, sled = firmware_sled()
    sled.move_to(x)
    steps = round((0.05 + x) / 777544e-9)
    assert ctrl.written[-1] == f"move {steps}\n".encode() and ctrl.pos == steps
    assert abs(sled.position() - x) <= 777544e-9 / 2 + 1e-12
    sled.move_to(x)
    assert ctrl.written[-1] == f"move {steps}\n".encode()
    assert len(ctrl.written) == 3


@pytest.mark.parametrize("x", [-0.0505, 0.9, 2.0])
def test_firmware_out_of_travel(x):
    ctrl, sled = firmware_sled()
    with pytest.raises(SledError, match="outside the travel"):
        sled.move_to(x)
    assert len(ctrl.written) == 2


def test_firmware_rehome_and_home():
    ctrl, sled = firmware_sled()
    sled.move_to(0.3)
    sled.rehome()
    assert ctrl.written[-1] == b"home\n" and sled.position() == pytest.approx(-0.05)
    sled.move_to(0.3)
    sled.home()
    assert ctrl.written[-1] == f"move {round(0.05 / 777544e-9)}\n".encode()


def test_firmware_errors():
    with pytest.raises(SledError, match="err home timeout to 'home'"):
        firmware_sled(FakeFirmware(switch=False))
    ctrl, sled = firmware_sled(FakeFirmware(limit=True))
    with pytest.raises(SledError, match="err limit to 'move"):
        sled.move_to(0.2)
    with pytest.raises(SledError, match="unknown"):
        sled.position()
    assert not ctrl.homed
    with pytest.raises(SledError, match="err not homed"):
        sled.move_to(0.1)


def test_firmware_no_reply(monkeypatch):
    ctrl = FakeFirmware()
    monkeypatch.setattr(ctrl, "write", ctrl.written.append)
    with pytest.raises(SledError, match="no reply to 'status\\?'"):
        firmware_sled(ctrl)

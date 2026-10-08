"""Sled controller driver against an emulation of radar_scan_platform.ino."""

import collections
import math
import types

import numpy as np
import pytest
import serial
from serial.tools import list_ports

from qmrdk import sled as sled_mod
from qmrdk.sled import HardwareSled, SledError

DISPLACEMENT = np.float32(np.float32(49.5) * np.float32(math.pi)) / np.float32(200)


class FakeController:
    """The sketch's serial behaviour: optional boot and homing lines, then per
    command line an echo, `error` above the travel, and `ready`."""

    def __init__(self, boot=True, homes=True, echo=True, travel=950):
        self.lines = collections.deque()
        if boot:
            self.lines.extend(["boot", "homing"] + ["ready"] * homes)
        self.echo, self.travel = echo, travel
        self.steps, self.written, self.closed = 0, [], False

    def readline(self):
        return f"{self.lines.popleft()}\r\n".encode() if self.lines else b""

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

    def reset_input_buffer(self):
        self.lines.clear()

    def close(self):
        self.closed = True


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

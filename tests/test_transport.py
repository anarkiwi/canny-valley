"""USBTMC transport against the simulated bus (qmrdk.sim.scpi)."""

import numpy as np
import pytest

from qmrdk import transport
from qmrdk.sim.scpi import SimBoard, SimManager
from qmrdk.transport import (
    DeviceError,
    DeviceTimeout,
    Disconnected,
    ForbiddenCommand,
    Idn,
    ScpiError,
    Transport,
)


class OtherBoard(SimBoard):
    """Same vendor ID, different instrument."""

    def _queries(self):
        return super()._queries() | {"*IDN?": lambda t: "Other,X,1,1"}


@pytest.fixture(name="fast_poll")
def fixture_fast_poll(monkeypatch):
    monkeypatch.setattr(transport, "POLL_S", 0.01)


@pytest.mark.parametrize(
    "name, serial",
    [
        ("USB0::8210::19::0021::0::INSTR", "0021"),
        ("USB0::0x2012::0x0017::0007::INSTR", "0007"),
        ("USB0::0x2013::0x0017::0007::INSTR", None),
        ("ASRL1::INSTR", None),
        ("not a resource", None),
    ],
)
def test_usb_serial(name, serial):
    assert transport.usb_serial(name) == serial


def test_resources_by_serial():
    manager = SimManager([SimBoard("0042"), SimBoard("0007")])
    assert len(transport.resources(manager)) == 2
    assert transport.resources(manager, "7") == [SimManager.name(manager.boards[1])]
    assert not transport.resources(manager, "8")
    assert not transport.resources()


@pytest.mark.parametrize(
    "msg",
    [
        "FACT:BIASGATE 1",
        "factory:biasgate?",
        ":SYST:POWE 1",
        "SYSTEM:POWER?",
        "SYST:ATTN 3",
        "SYST:TIME?",
        "SYST:CURR?",
        "SYST:MODULESTATUS?",
        "SYST:FIRMID?",
        "SYST:MODU?",
        "SYST:VENDOR?",
        "SYST:ALTERA?",
        "SYST:SENDSTR x",
        "SYST:READSTR?",
        "LCD:TEXT hi",
        "CAPT:STRE 1",
        "CAPTURE:STREAM?",
        "*CLS; SWEEP:STOP;FACT:X 1",
    ],
)
def test_forbidden(msg):
    assert transport.forbidden(msg)


@pytest.mark.parametrize(
    "msg", ["SYST:FIRM?", "SYST:MODNUM?", "SYST:TEMP?", "CAPT:FRAM?", "*IDN?", ""]
)
def test_allowed(msg):
    assert transport.forbidden(msg) is None


def test_idn_and_entries():
    idn = Idn.parse("Quonset Microwave,QM4004,0042,V1.1.0\n")
    assert (idn.serial, idn.firmware) == ("0042", "V1.1.0")
    for bad in ("Quonset Microwave,QM4004,0042", "Other,QM4004,1,2"):
        with pytest.raises(DeviceError, match="not a QM-RDK"):
            Idn.parse(bad)
    assert transport.parse_entry('-222, "Data out of range"') == (
        -222,
        "Data out of range",
    )
    with pytest.raises(DeviceError, match="malformed"):
        transport.parse_entry("Not Ready")


def test_connect_selects_board():
    a, b = SimBoard("0042"), SimBoard("0007")
    manager = SimManager([OtherBoard("0001"), a, b])
    t = Transport(manager=manager)
    assert t.idn.serial == "0042" and t.res.timeout == transport.TIMEOUT_MS
    assert t.res.read_termination == t.res.write_termination == "\n"
    assert manager.boards[0].log == ["*IDN?"]
    assert Transport(serial="7", manager=manager).idn.serial == "0007"
    with pytest.raises(DeviceError, match="no QM-RDK found with serial 9"):
        Transport(serial="9", manager=manager)
    with pytest.raises(DeviceError, match="serial 0042, not 7"):
        Transport(SimManager.name(a), serial="7", manager=manager)
    with pytest.raises(DeviceError, match="not a QM-RDK"):
        Transport(SimManager.name(manager.boards[0]), manager=manager)
    with pytest.raises(DeviceError, match="USB0::1::2::3::INSTR"):
        Transport("USB0::1::2::3::INSTR", manager=manager)


def test_default_manager_is_patched(attach):
    with pytest.raises(DeviceError, match="no QM-RDK found"):
        Transport()
    board = attach(SimBoard())
    assert Transport().idn.serial == board.serial


def test_error_policy():
    board = SimBoard()
    t = Transport(manager=SimManager([board]))
    t.command("SWEEP:RAMPTIME 8")
    assert board.state["SWEEP:RAMPTIME"] == 8 and not board.errs
    with pytest.raises(ScpiError) as err:
        t.command("SWEEP:RAMPTIME 1.5")
    assert (err.value.code, err.value.text) == (-102, "Syntax error")
    assert err.value.cmd == "SWEEP:RAMPTIME 1.5" and "-102" in str(err.value)
    t.write("SWEEP:FREQSTAR 3;SWEEP:TYPE AUTO")
    with pytest.raises(ScpiError) as err:
        t.check()
    assert err.value.queue == [(-222, "Data out of range"), (-102, "Syntax error")]
    board.errs = [(-500, "Power on")]
    assert not t.errors()
    board.errs = [(-500, "Power on")]
    assert t.errors(power_on=True) == [(-500, "Power on")]
    for _ in range(12):
        t.write("BOGUS")
    queue = t.errors()
    assert len(queue) == 10 and queue[-1] == (-350, "Queue overflow")
    board.message = lambda msg, timeout: '-113,"Undefined header"'
    with pytest.raises(DeviceError, match="does not drain"):
        t.errors()


def test_timeouts():
    board = SimBoard()
    t = Transport(manager=SimManager([board]))
    with pytest.raises(DeviceTimeout):
        t.query("BOGUS?")
    assert t.errors() == [(-113, "Undefined header")]
    t.write("CAPT:FRAM 4096")
    with pytest.raises(DeviceTimeout):
        t.query("CAPT:FRAM?", timeout=100)
    assert t.res.timeout == transport.TIMEOUT_MS
    assert len(t.query("CAPT:FRAM?", timeout=500)) == 124


def test_forbidden_never_sent():
    board = SimBoard()
    t = Transport(manager=SimManager([board]))
    sent = len(board.log)
    for call in (t.write, t.query, t.command):
        with pytest.raises(ForbiddenCommand):
            call("SYST:POWE?")
    assert len(board.log) == sent


def test_flush():
    board = SimBoard(source=lambda n, sweep: np.zeros(n, np.uint16))
    t = Transport(manager=SimManager([board]))
    t.write("CAPT:FRAM 4096")
    t.flush()
    assert board.log[-1] == "CAPT:FRAM?" and t.query("CAPT:FRAM?") == "Not Ready"
    board.inject("stall")
    t.write("CAPT:FRAM 100")
    t.flush()
    assert board.log[-7:] == ["CAPT:FRAM 100"] + ["CAPT:FRAM?"] * 6
    board._chunk = lambda timeout: "0000"  # pylint: disable=protected-access
    with pytest.raises(DeviceError, match="does not drain"):
        t.flush()


@pytest.mark.usefixtures("fast_poll")
def test_disconnect_and_reopen():
    board = SimBoard(reboot_s=0.05)
    t = Transport(manager=SimManager([board]))
    t.write("*RST")
    t.wait_gone()
    with pytest.raises(Disconnected):
        t.query("*IDN?")
    t.reopen()
    assert t.idn.serial == "0042" and t.errors(power_on=True) == [(-500, "Power on")]
    assert board.boot == 1
    t.wait_gone(0.05)
    t.close()
    t.close()
    with pytest.raises(Disconnected, match="not connected"):
        t.write("*CLS")
    board.unplug()
    with pytest.raises(DeviceError, match="did not re-enumerate"):
        t.reopen(0.05)


def test_closed_session_is_disconnected():
    t = Transport(manager=SimManager([SimBoard()]))
    t.res.close()
    with pytest.raises(Disconnected, match="Invalid session"):
        t.query("*IDN?")

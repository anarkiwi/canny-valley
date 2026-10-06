"""Simulated board: SCPI state machine and resource interface against
docs/protocol.md."""

# pylint: disable=protected-access

import numpy as np
import pytest
from pyvisa import errors
from pyvisa.constants import StatusCode

from qmrdk.config import Sweep
from qmrdk.constants import FS_NOMINAL
from qmrdk.dsp.segment import mirror_correlation
from qmrdk.sim import scpi
from qmrdk.sim.scpi import SimBoard, SimManager


@pytest.fixture(name="board")
def fixture_board():
    return SimBoard(source=lambda n, sweep: np.arange(n, dtype=np.uint16))


@pytest.fixture(name="res")
def fixture_res(board):
    manager = SimManager([board])
    return manager.open_resource(manager.list_resources()[1])


def errs(res):
    out = []
    while (e := res.query("SYST:ERR?")) != '0,"No error"':
        out.append(int(e.split(",")[0]))
    return out


def test_power_up(res):
    assert res.query("*IDN?") == "Quonset Microwave,QM4004,0042,V1.1.0"
    replies = {
        "SWEEP:FREQSTAR?": "2.400",
        "SWEEP:FREQSTOP?": "2.500",
        "SWEEP:RAMPTIME?": "16.00",
        "SWEEP:TYPE?": "2",
        "FREQ:REF:DIV?": "1",
        "POWE:RF?": "1",
        "FREQ:LOCK?": "1",
        "SYST:TEMP?": "31.25",
        "SYST:IDEN?": "QM4004",
        "SYST:SERNUM?": "0042",
        "SYST:MODNUM?": "QM4004",
        "SYST:FIRM?": "V1.1.0",
        "SYST:VERS?": "1999.0",
        "SYST:STAT?": '0,"Operational"',
        "*OPT?": "0",
        "*TST?": "0",
        "*OPC?": "1",
        "CAPT:STRE?": "0",
        "CAPT:FRAM?": "Not Ready",
        "*ESR?": "128",
    }
    assert {q: res.query(q) for q in replies} == replies
    assert res.query("SYST:ERR?") == '-500,"Power on"'
    assert res.query("SYST:ERR?") == '0,"No error"'


@pytest.mark.parametrize(
    "cmd, code, key, value",
    [
        ("SWEEP:FREQSTAR 2.45", None, "SWEEP:FREQSTAR", 2.45),
        ("sweep:freqstop 2.4e0", None, "SWEEP:FREQSTOP", 2.4),
        ("SWEEP:FREQSTAR 2.6", -222, "SWEEP:FREQSTAR", 2.4),
        ("SWEEP:FREQSTAR two", -102, "SWEEP:FREQSTAR", 2.4),
        ("SWEEP:RAMPTIME 0", None, "SWEEP:RAMPTIME", 0),
        ("SWEEP:RAMPTIME 8.0", -102, "SWEEP:RAMPTIME", 16),
        ("SWEEP:RAMPTIME 65537", -222, "SWEEP:RAMPTIME", 16),
        ("SWEEP:TYPE AUTO", -102, "SWEEP:TYPE", 2),
        ("SWEEP:TYPE 4", -222, "SWEEP:TYPE", 2),
        ("FREQ:REF:DIV 257", -222, "FREQ:REF:DIV", 1),
        ("FREQ:REF:DIV\t3", None, "FREQ:REF:DIV", 3),
        ("SWEEP:RAMPTIME", -109, "SWEEP:RAMPTIME", 16),
    ],
)
def test_settings(res, board, cmd, code, key, value):
    errs(res)
    res.write(cmd)
    assert errs(res) == ([] if code is None else [code])
    assert board.state[key] == value


@pytest.mark.parametrize(
    "cmd, code",
    [
        ("BOGUS:CMD 1", -113),
        ("SYST:BLUE?", -113),
        ("FACT:BIASGATE 1", -113),
        ("SWEEP:FREQUENCYSTART 2.4", -112),
        ("SWEEP:START 1", -108),
        ("SWEEP:TYPE? 1", -108),
        ("*ESE", -109),
        ("POWE:RF maybe", -224),
        ("CAPT:FRAM 4097", -222),
        ("*TRG", -211),
        ("SYST:CLRM 0", -222),
        ("*RCL 5", -222),
        ("; :BOGUS", -113),
    ],
)
def test_errors(res, cmd, code):
    errs(res)
    res.write(cmd)
    assert errs(res) == [code]


def test_error_queue_and_status_registers(res):
    assert res.query("*STB?") == "4"
    res.write("*CLS")
    assert res.query("*STB?") == "0" and res.query("*ESR?") == "0"
    res.write("*ESE 48;*SRE 32")
    assert (res.query("*ESE?"), res.query("*SRE?")) == ("48", "32")
    res.write("BOGUS")
    assert res.query("*STB?") == str(4 | 32 | 64)
    assert res.query("*ESR?") == "32" and res.query("*ESR?") == "0"
    res.write("SWEEP:TYPE 9")
    res.write("*OPC")
    assert res.query("*ESR?") == str(16 | 1)
    res.write("*CLS")
    for _ in range(12):
        res.write("BOGUS")
    assert errs(res) == [-113] * 9 + [-350]


def test_sweep_and_lock(res, board):
    res.write("SWEEP:TYPE 1")
    assert (res.query("FREQ:LOCK?"), res.query("POWE:RF?")) == ("0", "1")
    res.write("*TRG")
    assert res.query("FREQ:LOCK?") == "1"
    res.write("SWEEP:STOP")
    assert (res.query("FREQ:LOCK?"), res.query("POWE:RF?")) == ("0", "0")
    res.write("SWEEP:TYPE 2;SWEEP:RAMPTIME 10000;SWEEP:START")
    assert res.query("FREQ:LOCK?") == "0"
    res.write("FREQ:REF:DIV 2")
    assert res.query("FREQ:LOCK?") == "1"
    res.write("SWEEP:TYPE 3;SWEEP:START;POWE:RF OFF")
    assert res.query("FREQ:LOCK?") == "1" and not board.rf
    res.write("POWE:RF ON")
    assert board.rf
    board.stuck_unlocked = True
    assert res.query("FREQ:LOCK?") == "0"
    res.write("SWEEP:RAMPTIME 0;SWEEP:TYPE 2;SWEEP:START")
    board.stuck_unlocked = False
    assert res.query("FREQ:LOCK?") == "0"


def test_paging(res, board):
    res.write("CAPT:FRAM 70")
    chunks = [res.query("CAPT:FRAM?") for _ in range(4)]
    assert [len(c) for c in chunks[:3]] == [124, 124, 32]
    assert chunks[3] == "Not Ready" and chunks[0][:8] == "00000001"
    assert np.array_equal(
        np.frombuffer(bytes.fromhex("".join(chunks[:3])), ">u2"), np.arange(70)
    )
    res.write("SWEEP:STOP;CAPT:FRAM 31")
    assert res.query("CAPT:FRAM?") == "8000" * 31
    res.timeout = 100
    res.write("CAPT:FRAM 4096")
    with pytest.raises(errors.VisaIOError):
        res.read()
    with pytest.raises(errors.VisaIOError) as err:
        res.query("CAPT:FRAM?")
    assert err.value.error_code == StatusCode.error_timeout
    assert len(res.query("CAPT:FRAM?")) == 124
    assert board.log[-3:] == ["CAPT:FRAM 4096", "CAPT:FRAM?", "CAPT:FRAM?"]


def test_last_response_only(res):
    assert res.query("SWEEP:TYPE?;*IDN?;SWEEP:RAMPTIME?") == "16.00"
    res.write("SWEEP:TYPE?")
    assert res.query("FREQ:REF:DIV?") == "1"


def test_memory(res, board):
    res.write("SWEEP:RAMPTIME 20;*SAV 1;SWEEP:RAMPTIME 30;*RCL 1")
    assert board.state["SWEEP:RAMPTIME"] == 20
    res.write("SYST:CLRM 1;*RCL 1")
    assert errs(res)[-1] == -222
    res.write("SWEEP:RAMPTIME 40;*SAV 0;SWEEP:RAMPTIME 50;SYST:PRES")
    assert board.state["SWEEP:RAMPTIME"] == 40
    res.write("SYST:REST;SYST:PRES")
    assert board.state["SWEEP:RAMPTIME"] == 16


def test_reboot_and_unplug(board):
    board.reboot_s = 0.05
    manager = SimManager([board])
    name = manager.list_resources()[1]
    res = manager.open_resource(name)
    res.write("SWEEP:STOP;*CLS;*RST")
    with pytest.raises(errors.VisaIOError) as err:
        res.query("*IDN?")
    assert err.value.error_code == StatusCode.error_connection_lost
    assert manager.list_resources() == ("ASRL1::INSTR",)
    with pytest.raises(errors.VisaIOError):
        manager.open_resource(name)
    while not board.online:
        pass
    res = manager.open_resource(name)
    assert board.rf and res.query("SYST:ERR?") == '-500,"Power on"'
    res.close()
    with pytest.raises(errors.InvalidSession):
        res.write("*CLS")
    board.unplug()
    assert not board.online and not board.rf
    manager.close()


def test_faults(board):
    with pytest.raises(ValueError, match="melt"):
        board.inject("melt")
    res = SimManager([board]).open_resource(SimManager.name(board))
    board.inject("nonhex", "truncate", "not_ready")
    res.write("CAPT:FRAM 40")
    assert res.query("CAPT:FRAM?")[0] == "G"
    res.write("CAPT:FRAM 40")
    assert len(res.query("CAPT:FRAM?")) == 120 and len(res.query("CAPT:FRAM?")) == 36
    res.write("CAPT:FRAM 40")
    assert res.query("CAPT:FRAM?") == "Not Ready"


@pytest.mark.parametrize("kind, mirrored", [("tri", True), ("ramp", False)])
def test_scene_source_single_sweep(kind, mirrored):
    sweep = Sweep(ramp_time=80e-3, kind=kind)
    codes = scpi.scene_source()(4096, sweep)
    hw = scpi.Hardware()
    end = int(np.ceil(hw.fs * (hw.t_start + (2 if mirrored else 1) * 80e-3)))
    assert np.all(codes[end:] == scpi.MID_SCALE) and np.ptp(codes[:end]) > 100
    nr = int(sweep.ramp_time * FS_NOMINAL)
    j = mirror_correlation(
        codes.astype(float), np.arange(nr, nr + nr // 4), np.arange(16, nr // 2)
    )
    assert (j.max() > 0.5) == mirrored


def test_mirror_correlation():
    x = np.r_[np.arange(5.0), np.arange(5.0)[::-1][1:], 0.0, 0.0]
    j = mirror_correlation(x, [2, 4, 6], np.arange(1, 3))
    np.testing.assert_allclose(j, [-1.0, 1.0, -1.0])
    assert mirror_correlation(np.ones(9), [4], np.arange(1, 3))[0] == 0.0


def test_default_manager():
    manager = scpi.default_manager()
    assert manager is scpi.default_manager() and len(manager.boards) == 1

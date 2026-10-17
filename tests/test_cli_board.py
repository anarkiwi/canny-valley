"""`qmrdk` board commands against the simulated board."""

import json

import numpy as np
import pytest

from qmrdk import cli
from qmrdk.config import Sweep
from qmrdk.recording import Recording
from qmrdk.sim import scpi
from qmrdk.sim.scpi import SimBoard


def run(capsys, *argv):
    code = cli.main(list(argv))
    out = capsys.readouterr()
    return code, out.out, out.err


def sim_board():
    return scpi.default_manager().boards[0]


def test_list(capsys, attach):
    assert run(capsys, "list")[0] == 1
    attach(SimBoard("0007"), SimBoard("0042"))
    code, out, _ = run(capsys, "list")
    assert code == 0 and out.splitlines() == [
        "USB0::8210::19::0007::0::INSTR\t0007\tV1.1.0",
        "USB0::8210::19::0042::0::INSTR\t0042\tV1.1.0",
    ]
    code, out, _ = run(capsys, "list", "--sim", "--serial", "42")
    assert code == 0 and out.split("\t")[1] == "0042" and not sim_board().rf
    code, _, err = run(capsys, "list", "--resource", "USB0::1::2::3::INSTR")
    assert code == 0 and "no QM-RDK found" in err


def test_info(capsys):
    code, out, _ = run(capsys, "info", "--sim")
    info = json.loads(out)
    assert code == 0 and info["idn"]["serial"] == "0042"
    assert info["sweep"]["kind"] == "triangle" and info["rf"] and info["locked"]
    assert info["errors"] == [[-500, "Power on"]] and info["temperature"] == 31.25
    assert info["status"] == {"code": 0, "text": "Operational"}
    assert not sim_board().rf


def test_set_and_rf(capsys):
    argv = ["--sim", "--f0", "2.41", "--ramp-time", "8", "--type", "cw"]
    code, out, _ = run(capsys, "set", *argv)
    assert code == 0 and json.loads(out)["sweep"]["kind"] == "cw"
    board = sim_board()
    assert board.rf and board.sweep == Sweep(f0=2.41e9, ramp_time=8e-3, kind="cw")
    assert run(capsys, "rf", "off", "--sim")[0] == 0 and not board.rf
    code, out, _ = run(capsys, "rf", "on", "--sim")
    assert code == 0 and board.rf and json.loads(out)["locked"]
    code, _, err = run(capsys, "set", "--sim", "--ramp-time", "8.5")
    assert code == 2 and "not an integer" in err and not board.rf


def test_scpi(capsys):
    assert run(capsys, "scpi", "--sim", "SYST:TEMP?")[1] == "31.25\n"
    assert run(capsys, "scpi", "--sim", "SWEEP:START")[1] == ""
    assert not sim_board().rf
    code, _, err = run(capsys, "scpi", "--sim", "BOGUS:CMD 1")
    assert code == 2 and "undocumented" in err
    code, _, err = run(capsys, "scpi", "--sim", "*SAV 0")
    assert code == 2 and "power-up state" in err
    assert run(capsys, "scpi", "--sim", "--force", "*SAV 0")[0] == 0
    assert run(capsys, "scpi", "--sim", "SWEEP:TYPE 7")[0] == 2


@pytest.mark.parametrize("kind", ["triangle", "tri"])
def test_capture_round_trip(capsys, attach, tmp_path, kind):
    rng = np.random.default_rng(3)
    served = []

    def source(n, sweep):
        assert sweep.kind == kind
        served.append(rng.integers(0, 65536, n, dtype=np.uint16))
        return served[-1]

    board = attach(SimBoard(source=source))
    board.inject("truncate")
    out = tmp_path / "rec.npz"
    argv = ["capture", "--frames", "3", "--n", "100", "--out", str(out)]
    code, stdout, _ = run(capsys, *argv, "--ramp-time", "10", "--type", kind)
    assert code == 0 and "3 frames of 100 samples" in stdout
    rec = Recording.load(out)
    np.testing.assert_array_equal(rec.codes, np.stack(served[1:]))
    assert rec.codes.dtype == np.uint16 and np.all(np.diff(rec.t_host) >= 0)
    assert rec.sweep == Sweep(ramp_time=10e-3, kind=kind) and rec.x_pos is None
    assert rec.extra["idn"]["serial"] == "0042" and rec.extra["ref_div"] == 1
    assert rec.extra["fs"] == 21_977.0 and rec.extra["resource"].startswith("USB0")
    assert not board.rf


def test_capture_temperature(capsys, tmp_path):
    out = tmp_path / "rec.npz"
    argv = ["capture", "--sim", "--frames", "2", "--n", "64", "--out", str(out)]
    assert run(capsys, *argv, "--temperature", "--interval", "0.05")[0] == 0
    rec = Recording.load(out)
    np.testing.assert_array_equal(rec.temperature, [31.25, 31.25])
    assert rec.t_host[1] - rec.t_host[0] >= 0.05
    assert run(capsys, *argv)[0] == 0 and Recording.load(out).temperature is None


CV = "CV1.0.0"
CV_TIMING = {"fs": 25_000.0, "n0": 0.018125, "first_up": True}


def cv_source(served):
    rng = np.random.default_rng(4)

    def source(n, sweep, timing):
        served.append((sweep, timing, rng.integers(0, 65536, n, dtype=np.uint16)))
        return served[-1][2]

    return source


@pytest.mark.timeout(60)
def test_capture_cv_records_timing(capsys, attach, tmp_path):
    served = []
    board = attach(SimBoard(firmware=CV, source=cv_source(served)))
    board.inject("overflow")
    out = tmp_path / "rec.npz"
    argv = ["capture", "--frames", "2", "--n", "6000", "--out", str(out)]
    argv += ["--type", "saw", "--ramp-time", "7.3", "--dwell", "2.51", "--f0", "2.3"]
    code, stdout, _ = run(capsys, *argv)
    assert code == 0 and "2 frames of 6000 samples" in stdout
    rec = Recording.load(out)
    np.testing.assert_array_equal(rec.codes, np.stack([s[2] for s in served[1:]]))
    assert rec.sweep.kind == "saw" and rec.sweep.dwell == 2.52e-3
    assert rec.sweep.ramp_time == 7.32e-3 and rec.sweep.f0 == 2.3e9
    assert rec.extra["fs"] == 25_000.0 and rec.extra["timing"] == CV_TIMING
    assert rec.extra["ref_div"] == 1 and not board.rf


def test_saw_needs_cv(capsys, attach):
    board = attach(SimBoard())
    code, _, err = run(capsys, "set", "--type", "saw")
    assert code == 2 and "need CV firmware" in err and not board.rf
    code, _, err = run(capsys, "set", "--dwell", "1")
    assert code == 2 and "sawtooth sweeps only" in err


def test_info_and_limits_cv(capsys, attach):
    attach(SimBoard(firmware=CV))
    code, out, _ = run(capsys, "info")
    info = json.loads(out)
    assert code == 0 and info["timing"] == CV_TIMING and not info["rf"]
    assert info["limits"]["f_min"] == 2.25e9
    code, _, err = run(capsys, "set", "--f0", "2.2")
    assert code == 2 and "outside 2.25..2.5 GHz" in err
    code, out, _ = run(capsys, "set", "--f0", "2.25", "--ramp-time", "0.2")
    assert code == 0 and json.loads(out)["sweep"]["ramp_time"] == 0.2e-3

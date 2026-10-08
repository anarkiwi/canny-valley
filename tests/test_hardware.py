"""Device layer on the board. Every test runs against the simulated board and,
with `-m hardware` and a board attached, against the board itself; results
of the board runs are written to artifacts/hardware/."""

import json
import pathlib

import numpy as np
import pytest

from qmrdk import transport
from qmrdk.config import Sweep
from qmrdk.constants import FS_NOMINAL
from qmrdk.device import Device
from qmrdk.dsp.convert import codes_to_volts
from qmrdk.dsp.segment import mirror_correlation
from qmrdk.sim.scpi import SimBoard, SimManager

ARTIFACTS = pathlib.Path(__file__).resolve().parents[1] / "artifacts" / "hardware"


@pytest.fixture(
    name="manager", params=["sim", pytest.param("board", marks=pytest.mark.hardware)]
)
def fixture_manager(request):
    if request.param == "sim":
        return SimManager([SimBoard()])
    import pyvisa  # pylint: disable=import-outside-toplevel

    manager = pyvisa.ResourceManager("@py")
    if not transport.resources(manager):
        pytest.skip("no QM-RDK attached")
    return manager


@pytest.fixture(name="dev")
def fixture_dev(manager):
    with Device(manager=manager) as dev:
        yield dev


def record(dev, name, result):
    """Keep a board result; simulated runs are not recorded."""
    if isinstance(dev.transport.manager, SimManager):
        return
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    result = {"idn": dev.idn.__dict__, **result}
    (ARTIFACTS / f"{name}.json").write_text(json.dumps(result, indent=1))


def test_identify_and_read_back(dev):
    assert dev.identify() == dev.idn and dev.idn.model == "QM4004"
    sweep = Sweep(f0=2.41e9, f1=2.49e9, ramp_time=20e-3)
    assert dev.configure(sweep) == sweep
    settings = dev.settings()
    assert settings.locked and settings.rf and settings.ref_div == 1
    assert not dev.errors()


@pytest.mark.parametrize("n", [1, 31, 32, 4096])
def test_frame_lengths(dev, n):
    dev.configure(Sweep())
    codes, t_host, _ = dev.capture_many(n, 2)
    assert codes.shape == (2, n) and t_host[1] > t_host[0]
    assert not dev.errors()


def test_frames_synchronised(dev):
    dev.configure(Sweep())
    codes, _, _ = dev.capture_many(4096, 2)
    rho = np.corrcoef(codes_to_volts(codes))[0, 1]
    record(dev, "synchronised", {"correlation": float(rho)})
    assert rho > 0.99


def test_type1_sweeps_down(dev):
    """Open question 5: a type 1 (TRI) frame mirrors about the end of its
    up-ramp as an AUTO frame's first triangle does, if TRI sweeps back down."""
    ramp = 80e-3
    nr = int(ramp * FS_NOMINAL)
    centres, offsets = np.arange(nr, nr + nr // 4), np.arange(16, nr // 2)
    score = {}
    for kind in ("tri", "triangle", "ramp"):
        dev.configure(Sweep(ramp_time=ramp, kind=kind))
        x = codes_to_volts(dev.capture(4096))
        score[kind] = float(mirror_correlation(x, centres, offsets).max())
    record(dev, "question5", {"ramp_time": ramp, "mirror_correlation": score})
    assert score["ramp"] < 0.5 * score["triangle"]
    assert score["tri"] > 0.5 * score["triangle"], score


def test_rf_off_after_close(manager):
    with Device(manager=manager) as dev:
        dev.configure(Sweep())
        assert dev.settings().rf
    with Device(manager=manager) as dev:
        assert not dev.settings().rf


def test_reset_reopens(dev):
    serial = dev.idn.serial
    dev.configure(Sweep(ramp_time=20e-3))
    dev.reset()
    assert dev.idn.serial == serial and (-500, "Power on") in dev.boot_errors
    settings = dev.settings()
    assert not settings.rf
    dev.configure(Sweep())
    assert dev.capture(64).shape == (64,)

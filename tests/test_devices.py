"""Simulated sled and radar (docs/simulation.md §5)."""

import numpy as np
import pytest

from qmrdk.config import ScanGeometry, Sweep
from qmrdk.sim import propagation
from qmrdk.sim.devices import SimRadar, SimSled
from qmrdk.sim.hardware import Hardware, synthesize


def test_sled_statistics():
    sled = SimSled(sigma=0.01, bias=0.003, seed=1)
    n = 20000
    true = np.empty(n)
    for i in range(n):
        sled.move_to(1.0)
        true[i] = sled.true_position
    assert sled.position() == 1.0
    assert np.mean(true) == pytest.approx(1.003, abs=5 * 0.01 / np.sqrt(n))
    assert np.std(true) == pytest.approx(0.01, rel=5 / np.sqrt(2 * n))
    sled.home()
    assert sled.position() == 0.0


def test_sled_exact():
    sled = SimSled(bias=-0.25)
    sled.move_to(0.25)
    assert (sled.position(), sled.true_position) == (0.25, 0.0)


class Recorder:
    """Stand-in for propagation.paths that records its calls."""

    def __init__(self):
        self.calls = []

    def __call__(self, geom, tx, rx, lam, antenna):
        self.calls.append((geom, np.array(tx), np.array(rx), lam, antenna))
        return propagation.Paths.from_ranges(5.0 + tx[0], 1e-4)


@pytest.fixture(name="recorder")
def fixture_recorder(monkeypatch):
    rec = Recorder()
    monkeypatch.setattr(propagation, "paths", rec, raising=False)
    return rec


def test_radar_true_position_and_cache(recorder):
    hw = Hardware()
    sweep = Sweep()
    scan = ScanGeometry(height=1.3)
    sled = SimSled(bias=0.02)
    radar = SimRadar("geom", hw, sweep, sled, scan, seed=3)
    sled.move_to(0.5)
    a = radar.capture(512)
    b = radar.capture(512)
    assert len(recorder.calls) == 1
    geom, tx, rx, lam, antenna = recorder.calls[0]
    assert geom == "geom" and lam == sweep.lam and antenna is hw.antenna
    np.testing.assert_allclose(tx, np.array([0.52, 0.0, 1.3]) + hw.tx_offset)
    np.testing.assert_allclose(rx, np.array([0.52, 0.0, 1.3]) + hw.rx_offset)
    rng = np.random.default_rng(3)
    paths = propagation.Paths.from_ranges(5.0 + tx[0], 1e-4)
    np.testing.assert_array_equal(a, synthesize(paths, sweep, hw, 512, rng))
    np.testing.assert_array_equal(b, synthesize(paths, sweep, hw, 512, rng))
    sled.move_to(0.7)
    radar.capture(16)
    sled.move_to(0.5)
    radar.capture(16)
    assert len(recorder.calls) == 2


class ReportOnly:
    """A sled exposing only the reported position."""

    def position(self):
        return 0.25


def test_radar_reported_position_fallback(recorder):
    radar = SimRadar("g", Hardware(), Sweep(), ReportOnly(), ScanGeometry(), seed=0)
    assert radar.capture(8).shape == (8,)
    assert recorder.calls[0][1][0] == pytest.approx(0.25 + Hardware().tx_offset[0])

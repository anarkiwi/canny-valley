"""Recording round trip and scan sequencing against stub devices."""

import numpy as np

from qmrdk.config import ScanGeometry, Sweep
from qmrdk.recording import Recording
from qmrdk.scan import frames, run_scan, scan_positions


class _Sled:
    def __init__(self):
        self.x = None
        self.homed = False

    def home(self):
        self.homed = True
        self.x = 0.0

    def move_to(self, x):
        self.x = x

    def position(self):
        return self.x


class _Radar:
    sweep = Sweep()

    def __init__(self, sled):
        self.sled = sled

    def capture(self, n):
        return np.full(n, int(round(self.sled.x * 1000)), dtype=np.uint16)


def test_scan_positions_spacing():
    sweep = Sweep()
    x = scan_positions(sweep, 1.5)
    assert x[0] == 0.0 and x[-1] == 1.5
    assert np.max(np.diff(x)) <= sweep.lam_min / 4
    assert np.allclose(scan_positions(sweep, 1.0, 0.25), [0, 0.25, 0.5, 0.75, 1.0])


def test_scan_and_round_trip(tmp_path):
    sled = _Sled()
    radar = _Radar(sled)
    rec = run_scan(radar, sled, [0.0, 0.5, 1.0], n=8, geometry=ScanGeometry(1.2))
    assert sled.homed
    assert np.array_equal(rec.x_pos, [0.0, 0.5, 1.0])
    assert np.array_equal(rec.codes[:, 0], [0, 500, 1000])
    rec.extra["target"] = [1.0, 2.0, 3.0]
    path = tmp_path / "scan.npz"
    rec.save(path)
    back = Recording.load(path)
    assert np.array_equal(back.codes, rec.codes) and back.codes.dtype == np.uint16
    assert np.array_equal(back.x_pos, rec.x_pos)
    assert back.sweep == rec.sweep and back.geometry == ScanGeometry(1.2)
    assert back.extra == {"target": [1.0, 2.0, 3.0]}


def test_recording_without_positions(tmp_path):
    sled = _Sled()
    sled.home()
    sled.move_to(0.25)
    codes = frames(_Radar(sled), 4, 3)
    assert codes.shape == (3, 4) and np.all(codes == 250)
    rec = Recording(codes=codes, sweep=Sweep(kind="cw"), t_host=np.arange(3.0))
    rec.save(tmp_path / "r.npz")
    back = Recording.load(tmp_path / "r.npz")
    assert back.x_pos is None and back.geometry is None and back.sweep.kind == "cw"
    assert back.temperature is None
    rec.temperature = np.array([30.0, 30.5, 31.25])
    rec.save(tmp_path / "t.npz")
    np.testing.assert_array_equal(
        Recording.load(tmp_path / "t.npz").temperature, rec.temperature
    )

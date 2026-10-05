"""`qmrdk` command line: SAR scan, image and demo."""

# pylint: disable=protected-access

import importlib
import importlib.util
import json
import sys
import types

import numpy as np
import pytest
from PIL import Image
from sarscene import HW, SWEEP, tiny

from qmrdk import cli
from qmrdk.config import ScanGeometry
from qmrdk.recording import Recording
from qmrdk.sim.devices import SimRadar, SimSled
from qmrdk.sim.scene import compile_scene

SMALL = ["--length", "0.3", "--n", "2048", "--gain", "2e3", "--seed", "1"]


@pytest.fixture(name="scene_path")
def fixture_scene_path(tmp_path):
    path = tmp_path / "tiny.json"
    path.write_text(json.dumps(tiny()), encoding="utf-8")
    return str(path)


@pytest.fixture(name="cal_path")
def fixture_cal_path(tmp_path):
    path = tmp_path / "cal.json"
    HW.calibration(SWEEP).save(path)
    return str(path)


def test_scan_image_round_trip(tmp_path, scene_path, cal_path, capsys):
    rec_path = str(tmp_path / "scan.npz")
    assert cli.main(["sar", "scan", "--sim", "--scene", scene_path, "--out", rec_path]
                    + SMALL) == 0  # fmt: skip
    rec = Recording.load(rec_path)
    assert rec.codes.shape == (12, 2048) and rec.x_pos[-1] == pytest.approx(0.3)
    assert rec.extra == {"scene": "tiny", "extent": [-4, 4, 2, 12]}
    assert rec.geometry.height == 1.0
    png = tmp_path / "img.png"
    args = ["sar", "image", rec_path, "--out", str(png), "--cal", cal_path]
    assert cli.main(args + ["--scene", scene_path, "--pixel", "0.2", "0.5"]) == 0
    assert "first_up=True" in capsys.readouterr().out
    wide = Image.open(png).size
    assert (
        cli.main(args + ["--extent", "-2", "2", "3", "7", "--background", "none"]) == 0
    )
    assert Image.open(png).size[0] < wide[0]
    assert cli.main(["sar", "image", rec_path, "--out", str(png)]) == 0
    rec.extra = {}
    rec.save(rec_path)
    assert cli.main(["sar", "image", rec_path, "--out", str(png)]) == 0


def test_image_needs_positions(tmp_path, capsys):
    path = tmp_path / "plain.npz"
    Recording(np.zeros((1, 8), np.uint16), SWEEP, np.zeros(1)).save(path)
    assert cli.main(["sar", "image", str(path), "--out", str(tmp_path / "x.png")]) == 1
    assert "no x_pos" in capsys.readouterr().err


def test_scan_hardware_unavailable(tmp_path, capsys):
    assert cli.main(["sar", "scan", "--out", str(tmp_path / "s.npz")]) == 2
    assert "use --sim" in capsys.readouterr().err


def test_scan_hardware_path(tmp_path, monkeypatch):
    geom = compile_scene(tiny(), SWEEP.lam)
    sled = SimSled()
    radar = SimRadar(geom, HW, SWEEP, sled, ScanGeometry(1.5), seed=0)
    monkeypatch.setattr(cli, "UsbRadar", lambda: radar)
    monkeypatch.setattr(cli, "HardwareSled", lambda: sled)
    path = str(tmp_path / "hw.npz")
    argv = ["sar", "scan", "--out", path, "--length", "0.1", "--n", "1024"]
    assert cli.main(argv + ["--height", "1.5"]) == 0
    rec = Recording.load(path)
    assert rec.codes.shape == (5, 1024) and rec.geometry.height == 1.5


def test_clipping_warning(capsys):
    assert cli._clipping(np.full(4, 1000, np.uint16), "a") == 0.0
    assert capsys.readouterr().err == ""
    assert cli._clipping(np.array([0, 1, 65535, 2], np.uint16), "a") == 0.5
    assert "50.0% of ADC samples clipped" in capsys.readouterr().err


def _demo(tmp_path, scene_path, cal):
    out = tmp_path / "demo" / "d.png"
    argv = ["sar", "demo", "--scene", scene_path, "--out", str(out), "--cal", cal,
            "--frames", "3", "--pixel", "0.2", "0.5"]  # fmt: skip
    assert cli.main(argv + SMALL) == 0
    with Image.open(out) as im:
        assert im.n_frames == 3
    assert Image.open(out.with_name("d_final.png")).format == "PNG"
    return json.loads(out.with_name("d_cal.json").read_text(encoding="utf-8"))


def test_demo_truth(tmp_path, scene_path):
    meta = _demo(tmp_path, scene_path, "truth")
    assert meta["source"] == "truth" and meta["report"] is None
    assert meta["gain"] == 2e3 and meta["clipped"] == 0.0
    assert meta["calibration"]["r_cal"] == HW.r_cal
    assert meta["calibration"]["n0"] == pytest.approx(HW.calibration(SWEEP).n0)


def test_demo_cal_file(tmp_path, scene_path, cal_path):
    meta = _demo(tmp_path, scene_path, cal_path)
    assert meta["calibration"]["tx_offset"] == list(HW.tx_offset)


@pytest.mark.skipif(
    importlib.util.find_spec("qmrdk.calib") is None
    or not hasattr(importlib.import_module("qmrdk.calib"), "calibrate_sim"),
    reason="qmrdk.calib.calibrate_sim not available",
)
def test_demo_procedure(tmp_path, scene_path):
    meta = _demo(tmp_path, scene_path, "procedure")
    assert meta["source"] == "procedure" and meta["report"] is not None


def test_json_default():
    assert cli._json_default(np.arange(2)) == [0, 1]
    assert cli._json_default(object).startswith("<class")


def test_calib_unavailable(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "qmrdk.calib", None)
    assert cli.main(["calib"]) == 1
    assert "calib commands unavailable" in capsys.readouterr().err


def test_calib_commands(monkeypatch):
    def add_commands(sub):
        sub.add_parser("probe").set_defaults(func=lambda args: 7)

    monkeypatch.setitem(
        sys.modules, "qmrdk.calib", types.SimpleNamespace(add_commands=add_commands)
    )
    assert cli.main(["calib", "probe"]) == 7


def test_sweep_options(tmp_path, scene_path):
    rec_path = str(tmp_path / "wide.npz")
    argv = ["sar", "scan", "--sim", "--scene", scene_path, "--out", rec_path,
            "--f0", "2.25", "--f1", "2.5", "--ramp-time", "20"]  # fmt: skip
    assert cli.main(argv + SMALL) == 0
    sweep = Recording.load(rec_path).sweep
    assert (sweep.f0, sweep.f1, sweep.ramp_time) == pytest.approx((2.25e9, 2.5e9, 0.02))

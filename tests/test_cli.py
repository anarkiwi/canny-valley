"""`qmrdk` command line: SAR scan, image and demo."""

# pylint: disable=protected-access

import argparse
import dataclasses
import importlib
import importlib.util
import io
import json
import sys
import types

import numpy as np
import pytest
from PIL import Image
from sarscene import HW, SWEEP, scan, tiny

from qmrdk import cli
from qmrdk.config import Calibration, ScanGeometry
from qmrdk.constants import C
from qmrdk.recording import Recording
from qmrdk.render import box_peaks
from qmrdk.sim.devices import SimRadar, SimSled
from qmrdk.sim.scene import compile_scene
from qmrdk.sim.scpi import SimBoard

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


@pytest.fixture(name="change", scope="module")
def fixture_change(tmp_path_factory):
    """Scan of the tiny scene and an independent-noise rescan with an extra
    point target, with a calibration file."""
    path = tmp_path_factory.mktemp("change")
    scene = tiny()
    scene["objects"].append({"type": "point", "pos": NEW, "rcs": 1.0})
    for name, (_, rec) in (("ref", scan(seed=1)), ("new", scan(scene=scene, seed=2))):
        rec.save(path / f"{name}.npz")
    HW.calibration(SWEEP).save(path / "cal.json")
    return path


NEW = [-2.0, 8.5, 1.0]


def _image(monkeypatch, path, *extra):
    """(exit code, captured save_image arguments) of `sar image` of new.npz."""
    saved = []
    monkeypatch.setattr(cli, "save_image", lambda *a, **k: saved.append((a, k)))
    argv = ["sar", "image", str(path / "new.npz"), "--out", str(path / "x.png"),
            "--cal", str(path / "cal.json"), "--pixel", "0.05", "0.05",
            "--extent", "-4", "4", "2", "12", *extra]  # fmt: skip
    return cli.main(argv), saved


def test_change_image(monkeypatch, change):
    code, plain = _image(monkeypatch, change)
    assert code == 0 and plain[0][1]["labels"][0] == ""
    code, diff = _image(monkeypatch, change, "--reference", str(change / "ref.npz"))
    assert code == 0 and diff[0][1]["labels"] == ("change ", "y (m)")
    (_, _, img, _, x_pos, lam, _), _ = diff[0]
    plain = plain[0][0][2]
    assert img.gy[0] == 3.0
    iy, ix = divmod(int(np.argmax(np.abs(img.image))), img.gx.size)
    cross = lam * np.hypot(NEW[0], NEW[1]) / (2 * np.ptp(x_pos))
    assert abs(img.gx[ix] - NEW[0]) < cross / 2
    assert abs(img.gy[iy] - NEW[1]) < C / (4 * SWEEP.bandwidth)
    old = np.array([[0.0, 5.0], [-1.5, 3.0], [-2.5, 4.0], [2.5, 6.0]])

    def level(im):
        return box_peaks(im, old, 0.5) + 20 * np.log10(np.abs(im.image).max())

    assert np.all(level(plain) - level(img) > 20)


@pytest.mark.parametrize(
    "change_rec",
    [
        lambda r: dataclasses.replace(r, sweep=dataclasses.replace(r.sweep, f1=2.45e9)),
        lambda r: dataclasses.replace(r, codes=r.codes[:, :1024]),
        lambda r: dataclasses.replace(r, x_pos=r.x_pos + 2e-6),
        lambda r: dataclasses.replace(r, x_pos=None),
    ],
)
def test_change_reference_refused(monkeypatch, change, tmp_path, capsys, change_rec):
    ref = Recording.load(change / "ref.npz")
    change_rec(ref).save(tmp_path / "ref.npz")
    code, saved = _image(monkeypatch, change, "--reference", str(tmp_path / "ref.npz"))
    assert code == 1 and not saved
    assert "not comparable" in capsys.readouterr().err


def test_change_reference_position_tolerance(monkeypatch, change, tmp_path):
    ref = Recording.load(change / "ref.npz")
    dataclasses.replace(ref, x_pos=ref.x_pos + 5e-7).save(tmp_path / "ref.npz")
    code, _ = _image(monkeypatch, change, "--reference", str(tmp_path / "ref.npz"))
    assert code == 0


def test_range_label_without_r_cal(monkeypatch, change, tmp_path):
    Calibration().save(tmp_path / "cal.json")
    code, saved = _image(monkeypatch, change, "--cal", str(tmp_path / "cal.json"))
    assert code == 0 and saved[0][1]["labels"] == ("", "y (m, incl. internal delay)")


def test_min_range(monkeypatch, change, capsys):
    rec = Recording.load(change / "new.npz")
    args = argparse.Namespace(extent=[-4, 4, 1, 12], min_range=3.0)
    assert cli._extent(args, None, rec) == (-4.0, 4.0, 3.0, 12.0)
    args.extent = [-4, 4, 5, 12]
    assert cli._extent(args, None, rec)[2] == 5.0
    args.extent = None
    assert cli._extent(args, tiny(), rec) == (-4.0, 4.0, 3.0, 12.0)
    args.min_range = -1.0
    assert cli._extent(args, tiny(), rec)[2] == -0.5
    code, saved = _image(monkeypatch, change, "--min-range", "12")
    assert code == 1 and not saved
    assert "empty extent" in capsys.readouterr().err


def test_scan_hardware_unavailable(tmp_path, capsys):
    assert cli.main(["sar", "scan", "--out", str(tmp_path / "s.npz")]) == 2
    assert "sled controller not found" in capsys.readouterr().err


def test_scan_hardware_path(tmp_path, monkeypatch, attach):
    geom = compile_scene(tiny(), SWEEP.lam)
    sled = SimSled()
    radar = SimRadar(geom, HW, SWEEP, sled, ScanGeometry(1.5), seed=0)
    board = attach(SimBoard(source=lambda n, sweep: radar.capture(n)))
    opened = []
    monkeypatch.setattr(cli, "HardwareSled", lambda *a: opened.append(a) or sled)
    path = str(tmp_path / "hw.npz")
    argv = ["sar", "scan", "--out", path, "--length", "0.1", "--n", "1024"]
    assert cli.main(argv + ["--height", "1.5"]) == 0
    rec = Recording.load(path)
    assert rec.codes.shape == (5, 1024) and rec.geometry.height == 1.5
    assert rec.sweep == SWEEP and not board.rf and not board.sweeping
    assert opened == [(None, 0.05, 0.3)]


def test_scan_manual(tmp_path, monkeypatch, attach, capsys):
    attach(SimBoard(source=lambda n, sweep: np.full(n, 30000, np.uint16)))
    monkeypatch.setattr("sys.stdin", io.StringIO("\n" * 5))
    path = str(tmp_path / "manual.npz")
    argv = ["sar", "scan", "--manual", "--out", path, "--length", "0.1"]
    assert cli.main(argv + ["--dx", "0.025", "--n", "64"]) == 0
    np.testing.assert_allclose(Recording.load(path).x_pos, np.arange(5) * 0.025)
    assert capsys.readouterr().err.count("Enter when placed") == 5
    with pytest.raises(SystemExit):
        cli.parser().parse_args(argv + ["--sim"])


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
        group = sub.add_parser("calib").add_subparsers(required=True)
        group.add_parser("probe").set_defaults(func=lambda args: 7)

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

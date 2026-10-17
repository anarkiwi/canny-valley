"""Rough r_cal from one surveyed target (`calib target`)."""

import dataclasses
import json
import math

import numpy as np
import pytest
from sarscene import HW, SWEEP, scan, tiny

from qmrdk import calib, cli
from qmrdk.config import Calibration
from qmrdk.dsp.sar import SarImage

LENGTH = 0.85


def _scene(y):
    """A reflector at range `y` broadside of mid-rail at rail height."""
    scene = tiny()
    scene["objects"] = [
        {
            "type": "reflector",
            "pos": [LENGTH / 2, y, 1.0],
            "rcs": 10.0,
            "boresight": -90,
        }
    ]
    scene["ground"].update(material="grass", extent=[-4, 4, 1, 14])
    return scene


@pytest.fixture(name="recordings", scope="module")
def fixture_recordings():
    return {y: scan(length=LENGTH, scene=_scene(y))[1] for y in (5.0, 8.0)}


@pytest.mark.parametrize("y", [5.0, 8.0])
@pytest.mark.parametrize("offset", [-0.8, 0.0, 0.6])
def test_recovers_r_cal(recordings, y, offset):
    truth = HW.calibration(SWEEP)
    start = dataclasses.replace(truth, r_cal=truth.r_cal + offset)
    distance = math.hypot(y, 0.02)
    r_cal, report = calib.estimate_r_cal_target(recordings[y], start, distance)
    assert r_cal == pytest.approx(truth.r_cal, abs=0.03)
    assert report["peak"][0] == pytest.approx(0.0, abs=0.1)
    assert r_cal == pytest.approx(start.r_cal + report["slant"] - distance)


def test_band_excludes_leakage(recordings):
    truth = HW.calibration(SWEEP)
    _, report = calib.estimate_r_cal_target(recordings[8.0], truth, 8.0, band=1.0)
    assert 7.0 <= report["peak"][1] <= 9.0


def test_cli(recordings, tmp_path, capsys):
    rec_path, cal_path = tmp_path / "scan.npz", tmp_path / "cal.json"
    recordings[5.0].save(rec_path)
    truth = HW.calibration(SWEEP)
    dataclasses.replace(truth, r_cal=0.0).save(cal_path)
    argv = ["calib", "target", str(rec_path), "--cal", str(cal_path)]
    assert cli.main(argv + ["--distance", "5.0"]) == 0
    report = json.loads(capsys.readouterr().out)
    saved = Calibration.load(cal_path)
    assert saved.r_cal == pytest.approx(report["r_cal"])
    assert saved.r_cal == pytest.approx(truth.r_cal, abs=0.03)
    assert dataclasses.replace(saved, r_cal=0.0) == dataclasses.replace(
        truth, r_cal=0.0
    )
    new = tmp_path / "new.json"
    argv = ["calib", "target", str(rec_path), "--cal", str(new), "--distance", "5"]
    assert cli.main(argv) == 0
    assert Calibration.load(new).r_cal != 0.0


def test_no_interior_peak(recordings, monkeypatch, tmp_path, capsys):
    def flat(_codes, _x, _sweep, _cal, _geometry, gx, gy, *_args, **_kwargs):
        return SarImage(np.zeros((gy.size, gx.size)), gx, gy, 1.0, True, 0.0)

    monkeypatch.setattr(calib, "form_image", flat)
    rec_path = tmp_path / "scan.npz"
    recordings[5.0].save(rec_path)
    argv = ["calib", "target", str(rec_path), "--cal", str(tmp_path / "c.json")]
    assert cli.main(argv + ["--distance", "5"]) == 2
    assert "no return" in capsys.readouterr().err
    assert not (tmp_path / "c.json").exists()

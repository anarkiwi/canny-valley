"""Calibration estimators against the simulator's truths (docs/calibration.md).

Tolerances: `K_T` two-sided 99.9 % Student-t quantiles on the estimators'
standard errors, `CHI2_3` the 99.9 % chi-square quantile for the joint
(r_cal, ox, oy) Mahalanobis distance under the reported covariance.
"""

import argparse
import dataclasses
import json
import pathlib

import numpy as np
import pytest
from fakeboard import FakeBoard, install
from scipy import signal, stats

from qmrdk import calib
from qmrdk.config import Calibration, ScanGeometry, Sweep
from qmrdk.constants import FS_NOMINAL, C
from qmrdk.dsp.range import range_spectrum
from qmrdk.dsp.segment import interpolation_matrix, ramp_length
from qmrdk.recording import Recording
from qmrdk.scan import frames as capture_frames
from qmrdk.sim.hardware import Hardware, synthesize, synthesize_volts
from qmrdk.sim.propagation import Paths

SWEEP = Sweep()
GEOM = ScanGeometry()
ALT = {
    "fs": 21950.0,
    "t_start": 3.1e-3,
    "first_up": False,
    "r_cal": 0.52,
    "tx_offset": (-0.03, 0.035, 0.0),
    "rx_offset": (0.09, 0.035, 0.0),
}
UP = {
    "fs": 22010.0,
    "t_start": 1.7e-3,
    "first_up": True,
    "r_cal": 0.21,
    "tx_offset": (-0.08, -0.025, 0.0),
    "rx_offset": (0.04, -0.025, 0.0),
}
P = 1e-3
CHI2_3 = stats.chi2.ppf(1 - P, 3)
TARGET = calib.default_target(GEOM)


def k_t(dof):
    return stats.t.ppf(1 - P / 2, dof)


def phase_delay_n0(hw, rng):
    """First turnaround in [0, nr) delayed by the IF phase delay at the beat
    frequency of a scatterer at one-way range `rng`."""
    fb = SWEEP.slope * 2.0 * (rng + hw.r_cal) / C
    z, p, k = hw.if_zpk()
    h = signal.freqz_zpk(z, p, k, worN=[fb], fs=hw.fsim)[1][0]
    return float(
        np.mod(hw.fs * (hw.t_start - np.angle(h) / (2 * np.pi * fb)), hw.nr(SWEEP))
    )


def tone_frames(hw, rng, count, seed=0):
    paths = Paths.from_ranges([rng], [1e-2 / rng**2])
    gen = np.random.default_rng(seed)
    return np.stack([synthesize(paths, SWEEP, hw, 4096, gen) for _ in range(count)])


def mahalanobis(res, hw):
    mid = 0.5 * (np.array(hw.tx_offset) + hw.rx_offset)
    est = 0.5 * (np.array(res.tx_offset) + res.rx_offset)
    err = np.r_[res.r_cal - hw.r_cal, est[:2] - mid[:2]]
    return float(err @ np.linalg.solve(res.cov, err))


def test_window_lobes():
    half, width, psl = calib.window_lobes("hann", 512)
    assert half == pytest.approx(2.0, abs=0.02)
    assert width == pytest.approx(1.44, abs=0.01)
    assert psl == pytest.approx(-31.47, abs=0.05)
    assert calib.window_lobes("boxcar", 512)[1] == pytest.approx(0.886, abs=0.01)


@pytest.mark.parametrize(
    "truths, rng",
    [
        ({}, 20.0),
        (ALT, 20.0),
        (UP, 20.0),
        (ALT | {"hp_order": 0}, 3.0),
        (UP | {"hp_order": 0}, 8.0),
    ],
)
def test_timing_recovers_clock_and_phase_delay_turnaround(truths, rng):
    hw = Hardware(leak_amp=0.0, **truths)
    res = calib.estimate_timing(tone_frames(hw, rng, 16), SWEEP)
    k = k_t(res.turnarounds - 2)
    assert abs(res.fs - hw.fs) <= k * res.se_fs
    assert abs(res.n0 - phase_delay_n0(hw, rng)) <= k * res.se_n0
    assert res.locked
    assert res.beat_frequency == pytest.approx(
        SWEEP.slope * 2 * (rng + hw.r_cal) / C, rel=0.02
    )
    assert res.n0_jitter > 0.0
    json.dumps(res.summary())


def test_timing_frame_pair_cancels_leakage():
    hw = Hardware(**ALT)
    a, b = (
        tone_frames(hw, r, 8, seed=s) for s, r in ((1, 20.0), (2, 20.0 + SWEEP.lam / 4))
    )
    res = calib.estimate_timing(np.stack([a, b]), SWEEP)
    assert abs(res.fs - hw.fs) <= k_t(res.turnarounds - 2) * res.se_fs
    assert res.noise_var == pytest.approx(hw.noise**2, rel=0.1)


def test_timing_needs_ramps():
    with pytest.raises(ValueError):
        calib.estimate_timing(tone_frames(Hardware(), 8.0, 2)[:, :900], SWEEP)


def guard_truth(paths, hw, n0, guards, window="hann", zpad=4):
    """Actual transient level per guard: extracted ramps against the same
    samples of a frame whose sweep crosses the ramp's band far from any
    turnaround, beyond the window main lobe of zero range."""
    n = 4096
    x = synthesize_volts(paths, SWEEP, hw, n)
    nr = hw.nr(SWEEP)
    refs = []
    for k in range(2, int((n - n0) // nr) - 1):
        pad = SWEEP.slope * (hw.t_start + k * SWEEP.ramp_time - hw.t_reset)
        ref_sweep = Sweep(
            SWEEP.f0 - pad,
            SWEEP.f1 + pad,
            SWEEP.ramp_time * (1 + 2 * pad / SWEEP.bandwidth),
        )
        rising = (k % 2 == 0) == hw.first_up
        ref_hw = dataclasses.replace(hw, t_start=hw.t_reset, first_up=rising)
        refs.append((n0 + k * nr, synthesize_volts(paths, ref_sweep, ref_hw, n)))
    lo = int(np.ceil(calib.window_lobes(window, ramp_length(nr, 0))[0] * zpad))
    out = []
    for g in guards:
        level = 0.0
        for start, ref in refs:
            m = interpolation_matrix(start + g + np.arange(ramp_length(nr, g)), n)
            err, clean = (
                np.abs(range_spectrum(v, window, zpad))[lo:]
                for v in (m @ (x - ref), m @ ref)
            )
            level = max(level, err.max() / clean.max())
        out.append(20 * np.log10(level))
    return np.array(out)


def pair(hw, count, seed=0):
    """Static frame pair at the sled home and one quarter-wave step on, as
    the procedure takes it, and the paths of their difference."""
    radar, sled = calib.sim_devices(hw, SWEEP, GEOM, TARGET, seed)
    sled.home()
    first = radar.paths()
    frames = calib.run_timing(
        radar, 4096, count, sled, calib.pair_step(SWEEP, GEOM, TARGET)
    )
    second = radar.paths()
    diff = Paths(
        np.concatenate([first.delay, second.delay]),
        np.concatenate([first.amp, -second.amp]),
        np.zeros(len(first) + len(second)),
        np.concatenate([first.kind, second.kind]),
    )
    return frames, diff


def test_guard_meets_criterion_and_is_smallest():
    hw = Hardware(hp_order=0, pll_fn=250.0, pll_zeta=1.0)
    frames, diff = pair(hw, 32)
    timing = calib.estimate_timing(frames, SWEEP)
    res = calib.estimate_guard(frames, SWEEP, Calibration(fs=timing.fs, n0=timing.n0))
    assert res.noise_ok and res.frames_needed <= res.frames
    assert res.ng > 0
    truth = guard_truth(
        diff, dataclasses.replace(hw, leak_amp=0.0), timing.n0, [res.ng - 1, res.ng]
    )
    assert truth[1] < res.psl_db <= truth[0]
    json.dumps(res.summary())


def test_guard_default_board_needs_none():
    hw = Hardware(**ALT)
    frames, diff = pair(hw, 16)
    timing = calib.estimate_timing(frames, SWEEP)
    res = calib.estimate_guard(frames, SWEEP, Calibration(fs=timing.fs, n0=timing.n0))
    assert res.ng == 0
    truth = guard_truth(diff, dataclasses.replace(hw, leak_amp=0.0), timing.n0, [0])
    assert truth[0] < res.psl_db
    assert res.transient.shape == (2, int(hw.nr(SWEEP) // 2))


def test_guard_noise_check_asks_for_frames():
    hw = Hardware(noise=2e-2)
    frames, _ = pair(hw, 3)
    rng = np.linalg.norm(np.asarray(TARGET) - [0, 0, GEOM.height])
    res = calib.estimate_guard(frames, SWEEP, Calibration(n0=phase_delay_n0(hw, rng)))
    assert res.ng is None and not res.noise_ok
    assert res.frames_needed > res.frames
    assert "ng" in res.summary()


def test_guard_errors():
    with pytest.raises(ValueError):
        calib.estimate_guard(
            tone_frames(Hardware(), 8.0, 2)[:, :800], SWEEP, Calibration()
        )


def scan(truths, seed=0, sled_sigma=0.0, scans=1):
    hw = Hardware(**truths)
    radar, sled = calib.sim_devices(hw, SWEEP, GEOM, TARGET, seed, sled_sigma)
    recs = [calib.run_reflector(radar, sled, GEOM, TARGET) for _ in range(scans)]
    rng = np.linalg.norm(np.asarray(TARGET) - [0.75, 0, GEOM.height])
    start = calib.nominal(hw)
    cal = dataclasses.replace(start, fs=hw.fs, n0=phase_delay_n0(hw, rng), ng=16)
    return hw, cal, recs


@pytest.fixture(name="alt_scan", scope="module")
def fixture_alt_scan():
    return scan(ALT | {"hp_order": 0}, seed=1)


@pytest.mark.parametrize("truths", [ALT, UP])
def test_reflector_recovers_truths(truths):
    hw, cal, (rec,) = scan(truths | {"hp_order": 0}, seed=2)
    res = calib.estimate_reflector(
        rec, SWEEP, cal, GEOM, TARGET, 0.12, noise_var=hw.noise**2
    )
    assert res.first_up == hw.first_up
    assert mahalanobis(res, hw) < CHI2_3
    assert res.cost_ratio > 1.0
    assert res.chi2 == pytest.approx(1.0, abs=0.5)
    assert res.focus["ok"]
    assert np.allclose(res.focus["widths"], res.focus["predicted_widths"], rtol=0.15)
    assert res.range_rms < SWEEP.lam / 8
    json.dumps(res.summary())


def test_reflector_tuple_input(alt_scan):
    hw, cal, (rec,) = alt_scan
    res = calib.estimate_reflector(
        (rec.codes, rec.x_pos), SWEEP, cal, GEOM, TARGET, 0.12, check=False
    )
    assert not res.focus and np.isnan(res.chi2)
    assert mahalanobis(res, hw) < CHI2_3


@pytest.mark.slow
@pytest.mark.parametrize("sigma", [0.0, 1e-3])
def test_repeatability_recovers_sled_sigma(sigma):
    _, cal, recs = scan(ALT | {"hp_order": 0}, seed=3, sled_sigma=sigma, scans=4)
    res = calib.estimate_repeatability(recs, SWEEP, cal, GEOM, TARGET, 0.12)
    assert abs(res.sled_sigma - sigma) <= stats.norm.ppf(1 - P / 2) * res.se
    assert res.gain_loss == pytest.approx(
        np.exp(-((4 * np.pi * res.sled_sigma / SWEEP.lam) ** 2))
    )
    assert res.scans == 4 and 0 < res.positions <= recs[0].x_pos.size
    json.dumps(res.summary())


def test_repeatability_needs_two_scans(alt_scan):
    _, cal, recs = alt_scan
    with pytest.raises(ValueError):
        calib.estimate_repeatability(recs, SWEEP, cal, GEOM, TARGET, 0.12)


def test_pair_step_quarter_wave():
    dx = calib.pair_step(SWEEP, GEOM, TARGET)
    rng = np.linalg.norm(np.asarray(TARGET) - [[dx, 0, 1.0], [0.0, 0, 1.0]], axis=1)
    assert abs(rng[0] - rng[1]) == pytest.approx(SWEEP.lam / 4, rel=0.05)
    assert calib.pair_step(SWEEP, GEOM, (0.0, 8.0, 1.0), length=1.5) == 1.5


@pytest.mark.slow
@pytest.mark.parametrize("truths, sigma", [({}, 5e-4), (ALT, 0.0)])
def test_calibrate_sim(truths, sigma):
    """Steps 1-4 chained, each fed the previous steps' estimates. The step-3
    model leaves out the reflector's own IF high-pass turnaround transient: with
    a perfect sled that gap dominates (reported as chi2 > 1) and the result is
    held to the SAR budget (lam / 16 midpoint, 1/16 range cell)."""
    hw = Hardware(**truths)
    cal, report = calib.calibrate_sim(hw, SWEEP, sled_sigma=sigma)
    t, r = report["timing"], report["reflector"]
    assert abs(cal.fs - hw.fs) <= k_t(t["turnarounds"] - 2) * t["se_fs"]
    assert cal.first_up == hw.first_up
    assert cal.ng == report["guard"]["ng"]
    res = calib.ReflectorResult(
        **(r | {"cov": np.array(r["cov"]), "rho": complex(*r["rho"])})
    )
    if sigma > 0:
        assert mahalanobis(res, hw) < CHI2_3
    else:
        assert r["chi2"] > 1.0
        mid = 0.5 * (
            np.array(cal.tx_offset) + cal.rx_offset - hw.tx_offset - hw.rx_offset
        )
        assert np.all(np.abs(mid) < SWEEP.lam / 16)
        assert abs(cal.r_cal - hw.r_cal) < r["focus"]["predicted_widths"][0] / 16
    assert r["focus"]["ok"]
    rep = report["repeat"]
    assert abs(cal.sled_sigma - sigma) <= stats.norm.ppf(1 - P / 2) * rep["se"]
    assert report["runtime"] > 0
    json.dumps(report)


def run_cli(argv):
    parser = argparse.ArgumentParser()
    calib.add_commands(parser.add_subparsers(dest="command"))
    args = parser.parse_args(argv)
    return args.func(args)


@pytest.mark.slow
def test_cli_steps(tmp_path, capsys):
    cal, rep = tmp_path / "cal.json", tmp_path / "report.json"
    common = ["--sim", "--cal", str(cal), "--report", str(rep), "--frames", "8"]
    for step in ("timing", "guard", "reflector"):
        assert run_cli(["calib", step, *common]) == 0
        assert step in json.loads(rep.read_text())
    assert (
        run_cli(["calib", "repeat", *common, "--scans", "2", "--sled-sigma", "1e-3"])
        == 0
    )
    result = Calibration.load(cal)
    hw = Hardware()
    assert result.fs == pytest.approx(hw.fs, abs=1.0)
    assert result.first_up == hw.first_up
    assert result.sled_sigma > 0
    assert "repeat" in capsys.readouterr().out


def test_cli_sim(tmp_path):
    cal = tmp_path / "cal.json"
    assert (
        run_cli(["calib", "sim", "--cal", str(cal), "--scans", "1", "--frames", "8"])
        == 0
    )
    assert Calibration.load(cal).first_up == Hardware().first_up


@pytest.mark.parametrize("step", ["timing", "guard", "reflector", "repeat", "sim"])
def test_cli_without_sim_reports_device_error(step, capsys, monkeypatch):
    if step == "sim":
        monkeypatch.setattr(calib, "calibrate_sim", _raise)
    assert run_cli(["calib", step]) == 2
    assert "qmrdk calib:" in capsys.readouterr().err


def _raise(*_args, **_kwargs):
    raise NotImplementedError("no device")


def static_sets(moves, frames=8):
    """Static sets of the simulated board at sled positions `moves` (m)."""
    hw = Hardware()
    radar, sled = calib.sim_devices(hw, SWEEP, GEOM, TARGET, seed=3)
    out = []
    for x in moves:
        sled.move_to(x)
        out.append(capture_frames(radar, 4096, frames))
    return np.stack(out)


def read_report(path):
    return json.loads(pathlib.Path(path).read_text(encoding="utf-8"))


@pytest.mark.parametrize("quarters, expect", [(1.0, 180.0), (0.5, 90.0), (0.0, 0.0)])
def test_pair_phase(quarters, expect):
    dx = quarters * calib.pair_step(SWEEP, GEOM, TARGET)
    a, b = static_sets([0.0, dx])
    phi = calib.pair_phase(a, b, SWEEP, FS_NOMINAL)
    assert -180.0 < phi <= 180.0
    assert abs(abs(phi) - expect) < 10.0
    back = calib.pair_phase(b, a, SWEEP, FS_NOMINAL)
    assert abs(np.mod(back + phi + 180.0, 360.0) - 180.0) < 5.0


@pytest.fixture(name="board")
def fixture_board(monkeypatch):
    """Fake board serving simulated frames; its sled moves the radar."""
    radar, sled = calib.sim_devices(Hardware(), SWEEP, GEOM, TARGET, seed=5)
    board = install(monkeypatch, FakeBoard(radar.capture))
    board.sled = sled
    return board


def test_cli_hardware_pair(board, tmp_path, capsys, monkeypatch):
    a, b, rep = (str(tmp_path / f) for f in ("a.npz", "b.npz", "rep.json"))
    common = ["--frames", "8", "--check-frames", "4", "--report", rep]
    assert run_cli(["calib", "timing", "--frames-file", a, *common]) == 0
    assert board.closed and board.log[-1] == "SWEEP:STOP"
    single = read_report(rep)["timing"]
    assert run_cli(["calib", "timing", "--frames-file", b, "--pair", a, *common]) == 2
    assert "move the radar again" in capsys.readouterr().err
    assert not pathlib.Path(b).exists()
    board.sled.move_to(calib.pair_step(SWEEP, GEOM, TARGET))
    assert run_cli(["calib", "timing", "--frames-file", b, "--pair", a, *common]) == 0
    assert "pair phase change" in capsys.readouterr().err
    paired = read_report(rep)["timing"]
    assert paired["frames"] == single["frames"] == 8
    k = k_t(paired["turnarounds"] - 2)
    assert abs(paired["fs"] - Hardware().fs) <= k * paired["se_fs"]
    install(monkeypatch)
    cal = str(tmp_path / "cal.json")
    for step in ("timing", "guard"):
        argv = ["calib", step, "--frames-file", b, "--pair", a, "--cal", cal]
        assert run_cli(argv + common) == 0
    assert read_report(rep)["guard"]["frames"] == 8
    assert run_cli(["calib", "timing", "--frames-file", a, *common]) == 0
    assert read_report(rep)["timing"] == single


def test_cli_hardware_pair_mismatch(board, tmp_path, capsys):
    a, b = str(tmp_path / "a.npz"), str(tmp_path / "b.npz")
    assert run_cli(["calib", "guard", "--frames-file", a, "--frames", "4"]) == 0
    assert run_cli(["calib", "guard", "--frames-file", b, "--frames", "6"]) == 0
    assert board.log.count("SWEEP:STOP") == 2
    assert run_cli(["calib", "guard", "--frames-file", b, "--pair", a]) == 2
    assert "does not match" in capsys.readouterr().err


def test_recording_round_trip(alt_scan, tmp_path):
    _, _, (rec,) = alt_scan
    rec.save(tmp_path / "r.npz")
    assert Recording.load(tmp_path / "r.npz").extra["target"] == pytest.approx(
        list(TARGET)
    )

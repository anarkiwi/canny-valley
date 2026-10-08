"""Drift of a static capture: estimator on closed-form frames and the
simulated board, Allan deviation, `qmrdk drift`."""

import json

import allantools
import ideal
import numpy as np
import pytest
from scipy.signal import get_window

from qmrdk import cli
from qmrdk.config import Calibration, Sweep
from qmrdk.constants import C
from qmrdk.drift import allan_deviation, estimate_drift, frame_noise
from qmrdk.dsp.segment import ramp_length, ramp_positions
from qmrdk.recording import Recording
from qmrdk.sim.hardware import Hardware
from qmrdk.sim.scpi import SimBoard

SWEEP = Sweep()
N = 4096
TC = 2e-3
K = 64
SIDELOBE_BIAS = 2e-3


def _temperature(k):
    return 30.0 + 4.0 * np.sin(2.0 * np.pi * np.asarray(k) / 24.0) + 0.05 * k


def _ideal(first_up=True, noise=2e-3, temp=True, tc=TC):
    """Static frames of a target at 8 m and leakage at 0.4 m whose common
    delay grows by `2 tc / c` per degree, one second apart. The phase of a
    windowed bin carries the image and the neighbouring line's sidelobes,
    bounded by `SIDELOBE_BIAS` relative to the true coefficient."""
    cal = Calibration(n0=37.2, ng=16, first_up=first_up)
    temperature = _temperature(np.arange(K))
    tau = ideal.range_tau(np.add.outer(tc * temperature, [8.0, 0.4]))
    x = ideal.if_signal(
        N, SWEEP, cal.fs, cal.n0, first_up, tau, [0.3, 0.1], noise=noise
    )
    rec = Recording(
        ideal.to_codes(x),
        SWEEP,
        1e9 + np.arange(K, dtype=np.float64),
        temperature=temperature if temp else None,
    )
    return rec, cal


@pytest.mark.parametrize("first_up", [True, False])
def test_recovers_delay_temperature_coefficient(first_up):
    rec, cal = _ideal(first_up)
    res = estimate_drift(rec, cal)
    s = res.summary()
    t = s["target"]
    assert res.sign_known and s["frames"] == K and abs(t["range_m"] - 8.0) < 0.3
    tol = 3 * t["se_temp_mm_per_degc"] + SIDELOBE_BIAS * 1e3 * TC
    assert abs(t["temp_mm_per_degc"] - 1e3 * TC) < tol
    deg_per_degc = np.degrees(4 * np.pi * res.f_centre * TC / C)
    tol = 3 * t["se_temp_deg_per_degc"] + SIDELOBE_BIAS * deg_per_degc
    assert abs(t["temp_deg_per_degc"] - deg_per_degc) < tol
    leak = s["leakage"]
    assert leak["temp_mm_per_degc"] > 10 * leak["se_temp_mm_per_degc"]
    m2 = np.abs(np.mean(np.exp(1j * res.lines["target"].phase))) ** 2
    assert t["residual_db"] == pytest.approx(10 * np.log10((1 - m2) / m2), abs=0.5)
    assert s["leakage"]["bin"] < t["bin"] and t["amplitude_std_db"] < 0.1
    np.testing.assert_allclose(
        res.lines["target"].range_drift,
        TC * (rec.temperature - rec.temperature[0]),
        atol=5 * t["range_noise_mm"] * 1e-3,
    )
    assert t["rate_mm_per_min"] == pytest.approx(
        TC * 0.05 * 60 * 1e3, abs=3 * t["se_rate_mm_per_min"] + 0.5
    )


def test_tracks_strongest_lines_separately():
    """A fixed line (no delay drift) beside a drifting stronger one: each is
    tracked at its own range with its own coefficient."""
    cal = Calibration(n0=37.2, ng=16, first_up=True)
    temperature = _temperature(np.arange(K))
    drifting = TC * temperature
    tau = ideal.range_tau(np.stack([8.0 + drifting, np.full(K, 4.0)], axis=1))
    x = ideal.if_signal(N, SWEEP, cal.fs, cal.n0, True, tau, [0.3, 0.2], noise=2e-3)
    rec = Recording(
        ideal.to_codes(x),
        SWEEP,
        np.arange(K, dtype=np.float64),
        None,
        temperature=temperature,
    )
    s = estimate_drift(rec, cal, count=2).summary()
    assert abs(s["target"]["range_m"] - 8.0) < 0.3
    assert abs(s["line2"]["range_m"] - 4.0) < 0.3
    assert s["target"]["temp_mm_per_degc"] == pytest.approx(1e3 * TC, rel=0.05)
    assert abs(s["line2"]["temp_mm_per_degc"]) < 0.05 * 1e3 * TC


def test_frame_noise_matches_snr():
    noise, amp = 2e-2, 0.3
    rec, cal = _ideal(noise=noise, tc=0.0)
    t = estimate_drift(rec, cal).summary()["target"]
    nr = cal.nr(SWEEP)
    ramps = ramp_positions(cal.n0, nr, cal.ng, N, True).shape[1]
    w = get_window("hann", ramp_length(nr, cal.ng), fftbins=False)
    var = noise**2 * np.sum(w**2) / np.sum(w) ** 2 / ramps / amp**2
    assert t["phase_noise_deg"] == pytest.approx(np.degrees(np.sqrt(var)), rel=0.2)
    assert t["temp_mm_per_degc"] == pytest.approx(0.0, abs=3 * t["se_temp_mm_per_degc"])


def test_unknown_direction_and_no_temperature():
    rec, cal = _ideal(first_up=None, temp=False)
    res = estimate_drift(rec, cal)
    s = res.summary()
    assert not res.sign_known and s["temperature_range_degc"] is None
    assert s["target"]["temp_mm_per_degc"] is None
    span = 1e3 * TC * np.ptp(_temperature(np.arange(K)))
    assert s["target"]["range_span_mm"] == pytest.approx(span, rel=0.05)


def test_needs_triangle():
    rec, cal = _ideal()
    rec.codes = rec.codes[:, :300]
    with pytest.raises(ValueError, match="no complete ramp"):
        estimate_drift(rec, cal)


@pytest.mark.parametrize("n", [3, 50, 257])
def test_allan_deviation_matches_allantools(n):
    y = np.random.default_rng(n).normal(size=n).cumsum() * 0.1
    tau, adev = allan_deviation(y, 0.5)
    ref_tau, ref, _, _ = allantools.oadev(y, rate=2.0, data_type="freq", taus=tau)
    np.testing.assert_allclose(ref_tau, tau)
    np.testing.assert_allclose(adev, ref, rtol=1e-10)
    assert allan_deviation(y[:2], 1.0)[0].size == 0 and frame_noise(y[:2]) is None


def test_white_noise_allan_slope():
    y = np.random.default_rng(1).normal(size=4096)
    tau, adev = allan_deviation(y, 1.0)
    np.testing.assert_allclose(adev[:6] * np.sqrt(tau[:6]), 1.0, rtol=0.15)
    assert frame_noise(y) == pytest.approx(1.0, rel=0.05)


def test_cli_simulated_board(attach, tmp_path, capsys):
    hw = Hardware(delay_tc=TC)
    attach(SimBoard(hardware=hw, temperature=_temperature))
    rec, cal = tmp_path / "static.npz", tmp_path / "cal.json"
    hw.calibration(SWEEP).save(cal)
    argv = ["capture", "--frames", "24", "--temperature", "--out", str(rec)]
    assert cli.main(argv) == 0
    out, report = tmp_path / "drift.png", tmp_path / "drift.json"
    argv = ["drift", str(rec), "--cal", str(cal), "--out", str(out)]
    assert cli.main(argv + ["--report", str(report)]) == 0
    s = json.loads(report.read_text(encoding="utf-8"))
    stdout = capsys.readouterr().out
    assert json.loads(stdout[stdout.index("{") :]) == s and out.stat().st_size > 0
    np.testing.assert_allclose(
        Recording.load(rec).temperature, np.round(_temperature(np.arange(24)), 2)
    )
    assert s["target"]["temp_mm_per_degc"] == pytest.approx(1e3 * TC, rel=0.02)
    assert s["leakage"]["temp_mm_per_degc"] > 0

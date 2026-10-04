"""Board model against closed forms (docs/simulation.md §4)."""

import dataclasses

import numpy as np
import pytest
from scipy import signal

from qmrdk.config import Sweep
from qmrdk.constants import C
from qmrdk.sim.hardware import Hardware, f_cmd, f_tx, synthesize, synthesize_volts
from qmrdk.sim.propagation import Paths

IDEAL = Hardware(pll_fn=1e9, hp_order=0, lp_order=0, r_cal=0.0, leak_amp=0.0, noise=0.0)
SWEEP = Sweep()


def triangle(t, sweep, hw, f_prev):
    """Ideal commanded frequency from its corner table."""
    k = np.arange(int(np.max(t) / sweep.ramp_time) + 2)
    lo, hi = (sweep.f0, sweep.f1) if hw.first_up else (sweep.f1, sweep.f0)
    f = np.interp(t, hw.t_start + k * sweep.ramp_time, np.where(k % 2, hi, lo))
    return np.where(t < hw.t_reset, f_prev, f)


def butter_analog(f, fc, order, highpass):
    """Normalised Butterworth response at frequency f."""
    if order == 0:
        return np.ones_like(f, dtype=complex)
    p = np.exp(1j * np.pi * (2 * np.arange(1, order + 1) + order - 1) / (2 * order))
    s = 1j * np.asarray(f, dtype=float)[..., None] / fc
    s = 1.0 / s if highpass else s
    return np.prod(1.0 / (s - p), axis=-1)


def bilinear_response(f, hw):
    """Digital IF response: each Butterworth section pre-warped at its corner."""
    fsim = hw.fsim

    def warp(fc):
        return fc * np.tan(np.pi * f / fsim) / np.tan(np.pi * fc / fsim)

    return butter_analog(warp(hw.hp_fc), hw.hp_fc, hw.hp_order, True) * butter_analog(
        warp(hw.lp_fc), hw.lp_fc, hw.lp_order, False
    )


def volts(code):
    return code.astype(float) * 5.0 / 65535.0 - 2.5


def test_static_path_closed_form():
    hw = dataclasses.replace(IDEAL, first_up=False)
    paths = Paths.from_ranges([4.3, 11.7], [3e-4 * np.exp(0.4j), 1e-4])
    v = synthesize_volts(paths, SWEEP, hw, 4096)
    t = np.arange(4096) / hw.fs
    f = triangle(t, SWEEP, hw, SWEEP.f1)
    ref = (
        np.sqrt(hw.pt)
        * hw.gain
        * np.sum(
            np.abs(paths.amp)
            * np.cos(2 * np.pi * f[:, None] * paths.delay + np.angle(paths.amp)),
            axis=1,
        )
    )
    np.testing.assert_allclose(v, ref, rtol=0, atol=1e-9 * np.abs(ref).max())


def test_triangle_mirror_symmetry_and_period():
    hw = dataclasses.replace(IDEAL, fs=22_000.0, t_start=100 / 22_000.0)
    nr = 352
    assert hw.nr(SWEEP) == pytest.approx(nr)
    np.testing.assert_allclose(
        hw.turnarounds(SWEEP, 1000), [100, 100 + nr, 100 + 2 * nr]
    )
    v = synthesize_volts(Paths.from_ranges([7.0, 23.0], [1e-4, 2e-4j]), SWEEP, hw, 2000)
    tol = 1e-9 * np.abs(v).max()
    j = np.arange(1, nr)
    for n0 in (100 + nr, 100 + 2 * nr):
        np.testing.assert_allclose(v[n0 + j], v[n0 - j], atol=tol)
    np.testing.assert_allclose(v[100 + 2 * nr :], v[100 : -2 * nr], atol=tol)


def test_beat_frequency():
    sweep = Sweep(ramp_time=0.1)
    hw = dataclasses.replace(IDEAL, pll_fn=4e3, t_start=0.01)
    r = 30.0
    v = synthesize_volts(Paths.from_ranges(r, 1e-4), sweep, hw, int(0.2 * hw.fs))
    seg = v[int(0.02 * hw.fs) : int(0.11 * hw.fs)]
    pad = 2**20
    spec = np.abs(np.fft.rfft(seg * np.hanning(seg.size), pad))
    f_peak = np.argmax(spec) * hw.fs / pad
    assert f_peak == pytest.approx(
        2 * r * sweep.bandwidth / (C * sweep.ramp_time), abs=0.05
    )


def test_r_cal_is_a_delay():
    hw = Hardware(leak_amp=0.0, r_cal=0.5)
    a = synthesize_volts(Paths.from_ranges(9.0, 1e-4), SWEEP, hw, 3000)
    b = synthesize_volts(
        Paths.from_ranges(9.5, 1e-4), SWEEP, dataclasses.replace(hw, r_cal=0.0), 3000
    )
    np.testing.assert_allclose(a, b, atol=1e-9 * np.abs(a).max())


def test_leakage():
    hw = dataclasses.replace(IDEAL, leak_amp=1e-3, r_cal=0.2)
    v = synthesize_volts(Paths.from_ranges(np.empty(0), 0), SWEEP, hw, 2048)
    t = np.arange(2048) / hw.fs
    tau = 2 * (hw.leak_range + hw.r_cal) / C
    ref = (
        np.sqrt(hw.pt)
        * hw.gain
        * hw.leak_amp
        * np.cos(2 * np.pi * triangle(t, SWEEP, hw, SWEEP.f1) * tau)
    )
    np.testing.assert_allclose(v, ref, atol=1e-9)


def test_cw_moving_path_tone():
    sweep = Sweep(kind="cw")
    hw = dataclasses.replace(IDEAL, pll_fn=4e3)
    speed, a, r = -7.5, 2e-4 * np.exp(-1.1j), 12.0
    v = synthesize_volts(Paths.from_ranges(r, a, speed), sweep, hw, 4096)
    t = np.arange(4096) / hw.fs
    ref = (
        np.sqrt(hw.pt)
        * hw.gain
        * np.abs(a)
        * np.cos(2 * np.pi * sweep.f0 * (2 * r / C + 2 * speed * t / C) + np.angle(a))
    )
    live = t > hw.t_reset + 2e-3
    np.testing.assert_allclose(v[live], ref[live], atol=1e-9 * np.abs(ref).max())
    assert abs(2 * speed / sweep.lam) == pytest.approx(2 * 7.5 * sweep.f0 / C)


@pytest.mark.parametrize("speed", [3.0, 50.0, 450.0])
def test_if_filter_tone(speed):
    sweep = Sweep(kind="cw")
    hw = Hardware(leak_amp=0.0, r_cal=0.0)
    n = 8192
    v = synthesize_volts(Paths.from_ranges(5.0, 1e-4, speed), sweep, hw, n)
    t = np.arange(n) / hw.fs
    fd = 2 * speed * sweep.f0 / C
    h = bilinear_response(np.array(fd), hw)
    ref = (
        np.sqrt(hw.pt)
        * hw.gain
        * 1e-4
        * np.abs(h)
        * np.cos(2 * np.pi * sweep.f0 * 2 * (5.0 + speed * t) / C + np.angle(h))
    )
    live = t > 0.15
    np.testing.assert_allclose(v[live], ref[live], atol=1e-8 * np.abs(ref).max())


def test_if_group_delay():
    hw = Hardware()
    f = np.array([20.0, 200.0, 2000.0, 8000.0])
    df = 1e-3
    ratio = bilinear_response(f + df, hw) / bilinear_response(f - df, hw)
    gd = -np.angle(ratio) / (2 * np.pi * 2 * df)
    np.testing.assert_allclose(hw.if_group_delay(f), gd, rtol=1e-6)
    off = Hardware(hp_order=0, lp_order=0)
    np.testing.assert_array_equal(off.if_group_delay(f), 0.0)
    cal = hw.calibration(SWEEP)
    assert cal.n0 == pytest.approx(hw.fs * (hw.t_start + hw.if_group_delay(hw.fs / 4)))
    assert (cal.fs, cal.r_cal, cal.first_up) == (hw.fs, hw.r_cal, hw.first_up)
    assert cal.tx_offset == hw.tx_offset and cal.rx_offset == hw.rx_offset
    assert cal.nr(SWEEP) == hw.nr(SWEEP)


def _step_closed(tau, zeta, w):
    if zeta < 1:
        wd = w * np.sqrt(1 - zeta**2)
        return 1 - np.exp(-zeta * w * tau) * (
            np.cos(wd * tau) - zeta * w / wd * np.sin(wd * tau)
        )
    if zeta == 1:
        return 1 - np.exp(-w * tau) * (1 - w * tau)
    l1, l2 = -zeta * w + w * np.sqrt(zeta**2 - 1), -zeta * w - w * np.sqrt(zeta**2 - 1)
    return 1 - (l1 * np.exp(l1 * tau) - l2 * np.exp(l2 * tau)) / (l1 - l2)


def _ramp_error_closed(tau, zeta, w):
    if zeta < 1:
        wd = w * np.sqrt(1 - zeta**2)
        return np.exp(-zeta * w * tau) * np.sin(wd * tau) / wd
    if zeta == 1:
        return tau * np.exp(-w * tau)
    l1, l2 = -zeta * w + w * np.sqrt(zeta**2 - 1), -zeta * w - w * np.sqrt(zeta**2 - 1)
    return (np.exp(l1 * tau) - np.exp(l2 * tau)) / (l1 - l2)


@pytest.mark.parametrize("zeta", [0.3, 0.7, 1.0, 1.6])
def test_pll_step(zeta):
    hw = Hardware(t_reset=1e-3, t_start=20e-3, pll_zeta=zeta)
    t = np.linspace(0, 19e-3, 5001)
    jump = SWEEP.f0 - SWEEP.f1
    ref = SWEEP.f1 + jump * np.where(
        t > hw.t_reset, _step_closed(t - hw.t_reset, zeta, 2 * np.pi * hw.pll_fn), 0
    )
    np.testing.assert_allclose(f_tx(SWEEP, hw, t), ref, rtol=0, atol=1e-6)


@pytest.mark.parametrize(("zeta", "first_up"), [(0.7, True), (1.0, False), (2.0, True)])
def test_pll_ramp_corner_and_tracking(zeta, first_up):
    hw = Hardware(
        pll_zeta=zeta, first_up=first_up, f_prev=SWEEP.f0 if first_up else SWEEP.f1
    )
    w = 2 * np.pi * hw.pll_fn
    tau = np.linspace(0, SWEEP.ramp_time, 3001)[:-1]
    t = hw.t_start + tau
    sign = 1 if first_up else -1
    ref = triangle(t, SWEEP, hw, hw.f_prev) - sign * SWEEP.slope * _ramp_error_closed(
        tau, zeta, w
    )
    np.testing.assert_allclose(f_tx(SWEEP, hw, t), ref, rtol=0, atol=1e-5)
    mid = hw.t_start + (np.arange(1, 6) + 0.5) * SWEEP.ramp_time
    np.testing.assert_allclose(
        f_tx(SWEEP, hw, mid), f_cmd(SWEEP, hw, mid), rtol=0, atol=1e-5
    )


def test_pll_matches_first_order_hold():
    sweep = Sweep(ramp_time=2e-3)
    hw = Hardware(t_reset=0.5e-3, t_start=1e-3, f_prev=sweep.f0)
    dt = 1e-6
    t = np.arange(12001) * dt
    w = 2 * np.pi * hw.pll_fn
    pll = signal.lti([2 * hw.pll_zeta * w, w * w], [1, 2 * hw.pll_zeta * w, w * w])
    _, y, _ = signal.lsim(
        pll, triangle(t, sweep, hw, sweep.f0) - sweep.f0, t, interp=True
    )
    np.testing.assert_allclose(f_tx(sweep, hw, t) - sweep.f0, y, rtol=0, atol=1e-4)


def test_f_tx_edges():
    hw = Hardware()
    assert f_tx(SWEEP, hw, np.empty(0)).size == 0
    np.testing.assert_array_equal(f_tx(SWEEP, hw, [0.0, hw.t_reset]), SWEEP.f1)
    cw = Sweep(kind="cw")
    assert f_tx(cw, hw, [1.0])[0] == pytest.approx(cw.f0, abs=1e-6)
    assert hw.turnarounds(cw, 4096).size == 0
    assert hw.calibration(cw).first_up is None
    assert hw.turnarounds(SWEEP, 10).size == 0
    with pytest.raises(ValueError):
        Hardware(t_reset=3e-3)
    with pytest.raises(ValueError):
        Hardware(pll_zeta=0.0)


def test_quantisation_and_dc():
    hw = dataclasses.replace(IDEAL, noise=0.0)
    empty = Paths.from_ranges(np.empty(0), 0)
    np.testing.assert_array_equal(synthesize(empty, SWEEP, hw, 16, 0), 32768)
    code = synthesize(empty, SWEEP, dataclasses.replace(hw, dc=1.0), 16, 0)
    assert code.dtype == np.uint16
    np.testing.assert_array_equal(code, round(3.5 * 65535 / 5))
    paths = Paths.from_ranges(6.0, 1e-4)
    big = dataclasses.replace(hw, gain=1e6)
    v = synthesize_volts(paths, SWEEP, big, 1024)
    code = synthesize(paths, SWEEP, big, 1024, 0)
    assert v.max() > 2.5 and v.min() < -2.5
    np.testing.assert_array_equal(code, np.clip(np.rint((v + 2.5) * 13107), 0, 65535))
    assert code.max() == 65535 and code.min() == 0
    small = synthesize(paths, SWEEP, hw, 1024, 0)
    np.testing.assert_array_less(
        np.abs(volts(small) - synthesize_volts(paths, SWEEP, hw, 1024)),
        2.5 / 65535 + 1e-12,
    )


def test_noise_and_reproducibility():
    hw = dataclasses.replace(IDEAL, noise=0.01)
    paths = Paths.from_ranges(3.0, 1e-5)
    n = 4096
    a = synthesize(paths, SWEEP, hw, n, 7)
    np.testing.assert_array_equal(
        a, synthesize(paths, SWEEP, hw, n, np.random.default_rng(7))
    )
    assert np.any(a != synthesize(paths, SWEEP, hw, n, 8))
    resid = volts(a) - synthesize_volts(paths, SWEEP, hw, n)
    assert np.std(resid) == pytest.approx(0.01, rel=5 / np.sqrt(2 * n))
    assert abs(np.mean(resid)) < 5 * 0.01 / np.sqrt(n)

"""SAR phase history and backprojection imaging (signal-processing §11)."""

import dataclasses
import ideal
import numpy as np
import pytest
from scipy.optimize import brentq
from scipy.signal import get_window

from qmrdk.config import Calibration, ScanGeometry, Sweep
from qmrdk.constants import C
from qmrdk.dsp.convert import codes_to_volts
from qmrdk.dsp.sar import (
    _backproject,
    backproject,
    default_grid,
    form_image,
    phase_history,
    sharpness,
)
from qmrdk.dsp.segment import (
    extract_ramps,
    interpolation_matrix,
    ramp_length,
    ramp_positions,
)

SWEEP = Sweep()
GEOMETRY = ScanGeometry(height=1.0)
CAL = Calibration(n0=13.3, ng=16, first_up=True, r_cal=0.2)
N = 4096
TARGET = np.array([0.75, 8.0, 1.0])


def _scan(dx, count, cal=CAL, first_up=True, points=(TARGET,), amp=0.5):
    x_pos = dx * np.arange(count)
    tx, rx = GEOMETRY.antenna_positions(x_pos, cal)
    tau = ideal.bistatic_tau(np.asarray(points), tx, rx, cal.r_cal)
    x = ideal.if_signal(
        N, SWEEP, cal.fs, cal.n0, first_up, tau, np.full(tau.shape, amp)
    )
    return x_pos, ideal.to_codes(x)


@pytest.fixture(name="dense", scope="module")
def fixture_dense():
    return _scan(SWEEP.lam_min / 4, 51)


def _three_db_bins(window, n):
    w = get_window(window, n, fftbins=False)
    m = np.arange(n) - (n - 1) / 2

    def gain(f):
        return np.sum(w * np.cos(2 * np.pi * f * m)) ** 2 / w.sum() ** 2 - 0.5

    return 2 * n * brentq(gain, 1e-6, 1.0 / n * 2)


def _three_db_width(cut, step):
    p = np.abs(cut) ** 2 / np.max(np.abs(cut) ** 2)
    i = int(np.argmax(p))
    lo = i - np.argmax(p[i::-1] < 0.5)
    hi = i + np.argmax(p[i:] < 0.5)
    left = lo + (0.5 - p[lo]) / (p[lo + 1] - p[lo])
    right = hi - 1 + (p[hi - 1] - 0.5) / (p[hi - 1] - p[hi])
    return (right - left) * step


@pytest.mark.parametrize("window, aperture", [("boxcar", None), ("hann", "hann")])
def test_point_response(dense, window, aperture):
    x_pos, codes = dense
    step = 0.01
    gx = TARGET[0] + step * np.arange(-150, 151)
    gy = TARGET[1] + step * np.arange(-300, 301)
    img = form_image(
        codes,
        x_pos,
        SWEEP,
        CAL,
        GEOMETRY,
        gx,
        gy,
        background=None,
        aperture_window=aperture,
        window=window,
        zpad=16,
    )
    iy, ix = divmod(int(np.argmax(np.abs(img.image))), img.image.shape[1])
    nu = ramp_length(CAL.nr(SWEEP), CAL.ng)
    f_m = SWEEP.f0 + SWEEP.slope * (CAL.ng + (nu - 1) / 2) / CAL.fs
    length = x_pos.size * (x_pos[1] - x_pos[0])
    down = C / (2 * SWEEP.slope * nu / CAL.fs)
    cross = C / f_m * TARGET[1] / (2 * length)
    assert abs(gx[ix] - TARGET[0]) < cross / 2 and abs(gy[iy] - TARGET[1]) < down / 2
    wy = _three_db_width(img.image[:, ix], step)
    wx = _three_db_width(img.image[iy], step)
    np.testing.assert_allclose(wy, down * _three_db_bins(window, nu), rtol=0.01)
    np.testing.assert_allclose(
        wx, cross * _three_db_bins(aperture or "boxcar", x_pos.size), rtol=0.01
    )
    assert img.first_up and img.z == GEOMETRY.height


@pytest.mark.parametrize("first_up", [True, False])
@pytest.mark.parametrize("n0", [0.0, 87.3, 351.0, 500.55])
def test_ramp_direction_from_sharpness(n0, first_up):
    cal = dataclasses.replace(CAL, n0=n0, first_up=None)
    x_pos, codes = _scan(SWEEP.lam_min / 4, 51, cal, first_up)
    gx, gy = default_grid(SWEEP, x_pos, (-1.0, 2.5), (5.0, 11.0))
    img = form_image(codes, x_pos, SWEEP, cal, GEOMETRY, gx, gy)
    assert img.first_up == first_up


def _off_axis_peak(dx, count):
    x_pos, codes = _scan(dx, count)
    centre = x_pos.mean()
    gx = centre + 0.05 * np.arange(-120, 121)
    gy = 4.0 + 0.05 * np.arange(121)
    cal = dataclasses.replace(CAL, first_up=True)
    img = np.abs(
        form_image(codes, x_pos, SWEEP, cal, GEOMETRY, gx, gy, window="hann").image
    )
    angle = np.arctan2(gx[None, :] - centre, gy[:, None])
    off = np.where(np.abs(angle) > np.radians(10), img, 0)
    iy, ix = divmod(int(np.argmax(off)), off.shape[1])
    return (
        20 * np.log10(off[iy, ix] / img.max()),
        angle[iy, ix],
        x_pos[-1] - x_pos[0] + dx,
    )


def test_grating_lobes():
    nu = ramp_length(CAL.nr(SWEEP), CAL.ng)
    lam = C / (SWEEP.f0 + SWEEP.slope * (CAL.ng + (nu - 1) / 2) / CAL.fs)
    level, _, _ = _off_axis_peak(SWEEP.lam_min / 4, 51)
    assert level < -13.26
    level, angle, length = _off_axis_peak(lam, 13)
    predicted = np.arcsin(lam / (2 * lam))
    assert level > -3
    assert abs(abs(angle) - predicted) < lam / (2 * length * np.cos(predicted)) / 2


def test_background_mean_removes_constant_return(dense):
    _, codes = dense
    still = np.broadcast_to(codes[:1], codes.shape)
    ph = phase_history(still, SWEEP, CAL, True)
    assert np.abs(ph.profiles).max() < 1e-12
    assert (
        np.abs(phase_history(still, SWEEP, CAL, True, background=None).profiles).max()
        > 0.1
    )


def test_background_reference_scan(dense):
    x_pos, codes = dense
    leak = ideal.to_codes(
        np.broadcast_to(
            ideal.if_signal(
                N, SWEEP, CAL.fs, CAL.n0, True, ideal.range_tau([0.3]), [1.0]
            ),
            codes.shape,
        )
    )
    tx, rx = GEOMETRY.antenna_positions(x_pos, CAL)
    tau = ideal.bistatic_tau(TARGET[None], tx, rx, CAL.r_cal)
    combined = ideal.if_signal(
        N,
        SWEEP,
        CAL.fs,
        CAL.n0,
        True,
        np.concatenate([tau, np.full_like(tau, ideal.range_tau(0.3))], 1),
        np.tile([0.5, 1.0], (tau.shape[0], 1)),
    )
    nr = CAL.nr(SWEEP)
    ref = extract_ramps(codes_to_volts(leak), CAL.n0, nr, CAL.ng, True).mean(
        axis=(-3, -2)
    )
    got = phase_history(
        ideal.to_codes(combined), SWEEP, CAL, True, background=ref, window="boxcar"
    )
    want = phase_history(codes, SWEEP, CAL, True, background=None, window="boxcar")
    gain = (
        np.abs(interpolation_matrix(ramp_positions(CAL.n0, nr, CAL.ng, N, True), N))
        .sum(axis=1)
        .max()
    )
    q = 5.0 / 65535
    np.testing.assert_allclose(
        got.profiles, want.profiles, rtol=0, atol=2 * 1.5 * q * gain
    )
    with pytest.raises(ValueError):
        phase_history(codes, SWEEP, CAL, True, background="median")


def test_backproject_accumulates_and_matches_reference(dense):
    x_pos, codes = dense
    tx, rx = GEOMETRY.antenna_positions(x_pos, CAL)
    ph = phase_history(codes, SWEEP, CAL, True)
    gx, gy = np.linspace(0.0, 1.5, 7), np.array([-1.0, 4.0, 8.0, 400.0])
    full = backproject(ph, tx, rx, gx, gy, 1.0, CAL.r_cal, weights=np.ones(len(tx)))
    half = slice(0, 20), slice(20, None)
    parts = [dataclasses.replace(ph, profiles=ph.profiles[s]) for s in half]
    acc = backproject(parts[0], tx[half[0]], rx[half[0]], gx, gy, 1.0, CAL.r_cal)
    backproject(parts[1], tx[half[1]], rx[half[1]], gx, gy, 1.0, CAL.r_cal, out=acc)
    np.testing.assert_allclose(acc, full, rtol=1e-12, atol=1e-12)
    ref = np.zeros_like(full)
    r = ph.r_axis
    _backproject.py_func(
        ph.profiles,
        r[0],
        r[1] - r[0],
        tx,
        rx,
        gx,
        gy,
        1.0,
        CAL.r_cal,
        4 * np.pi * ph.f_m / C,
        np.ones(len(tx)),
        ref,
    )
    np.testing.assert_allclose(full, ref, rtol=1e-10, atol=1e-12)
    assert np.all(full[-1] == 0) and np.all(full[:-1] != 0)


def test_phase_history_rejects_short_frames():
    with pytest.raises(ValueError):
        phase_history(np.zeros((2, 300), np.uint16), SWEEP, CAL, True)


def test_sharpness():
    assert sharpness(np.eye(1)) == 1.0
    np.testing.assert_allclose(sharpness(np.ones((4, 5))), 1 / 20)


def test_default_grid():
    x_pos = np.linspace(0, 1.5, 51)
    gx, gy = default_grid(SWEEP, x_pos, (-1.0, 1.0), (2.0, 10.0), r_ref=6.0)
    np.testing.assert_allclose(np.diff(gy), C / (4 * SWEEP.bandwidth))
    np.testing.assert_allclose(np.diff(gx), SWEEP.lam * 6.0 / (4 * 1.5))
    assert gx[0] == -1.0 and gx[-1] <= 1.0 and gy[0] == 2.0 and gy[-1] <= 10.0
    assert np.array_equal(default_grid(SWEEP, x_pos, (-1.0, 1.0), (2.0, 10.0))[0], gx)

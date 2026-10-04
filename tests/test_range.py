"""Range profile, level calibration and vendor spectrum (signal-processing §5)."""

import dataclasses

import ideal
import numpy as np
import pytest
from scipy.fft import next_fast_len

from qmrdk.config import Calibration, Sweep
from qmrdk.constants import C
from qmrdk.dsp.convert import codes_to_volts
from qmrdk.dsp.range import (
    centre_frequency,
    range_axis,
    range_profile,
    range_spectrum,
    vendor_spectrum,
)
from qmrdk.dsp.segment import INTERP_ATTEN_DB, extract_ramps, ramp_length

SWEEP = Sweep()
N = 4096
WINDOWS = [
    "boxcar",
    "hann",
    "hamming",
    "blackman",
    "blackmanharris",
    "flattop",
    ("kaiser", 8.0),
]


def _peak(level, r):
    i = int(np.argmax(level))
    a, b, c = level[i - 1 : i + 2]
    return r[i] + 0.5 * (a - c) / (a - 2 * b + c) * (r[1] - r[0])


@pytest.mark.parametrize("window", WINDOWS)
def test_level_full_scale_bin_centred_tone(window):
    nu, k0 = 320, 37
    m = np.arange(nu)
    spec = range_spectrum(2.5 * np.cos(2 * np.pi * k0 * m / nu + 0.7), window, zpad=4)
    assert spec.shape == (nu * 4 // 2 + 1,)
    assert abs(20 * np.log10(np.abs(spec[4 * k0]) / 2.5)) < 1e-3


def test_range_profile_full_scale_level():
    cal = Calibration(fs=352.5 / SWEEP.ramp_time, n0=21.4, ng=16)
    nu = ramp_length(cal.nr(SWEEP), cal.ng)
    assert next_fast_len(4 * nu, real=True) == 4 * nu
    f_b = 40 * cal.fs / nu
    tau = f_b / SWEEP.slope
    codes = ideal.to_codes(
        ideal.if_signal(N, SWEEP, cal.fs, cal.n0, True, [tau], [2.5])
    )
    _, level = range_profile(codes, SWEEP, cal)
    assert abs(level.max()) < 1e-3


def test_centre_frequency():
    cal = Calibration(ng=7)
    assert centre_frequency(SWEEP, cal, 101) == SWEEP.f0 + SWEEP.slope * 57 / cal.fs


def test_range_axis():
    cal = Calibration(fs=20_000.0)
    r = range_axis(1000, SWEEP, cal)
    assert r.size == 501
    np.testing.assert_allclose(r[1], 20.0 * C * SWEEP.ramp_time / (2 * SWEEP.bandwidth))


@pytest.mark.parametrize("coherent", [True, False])
@pytest.mark.parametrize("first_up", [True, False])
@pytest.mark.parametrize("n0", [3.7, 250.1])
def test_range_peak_within_one_bin(n0, first_up, coherent):
    cal = Calibration(n0=n0, ng=16, first_up=first_up, r_cal=0.4)
    r_true = 9.3
    x = ideal.if_signal(
        N, SWEEP, cal.fs, n0, first_up, ideal.range_tau([r_true], cal.r_cal), [0.3]
    )
    r, level = range_profile(ideal.to_codes(x), SWEEP, cal, coherent=coherent)
    assert abs(_peak(level, r) - r_true) < r[1] - r[0]


def test_range_profile_batched_and_noise_averaging():
    cal = Calibration(n0=0.0, ng=16, first_up=True)
    x = ideal.if_signal(
        N, SWEEP, cal.fs, 0.0, True, ideal.range_tau([5.0]), [0.1], noise=0.05
    )
    codes = ideal.to_codes(np.stack([x, x]))
    _, coh = range_profile(codes, SWEEP, cal, coherent=True)
    _, inc = range_profile(codes, SWEEP, cal, coherent=False)
    assert coh.shape == inc.shape and coh.shape[0] == 2
    assert np.median(coh) < np.median(inc) - 3


def test_range_profile_rejects_short_frame():
    with pytest.raises(ValueError):
        range_profile(np.zeros(300, np.uint16), SWEEP, Calibration())


@pytest.mark.parametrize("separation, resolved", [(2.0, True), (0.5, False)])
def test_two_point_resolution(separation, resolved):
    cal = Calibration(n0=11.2, ng=0, first_up=True)
    nu = ramp_length(cal.nr(SWEEP), cal.ng)
    f_m = SWEEP.f0 + SWEEP.slope * (nu - 1) / 2 / cal.fs
    cell = C / (2 * SWEEP.bandwidth)
    rr = np.array([6.0, 6.0 + separation * cell])
    theta = -4 * np.pi * f_m * rr / C
    x = ideal.if_signal(
        N, SWEEP, cal.fs, cal.n0, True, ideal.range_tau(rr), [0.5, 0.5], theta
    )
    r, level = range_profile(ideal.to_codes(x), SWEEP, cal, window="boxcar", zpad=16)
    seg = level[(r >= rr[0] - cell / 4) & (r <= rr[1] + cell / 4)]
    maxima = np.sum((seg[1:-1] > seg[:-2]) & (seg[1:-1] > seg[2:]))
    assert maxima == (2 if resolved else 1)


@pytest.mark.parametrize("first_up", [True, False])
@pytest.mark.parametrize("n0", [0.0, 123.45])
def test_peak_phase_referenced_to_ramp_centre(n0, first_up):
    fs, ng, r_true = 21_977.0, 16, 23.0
    nr = SWEEP.ramp_time * fs
    nu = ramp_length(nr, ng)
    x = ideal.if_signal(N, SWEEP, fs, n0, first_up, ideal.range_tau([r_true]), [1.0])
    tol = 10 * 10 ** (-INTERP_ATTEN_DB / 20)
    for up, f_c, sign in (
        (first_up, SWEEP.f0 + SWEEP.slope * (ng + (nu - 1) / 2) / fs, 1),
        (not first_up, SWEEP.f0 + SWEEP.slope * (nr - ng - (nu - 1) / 2) / fs, -1),
    ):
        spec = range_spectrum(extract_ramps(x, n0, nr, ng, up))
        nfft = next_fast_len(nu * 4, real=True)
        k = int(round(SWEEP.slope * 2 * r_true / C * nfft / fs))
        expected = np.exp(1j * sign * 4 * np.pi * f_c * r_true / C)
        for d in (-1, 0, 1):
            np.testing.assert_allclose(
                spec[..., k + d] / np.abs(spec[..., k + d]), expected, atol=tol
            )


def test_vendor_spectrum_matches_direct_formula():
    rng = np.random.default_rng(1)
    codes = rng.integers(0, 65536, (2, 48)).astype(np.uint16)
    fs = 21_977.0
    r, level = vendor_spectrum(codes, SWEEP, fs)
    nfft = 7 * 48
    k = np.arange(nfft // 2 + 1)
    dft = codes_to_volts(codes) @ np.exp(
        -2j * np.pi * np.outer(np.arange(48), k) / nfft
    )
    np.testing.assert_allclose(level, 20 * np.log10(np.abs(dft) / nfft), atol=1e-9)
    np.testing.assert_allclose(
        r, k * fs / nfft * C * SWEEP.ramp_time / (2 * SWEEP.bandwidth)
    )


def test_r_cal_shifts_axis():
    cal = Calibration(n0=5.0, ng=16, first_up=True)
    codes = ideal.to_codes(
        ideal.if_signal(N, SWEEP, cal.fs, 5.0, True, ideal.range_tau([8.0]), [0.2])
    )
    r0, l0 = range_profile(codes, SWEEP, cal)
    r1, l1 = range_profile(codes, SWEEP, dataclasses.replace(cal, r_cal=1.25))
    np.testing.assert_allclose(r0 - r1, 1.25)
    np.testing.assert_array_equal(l0, l1)

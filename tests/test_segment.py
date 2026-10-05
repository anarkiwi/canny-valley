"""Sweep segmentation and ramp extraction (signal-processing §4)."""

import ideal
import numpy as np
import pytest

from qmrdk.config import Sweep
from qmrdk.dsp.segment import (
    INTERP_ATTEN_DB,
    INTERP_HALF_WIDTH,
    extract_ramps,
    mirror_turnaround,
    ramp_length,
    ramp_positions,
    turnaround_positions,
)

FS = 21_977.0
SWEEP = Sweep()
NR = SWEEP.ramp_time * FS
N = 4096
TAU = ideal.range_tau([3.0, 7.5, 12.0])
AMP = np.array([1.0, 0.5, 0.3])
THETA = np.array([0.3, 1.1, 2.0])
DESIGN_ERROR = 10 ** (-INTERP_ATTEN_DB / 20)


def _circular_error(pos, true, nr):
    return np.abs(np.mod(pos - true + 0.5 * nr, nr) - 0.5 * nr)


def test_turnaround_positions():
    np.testing.assert_allclose(
        turnaround_positions(-100.0, 350.5, 1000), [250.5, 601.0, 951.5]
    )
    np.testing.assert_allclose(
        turnaround_positions(1000.2, 350.5, 1000), [299.2, 649.7]
    )


@pytest.mark.parametrize("first_up", [True, False])
def test_mirror_turnaround_fractional_period(first_up):
    offsets = np.linspace(0.0, 2 * NR, 157, endpoint=False)
    x = np.stack(
        [ideal.if_signal(N, SWEEP, FS, o, first_up, TAU, AMP, THETA) for o in offsets]
    )
    pos, quality = mirror_turnaround(x, NR)
    assert pos.shape == offsets.shape
    assert np.all((pos >= 0) & (pos < NR))
    assert _circular_error(pos, offsets, NR).max() < 0.5
    assert np.all(quality > 0.99)


def test_mirror_turnaround_exact_on_lag_grid():
    nr = 350.5
    sweep = Sweep(ramp_time=nr / FS)
    offsets = np.arange(int(2 * nr)) / 2
    x = np.stack(
        [ideal.if_signal(N, sweep, FS, o, True, TAU, AMP, THETA) for o in offsets]
    )
    pos, quality = mirror_turnaround(x[None], nr)
    assert _circular_error(pos.ravel(), offsets, nr).max() < 0.5
    np.testing.assert_allclose(quality, 1.0, atol=1e-9)


def test_mirror_turnaround_noise_lowers_quality():
    x = ideal.if_signal(N, SWEEP, FS, 123.4, True, TAU, AMP, THETA, noise=0.5)
    pos, quality = mirror_turnaround(x, NR)
    assert _circular_error(pos, 123.4, NR) < 0.5
    assert quality < 0.9


def test_mirror_turnaround_needs_two_ramps():
    with pytest.raises(ValueError):
        mirror_turnaround(np.ones(500), NR)


@pytest.mark.parametrize("first_up", [True, False])
@pytest.mark.parametrize("n0", [-500.3, 0.0, 10.7, 200.25, 2 * NR + 3.9])
def test_ramp_positions_index_increasing_frequency(n0, first_up):
    ng = 5
    pos = ramp_positions(n0, NR, ng, N, first_up)
    nu = ramp_length(NR, ng)
    assert pos.shape[0] == 2 and pos.shape[1] >= 4 and pos.shape[2] == nu
    m = np.arange(nu)
    f = ideal.sweep_frequency(pos, SWEEP, FS, n0, first_up)
    np.testing.assert_allclose(
        f, np.broadcast_to(SWEEP.f0 + SWEEP.slope * (ng + m) / FS, f.shape), rtol=1e-12
    )
    np.testing.assert_allclose(
        np.diff(pos, axis=-1),
        np.broadcast_to([[[1.0]], [[-1.0]]], (2,) + pos.shape[1:2] + (nu - 1,)),
    )
    assert pos.min() >= INTERP_HALF_WIDTH - 1 and pos.max() <= N - 1 - INTERP_HALF_WIDTH
    nk = turnaround_positions(n0, NR, N)
    starts = np.concatenate([nk, nk[:1] - NR])
    assert np.isclose(pos[0, :, 0, None] - ng, starts).any(axis=1).all()
    assert np.isclose(pos[1, :, 0, None] + ng, starts + NR).any(axis=1).all()


@pytest.mark.parametrize("f", [0.01, 0.1, 0.25])
@pytest.mark.parametrize("first_up", [True, False])
def test_extract_ramps_tone_at_fractional_positions(f, first_up):
    n0 = 37.31
    x = np.cos(2 * np.pi * f * np.arange(N) + 0.3)
    ramps = extract_ramps(np.stack([x, -x]), n0, NR, 5, first_up)
    pos = ramp_positions(n0, NR, 5, N, first_up)
    expected = np.cos(2 * np.pi * f * pos + 0.3)
    assert ramps.shape == (2,) + pos.shape
    np.testing.assert_allclose(ramps[0], expected, atol=DESIGN_ERROR)
    np.testing.assert_allclose(ramps[1], -expected, atol=DESIGN_ERROR)


@pytest.mark.parametrize("first_up", [True, False])
@pytest.mark.parametrize("n0", [0.0, 37.3, 400.81])
def test_extract_ramps_sample_sweep_frequency_grid(n0, first_up):
    ng = INTERP_HALF_WIDTH
    x = ideal.if_signal(N, SWEEP, FS, n0, first_up, TAU, AMP, THETA)
    ramps = extract_ramps(x, n0, NR, ng, first_up)
    f = SWEEP.f0 + SWEEP.slope * (ng + np.arange(ramp_length(NR, ng))) / FS
    expected = np.sum(
        AMP[:, None] * np.cos(2 * np.pi * f * TAU[:, None] + THETA[:, None]), 0
    )
    np.testing.assert_allclose(
        ramps, np.broadcast_to(expected, ramps.shape), atol=DESIGN_ERROR * AMP.sum()
    )


def test_extract_ramps_short_frame_is_empty():
    assert extract_ramps(np.zeros(400), 0.0, NR, 0, True).shape == (
        2,
        0,
        ramp_length(NR, 0),
    )

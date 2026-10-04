"""Closed-form IF source for tests (signal-processing §3)."""

import numpy as np

from qmrdk.constants import A_FS, ADC_MAX, C


def sweep_frequency(pos, sweep, fs, n0, first_up):
    """Transmit frequency at fractional sample positions of a triangle sweep
    whose turnaround `n0` begins a rising ramp iff `first_up`."""
    u = (np.asarray(pos, dtype=np.float64) - n0) / (sweep.ramp_time * fs)
    k = np.floor(u)
    frac = u - k
    up = (np.mod(k, 2) == 0) == bool(first_up)
    return np.where(
        up, sweep.f0 + sweep.bandwidth * frac, sweep.f1 - sweep.bandwidth * frac
    )


def if_signal(n, sweep, fs, n0, first_up, tau, amp, theta=0.0, noise=0.0, rng=None):
    """IF volts `[..., n]`: sum over the last axis of `tau`, `amp`, `theta` of
    `amp * cos(2 pi f_tx(t) tau + theta)` plus white noise of std `noise`."""
    f = sweep_frequency(np.arange(n), sweep, fs, n0, first_up)
    tau, amp, theta = (
        np.asarray(v, dtype=np.float64)[..., None] for v in (tau, amp, theta)
    )
    x = np.sum(amp * np.cos(2 * np.pi * f * tau + theta), axis=-2)
    if noise:
        x = x + (rng or np.random.default_rng(0)).normal(0.0, noise, x.shape)
    return x


def to_codes(x):
    """Volts to 16-bit codes as the device returns them."""
    return np.clip(np.rint((x + A_FS) * ADC_MAX / (2 * A_FS)), 0, ADC_MAX).astype(
        np.uint16
    )


def bistatic_tau(points, tx, rx, r_cal=0.0):
    """Delays `[P, S]` of scatterers `points[S, 3]` seen from antennas `[P, 3]`."""
    points = np.asarray(points, dtype=np.float64)
    d = np.linalg.norm(points - tx[:, None], axis=-1) + np.linalg.norm(
        points - rx[:, None], axis=-1
    )
    return (d + 2 * r_cal) / C


def range_tau(r, r_cal=0.0):
    """Monostatic delay of a scatterer at range `r`."""
    return 2 * (np.asarray(r, dtype=np.float64) + r_cal) / C

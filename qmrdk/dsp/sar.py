"""Rail SAR phase history and backprojection imaging (signal-processing §11)."""

import dataclasses
import math

import numba
import numpy as np
from scipy.fft import next_fast_len
from scipy.signal import get_window

from qmrdk.constants import C
from qmrdk.dsp.convert import as_volts
from qmrdk.dsp.range import centre_frequency, range_axis, range_spectrum
from qmrdk.dsp.segment import extract_ramps


@dataclasses.dataclass
class PhaseHistory:
    """Complex range profiles `[P, K]` on the raw range axis `r_axis[K]`,
    phase-referenced to the sweep frequency `f_m`."""

    profiles: np.ndarray
    r_axis: np.ndarray
    f_m: float
    first_up: bool


@dataclasses.dataclass
class SarImage:
    """Complex image `[ny, nx]` on grid `gx`, `gy` in the plane `z`."""

    image: np.ndarray
    gx: np.ndarray
    gy: np.ndarray
    z: float
    first_up: bool
    sharpness: float


def phase_history(
    codes, sweep, cal, first_up, background="mean", window="hann", zpad=4
):
    """Per-position complex range profiles of frames `codes[P, N]` (§11.2),
    unsigned ADC codes or volts (e.g. a scan minus a reference scan).

    `background`: "mean" subtracts the mean ramp over positions, an array
    (mean ramp `[Nu]` or `[P, Nu]` of an empty-scene scan) is subtracted, None
    leaves the ramps unchanged.
    """
    ramps = extract_ramps(as_volts(codes), cal.n0, cal.nr(sweep), cal.ng, first_up)
    if ramps.shape[-2] == 0:
        raise ValueError("frames hold no complete ramp of each direction")
    mean = ramps.mean(axis=(-3, -2))
    if isinstance(background, str):
        if background != "mean":
            raise ValueError(f"unknown background {background!r}")
        mean = mean - mean.mean(axis=0)
    elif background is not None:
        mean = mean - np.asarray(background, dtype=np.float64)
    nu = mean.shape[-1]
    profiles = range_spectrum(mean, window, zpad)
    nfft = next_fast_len(nu * zpad, real=True)
    return PhaseHistory(
        profiles,
        range_axis(nfft, sweep, cal),
        centre_frequency(sweep, cal, nu),
        bool(first_up),
    )


@numba.njit(parallel=True, cache=True)
def _backproject(profiles, r0, dr, tx, rx, gx, gy, z, r_cal, kr, weights, out):
    ny, nx = out.shape
    npos, nk = profiles.shape
    for q in numba.prange(ny * nx):  # pylint: disable=not-an-iterable
        iy = q // nx
        ix = q - iy * nx
        px = gx[ix]
        py = gy[iy]
        acc = 0j
        for n in range(npos):
            dt = math.sqrt(
                (px - tx[n, 0]) ** 2 + (py - tx[n, 1]) ** 2 + (z - tx[n, 2]) ** 2
            )
            dv = math.sqrt(
                (px - rx[n, 0]) ** 2 + (py - rx[n, 1]) ** 2 + (z - rx[n, 2]) ** 2
            )
            rn = 0.5 * (dt + dv) + r_cal
            u = (rn - r0) / dr
            if u < 0.0 or u >= nk - 1:
                continue
            i = int(u)
            f = u - i
            v = profiles[n, i] * (1.0 - f) + profiles[n, i + 1] * f
            ph = kr * rn
            acc += weights[n] * v * complex(math.cos(ph), -math.sin(ph))
        out[iy, ix] += acc


def backproject(ph, tx, rx, gx, gy, z, r_cal, weights=None, out=None):
    """Backprojection image `[ny, nx]` (§11.3) accumulated into `out`.

    `tx`, `rx`: antenna phase centres `[P, 3]`; `weights`: aperture weights `[P]`.
    """
    profiles = np.ascontiguousarray(ph.profiles, dtype=np.complex128)
    gx = np.ascontiguousarray(gx, dtype=np.float64)
    gy = np.ascontiguousarray(gy, dtype=np.float64)
    if out is None:
        out = np.zeros((gy.size, gx.size), dtype=np.complex128)
    weights = (
        np.ones(profiles.shape[0])
        if weights is None
        else np.asarray(weights, np.float64)
    )
    r = np.asarray(ph.r_axis, dtype=np.float64)
    _backproject(
        profiles,
        r[0],
        r[1] - r[0],
        np.ascontiguousarray(tx, dtype=np.float64),
        np.ascontiguousarray(rx, dtype=np.float64),
        gx,
        gy,
        float(z),
        float(r_cal),
        4.0 * np.pi * ph.f_m / C,
        np.ascontiguousarray(weights),
        out,
    )
    return out


def aperture_weights(window, n):
    """Weights `[n]` of the scipy window `window` over positions; ones for
    None or "none"."""
    if window is None or window == "none":
        return np.ones(n)
    return get_window(window, n, fftbins=False)


def sharpness(img):
    """Image sharpness `sum |I|^4 / (sum |I|^2)^2`."""
    p = np.abs(img) ** 2
    return float(np.sum(p * p) / np.sum(p) ** 2)


def form_image(
    codes,
    x_pos,
    sweep,
    cal,
    geometry,
    gx,
    gy,
    z=None,
    background="mean",
    aperture_window="hann",
    window="hann",
    zpad=4,
):
    """Backprojection image of a rail scan `codes[P, N]` (codes or volts) at
    rail positions `x_pos`, weighted by `aperture_weights(aperture_window)`.

    With `cal.first_up` unknown both ramp directions are imaged and the
    sharper image is kept (§11.2).
    """
    tx, rx = geometry.antenna_positions(x_pos, cal)
    z = geometry.height if z is None else z
    weights = aperture_weights(aperture_window, len(tx))
    best = None
    for up in (True, False) if cal.first_up is None else (cal.first_up,):
        ph = phase_history(codes, sweep, cal, up, background, window, zpad)
        img = backproject(ph, tx, rx, gx, gy, z, cal.r_cal, weights)
        s = sharpness(img)
        if best is None or s > best.sharpness:
            best = SarImage(img, np.asarray(gx), np.asarray(gy), z, up, s)
    return best


def default_grid(sweep, x_pos, x_range, y_range, r_ref=None):
    """Grid axes `(gx, gy)` spaced at half the down-range resolution `c / 2B`
    and half the cross-range resolution `lam * r_ref / 2L` (default `r_ref`:
    centre of `y_range`)."""
    r_ref = 0.5 * (y_range[0] + y_range[1]) if r_ref is None else r_ref
    dy = 0.25 * C / sweep.bandwidth
    dx = 0.25 * sweep.lam * r_ref / np.ptp(np.asarray(x_pos, dtype=np.float64))
    return _axis(x_range, dx), _axis(y_range, dy)


def _axis(span, step):
    return span[0] + step * np.arange(int(np.floor((span[1] - span[0]) / step)) + 1)

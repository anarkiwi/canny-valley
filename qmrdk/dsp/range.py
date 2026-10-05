"""Range profiles and the vendor-equivalent spectrum (signal-processing §5)."""

import numpy as np
from scipy.fft import next_fast_len, rfft
from scipy.signal import get_window

from qmrdk.constants import C
from qmrdk.dsp.convert import codes_to_volts, dbfs
from qmrdk.dsp.segment import extract_ramps, ramp_length


def range_spectrum(ramps, window="hann", zpad=4):
    """Complex positive-half spectra `[..., Nfft // 2 + 1]` of ramps `[..., Nu]`.

    |X| is the tone amplitude in volts; phase is referenced to the ramp centre
    (§11.2 step 4), so a window kernel is real about each peak.
    """
    ramps = np.asarray(ramps, dtype=np.float64)
    nu = ramps.shape[-1]
    w = get_window(window, nu, fftbins=False)
    nfft = next_fast_len(nu * zpad, real=True)
    spec = rfft((ramps - ramps.mean(axis=-1, keepdims=True)) * w, nfft)
    k = np.arange(nfft // 2 + 1)
    return spec * (2.0 / w.sum() * np.exp(1j * np.pi * k * (nu - 1) / nfft))


def _bin_range(nfft, fs, sweep):
    return (
        np.arange(nfft // 2 + 1)
        * (fs / nfft)
        * C
        * sweep.ramp_time
        / (2.0 * sweep.bandwidth)
    )


def range_axis(nfft, sweep, cal):
    """Raw apparent range of bins `0 .. nfft // 2`, without the `r_cal` correction."""
    return _bin_range(nfft, cal.fs, sweep)


def centre_frequency(sweep, cal, nu):
    """Sweep frequency `f_m` at the centre of the used part of a ramp."""
    return sweep.f0 + sweep.slope * (cal.ng + 0.5 * (nu - 1)) / cal.fs


def _ramps(codes, sweep, cal, first_up):
    x = codes_to_volts(codes)
    ramps = extract_ramps(x, cal.n0, cal.nr(sweep), cal.ng, first_up)
    if ramps.shape[-2] == 0:
        raise ValueError("frame holds no complete ramp of each direction")
    return ramps


def range_profile(codes, sweep, cal, coherent=True, window="hann", zpad=4):
    """Range profile `(r, level_dbfs)` of frames `codes[..., N]` (§5).

    Coherent: complex mean over the ramps of each set, then the mean of the
    two sets' powers. Noncoherent: mean power over all ramps.
    """
    first_up = True if cal.first_up is None else cal.first_up
    spec = range_spectrum(_ramps(codes, sweep, cal, first_up), window, zpad)
    if coherent:
        power = np.mean(np.abs(spec.mean(axis=-2)) ** 2, axis=-2)
    else:
        power = np.mean(np.abs(spec) ** 2, axis=(-3, -2))
    nfft = next_fast_len(ramp_length(cal.nr(sweep), cal.ng) * zpad, real=True)
    return range_axis(nfft, sweep, cal) - cal.r_cal, dbfs(np.sqrt(power))


def vendor_spectrum(codes, sweep, fs):
    """Vendor-equivalent unsegmented spectrum `(r, level)` (§5.1)."""
    x = codes_to_volts(codes)
    nfft = 7 * x.shape[-1]
    with np.errstate(divide="ignore"):
        level = 20.0 * np.log10(np.abs(rfft(x, nfft)) / nfft)
    return _bin_range(nfft, fs, sweep), level

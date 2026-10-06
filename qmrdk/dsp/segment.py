"""Sweep segmentation and fractional ramp extraction (signal-processing §4)."""

import numpy as np
import scipy.sparse
from scipy.fft import irfft, next_fast_len, rfft
from scipy.signal import kaiser_beta

INTERP_HALF_WIDTH = 16
INTERP_ATTEN_DB = 80.0
_BETA = kaiser_beta(INTERP_ATTEN_DB)


def mirror_turnaround(x, nr):
    """Mirror-symmetry turnaround estimate (§4.2), batched over leading axes.

    Returns `(pos, quality)`: the first turnaround position in `[0, nr)` and
    the folded normalised mirror correlation `J` at its maximum.
    """
    x = np.asarray(x, dtype=np.float64)
    x = x - x.mean(axis=-1, keepdims=True)
    n = x.shape[-1]
    if n < 2 * nr:
        raise ValueError("mirror_turnaround needs at least 2 * nr samples")
    nlag = 2 * n - 1
    size = next_fast_len(nlag, real=True)
    period = 2.0 * nr
    shift = period * np.arange(int((nlag - 1) // period) + 1)
    base = np.floor(shift).astype(np.int64)
    frac = shift - base
    spec = rfft(x, size) ** 2
    k = np.arange(size // 2 + 1)
    conv = irfft(
        spec[..., None, :] * np.exp(2j * np.pi * frac[:, None] * k / size), size
    )

    p = np.arange(-1, int(np.ceil(period)) + 1)
    lag = p + shift[:, None]
    idx = p + base[:, None]
    valid = np.minimum(lag + 1, nlag - lag) >= nr
    idx = np.clip(idx, 0, nlag - 2)

    cs = np.concatenate([np.zeros(x.shape[:-1] + (1,)), np.cumsum(x * x, axis=-1)], -1)
    m = np.arange(nlag)
    energy = cs[..., np.minimum(m, n - 1) + 1] - cs[..., np.maximum(0, m - n + 1)]
    w = frac[:, None]
    energy = (1.0 - w) * energy[..., idx] + w * energy[..., idx + 1]
    a = np.take_along_axis(conv, np.broadcast_to(idx, x.shape[:-1] + idx.shape), -1)
    rho = np.divide(a, energy, out=np.zeros_like(a), where=valid & (energy > 0))
    fold = rho.sum(axis=-2) / valid.sum(axis=0)

    i = np.argmax(fold[..., 1:-1], axis=-1)[..., None] + 1
    jm, j0, jp = (np.take_along_axis(fold, i + d, -1)[..., 0] for d in (-1, 0, 1))
    curv = jm - 2.0 * j0 + jp
    delta = np.divide(0.5 * (jm - jp), curv, out=np.zeros_like(curv), where=curv < 0)
    pos = np.mod(0.5 * (p[i[..., 0]] + delta), nr)
    return pos, j0


def mirror_correlation(x, centres, offsets):
    """Normalised correlation of `x[c + d]` with `x[c - d]` over `offsets` d,
    for each integer centre c; each side has its mean removed."""
    x = np.asarray(x, dtype=np.float64)
    c = np.asarray(centres, dtype=np.int64)[:, None]
    a, b = x[c + offsets], x[c - offsets]
    a = a - a.mean(axis=-1, keepdims=True)
    b = b - b.mean(axis=-1, keepdims=True)
    den = np.sqrt(np.sum(a * a, axis=-1) * np.sum(b * b, axis=-1))
    num = np.sum(a * b, axis=-1)
    return np.divide(num, den, out=np.zeros_like(num), where=den > 0)


def turnaround_positions(n0, nr, n):
    """Turnaround positions `n0 + k * nr` lying in `[0, n)`."""
    first = n0 + np.ceil(-n0 / nr) * nr
    return first + nr * np.arange(int(np.ceil((n - first) / nr)))


def ramp_length(nr, ng):
    """Usable samples per ramp, `floor(nr - 2 * ng)`."""
    return int(np.floor(nr - 2 * ng))


def ramp_positions(n0, nr, ng, n, first_up):
    """Fractional sample positions `[2, M, Nu]` of the ramps that
    `extract_ramps` takes from an `n`-sample frame."""
    k0 = np.ceil(-n0 / nr)
    start = n0 + k0 * nr
    up0 = bool(first_up) != bool(int(k0) % 2)
    nu = ramp_length(nr, ng)
    k = np.arange(-1, int(np.ceil(n / nr)) + 1)
    nk = start + nr * k
    m = np.arange(nu)
    up = (k % 2 == 0) == up0
    pos = np.where(up[:, None], nk[:, None] + ng + m, nk[:, None] + nr - ng - m)
    lo = np.floor(pos.min(axis=1)) - INTERP_HALF_WIDTH + 1
    hi = np.floor(pos.max(axis=1)) + INTERP_HALF_WIDTH
    ok = (lo >= 0) & (hi <= n - 1)
    sets = pos[up & ok], pos[~up & ok]
    count = min(len(s) for s in sets)
    return np.stack([s[:count] for s in sets])


def interpolation_matrix(pos, n):
    """Sparse `[pos.size, n]` Kaiser-windowed-sinc interpolator evaluating a
    frame at fractional positions `pos`."""
    pos = np.ravel(pos)
    taps = np.arange(1 - INTERP_HALF_WIDTH, INTERP_HALF_WIDTH + 1)
    cols = np.floor(pos).astype(np.int64)[:, None] + taps
    d = pos[:, None] - cols
    arg = np.sqrt(np.clip(1.0 - (d / INTERP_HALF_WIDTH) ** 2, 0.0, None))
    w = np.sinc(d) * np.i0(_BETA * arg) / np.i0(_BETA)
    indptr = np.arange(pos.size + 1) * taps.size
    return scipy.sparse.csr_matrix(
        (w.ravel(), cols.ravel(), indptr), shape=(pos.size, n)
    )


def extract_ramps(x, n0, nr, ng, first_up):
    """Ramps `[..., 2, M, Nu]` of frames `x[..., N]` (§4.3).

    Set 0: rising-frequency ramps; set 1: falling ones time-reversed. Both are
    indexed by increasing frequency from `f0 + mu * ng / fs`. `first_up`
    labels the ramp beginning at the turnaround `n0`.
    """
    x = np.asarray(x, dtype=np.float64)
    n = x.shape[-1]
    pos = ramp_positions(n0, nr, ng, n, first_up)
    out = interpolation_matrix(pos, n) @ x.reshape(-1, n).T
    return out.T.reshape(x.shape[:-1] + pos.shape)

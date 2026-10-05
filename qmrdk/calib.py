"""Board calibration procedure (docs/calibration.md): estimators, drivers, CLI."""

# pylint: disable=too-many-lines

import dataclasses
import json
import math
import pathlib
import sys
import time

import numpy as np
from scipy import ndimage
from scipy.fft import next_fast_len, rfft
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.signal import butter, find_peaks, get_window, sosfiltfilt

from qmrdk.config import Calibration, ScanGeometry, Sweep
from qmrdk.constants import C, FS_NOMINAL
from qmrdk.dsp.convert import codes_to_volts
from qmrdk.dsp.range import range_spectrum
from qmrdk.dsp.sar import PhaseHistory, backproject, phase_history
from qmrdk.dsp.segment import (
    INTERP_HALF_WIDTH,
    extract_ramps,
    interpolation_matrix,
    mirror_turnaround,
    ramp_length,
    ramp_positions,
    turnaround_positions,
)
from qmrdk.recording import Recording
from qmrdk.scan import frames as capture_frames
from qmrdk.scan import run_scan, scan_positions

LOCK_RMS = 0.1
EQ_TAPS = 8
HP_ORDER = 4
P_FALSE = 1e-3
TARGET_RANGE = 8.0
TARGET_ANGLE = 25.0


def _floats(d):
    """JSON-ready copy of a dict: numpy scalars and arrays to Python types."""
    return json.loads(json.dumps(d, default=lambda v: np.asarray(v).tolist()))


def window_lobes(window, n, over=64):
    """Main-lobe half width (bins), −3 dB full width (bins) and peak sidelobe
    level (dB) of the symmetric window of length `n`."""
    w = get_window(window, n, fftbins=False)
    s = np.abs(rfft(w, over * n))
    null = int(np.argmax(np.diff(s) > 0))
    i = int(np.argmax(s < s[0] / math.sqrt(2.0)))
    half = i - 1 + (s[i - 1] - s[0] / math.sqrt(2.0)) / (s[i - 1] - s[i])
    psl = -np.inf if null == 0 else 20.0 * np.log10(s[null:].max() / s[0])
    return null / over, 2.0 * half / over, float(psl)


def _volts(frames):
    """Frames `[K, N]` in volts; a pair of frame sets `[2, K, N]` is differenced."""
    x = np.asarray(frames)
    x = codes_to_volts(x) if x.dtype.kind == "u" else x.astype(np.float64)
    return x[0] - x[1] if x.ndim == 3 else np.atleast_2d(x)


def _highpass(x, cutoff, order=HP_ORDER):
    """Zero-phase (forward-backward Butterworth) high-pass at `cutoff`
    cycles/sample; mirror points are unchanged."""
    sos = butter(order, 2.0 * cutoff, "highpass", output="sos")
    return sosfiltfilt(sos, x - x.mean(axis=-1, keepdims=True), axis=-1)


def _frame_stats(frames):
    """Mean frame, per-sample noise variance of one capture and frame count."""
    x = _volts(frames)
    k = x.shape[0]
    var = float(x.var(axis=0, ddof=1).mean()) if k > 1 else float("nan")
    return x.mean(axis=0), var / (2.0 if np.ndim(frames) == 3 else 1.0), k


@dataclasses.dataclass
class TimingResult:
    """Sweep period and first apparent turnaround (step 1)."""

    fs: float
    n0: float
    nr: float
    se_fs: float
    se_n0: float
    se_nr: float
    residual_rms: float
    n0_jitter: float
    beat_frequency: float
    turnarounds: int
    noise_var: float
    frames: int

    @property
    def locked(self):
        return self.residual_rms <= LOCK_RMS

    def apply(self, cal):
        return dataclasses.replace(cal, fs=self.fs, n0=self.n0)

    def summary(self):
        return _floats(dataclasses.asdict(self) | {"locked": self.locked})


def _centred_ramps(x, n0, nr, guard):
    """Forward ramps `[..., J, Nu]` centred between turnarounds `n0 + j nr`,
    from the second turnaround on, inside the frame; also returns `j`."""
    n = x.shape[-1]
    nu = ramp_length(nr, guard)
    j = np.arange(1, int(np.ceil((n - n0) / nr)))
    pos = (n0 + (j + 0.5) * nr)[:, None] + (np.arange(nu) - 0.5 * (nu - 1))
    ok = (np.floor(pos[:, 0]) - INTERP_HALF_WIDTH + 1 >= 0) & (
        np.floor(pos[:, -1]) + INTERP_HALF_WIDTH <= n - 1
    )
    j, pos = j[ok], pos[ok]
    out = interpolation_matrix(pos, n) @ x.reshape(-1, n).T
    return out.T.reshape(x.shape[:-1] + pos.shape), j


def _mirror_score(x, n0, nr, guard):
    """Normalised mirror correlation of frame `x` about the interior
    turnarounds `n0 + k nr`, over `guard <= d < nr / 2`."""
    n = x.size
    d = np.arange(guard, int(nr // 2))
    nk = turnaround_positions(n0, nr, n)[1:]
    reach = d[-1] + INTERP_HALF_WIDTH
    nk = nk[(nk - reach >= 0) & (nk + reach <= n - 1)]
    pos = np.concatenate([nk[:, None] - d, nk[:, None] + d])
    v = (interpolation_matrix(pos, n) @ x).reshape(pos.shape)
    a, b = v[: nk.size], v[nk.size :]
    return float(np.sum(a * b) / np.sqrt(np.sum(a * a) * np.sum(b * b)))


def _turnarounds(spec, kp, omega, n0, nr, turn):
    """Turnarounds where the ramps either side carry equal phase at bin `kp`."""
    z = spec[..., kp]
    return n0 + turn * nr - np.angle(z[..., 1:] * z[..., :-1]) / (2.0 * omega)


def estimate_timing(
    frames,
    sweep,
    fs_nominal=FS_NOMINAL,
    guard=INTERP_HALF_WIDTH,
    window="hann",
    iterations=4,
):
    """Clock ratio `fs` and apparent first turnaround `n0` from static frames.

    Each turnaround is where the ramps either side have equal phase at the
    strongest beat line beyond the DC main lobe, from the mirror estimate of
    the high-passed frame (its correlation picks the half-beat-period branch).
    """
    x, noise_var, k_frames = _frame_stats(frames)
    nr = sweep.ramp_time * fs_nominal
    lobe = window_lobes(window, ramp_length(nr, guard))[0]
    xh = _highpass(x, lobe / ramp_length(nr, guard))
    n0 = float(mirror_turnaround(xh, nr)[0])
    for _ in range(2):
        for _ in range(iterations):
            ramps, j = _centred_ramps(x, n0, nr, guard)
            if j.size < 3:
                raise ValueError("timing needs at least three complete ramps")
            spec = range_spectrum(ramps, window)
            nfft = 2 * (spec.shape[-1] - 1)
            mag = 20.0 * np.log10(np.abs(spec).mean(axis=0) + 1e-300)
            peaks = find_peaks(mag)[0]
            peaks = peaks[peaks >= int(np.ceil(lobe * nfft / ramps.shape[-1]))]
            kp = int(peaks[np.argmax(mag[peaks])])
            a, b, c = mag[kp - 1 : kp + 2]
            omega = 2.0 * np.pi * (kp + 0.5 * (a - c) / (a - 2.0 * b + c)) / nfft
            turn = j[1:].astype(np.float64)
            nk = _turnarounds(spec, kp, omega, n0, nr, turn)
            design = np.column_stack([np.ones_like(turn), turn])
            coef = np.linalg.lstsq(design, nk, rcond=None)[0]
            resid = nk - design @ coef
            n0, nr = float(coef[0]), float(coef[1])
        branch = n0 + np.pi / omega * np.arange(
            -np.ceil(guard * omega / np.pi), np.ceil(guard * omega / np.pi) + 1
        )
        best = branch[np.argmax([_mirror_score(xh, b, nr, guard) for b in branch])]
        if best == n0:
            break
        n0 = float(best)
    cov = resid @ resid / max(turn.size - 2, 1) * np.linalg.inv(design.T @ design)
    jitter = 0.0
    if k_frames > 1:
        per = range_spectrum(_centred_ramps(_volts(frames), n0, nr, guard)[0], window)
        nkf = _turnarounds(per, kp, omega, n0, nr, turn)
        jitter = float((nkf - turn * nr).mean(axis=1).std(ddof=1))
    se = np.sqrt(np.diag(cov))
    return TimingResult(
        fs=nr / sweep.ramp_time,
        n0=float(np.mod(n0, nr)),
        nr=nr,
        se_fs=float(se[1] / sweep.ramp_time),
        se_n0=float(se[0]),
        se_nr=float(se[1]),
        residual_rms=float(np.sqrt(np.mean(resid**2))),
        n0_jitter=jitter,
        beat_frequency=float(omega / (2.0 * np.pi) * nr / sweep.ramp_time),
        turnarounds=int(turn.size),
        noise_var=noise_var,
        frames=k_frames,
    )


@dataclasses.dataclass
class GuardResult:
    """Turnaround guard (step 2), None if the criterion is not met within a
    quarter ramp. `level_db[G]` is the largest transient spectral level
    relative to the ramp peak for guard `G`."""

    ng: int | None
    psl_db: float
    level_db: np.ndarray
    transient: np.ndarray
    noise_db: float
    frames: int
    frames_needed: int

    @property
    def noise_ok(self):
        return self.noise_db < self.psl_db

    def apply(self, cal):
        return cal if self.ng is None else dataclasses.replace(cal, ng=self.ng)

    def summary(self):
        d = dataclasses.asdict(self)
        d.pop("transient")
        return _floats(d | {"noise_ok": self.noise_ok})


def estimate_guard(frames, sweep, cal, window="hann", zpad=4, taps=EQ_TAPS):
    """Smallest guard from which on the turnaround transient's range spectrum
    stays below the window's peak sidelobe relative to the ramp peak, both
    beyond the window main lobe of zero range (frames high-passed there)."""
    x, noise_var, k_frames = _frame_stats(frames)
    n = x.shape[-1]
    nr = cal.nr(sweep)
    half = int(nr // 2)
    lobe, _, psl = window_lobes(window, ramp_length(nr, 0))
    x = _highpass(x, lobe / ramp_length(nr, 0))
    d = np.arange(-taps, half + taps)
    nk = turnaround_positions(cal.n0, nr, n)[1:]
    reach = d[-1] + INTERP_HALF_WIDTH
    nk = nk[(nk - reach >= 0) & (nk + reach <= n - 1)]
    if nk.size < 2:
        raise ValueError("guard estimation needs two interior turnarounds")
    pos = np.concatenate([nk[:, None] - d, nk[:, None] + d])
    v = (interpolation_matrix(pos, n) @ x).reshape(pos.shape)
    a, b = v[: nk.size], v[nk.size :]
    lags = np.arange(-taps, taps + 1)
    settled = np.arange(half // 2, half)
    full = np.arange(half)
    psl_lin = 10.0 ** (psl / 20.0)
    transient, gain2 = [], []
    for kind in (0, 1):
        ak, bk = a[kind::2], b[kind::2]
        mirror = bk[:, taps : taps + half] - ak[:, taps : taps + half]
        g = np.linalg.lstsq(
            ak[:, settled[:, None] - lags + taps].reshape(-1, lags.size),
            mirror[:, settled].ravel(),
            rcond=psl_lin,
        )[0]
        e = mirror[:, full] - ak[:, full[:, None] - lags + taps] @ g
        transient.append(e.mean(axis=0))
        gain2.append((2.0 + 2.0 * g[taps] + g @ g) / ak.shape[0])
    transient = np.array(transient)
    guards = np.arange(half // 2)
    level = np.empty(guards.size)
    noise = np.empty(guards.size)
    for i, g in enumerate(guards):
        nu = ramp_length(nr, g)
        lo = int(np.ceil(lobe * zpad))
        ramps = extract_ramps(x, cal.n0, nr, g, True)
        peak = np.abs(range_spectrum(ramps, window, zpad)).mean(axis=(0, 1))[lo:].max()
        seg = np.zeros((2, nu))
        m = min(nu, half - g)
        seg[:, :m] = transient[:, g : g + m]
        level[i] = np.abs(range_spectrum(seg, window, zpad))[:, lo:].max() / peak
        w = get_window(window, nu, fftbins=False)
        sig = 2.0 / w.sum() * math.sqrt(w @ w * noise_var / k_frames * max(gain2))
        noise[i] = sig * math.sqrt(math.log((nu // 2 + 1) / P_FALSE)) / peak
    bad = np.flatnonzero(level + noise >= psl_lin)
    ng = 0 if bad.size == 0 else int(guards[bad[-1]] + 1)
    at = min(ng, guards.size - 1)
    return GuardResult(
        ng=ng if ng < guards.size else None,
        psl_db=psl,
        level_db=20.0 * np.log10(level),
        transient=transient,
        noise_db=float(20.0 * np.log10(noise[at])),
        frames=k_frames,
        frames_needed=int(np.ceil(k_frames * (noise[at] / psl_lin) ** 2)),
    )


class _ReflectorModel:
    """Whitened profiles near the reflector and their model: direct and three
    ground-image paths (z = 0), each bounced leg scaled by `rho0 + rho1 (sin
    e - mean)` (e: bounce-ray elevation); amplitudes and background projected out."""

    def __init__(self, scan, sweep, cal, geometry, target, baseline, first_up, window):
        codes, x_pos = scan
        self.x = np.asarray(x_pos, dtype=np.float64)
        self.sweep, self.geometry = sweep, geometry
        self.target = np.asarray(target, dtype=np.float64)
        self.baseline, self.first_up = baseline, first_up
        mid = np.mean([cal.tx_offset, cal.rx_offset], axis=0)
        self.mid_z = float(mid[2])
        nr = cal.nr(sweep)
        nu = ramp_length(nr, cal.ng)
        self.ramp_pos = ramp_positions(
            cal.n0, nr, cal.ng, np.shape(codes)[-1], first_up
        )
        ph = phase_history(codes, sweep, cal, first_up, None, window, 1)
        self.f_m = ph.f_m
        self.cell = C / (2.0 * sweep.slope * nu / cal.fs)
        win = get_window(window, nu, fftbins=False)
        r_pred = 0.5 * self.lengths(mid)[0][:, 0].mean() + cal.r_cal
        mag = np.abs(ph.profiles).mean(axis=0)
        peaks = find_peaks(mag)[0]
        kp = int(peaks[np.argmin(np.abs(ph.r_axis[peaks] - r_pred))])
        self.r_peak = float(ph.r_axis[kp])
        h = int(np.ceil(window_lobes(window, nu)[0]))
        self.bins = np.arange(max(kp - h, 1), kp + h + 1)
        m = np.arange(nu) - 0.5 * (nu - 1)
        nfft = next_fast_len(nu, real=True)
        e = np.exp(-2j * np.pi * np.outer(self.bins, m) / nfft) * (2.0 / win.sum())
        self.white = np.linalg.inv(np.linalg.cholesky((e * win**2) @ e.conj().T))
        size = next_fast_len(64 * nu, real=True)
        grid = np.arange(size // 2 + 1) / size
        table = (rfft(win, size) * np.exp(2j * np.pi * grid * 0.5 * (nu - 1))).real
        self.wspline = CubicSpline(grid, table * (2.0 / win.sum()))
        self.kfreq = self.bins / nfft
        self.wk = self.wspline(self.kfreq)
        self.df = sweep.slope / cal.fs
        self.nu = nu
        self.y = self.whiten(ph.profiles)
        self.u_ref = float(self.lengths(mid)[2].mean())

    def whiten(self, profiles):
        """Real `[P, 2B]` whitened profile bins."""
        y = profiles[:, self.bins] @ self.white.T
        return np.concatenate([y.real, y.imag], axis=1)

    def lengths(self, mid, x=None):
        """Path lengths `[P, 4]`, spreading factors `[P, 4]` and bounce-ray
        elevation sines (tx, rx) for midpoint offset `mid`."""
        x = self.x if x is None else x
        ref = np.column_stack(
            [
                x + mid[0],
                np.full(x.size, mid[1]),
                np.full(x.size, self.geometry.height + mid[2]),
            ]
        )
        half = np.array([0.5 * self.baseline, 0.0, 0.0])
        img = np.array([1.0, 1.0, -1.0])
        tx, rx = ref - half, ref + half
        lt, lr, lti, lri = (
            np.linalg.norm(self.target - p, axis=1)
            for p in (tx, rx, tx * img, rx * img)
        )
        legs = np.stack([lt * lr, lti * lr, lt * lri, lti * lri], axis=1)
        length = np.stack([lt + lr, lti + lr, lt + lri, lti + lri], axis=1)
        zt = self.target[2]
        return (
            length,
            (lt * lr)[:, None] / legs,
            (tx[:, 2] + zt) / lti,
            (rx[:, 2] + zt) / lri,
        )

    def spectra(self, r_cal, mid, x=None):
        """Whitened spectra `(S+, S-)` `[P, 4, B]` of unit paths and conjugates:
        shifted window transforms less the ramp-mean term."""
        length, spread, ut, ur = self.lengths(mid, x)
        tau = (length + 2.0 * r_cal) / C
        base = (spread * np.exp(2j * np.pi * self.f_m * tau))[..., None]
        nu = (tau * self.df)[..., None]
        mean = np.sinc(self.nu * nu) / np.sinc(nu) * self.wk
        w = self.wspline
        sp = base * (w(np.abs(nu - self.kfreq)) - mean)
        sm = base.conj() * (w(nu + self.kfreq) - mean)
        return sp @ self.white.T, sm @ self.white.T, ut, ur

    def _alpha(self, theta, ut, ur):
        r0, i0, r1, i1 = theta[4:]
        rt = complex(r0, i0) + complex(r1, i1) * (ut - self.u_ref)
        rr = complex(r0, i0) + complex(r1, i1) * (ur - self.u_ref)
        return np.exp(1j * theta[3]) * np.stack([np.ones_like(rt), rt, rr, rt * rr], 1)

    def profiles(self, theta, x=None):
        """Complex model `[P, B]` for `(r_cal, ox, oy, phi, rho0, rho1)`."""
        sp, sm, ut, ur = self.spectra(theta[0], (theta[1], theta[2], self.mid_z), x)
        alpha = self._alpha(theta, ut, ur)
        return 0.5 * (
            np.einsum("np,npb->nb", alpha, sp)
            + np.einsum("np,npb->nb", alpha.conj(), sm)
        )

    def ramps(self, theta, amp):
        """Model mean ramps `[P, Nu]` with per-position amplitudes `amp`."""
        length, spread, ut, ur = self.lengths((theta[1], theta[2], self.mid_z))
        tau = (length + 2.0 * theta[0]) / C
        f = self.f_m + self.df * (np.arange(self.nu) - 0.5 * (self.nu - 1))
        z = np.exp(2j * np.pi * tau[..., None] * f) * spread[..., None]
        return (
            amp[:, None] * np.einsum("np,npm->nm", self._alpha(theta, ut, ur), z).real
        )

    def project(self, model, y=None):
        """Per-position amplitudes, background and residual `[P, 2B]`."""
        y = self.y if y is None else y
        m = np.concatenate([model.real, model.imag], axis=1)
        mh = m / np.linalg.norm(m, axis=1, keepdims=True)
        gram = y.shape[0] * np.eye(m.shape[1]) - mh.T @ mh
        bg = np.linalg.solve(gram, y.sum(axis=0) - mh.T @ np.sum(mh * y, axis=1))
        amp = np.sum(m * (y - bg), axis=1) / np.sum(m * m, axis=1)
        return amp, bg, y - bg - amp[:, None] * m

    def residual(self, theta):
        return self.project(self.profiles(theta))[2].ravel()

    def linear_init(self, r_cal, mid):
        """`(cost, phi, rho)` from free complex coefficients of the direct,
        single- and double-bounce classes at unit amplitudes."""
        sp, sm, _, _ = self.spectra(r_cal, mid)
        cols = []
        for p, q in (
            (sp[:, 0], sm[:, 0]),
            (sp[:, 1] + sp[:, 2], sm[:, 1] + sm[:, 2]),
            (sp[:, 3], sm[:, 3]),
        ):
            for v in (0.5 * (p + q), 0.5j * (p - q)):
                cols.append(np.concatenate([v.real, v.imag], axis=1).ravel())
        bg = np.tile(np.eye(self.y.shape[1]), (self.y.shape[0], 1))
        design = np.column_stack([np.column_stack(cols), bg])
        coef = np.linalg.lstsq(design, self.y.ravel(), rcond=None)[0]
        res = self.y.ravel() - design @ coef
        c0, c1 = complex(*coef[0:2]), complex(*coef[2:4])
        return float(res @ res), float(np.angle(c0)), c1 / c0

    def start(self, mid):
        """`(cost, theta0)`: best linear start on an `r_cal` grid over one cell."""
        r0 = self.r_peak - 0.5 * self.lengths(mid)[0][:, 0].mean()
        grid = r0 + self.cell * np.linspace(-1.0, 1.0, 33)
        cost, phi, rho, r_init = min(
            (self.linear_init(r, mid) + (r,) for r in grid), key=lambda t: t[0]
        )
        return cost, np.array([r_init, mid[0], mid[1], phi, rho.real, rho.imag, 0, 0])

    def fit(self, mid, theta0):
        """Nonlinear least squares from `theta0`, solved in `r_cal + grad R .
        offset` (what the envelope measures) to decorrelate the parameters."""
        h = 1e-4
        grad = [
            np.mean(
                np.diff(
                    [self.lengths(mid + s * h * e)[0][:, 0] for s in (-1, 1)], axis=0
                )
            )
            / (4.0 * h)
            for e in np.eye(3)[:2]
        ]
        mix = np.eye(theta0.size)
        mix[0, 1:3] = -np.asarray(grad)
        shift = np.zeros(theta0.size)
        shift[0] = np.dot(grad, mid[:2])
        sol = least_squares(
            lambda p: self.residual(mix @ p + shift),
            np.linalg.solve(mix, theta0 - shift),
            x_scale="jac",
            method="lm",
        )
        sol.x = mix @ sol.x + shift
        sol.jac = sol.jac @ np.linalg.inv(mix)
        return sol


@dataclasses.dataclass
class ReflectorResult:
    """Reflector-scan calibration (step 3). `cov` is the covariance of
    `(r_cal, ox, oy)`; `rho` the fitted ground reflection factor per leg."""

    first_up: bool
    r_cal: float
    tx_offset: tuple
    rx_offset: tuple
    se_r_cal: float
    se_offset: tuple
    cov: np.ndarray
    rho: complex
    range_rms: float
    chi2: float
    cost_ratio: float
    focus: dict
    model: object = dataclasses.field(default=None, repr=False, compare=False)
    theta: np.ndarray = dataclasses.field(default=None, repr=False, compare=False)

    def apply(self, cal):
        return dataclasses.replace(
            cal,
            first_up=self.first_up,
            r_cal=self.r_cal,
            tx_offset=self.tx_offset,
            rx_offset=self.rx_offset,
        )

    def summary(self):
        skip = ("model", "theta", "rho")
        d = {f.name: getattr(self, f.name) for f in dataclasses.fields(self)}
        d = {k: v for k, v in d.items() if k not in skip}
        return _floats(d | {"rho": [self.rho.real, self.rho.imag]})


def _three_db_width(cut, step):
    p = cut / cut.max()
    i = int(np.argmax(p))
    lo = i - int(np.argmax(p[i::-1] < 0.5))
    hi = i + int(np.argmax(p[i:] < 0.5))
    if i in (lo, hi):
        return float("nan")
    left = lo + (0.5 - p[lo]) / (p[lo + 1] - p[lo])
    right = hi - 1 + (p[hi - 1] - 0.5) / (p[hi - 1] - p[hi])
    return float((right - left) * step)


def focus_check(
    codes, x_pos, sweep, cal, geometry, target, window="hann", reflector=None
):
    """Image peak error and −3 dB widths along and across the line of sight,
    with the predicted widths (signal-processing §11.4, projected aperture).
    With a fitted `(model, theta, amp)` the error is taken against the image
    of the model scan (ground images included) instead of the target."""
    target = np.asarray(target, dtype=np.float64)
    x_pos = np.asarray(x_pos, dtype=np.float64)
    nu = ramp_length(cal.nr(sweep), cal.ng)
    centre = np.array([x_pos.mean(), 0.0, geometry.height])
    u = (target[:2] - centre[:2]) / np.linalg.norm(target[:2] - centre[:2])
    v = np.array([-u[1], u[0]])
    ph = phase_history(codes, sweep, cal, cal.first_up, None, window)
    aperture = np.ptp(x_pos) * x_pos.size / (x_pos.size - 1) * abs(u[1])
    rng = float(np.linalg.norm(target - centre))
    pred = np.array(
        [
            window_lobes(window, nu)[1] * C / (2.0 * sweep.slope * nu / cal.fs),
            window_lobes("boxcar", x_pos.size)[1] * C / ph.f_m * rng / (2 * aperture),
        ]
    )
    step = pred.min() / 16.0
    span = 2.0 * pred.max()
    ax = np.arange(-span, span + step / 2, step)
    tx, rx = geometry.antenna_positions(x_pos, cal)

    def power_of(history):
        img = backproject(
            history, tx, rx, target[0] + ax, target[1] + ax, target[2], cal.r_cal
        )
        return np.abs(img) ** 2

    def peak_of(power):
        iy, ix = divmod(int(np.argmax(power)), power.shape[1])
        return np.array([ax[ix], ax[iy]])

    power = power_of(ph)
    peak = peak_of(power)
    ref = np.zeros(2)
    if reflector is not None:
        model, theta, amp = reflector  # pylint: disable=unbalanced-tuple-unpacking
        spec = range_spectrum(model.ramps(theta, amp), window)
        ref = peak_of(power_of(PhaseHistory(spec, ph.r_axis, ph.f_m, ph.first_up)))
    t = np.arange(-span, span, step / 4)

    def width(direction):
        pts = (peak[:, None] + direction[:, None] * t + span) / step
        return _three_db_width(
            ndimage.map_coordinates(power, pts[::-1], order=1), step / 4
        )

    err = np.array([(peak - ref) @ u, (peak - ref) @ v])
    return {
        "peak_error": err.tolist(),
        "model_offset": [float(ref @ u), float(ref @ v)],
        "widths": [width(u), width(v)],
        "predicted_widths": pred.tolist(),
        "ok": bool(np.all(np.abs(err) <= 0.5 * pred)),
    }


def _scan_arrays(scan):
    return (scan.codes, scan.x_pos) if isinstance(scan, Recording) else scan


def estimate_reflector(
    scan,
    sweep,
    cal,
    geometry,
    target,
    baseline,
    window="hann",
    noise_var=None,
    check=True,
):
    """`first_up`, `r_cal` and antenna offsets from a reflector scan
    (`Recording` or `(codes, x_pos)`). `cal` gives `fs`, `n0`, `ng` and the
    nominal midpoint (its `z` is held); `noise_var` (step 1) gives `chi2`."""
    scan = _scan_arrays(scan)
    mid = np.mean([cal.tx_offset, cal.rx_offset], axis=0)
    starts = []
    for up in (True, False):
        model = _ReflectorModel(
            scan, sweep, cal, geometry, target, baseline, up, window
        )
        starts.append(model.start(mid) + (model,))
    (cost, theta0, model), (other, _, _) = sorted(starts, key=lambda f: f[0])
    sol = model.fit(mid, theta0)
    s2 = 2.0 * sol.cost / (sol.fun.size - sol.x.size - sum(model.y.shape))
    bread = np.linalg.pinv(sol.jac.T @ sol.jac)
    score = np.einsum(
        "nkp,nk->np",
        sol.jac.reshape(model.y.shape + (-1,)),
        sol.fun.reshape(model.y.shape),
    )
    npos = model.y.shape[0]
    cov = bread @ (score.T @ score) @ bread * npos / (npos - sol.x.size)
    se = np.sqrt(np.diag(cov))
    m = model.profiles(sol.x)
    amp, bg, _ = model.project(m)
    k = m.shape[1] // 2
    yb = model.y - bg
    dphi = np.angle((yb[:, k] + 1j * yb[:, m.shape[1] + k]) * np.conj(amp * m[:, k]))
    sigma_phi = np.sqrt(np.mean((dphi - dphi.mean()) ** 2))
    chi2 = float("nan")
    if noise_var:
        chi2 = float(s2 / (noise_var / (4.0 * model.ramp_pos.shape[1])))
    half = np.array([0.5 * baseline, 0.0, 0.0])
    mid_fit = np.array([sol.x[1], sol.x[2], mid[2]])
    new = dataclasses.replace(
        cal,
        first_up=model.first_up,
        r_cal=float(sol.x[0]),
        tx_offset=tuple(map(float, mid_fit - half)),
        rx_offset=tuple(map(float, mid_fit + half)),
    )
    return ReflectorResult(
        first_up=model.first_up,
        r_cal=new.r_cal,
        tx_offset=new.tx_offset,
        rx_offset=new.rx_offset,
        se_r_cal=float(se[0]),
        se_offset=(float(se[1]), float(se[2])),
        cov=cov[:3, :3],
        rho=complex(sol.x[4], sol.x[5]),
        range_rms=float(C * sigma_phi / (4.0 * np.pi * model.f_m)),
        chi2=chi2,
        cost_ratio=float(other / cost),
        focus=(
            focus_check(
                *scan, sweep, new, geometry, target, window, (model, sol.x, amp)
            )
            if check
            else {}
        ),
        model=model,
        theta=sol.x,
    )


@dataclasses.dataclass
class RepeatResult:
    """Sled repeatability (step 4) and the predicted SAR peak-gain factor."""

    sled_sigma: float
    se: float
    gain_loss: float
    noise_sigma: float
    positions: int
    scans: int

    def apply(self, cal):
        return dataclasses.replace(cal, sled_sigma=self.sled_sigma)

    def summary(self):
        return _floats(dataclasses.asdict(self))


def estimate_repeatability(
    scans, sweep, cal, geometry, target, baseline, reflector=None, window="hann"
):
    """`sled_sigma` from repeated reflector scans: each position's phase against
    the fitted model over the model's phase slope along the rail, at positions
    with at least half the largest slope, less the noise orthogonal to the model."""
    scans = [_scan_arrays(s) for s in scans]
    if len(scans) < 2:
        raise ValueError("repeatability needs at least two scans")
    if reflector is None:
        reflector = estimate_reflector(
            scans[0], sweep, cal, geometry, target, baseline, window, check=False
        )
    model, theta = reflector.model, reflector.theta

    def real(z):
        return np.concatenate([z.real, z.imag], axis=-1)

    m = model.profiles(theta)
    amp, bg, _ = model.project(m)
    unit = real(m) / np.linalg.norm(real(m), axis=1, keepdims=True)
    quad = real(1j * m) / np.linalg.norm(real(m), axis=1, keepdims=True)

    def phase(y):
        return np.arctan2(np.sum(y * quad, -1), np.sum(y * unit, -1))

    h = 1e-4
    slope = np.angle(
        np.exp(
            1j
            * (
                phase(real(model.profiles(theta, model.x + h)))
                - phase(real(model.profiles(theta, model.x - h)))
            )
        )
    ) / (2.0 * h)
    up = reflector.first_up
    cal = dataclasses.replace(cal, first_up=up)
    y = np.array(
        [
            model.whiten(phase_history(c, sweep, cal, up, None, window, 1).profiles)
            - bg
            for c, _ in scans
        ]
    )
    rot = np.exp(1j * phase(y))
    dphi = np.angle(rot * np.conj(rot.mean(axis=0)))
    keep = np.abs(slope) >= 0.5 * np.abs(slope).max()
    perp = (
        y
        - np.sum(y * unit, -1)[..., None] * unit
        - np.sum(y * quad, -1)[..., None] * quad
    )
    perp = (perp - perp.mean(axis=0))[:, keep]
    j = len(scans)
    s2 = float(np.sum(perp**2) / (perp[0].size - 2 * keep.sum()) / (j - 1))
    dx = dphi[:, keep] / slope[keep]
    dof = keep.sum() * (j - 1)
    var = float(np.sum((dx - dx.mean(axis=0)) ** 2) / dof)
    noise = float(
        np.mean(s2 / (amp[keep] * slope[keep]) ** 2 / np.sum(real(m[keep]) ** 2, -1))
    )
    sigma = math.sqrt(max(var - noise, 0.0))
    se_var = var * math.sqrt(2.0 / dof)
    return RepeatResult(
        sled_sigma=sigma,
        se=float(se_var / (2.0 * sigma) if sigma**2 > se_var else math.sqrt(se_var)),
        gain_loss=float(np.exp(-((4.0 * np.pi * sigma / sweep.lam) ** 2))),
        noise_sigma=math.sqrt(noise),
        positions=int(keep.sum()),
        scans=j,
    )


def pair_step(sweep, geometry, target, x0=0.0, length=1.5):
    """Sled step that changes the reflector's range by a quarter wavelength."""
    p = np.asarray(target, dtype=np.float64) - [x0, 0.0, geometry.height]
    slope = abs(p[0]) / np.linalg.norm(p)
    return float(min(sweep.lam / (4.0 * max(slope, 1e-12)), length))


def run_timing(radar, n=4096, frames=32, sled=None, dx=None):
    """Static frames `[frames, n]` at the current position or, with a sled,
    `[2, frames, n]` at the current position and `dx` further on."""
    first = capture_frames(radar, n, frames, desc="timing")
    if sled is None:
        return first
    sled.move_to(sled.position() + dx)
    return np.stack([first, capture_frames(radar, n, frames, desc="timing +dx")])


def run_reflector(radar, sled, geometry, target, length=1.5, n=4096, desc="reflector"):
    """Reflector scan over `[0, length]` at `dx = lam_min / 4`."""
    return run_scan(
        radar,
        sled,
        scan_positions(radar.sweep, length),
        n,
        geometry,
        extra={"target": list(map(float, target))},
        desc=desc,
    )


def run_repeat(radar, sled, geometry, target, length=1.5, n=4096, scans=5):
    """`scans` repeated reflector scans."""
    return [
        run_reflector(radar, sled, geometry, target, length, n, f"repeat {i + 1}")
        for i in range(scans)
    ]


def default_target(geometry, length=1.5):
    """Step 3 placement: `TARGET_RANGE` from the aperture centre,
    `TARGET_ANGLE` off boresight, at antenna height."""
    a = math.radians(TARGET_ANGLE)
    return (
        0.5 * length + TARGET_RANGE * math.sin(a),
        TARGET_RANGE * math.cos(a),
        geometry.height,
    )


def sim_devices(hw, sweep, geometry, target, seed=0, sled_sigma=0.0):
    """Simulated radar and sled on `builtin_scene("single")`, reflector at `target`."""
    # pylint: disable=import-outside-toplevel
    from qmrdk.sim.devices import SimRadar, SimSled
    from qmrdk.sim.scene import builtin_scene, compile_scene

    scene = builtin_scene("single")
    scene["objects"] = [dict(scene["objects"][0], pos=list(map(float, target)))]
    sled = SimSled(sigma=sled_sigma, seed=seed)
    geom = compile_scene(scene, sweep.lam)
    return SimRadar(geom, hw, sweep, sled, geometry, seed + 1), sled


def nominal(offsets, baseline=None):
    """Mechanical starting calibration: antennas symmetric about the sled
    reference at the measured `baseline` and height of `offsets`."""
    tx, rx = np.asarray(offsets.tx_offset), np.asarray(offsets.rx_offset)
    baseline = float(abs(rx[0] - tx[0])) if baseline is None else baseline
    z = float(0.5 * (tx[2] + rx[2]))
    return Calibration(
        tx_offset=(-0.5 * baseline, 0.0, z), rx_offset=(0.5 * baseline, 0.0, z)
    )


def calibrate(
    radar,
    sled,
    geometry,
    target,
    baseline,
    start,
    frames=32,
    repeats=3,
    length=1.5,
    n=4096,
):
    """Steps 1–4 in order on any radar and sled: `(Calibration, report)`."""
    t0 = time.time()
    sweep = radar.sweep
    sled.home()
    static = run_timing(
        radar, n, frames, sled, pair_step(sweep, geometry, target, 0.0, length)
    )
    timing = estimate_timing(static, sweep, start.fs)
    cal = timing.apply(start)
    guard = estimate_guard(static, sweep, cal)
    if guard.ng is None:
        raise ValueError(f"no guard meets the criterion: {guard.summary()}")
    cal = guard.apply(cal)
    scan = run_reflector(radar, sled, geometry, target, length, n)
    refl = estimate_reflector(
        scan, sweep, cal, geometry, target, baseline, noise_var=timing.noise_var
    )
    cal = refl.apply(cal)
    report = {
        "timing": timing.summary(),
        "guard": guard.summary(),
        "reflector": refl.summary(),
    }
    if repeats > 1:
        more = run_repeat(radar, sled, geometry, target, length, n, repeats - 1)
        rep = estimate_repeatability(
            [scan] + more, sweep, cal, geometry, target, baseline, refl
        )
        cal = rep.apply(cal)
        report["repeat"] = rep.summary()
    report["calibration"] = dataclasses.asdict(cal)
    report["runtime"] = time.time() - t0
    return cal, _floats(report)


def calibrate_sim(
    hw,
    sweep,
    geometry=ScanGeometry(),
    seed=0,
    frames=32,
    repeats=3,
    length=1.5,
    target=None,
    sled_sigma=0.0,
):
    """Steps 1–4 on the simulated board `hw` from nominal constants and the
    measured baseline; reflector at `target` (default `default_target`)."""
    target = default_target(geometry, length) if target is None else target
    radar, sled = sim_devices(hw, sweep, geometry, target, seed, sled_sigma)
    baseline = float(abs(hw.rx_offset[0] - hw.tx_offset[0]))
    return calibrate(
        radar, sled, geometry, target, baseline, nominal(hw), frames, repeats, length
    )


def run_step(args):
    """Handler of every `calib` command: run the step, update `--cal`, report."""
    # pylint: disable=import-outside-toplevel
    from qmrdk.radar import HardwareSled, UsbRadar
    from qmrdk.sim.hardware import Hardware

    path = pathlib.Path(args.cal) if args.cal else None
    cal = Calibration.load(path) if path and path.is_file() else Calibration()
    geometry = ScanGeometry(height=args.height)
    target = tuple(args.target or default_target(geometry, args.length))
    step = args.calib_command
    try:
        if step == "sim":
            cal, report = calibrate_sim(
                Hardware(),
                Sweep(),
                geometry,
                args.seed,
                args.frames,
                args.scans,
                args.length,
                target,
                args.sled_sigma,
            )
        else:
            if args.sim:
                hw = Hardware()
                radar, sled = sim_devices(
                    hw, Sweep(), geometry, target, args.seed, args.sled_sigma
                )
                cal = cal if path and path.is_file() else nominal(hw)
            else:
                radar, sled = UsbRadar(), HardwareSled()
            res = _STEPS[step](args, radar, sled, geometry, target, cal)
            cal, report = res.apply(cal), {step: res.summary()}
    except (NotImplementedError, ValueError) as exc:
        print(f"qmrdk calib: {exc}", file=sys.stderr)
        return 2
    report = _floats(report)
    if path:
        cal.save(path)
    if args.report:
        pathlib.Path(args.report).write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
    print(json.dumps(report, indent=2))
    return 0


def _static(args, radar, sled, geometry, target):
    sled.home()
    return run_timing(
        radar,
        args.n,
        args.frames,
        sled,
        pair_step(radar.sweep, geometry, target, 0.0, args.length),
    )


def _baseline(args, cal):
    return args.baseline or float(abs(cal.rx_offset[0] - cal.tx_offset[0]))


_STEPS = {
    "timing": lambda a, r, s, g, t, c: estimate_timing(
        _static(a, r, s, g, t), r.sweep, c.fs
    ),
    "guard": lambda a, r, s, g, t, c: estimate_guard(
        _static(a, r, s, g, t), r.sweep, c
    ),
    "reflector": lambda a, r, s, g, t, c: estimate_reflector(
        run_reflector(r, s, g, t, a.length, a.n), r.sweep, c, g, t, _baseline(a, c)
    ),
    "repeat": lambda a, r, s, g, t, c: estimate_repeatability(
        run_repeat(r, s, g, t, a.length, a.n, a.scans),
        r.sweep,
        c,
        g,
        t,
        _baseline(a, c),
    ),
}

_HELP = {
    "timing": "step 1: fs and n0 from static frame pairs",
    "guard": "step 2: turnaround guard ng",
    "reflector": "step 3: first_up, r_cal and antenna offsets",
    "repeat": "step 4: sled repeatability",
    "sim": "steps 1-4 on the simulated board",
}

_ARGS = (
    ("--sim", {"action": "store_true", "help": "use the simulated board"}),
    ("--cal", {"help": "calibration JSON to read and update"}),
    ("--report", {"help": "write the JSON report here"}),
    ("--n", {"type": int, "default": 4096, "help": "samples per frame"}),
    ("--frames", {"type": int, "default": 32, "help": "static frames per position"}),
    (
        "--target",
        {
            "type": float,
            "nargs": 3,
            "metavar": ("X", "Y", "Z"),
            "help": "reflector phase centre, m",
        },
    ),
    ("--baseline", {"type": float, "help": "tx-rx aperture separation, m"}),
    ("--length", {"type": float, "default": 1.5, "help": "scan length, m"}),
    ("--height", {"type": float, "default": 1.0, "help": "rail height, m"}),
    ("--seed", {"type": int, "default": 0, "help": "simulation seed"}),
    (
        "--sled-sigma",
        {"type": float, "default": 0.0, "help": "simulated sled sigma, m"},
    ),
)


def add_commands(subparsers):
    """Register the `calib` command group; every command sets `func=run_step`."""
    calib = subparsers.add_parser(
        "calib", help="board calibration (docs/calibration.md)"
    )
    sub = calib.add_subparsers(dest="calib_command", required=True)
    for name, text in _HELP.items():
        p = sub.add_parser(name, help=text)
        for flag, kwargs in _ARGS:
            p.add_argument(flag, **kwargs)
        p.add_argument(
            "--scans",
            type=int,
            default=5 if name == "repeat" else 3,
            help="reflector scans",
        )
        p.set_defaults(func=run_step)
    return calib

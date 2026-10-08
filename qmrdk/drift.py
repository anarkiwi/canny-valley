"""Phase, amplitude and temperature drift of a static capture
(docs/signal-processing.md §9.1)."""

import dataclasses
import json
import pathlib

import numpy as np
from matplotlib.figure import Figure
from scipy.fft import next_fast_len
from scipy.signal import find_peaks
from scipy.stats import linregress
from tqdm import tqdm

from qmrdk.calib import window_lobes
from qmrdk.config import Calibration
from qmrdk.constants import C
from qmrdk.dsp.convert import codes_to_volts
from qmrdk.dsp.range import centre_frequency, range_axis, range_spectrum
from qmrdk.dsp.segment import extract_ramps, ramp_length
from qmrdk.recording import Recording

CHUNK = 256


def line_values(codes, sweep, cal, window="hann", zpad=4):
    """Complex range spectra `[K, 2, F]` of the mean up and down ramp of each
    frame `codes[K, N]`, and the ramp length."""
    first_up = True if cal.first_up is None else cal.first_up
    parts = []
    for lo in tqdm(range(0, len(codes), CHUNK), desc="drift", unit="chunk"):
        x = codes_to_volts(np.asarray(codes[lo : lo + CHUNK]))
        ramps = extract_ramps(x, cal.n0, cal.nr(sweep), cal.ng, first_up)
        if ramps.shape[-2] == 0:
            raise ValueError("frames hold no complete ramp of each direction")
        parts.append(range_spectrum(ramps.mean(axis=-2), window, zpad))
    return np.concatenate(parts), ramps.shape[-1]


def select_bins(spec, nu, window="hann", count=1):
    """Line bins (the `count` strongest peaks beyond the zero-range main lobe,
    strongest first) and leakage bin (strongest inside it, DC excluded) of the
    mean frame of `spec[K, 2, F]`."""
    mag = np.abs(spec.mean(axis=0)).mean(axis=0)
    edge = int(np.ceil(window_lobes(window, nu)[0] * 2 * (spec.shape[-1] - 1) / nu))
    peaks = find_peaks(mag)[0]
    peaks = peaks[peaks >= edge]
    leak = 1 + int(np.argmax(mag[1:edge]))
    return peaks[np.argsort(mag[peaks])[::-1][:count]].tolist(), leak


def unwrapped_phase(z):
    """Phase (rad) of `z[K, 2]` relative to the first frame: each ramp
    direction unwrapped over frames, then averaged."""
    phi = np.unwrap(np.angle(z), axis=0)
    return (phi - phi[0]).mean(axis=1)


def allan_deviation(y, tau0):
    """Overlapping Allan deviation of samples `y` at spacing `tau0` for
    octave averaging factors: `(tau, adev)`."""
    y = np.asarray(y, dtype=np.float64)
    m = 2 ** np.arange(int(np.log2(max((y.size - 1) // 2, 1))) + 1)
    m = m[2 * m < y.size]
    s = np.concatenate([[0.0], np.cumsum(y)])
    adev = [
        np.sqrt(0.5 * np.mean(((s[2 * k :] - 2 * s[k:-k] + s[: -2 * k]) / k) ** 2))
        for k in m
    ]
    return m * tau0, np.asarray(adev)


def frame_noise(y):
    """Frame-to-frame white noise of `y`: rms second difference / sqrt(6),
    insensitive to drift linear over three frames."""
    return float(np.sqrt(np.mean(np.diff(y, 2) ** 2) / 6.0)) if len(y) > 2 else None


def _fit(x, y):
    """Least-squares slope and its standard error, None if `x` is constant."""
    if x is None or np.ptp(x) == 0 or len(x) < 3:
        return None, None
    fit = linregress(x, y)
    return float(fit.slope), float(fit.stderr)


@dataclasses.dataclass
class DriftLine:
    """Series of one spectral line over the frames."""

    bin: int
    range: float
    phase: np.ndarray
    range_drift: np.ndarray
    amplitude_db: np.ndarray
    residual_db: float


@dataclasses.dataclass
class DriftResult:
    """Drift of the target and leakage lines of a static capture. `t` is s
    from the first frame; `range_drift` (m) is `c phi / (4 pi f_centre)`, its
    sign meaningful only if `sign_known` (calibrated `first_up`)."""

    t: np.ndarray
    temperature: np.ndarray | None
    f_centre: float
    sign_known: bool
    lines: dict

    @property
    def tau0(self):
        """Median frame spacing, s."""
        return float(np.median(np.diff(self.t))) if self.t.size > 1 else 0.0

    def stats(self, line):
        """Summary statistics of one line."""
        d = self.lines[line]
        mm = 1e3 * d.range_drift
        deg = np.degrees(d.phase)
        rate, se_rate = _fit(self.t / 60.0, mm)
        tc_deg, se_deg = _fit(self.temperature, deg)
        tc_mm, se_mm = _fit(self.temperature, mm)
        tau, adev = allan_deviation(mm, self.tau0)
        return {
            "bin": d.bin,
            "range_m": d.range,
            "phase_noise_deg": frame_noise(deg),
            "range_noise_mm": frame_noise(mm),
            "phase_span_deg": float(np.ptp(deg)),
            "range_span_mm": float(np.ptp(mm)),
            "amplitude_std_db": float(np.std(d.amplitude_db)),
            "residual_db": d.residual_db,
            "rate_mm_per_min": rate,
            "se_rate_mm_per_min": se_rate,
            "temp_deg_per_degc": tc_deg,
            "se_temp_deg_per_degc": se_deg,
            "temp_mm_per_degc": tc_mm,
            "se_temp_mm_per_degc": se_mm,
            "adev_tau_s": tau.tolist(),
            "adev_mm": adev.tolist(),
        }

    def summary(self):
        t = self.temperature
        return {
            "frames": int(self.t.size),
            "duration_s": float(self.t[-1]),
            "f_centre": self.f_centre,
            "sign_known": self.sign_known,
            "temperature_range_degc": (
                None if t is None else [float(t.min()), float(t.max())]
            ),
            **{name: self.stats(name) for name in self.lines},
        }


def _line(z, k, r, f_c):
    """Series of the line at bin `k` (range `r`) from its values `z[K, 2]`."""
    phi = unwrapped_phase(z)
    amp = np.abs(z).mean(axis=1)
    zbar = z.mean(axis=0)
    return DriftLine(
        bin=int(k),
        range=float(r),
        phase=phi,
        range_drift=C * phi / (4.0 * np.pi * f_c),
        amplitude_db=20.0 * np.log10(amp / amp.mean()),
        residual_db=float(
            10.0 * np.log10(np.mean(np.abs(z - zbar) ** 2) / np.mean(np.abs(zbar) ** 2))
        ),
    )


def estimate_drift(rec: Recording, cal: Calibration, window="hann", zpad=4, count=1):
    """Phase and amplitude series of the `count` strongest lines beyond the
    zero-range main lobe of the mean frame ("target", then "line2", ...) and
    of the leakage inside that
    lobe over the frames of a static recording. The leakage overlaps its own
    image, so its `range_drift` scale is nominal; `residual_db` is its
    stability against a background reference."""
    spec, nu = line_values(rec.codes, rec.sweep, cal, window, zpad)
    peaks, leak = select_bins(spec, nu, window, count)
    r = range_axis(next_fast_len(nu * zpad, real=True), rec.sweep, cal) - cal.r_cal
    f_c = centre_frequency(rec.sweep, cal, ramp_length(cal.nr(rec.sweep), cal.ng))
    names = ["target"] + [f"line{i}" for i in range(2, len(peaks) + 1)]
    lines = {n: _line(spec[..., k], k, r[k], f_c) for n, k in zip(names, peaks)}
    lines["leakage"] = _line(spec[..., leak], leak, r[leak], f_c)
    t = np.asarray(rec.t_host, dtype=np.float64)
    temp = rec.temperature
    return DriftResult(
        t=t - t[0],
        temperature=None if temp is None else np.asarray(temp, dtype=np.float64),
        f_centre=float(f_c),
        sign_known=cal.first_up is not None,
        lines=lines,
    )


def plot_drift(res: DriftResult, path) -> None:
    """Range drift and temperature against time, and Allan deviation."""
    fig = Figure(figsize=(8.0, 9.0), layout="constrained")
    ax_r, ax_t, ax_a = fig.subplots(3, 1)
    minutes = res.t / 60.0
    for name, line in res.lines.items():
        ax_r.plot(minutes, 1e3 * line.range_drift, lw=1.5, label=name)
        tau, adev = allan_deviation(1e3 * line.range_drift, res.tau0)
        ax_a.loglog(tau, adev, "o-", lw=1.5, ms=4, label=name)
    ax_r.set(xlabel="time (min)", ylabel="range drift (mm)")
    ax_r.legend(frameon=False)
    if res.temperature is not None:
        ax_t.plot(minutes, res.temperature, color="0.3", lw=1.5)
    ax_t.set(xlabel="time (min)", ylabel="board temperature (degC)")
    ax_a.set(xlabel="tau (s)", ylabel="Allan deviation (mm)")
    ax_a.legend(frameon=False)
    for ax in (ax_r, ax_t, ax_a):
        ax.grid(alpha=0.3)
    fig.savefig(path, dpi=120)


def run(args):
    """`qmrdk drift`: estimate, print and optionally save the report and plot."""
    res = estimate_drift(
        Recording.load(args.recording), Calibration.load(args.cal), count=args.lines
    )
    text = json.dumps(res.summary(), indent=2)
    if args.report:
        pathlib.Path(args.report).write_text(text, encoding="utf-8")
    if args.out:
        plot_drift(res, args.out)
    print(text)
    return 0

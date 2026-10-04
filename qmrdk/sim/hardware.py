"""Board model: synthesiser, IF chain and ADC (docs/simulation.md §4)."""

import dataclasses

import finufft
import numba
import numpy as np
from scipy import signal

from qmrdk.config import Calibration, Sweep
from qmrdk.constants import A_FS, ADC_MAX, C
from qmrdk.sim.propagation import Antenna, Paths

PREROLL = 4e-3


@dataclasses.dataclass(frozen=True)
class Hardware:
    """True board parameters (docs/simulation.md §4.1).

    `f_prev` None means `sweep.f1`. `hp_order` or `lp_order` 0 removes that
    filter.
    """

    fs: float = 21_977.0
    t_start: float = 2.3e-3
    t_reset: float = 0.4e-3
    f_prev: float | None = None
    first_up: bool = True
    pll_fn: float = 4e3
    pll_zeta: float = 0.7
    hp_fc: float = 40.0
    hp_order: int = 1
    lp_fc: float = 9e3
    lp_order: int = 4
    r_cal: float = 0.35
    tx_offset: tuple[float, float, float] = (-0.06, 0.02, 0.0)
    rx_offset: tuple[float, float, float] = (0.06, 0.02, 0.0)
    antenna: Antenna = Antenna(g0=10.0, beamwidth=60.0)
    leak_amp: float = 1e-3
    leak_range: float = 0.1
    pt: float = 10e-3
    gain: float = 2.0e3
    noise: float = 2e-4
    dc: float = 0.0
    oversample: int = 8

    def __post_init__(self):
        if not 0.0 <= self.t_reset <= self.t_start:
            raise ValueError("need 0 <= t_reset <= t_start")
        if self.pll_fn <= 0 or self.pll_zeta <= 0:
            raise ValueError("pll_fn and pll_zeta must be positive")

    @property
    def fsim(self):
        """Analogue simulation rate, Hz."""
        return self.oversample * self.fs

    def nr(self, sweep: Sweep) -> float:
        """Samples per ramp."""
        return sweep.ramp_time * self.fs

    def turnarounds(self, sweep: Sweep, n: int) -> np.ndarray:
        """Commanded turnaround sample positions in a frame of `n` samples."""
        if sweep.kind == "cw":
            return np.empty(0)
        n0 = self.t_start * self.fs
        k = np.arange(max(0, int(np.ceil((n - n0) / self.nr(sweep)))))
        return n0 + k * self.nr(sweep)

    def if_zpk(self):
        """Digital zeros, poles and gain of the simulated IF cascade at `fsim`."""
        parts = [
            signal.butter(order, fc, btype, fs=self.fsim, output="zpk")
            for order, fc, btype in (
                (self.hp_order, self.hp_fc, "highpass"),
                (self.lp_order, self.lp_fc, "lowpass"),
            )
            if order > 0
        ]
        return (
            np.concatenate([np.empty(0, complex)] + [p[0] for p in parts]),
            np.concatenate([np.empty(0, complex)] + [p[1] for p in parts]),
            float(np.prod([p[2] for p in parts])),
        )

    def if_group_delay(self, f) -> np.ndarray:
        """Group delay (s) of the simulated IF cascade at frequency `f` (Hz),
        0 < f < fsim / 2."""
        z, p, _ = self.if_zpk()
        e = np.exp(2j * np.pi * np.asarray(f, dtype=np.float64) / self.fsim)[..., None]
        tau = np.sum((p / (e - p)).real, axis=-1) - np.sum((z / (e - z)).real, axis=-1)
        return tau / self.fsim

    def calibration(self, sweep: Sweep, f_ref: float | None = None) -> Calibration:
        """Ideal calibration: `n0` is the commanded first turnaround plus the IF
        group delay at `f_ref` (default fs / 4, the centre of the beat band),
        in samples; `r_cal` is the true extra delay as range; CW has no
        ramp direction."""
        f_ref = 0.25 * self.fs if f_ref is None else f_ref
        return Calibration(
            fs=self.fs,
            n0=float(self.fs * (self.t_start + self.if_group_delay(f_ref))),
            first_up=None if sweep.kind == "cw" else self.first_up,
            r_cal=self.r_cal,
            tx_offset=tuple(self.tx_offset),
            rx_offset=tuple(self.rx_offset),
        )


def f_cmd(sweep: Sweep, hw: Hardware, t) -> np.ndarray:
    """Commanded synthesiser frequency at times `t` (s from frame start)."""
    t = np.asarray(t, dtype=np.float64)
    f_prev = sweep.f1 if hw.f_prev is None else hw.f_prev
    if sweep.kind == "cw":
        return np.where(t < hw.t_reset, f_prev, sweep.f0)
    u = np.maximum(t - hw.t_start, 0.0) / sweep.ramp_time
    tri = sweep.bandwidth * (1.0 - np.abs(1.0 - np.mod(u, 2.0)))
    f = sweep.f0 + tri if hw.first_up else sweep.f1 - tri
    return np.where(t < hw.t_reset, f_prev, f)


def _knots(sweep: Sweep, hw: Hardware, t_max: float):
    """Breakpoints of the commanded frequency: times, jumps, slope changes."""
    f_prev = sweep.f1 if hw.f_prev is None else hw.f_prev
    if sweep.kind == "cw":
        return np.array([hw.t_reset]), np.array([sweep.f0 - f_prev]), np.zeros(1)
    f_start = sweep.f0 if hw.first_up else sweep.f1
    k = np.arange(max(0, int(np.floor((t_max - hw.t_start) / sweep.ramp_time)) + 1))
    sign = (1.0 if hw.first_up else -1.0) * (-1.0) ** k
    return (
        np.concatenate([[hw.t_reset], hw.t_start + k * sweep.ramp_time]),
        np.concatenate([[f_start - f_prev], np.zeros(k.size)]),
        np.concatenate([[0.0], sign * sweep.slope * np.where(k == 0, 1.0, 2.0)]),
    )


def f_tx(sweep: Sweep, hw: Hardware, t) -> np.ndarray:
    """Synthesiser output frequency at arbitrary times `t` (s from frame start).

    The tracking error (1 − H) s⁻² = 1 / (s² + 2ζωs + ω²) of the type-2 loop is
    summed in closed form over the step and slope breakpoints of the command.
    """
    t = np.asarray(t, dtype=np.float64)
    out = f_cmd(sweep, hw, t)
    if t.size == 0:
        return out
    w = 2.0 * np.pi * hw.pll_fn
    beta = w * np.sqrt(complex(hw.pll_zeta**2 - 1.0))
    lam = -hw.pll_zeta * w + beta
    for tk, jump, dslope in zip(*_knots(sweep, hw, float(t.max()))):
        tau = np.maximum(t - tk, 0.0)
        if beta == 0:
            ramp = tau * np.exp(lam * tau)
        else:
            ramp = np.exp(lam * tau) * -np.expm1(-2.0 * beta * tau) / (2.0 * beta)
        step = lam * ramp + np.exp((lam - 2.0 * beta) * tau)
        out = out - np.where(t >= tk, (jump * step + dslope * ramp).real, 0.0)
    return out


@numba.njit(parallel=True, cache=True)
def _moving_sum(f, t, delay, rate, amp):  # pragma: no cover
    out = np.empty(f.size)
    for m in numba.prange(f.size):  # pylint: disable=not-an-iterable
        acc = 0.0
        for p in range(delay.size):
            ph = 2.0 * np.pi * f[m] * (delay[p] + rate[p] * t[m])
            acc += amp[p].real * np.cos(ph) - amp[p].imag * np.sin(ph)
        out[m] = acc
    return out


def _if_sum(paths: Paths, hw: Hardware, f, t) -> np.ndarray:
    """Unfiltered IF for 1 W and unit gain at times `t`, synthesiser frequencies `f`."""
    extra = 2.0 * hw.r_cal / C
    static = paths.speed == 0
    delay = np.concatenate([[2.0 * hw.leak_range / C], paths.delay[static]]) + extra
    amp = np.concatenate([[complex(hw.leak_amp)], paths.amp[static]]).astype(
        np.complex128
    )
    s = finufft.nufft1d3(delay, amp, 2.0 * np.pi * f, eps=1e-12, isign=1).real
    if not static.all():
        s += _moving_sum(
            f,
            t,
            paths.delay[~static] + extra,
            2.0 * paths.speed[~static] / C,
            paths.amp[~static].astype(np.complex128),
        )
    return s


def synthesize_volts(paths: Paths, sweep: Sweep, hw: Hardware, n: int) -> np.ndarray:
    """Noiseless filtered IF (V) at the sample instants k / fs, k < n."""
    m0 = min(0, int(np.floor((hw.t_reset - PREROLL) * hw.fsim)))
    t = np.arange(m0, (n - 1) * hw.oversample + 1) / hw.fsim
    v = np.sqrt(hw.pt) * hw.gain * _if_sum(paths, hw, f_tx(sweep, hw, t), t)
    z, p, k = hw.if_zpk()
    if p.size:
        sos = signal.zpk2sos(z, p, k)
        v, _ = signal.sosfilt(sos, v, zi=signal.sosfilt_zi(sos) * v[0])
    return v[-m0 :: hw.oversample][:n]


def synthesize(
    paths: Paths, sweep: Sweep, hw: Hardware, n: int, rng=None
) -> np.ndarray:
    """One frame of `n` ADC codes (uint16)."""
    rng = np.random.default_rng(rng)
    v = synthesize_volts(paths, sweep, hw, n) + hw.dc + rng.normal(0.0, hw.noise, n)
    code = np.rint((v + A_FS) * ADC_MAX / (2.0 * A_FS))
    return np.clip(code, 0, ADC_MAX).astype(np.uint16)

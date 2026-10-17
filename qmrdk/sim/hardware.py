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
SETTLE = 1e-6
T_REF = 25.0


@dataclasses.dataclass(frozen=True)
class Hardware:
    """True board parameters (docs/simulation.md §4.1).

    `f_prev` None means `sweep.f1`. `hp_order` or `lp_order` 0 removes that
    filter. `continuous`: the sweep has been running since before the frame
    (no reset, no `f_prev`); `t_start` is then any real.
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
    delay_tc: float = 0.0
    temperature: float = T_REF
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
    continuous: bool = False

    def __post_init__(self):
        if not self.continuous and not 0.0 <= self.t_reset <= self.t_start:
            raise ValueError("need 0 <= t_reset <= t_start")
        if self.pll_fn <= 0 or self.pll_zeta <= 0:
            raise ValueError("pll_fn and pll_zeta must be positive")

    @property
    def r_extra(self):
        """Extra delay as range at `temperature`: `r_cal` at `T_REF` plus
        `delay_tc` (m/degC) per degree above it."""
        return self.r_cal + self.delay_tc * (self.temperature - T_REF)

    @property
    def fsim(self):
        """Analogue simulation rate, Hz."""
        return self.oversample * self.fs

    @property
    def t_first(self):
        """Start of the simulation grid, s: `PREROLL` before the restart or,
        for a continuous sweep, also until the slowest IF pole has decayed to
        `SETTLE` (the filters start in steady state for a constant input)."""
        if not self.continuous:
            return min(0.0, self.t_reset - PREROLL)
        p = self.if_zpk()[1]
        settle = np.log(SETTLE) / np.log(np.abs(p).max()) / self.fsim if p.size else 0
        return -max(PREROLL, float(settle))

    def nr(self, sweep: Sweep) -> float:
        """Samples per ramp."""
        return sweep.ramp_time * self.fs

    def turnarounds(self, sweep: Sweep, n: int) -> np.ndarray:
        """Commanded turnaround (sawtooth: ramp start) sample positions in a
        frame of `n` samples."""
        if sweep.kind == "cw":
            return np.empty(0)
        n0 = self.t_start * self.fs
        step = (sweep.ramp_time + sweep.dwell) * self.fs
        step = step if sweep.kind == "saw" else self.nr(sweep)
        k0 = int(np.ceil(-n0 / step)) if self.continuous else 0
        return n0 + step * np.arange(k0, max(k0, int(np.ceil((n - n0) / step))))

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
        delay = float(self.if_group_delay(f_ref))
        return Calibration(
            fs=self.fs,
            n0=float(self.fs * (self.t_start + delay)),
            if_delay=delay,
            first_up=None if sweep.kind == "cw" else self.first_up,
            r_cal=self.r_extra,
            tx_offset=tuple(self.tx_offset),
            rx_offset=tuple(self.rx_offset),
        )


def f_cmd(sweep: Sweep, hw: Hardware, t) -> np.ndarray:
    """Commanded synthesiser frequency at times `t` (s from frame start):
    triangle with turnarounds every `ramp_time`, or sawtooth ("saw") ramps of
    `ramp_time` from the start frequency, each followed by a hop back and a
    rest of `dwell`."""
    t = np.asarray(t, dtype=np.float64)
    f_prev = sweep.f1 if hw.f_prev is None else hw.f_prev
    if sweep.kind == "cw":
        if hw.continuous:
            return np.full_like(t, sweep.f0)
        return np.where(t < hw.t_reset, f_prev, sweep.f0)
    u = t - hw.t_start
    u = u if hw.continuous else np.maximum(u, 0.0)
    if sweep.kind == "saw":
        ph = np.mod(u, sweep.ramp_time + sweep.dwell)
        rise = sweep.slope * np.where(ph < sweep.ramp_time, ph, 0.0)
    else:
        u = u / sweep.ramp_time
        rise = sweep.bandwidth * (1.0 - np.abs(1.0 - np.mod(u, 2.0)))
    f = sweep.f0 + rise if hw.first_up else sweep.f1 - rise
    return f if hw.continuous else np.where(t < hw.t_reset, f_prev, f)


def _knots(sweep: Sweep, hw: Hardware, t_min: float, t_max: float):
    """Breakpoints of the commanded frequency: times, jumps, slope changes.
    A continuous sweep has those from `PREROLL` before `t_min` on, and tracks
    the command exactly before the first."""
    f_prev = sweep.f1 if hw.f_prev is None else hw.f_prev
    if sweep.kind == "cw":
        if hw.continuous:
            return np.empty(0), np.empty(0), np.empty(0)
        return np.array([hw.t_reset]), np.array([sweep.f0 - f_prev]), np.zeros(1)
    f_start = sweep.f0 if hw.first_up else sweep.f1
    up = 1.0 if hw.first_up else -1.0
    lo = t_min - PREROLL
    if sweep.kind == "saw":
        period = sweep.ramp_time + sweep.dwell
        j0 = (
            np.ceil((lo - hw.t_start - sweep.ramp_time) / period)
            if hw.continuous
            else 0
        )
        a = hw.t_start + period * np.arange(
            j0, np.floor((t_max - hw.t_start) / period) + 1
        )
        times = np.concatenate([a, a + sweep.ramp_time])
        jumps = np.concatenate(
            [np.zeros(a.size), np.full(a.size, -up * sweep.bandwidth)]
        )
        dslope = up * sweep.slope * np.repeat([1.0, -1.0], a.size)
    else:
        k0 = int(np.ceil((lo - hw.t_start) / sweep.ramp_time)) if hw.continuous else 0
        k = np.arange(
            k0, max(k0, int(np.floor((t_max - hw.t_start) / sweep.ramp_time)) + 1)
        )
        times = hw.t_start + k * sweep.ramp_time
        jumps = np.zeros(k.size)
        sign = up * (-1.0) ** k
        dslope = sign * sweep.slope * np.where((k == 0) & (not hw.continuous), 1.0, 2.0)
    if hw.continuous:
        keep = times >= lo
        return times[keep], jumps[keep], dslope[keep]
    return (
        np.concatenate([[hw.t_reset], times]),
        np.concatenate([[f_start - f_prev], jumps]),
        np.concatenate([[0.0], dslope]),
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
    for tk, jump, dslope in zip(*_knots(sweep, hw, float(t.min()), float(t.max()))):
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
    extra = 2.0 * hw.r_extra / C
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
    m0 = int(np.floor(hw.t_first * hw.fsim))
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

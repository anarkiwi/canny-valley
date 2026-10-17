"""Sweep parameters, board calibration and scan geometry."""

import dataclasses
import json
import pathlib

import numpy as np

from qmrdk.constants import C, FS_NOMINAL


@dataclasses.dataclass(frozen=True)
class Sweep:
    """Read-back sweep parameters of a capture.

    Attributes:
        kind: sweep type (protocol §3.1): "ramp" (0), "tri" (1), "triangle"
            (2, AUTO), "cw" (3) or, with CV firmware only, "saw" (4).
        ramp_time: one-way ramp time, s.
        dwell: rest at the start frequency between sawtooth ramps, s (0 keeps
            the board's setting).
    """

    f0: float = 2.4e9
    f1: float = 2.5e9
    ramp_time: float = 16e-3
    kind: str = "triangle"
    dwell: float = 0.0

    @property
    def bandwidth(self):
        return self.f1 - self.f0

    @property
    def slope(self):
        return self.bandwidth / self.ramp_time

    @property
    def fc(self):
        return self.f0 if self.kind == "cw" else 0.5 * (self.f0 + self.f1)

    @property
    def lam(self):
        return C / self.fc

    @property
    def lam_min(self):
        return C / max(self.f0, self.f1)


@dataclasses.dataclass(frozen=True)
class Timing:
    """Device sweep timing reported by CV firmware (protocol §6): exact
    sample rate `fs` (Hz), commanded first turnaround `n0` (samples from the
    acquisition start) and the direction of the ramp starting there."""

    fs: float
    n0: float
    first_up: bool


@dataclasses.dataclass(frozen=True)
class Calibration:
    """Board constants determined by the calibration procedure (docs/calibration.md).

    Attributes:
    fs: ADC sample rate relative to the sweep clock, so that a ramp spans
        `ramp_time * fs` samples.
    n0: apparent sample position of the first turnaround of a frame (fractional),
        as seen in the IF (includes IF filter delay).
    first_up: True if the ramp starting at `n0` sweeps up, False if down,
        None if unknown (SAR then uses the sharpness test).
    ng: guard samples discarded at each end of a ramp.
    r_cal: fixed delay of cables, antennas and IF filter expressed as range, m.
    tx_offset, rx_offset: antenna phase centres relative to the sled reference
        point (x along the rail, y towards the scene, z up), m.
    sled_sigma: measured sled position repeatability (1 sigma), m.
    if_delay: IF filter phase delay at the reflector's beat frequency, s;
        the apparent turnaround lags the commanded one by `if_delay * fs`.
    """

    fs: float = FS_NOMINAL
    n0: float = 0.0
    first_up: bool | None = None
    ng: int = 0
    r_cal: float = 0.0
    tx_offset: tuple[float, float, float] = (-0.05, 0.0, 0.0)
    rx_offset: tuple[float, float, float] = (0.05, 0.0, 0.0)
    sled_sigma: float = 0.0
    if_delay: float = 0.0

    def nr(self, sweep: Sweep) -> float:
        """Samples per ramp."""
        return sweep.ramp_time * self.fs

    def period(self, sweep: Sweep) -> float | None:
        """Samples per sawtooth period; None for the triangle-like kinds."""
        return (
            (sweep.ramp_time + sweep.dwell) * self.fs if sweep.kind == "saw" else None
        )

    def with_timing(self, fs: float, n0: float, first_up: bool) -> "Calibration":
        """Copy for the device timing of CV firmware (`Timing` fields): `fs`
        and `first_up` from the device, `n0` the commanded turnaround delayed
        by `if_delay`."""
        return dataclasses.replace(
            self, fs=fs, n0=n0 + self.if_delay * fs, first_up=bool(first_up)
        )

    def to_json(self) -> str:
        return json.dumps(dataclasses.asdict(self), indent=2)

    @classmethod
    def from_json(cls, text: str) -> "Calibration":
        d = json.loads(text)
        for k in ("tx_offset", "rx_offset"):
            if k in d:
                d[k] = tuple(d[k])
        return cls(**d)

    def save(self, path) -> None:
        pathlib.Path(path).write_text(self.to_json(), encoding="utf-8")

    @classmethod
    def load(cls, path) -> "Calibration":
        return cls.from_json(pathlib.Path(path).read_text(encoding="utf-8"))


@dataclasses.dataclass(frozen=True)
class ScanGeometry:
    """Rail placement: the sled reference point is at (x, 0, height) for rail
    position x; the scene lies towards +y, the ground is the plane z = 0."""

    height: float = 1.0

    def antenna_positions(self, x_pos, cal: Calibration):
        """Transmit and receive phase centres, each [positions, 3]."""
        x_pos = np.asarray(x_pos, dtype=np.float64)
        ref = np.zeros((x_pos.size, 3))
        ref[:, 0] = x_pos
        ref[:, 2] = self.height
        return ref + np.asarray(cal.tx_offset), ref + np.asarray(cal.rx_offset)

"""Propagation paths between antennas and a compiled scene (docs/simulation.md §3)."""

import dataclasses

import numpy as np

from qmrdk.constants import C


@dataclasses.dataclass(frozen=True)
class Antenna:
    """Antenna gain model: field pattern cos(theta)**q in front, 0 behind.

    g0: boresight power gain (linear). beamwidth: half-power full width, deg.
    yaw: boresight azimuth from +x, deg. elevation: boresight elevation, deg.
    """

    g0: float = 10.0
    beamwidth: float = 60.0
    yaw: float = 90.0
    elevation: float = 0.0


@dataclasses.dataclass(frozen=True)
class Paths:
    """Propagation paths for one tx/rx placement.

    delay: total path length / c, s. amp: complex field amplitude for 1 W
    transmitted, sqrt(W). speed: rate of change of total path length / 2
    (negative approaching), m/s. kind: 0 scatterer without bounce,
    1 scatterer with at least one bounce, 2 specular.
    """

    delay: np.ndarray
    amp: np.ndarray
    speed: np.ndarray
    kind: np.ndarray

    @classmethod
    def from_ranges(cls, ranges, amps, speeds=None):
        """Paths for monostatic point scatterers at one-way `ranges` (m)."""
        ranges = np.atleast_1d(np.asarray(ranges, dtype=np.float64))
        amps = np.broadcast_to(np.asarray(amps, dtype=np.complex128), ranges.shape)
        speeds = (
            np.zeros_like(ranges)
            if speeds is None
            else np.broadcast_to(np.asarray(speeds, dtype=np.float64), ranges.shape)
        )
        return cls(
            2.0 * ranges / C,
            amps.copy(),
            speeds.copy(),
            np.zeros(ranges.shape, dtype=np.int8),
        )

    def __len__(self):
        return self.delay.size

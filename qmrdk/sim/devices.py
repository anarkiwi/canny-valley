"""Simulated sled and radar behind the qmrdk.radar interfaces (docs/simulation.md §5)."""

import numpy as np

from qmrdk.config import ScanGeometry, Sweep
from qmrdk.sim import propagation
from qmrdk.sim.hardware import Hardware, synthesize


class SimSled:
    """Sled whose true position is the commanded one plus `bias` and N(0, sigma)."""

    def __init__(self, sigma: float = 0.0, bias: float = 0.0, seed=None):
        self.sigma = sigma
        self.bias = bias
        self._rng = np.random.default_rng(seed)
        self._x = 0.0
        self.true_position = bias

    def home(self) -> None:
        self.move_to(0.0)

    def move_to(self, x: float) -> None:
        self._x = float(x)
        self.true_position = (
            self._x + self.bias + self.sigma * self._rng.standard_normal()
        )

    def position(self) -> float:
        return self._x


class SimRadar:
    """Radar synthesising frames for the scene at the sled's true position."""

    def __init__(
        self,
        scene_geometry,
        hardware: Hardware,
        sweep: Sweep,
        sled,
        scan_geometry: ScanGeometry,
        seed=None,
    ):
        self.geometry = scene_geometry
        self.hardware = hardware
        self.sweep = sweep
        self.sled = sled
        self.scan_geometry = scan_geometry
        self._rng = np.random.default_rng(seed)
        self._paths = {}

    def paths(self) -> propagation.Paths:
        """Propagation paths at the sled's true position (cached)."""
        x = getattr(self.sled, "true_position", None)
        x = float(self.sled.position() if x is None else x)
        if x not in self._paths:
            tx, rx = self.scan_geometry.antenna_positions([x], self.hardware)
            self._paths[x] = propagation.paths(  # pylint: disable=no-member
                self.geometry, tx[0], rx[0], self.sweep.lam, self.hardware.antenna
            )
        return self._paths[x]

    def capture(self, n: int) -> np.ndarray:
        """One frame of `n` uint16 codes."""
        return synthesize(self.paths(), self.sweep, self.hardware, n, self._rng)

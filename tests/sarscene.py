"""Small scene and short simulated rail scan shared by the rendering and CLI tests."""

import copy

from qmrdk.config import ScanGeometry, Sweep
from qmrdk.scan import run_scan, scan_positions
from qmrdk.sim.devices import SimRadar, SimSled
from qmrdk.sim.hardware import Hardware
from qmrdk.sim.scene import compile_scene

TINY = {
    "name": "tiny",
    "seed": 1,
    "diffuse_density": 0.0,
    "ground": {"material": "soil", "clutter_density": 0.0, "extent": [-4, 4, 2, 12]},
    "objects": [
        {"type": "wall", "p0": [-3, 10], "p1": [3, 10], "z": [0, 3], "material": "concrete"},
        {"type": "box", "center": [2.5, 6], "size": [1.0, 0.5], "z": [0, 1.5], "material": "metal"},
        {"type": "cylinder", "center": [-2.5, 4], "radius": 0.05, "z": [0, 1], "material": "metal"},
        {"type": "reflector", "pos": [0.0, 5.0, 1.0], "rcs": 10.0, "boresight": -90},
        {"type": "reflector", "pos": [-1.0, 7.0, 1.0], "rcs": 1.0, "boresight": 90},
        {"type": "point", "pos": [-1.5, 3.0, 1.0], "rcs": 0.5},
    ],
}  # fmt: skip
SWEEP = Sweep()
GEOMETRY = ScanGeometry(height=1.0)
HW = Hardware(gain=2e3)


def tiny():
    return copy.deepcopy(TINY)


def scan(length=0.3, n=2048, scene=None, seed=1):
    """(compiled geometry, recording) of a short simulated scan."""
    geom = compile_scene(tiny() if scene is None else scene, SWEEP.lam)
    sled = SimSled(seed=seed)
    radar = SimRadar(geom, HW, SWEEP, sled, GEOMETRY, seed=seed)
    rec = run_scan(radar, sled, scan_positions(SWEEP, length), n, GEOMETRY)
    return geom, rec

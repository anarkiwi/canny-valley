"""SAR scan sequencer: move, settle, capture, record position."""

import time

import numpy as np
from tqdm import tqdm

from qmrdk.config import ScanGeometry, Sweep
from qmrdk.radar import Radar, Sled
from qmrdk.recording import Recording


def scan_positions(sweep: Sweep, length: float, dx: float | None = None) -> np.ndarray:
    """Rail positions 0..length with spacing at most `dx` (default lam_min / 4)."""
    dx = sweep.lam_min / 4.0 if dx is None else dx
    return np.linspace(0.0, length, int(np.ceil(length / dx)) + 1)


def frames(radar: Radar, n: int, count: int, desc: str = "capture") -> np.ndarray:
    """`count` frames at the current position, [count, n] codes."""
    return np.stack([radar.capture(n) for _ in tqdm(range(count), desc=desc)])


def run_scan(
    radar: Radar,
    sled: Sled,
    positions,
    n: int = 4096,
    geometry: ScanGeometry | None = None,
    extra: dict | None = None,
    desc: str = "scan",
) -> Recording:
    """One capture per rail position; records the sled's reported position."""
    positions = np.asarray(positions, dtype=np.float64)
    codes = np.empty((positions.size, n), dtype=np.uint16)
    t_host = np.empty(positions.size)
    x_pos = np.empty(positions.size)
    sled.home()
    for i, x in enumerate(tqdm(positions, desc=desc, unit="pos")):
        sled.move_to(float(x))
        x_pos[i] = sled.position()
        t_host[i] = time.time()
        codes[i] = radar.capture(n)
    return Recording(
        codes=codes,
        sweep=radar.sweep,
        t_host=t_host,
        x_pos=x_pos,
        geometry=geometry or ScanGeometry(),
        extra=dict(extra or {}),
    )

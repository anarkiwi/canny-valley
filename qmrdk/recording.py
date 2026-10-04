"""Native `.npz` recordings (docs/protocol.md §5)."""

import dataclasses
import json

import numpy as np

from qmrdk import __version__
from qmrdk.config import ScanGeometry, Sweep


@dataclasses.dataclass
class Recording:
    """A sequence of captures with their sweep, timestamps and, for SAR scans,
    sled positions. `extra` holds free-form metadata (e.g. surveyed target)."""

    codes: np.ndarray
    sweep: Sweep
    t_host: np.ndarray
    x_pos: np.ndarray | None = None
    geometry: ScanGeometry | None = None
    extra: dict = dataclasses.field(default_factory=dict)

    def save(self, path) -> None:
        meta = {
            "sweep": dataclasses.asdict(self.sweep),
            "geometry": (
                None if self.geometry is None else dataclasses.asdict(self.geometry)
            ),
            "software": __version__,
            "extra": self.extra,
        }
        arrays = {"codes": np.asarray(self.codes, dtype=np.uint16)}
        arrays["t_host"] = np.asarray(self.t_host, dtype=np.float64)
        if self.x_pos is not None:
            arrays["x_pos"] = np.asarray(self.x_pos, dtype=np.float64)
        np.savez_compressed(path, meta=np.array(json.dumps(meta)), **arrays)

    @classmethod
    def load(cls, path) -> "Recording":
        with np.load(path, allow_pickle=False) as f:
            meta = json.loads(str(f["meta"]))
            geometry = meta.get("geometry")
            return cls(
                codes=f["codes"],
                sweep=Sweep(**meta["sweep"]),
                t_host=f["t_host"],
                x_pos=f["x_pos"] if "x_pos" in f else None,
                geometry=None if geometry is None else ScanGeometry(**geometry),
                extra=meta.get("extra", {}),
            )

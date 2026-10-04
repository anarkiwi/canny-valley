"""Scene description and compilation to flat geometry arrays (docs/simulation.md §2)."""

import dataclasses
import json
import math
import pathlib

import numba
import numpy as np

ISOTROPIC, LAMBERTIAN, TRIHEDRAL, GLINT = 0, 1, 2, 3

PEC = complex(np.inf, 0.0)

MATERIALS = {
    "metal": {"eps_r": PEC, "sigma0": 0.01, "smooth": True},
    "concrete": {"eps_r": 6.0 - 0.6j, "sigma0": 0.05, "smooth": True},
    "brick": {"eps_r": 4.5 - 0.4j, "sigma0": 0.08, "smooth": True},
    "wood": {"eps_r": 2.0 - 0.2j, "sigma0": 0.03, "smooth": True},
    "glass": {"eps_r": 6.5 - 0.1j, "sigma0": 0.002, "smooth": True},
    "foliage": {"eps_r": 1.5 - 0.3j, "sigma0": 0.2, "smooth": False},
    "soil": {"eps_r": 10.0 - 2.0j, "sigma0": 0.002, "smooth": True},
    "grass": {"eps_r": 8.0 - 2.5j, "sigma0": 0.01, "smooth": False},
}

SCENE_DIR = pathlib.Path(__file__).resolve().parent.parent / "scenes"


def _gamma_py(eps, c, tm):
    """Fresnel coefficient in the image convention (PEC gives -1)."""
    if math.isinf(eps.real):
        return -1.0 + 0.0j
    root = np.sqrt(eps - 1.0 + c * c)
    if tm:
        return (root - eps * c) / (root + eps * c)
    return (c - root) / (c + root)


gamma = numba.njit(cache=True)(_gamma_py)
_gamma_uf = numba.vectorize(["complex128(complex128, float64, boolean)"], cache=True)(
    _gamma_py
)


def fresnel(eps_r, cos_incidence, pol):
    """Fresnel reflection coefficient relative to the mirror image of the
    incident field, so a perfect conductor (eps_r real part inf) gives -1.

    Args:
        eps_r: complex relative permittivity.
        cos_incidence: cosine of the angle from the surface normal.
        pol: "h" field parallel to the surface (TE) or "v" (TM).
    """
    if pol not in ("h", "v"):
        raise ValueError(f"polarisation must be 'h' or 'v', not {pol!r}")
    c = np.clip(np.asarray(cos_incidence, dtype=np.float64), 0.0, 1.0)
    return _gamma_uf(np.asarray(eps_r, dtype=np.complex128), c, pol == "v")


def pattern_exponent(beamwidth):
    """Exponent q of a cos(theta)**q field pattern that is -3 dB in power at
    half the full beamwidth (deg)."""
    return math.log(0.5) / (2.0 * math.log(math.cos(math.radians(beamwidth) / 2.0)))


def material(spec):
    """Material by name or inline {"eps_r": [re, im] or null, "sigma0", "smooth"}."""
    if isinstance(spec, str):
        return MATERIALS[spec]
    eps = spec["eps_r"]
    return {
        "eps_r": PEC if eps is None else complex(*eps),
        "sigma0": float(spec["sigma0"]),
        "smooth": bool(spec.get("smooth", True)),
    }


def load_scene(path_or_dict):
    """Scene dict from a JSON file path or an already parsed dict."""
    if isinstance(path_or_dict, dict):
        return path_or_dict
    return json.loads(pathlib.Path(path_or_dict).read_text(encoding="utf-8"))


def builtin_scene(name):
    """Scene shipped in qmrdk/scenes/<name>.json."""
    return load_scene(SCENE_DIR / f"{name}.json")


@dataclasses.dataclass(frozen=True)
class Geometry:
    """Compiled scene: F vertical facets, S scatterers and the ground plane z = 0.

    Attributes:
        scene, lam, pol: source scene dict, wavelength, polarisation ("h"/"v").
        ground_eps, ground_mirror: ground permittivity and mirror flag.
        facet_p0, facet_p1, facet_z: (F, 2) endpoints (xy) and (z0, z1).
        facet_normal: (F, 2) unit normal, to the right of p0 -> p1.
        facet_eps, facet_sigma0, facet_mirror: material (eps inf for metal).
        facet_obj: index of the owning object in scene["objects"].
        scat_pos, scat_amp: (S, 3) positions, sqrt(rcs) e^{j phi}.
        scat_kind: ISOTROPIC, LAMBERTIAN, TRIHEDRAL or GLINT.
        scat_par: (S, 3) trihedral (boresight x, y, q) or glint (radius, 0, 0).
        scat_host, scat_normal: host facet (-1 none), (S, 3) normal (0 none).
    """

    scene: dict
    lam: float
    pol: str
    ground_eps: complex
    ground_mirror: bool
    facet_p0: np.ndarray
    facet_p1: np.ndarray
    facet_z: np.ndarray
    facet_normal: np.ndarray
    facet_eps: np.ndarray
    facet_sigma0: np.ndarray
    facet_mirror: np.ndarray
    facet_obj: np.ndarray
    scat_pos: np.ndarray
    scat_amp: np.ndarray
    scat_kind: np.ndarray
    scat_par: np.ndarray
    scat_host: np.ndarray
    scat_normal: np.ndarray

    @property
    def n_facets(self):
        return self.facet_obj.size

    @property
    def n_scatterers(self):
        return self.scat_kind.size


def _polygon(obj):
    """Counter-clockwise footprint vertices (xy) of a box or cylinder."""
    cx, cy = obj["center"]
    if obj["type"] == "box":
        a = math.radians(obj.get("angle", 0.0))
        hx, hy = 0.5 * np.asarray(obj["size"], dtype=np.float64)
        local = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
        rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        return local @ rot.T + (cx, cy)
    n = obj.get("n_facets", 16)
    phi = 2.0 * np.pi * np.arange(n) / n
    return np.column_stack(
        (cx + obj["radius"] * np.cos(phi), cy + obj["radius"] * np.sin(phi))
    )


def _inside_footprints(xy, objects):
    """Mask of points inside any box or cylinder footprint."""
    mask = np.zeros(len(xy), dtype=bool)
    for obj in objects:
        if obj["type"] == "cylinder":
            mask |= np.hypot(*(xy - obj["center"]).T) < obj["radius"]
        elif obj["type"] == "box":
            a = math.radians(obj.get("angle", 0.0))
            d = xy - obj["center"]
            u = d @ (math.cos(a), math.sin(a))
            v = d @ (-math.sin(a), math.cos(a))
            hx, hy = 0.5 * np.asarray(obj["size"])
            mask |= (np.abs(u) < hx) & (np.abs(v) < hy)
    return mask


def _facets(objects):
    """Facet rows (p0x, p0y, p1x, p1y, z0, z1), object ids, mirror flags,
    materials and two-sidedness (walls)."""
    rows, ids, mirror, mats, two_sided = [], [], [], [], []
    for k, obj in enumerate(objects):
        kind = obj["type"]
        if kind not in ("wall", "box", "cylinder"):
            continue
        mat = material(obj["material"])
        if kind == "wall":
            verts = np.array([obj["p0"], obj["p1"]], dtype=np.float64)
            edges = [(0, 1)]
        else:
            verts = _polygon(obj)
            edges = [(i, (i + 1) % len(verts)) for i in range(len(verts))]
        for i, j in edges:
            rows.append([*verts[i], *verts[j], *obj["z"]])
            ids.append(k)
            mirror.append(mat["smooth"] and kind != "cylinder")
            mats.append(mat)
            two_sided.append(kind == "wall")
    rows = np.array(rows, dtype=np.float64).reshape(-1, 6)
    return rows, ids, mirror, mats, np.array(two_sided, dtype=bool)


def _block(pos, rcs, phase, kind, par=None, host=-1, normal=None):
    """Scatterer arrays for points sharing a kind."""
    pos = np.atleast_2d(np.asarray(pos, dtype=np.float64))
    n = len(pos)
    amp = np.sqrt(rcs) * np.exp(1j * np.asarray(phase))
    return {
        "pos": pos,
        "amp": np.broadcast_to(amp, (n,)).astype(np.complex128),
        "kind": np.full(n, kind, dtype=np.int64),
        "par": np.zeros((n, 3)) if par is None else np.atleast_2d(par).astype(float),
        "host": np.broadcast_to(np.asarray(host, dtype=np.int64), (n,)).copy(),
        "normal": np.broadcast_to(
            np.zeros(3) if normal is None else np.asarray(normal, dtype=float), (n, 3)
        ).copy(),
    }


def _diffuse(rng, rows, normals, sigma0, two_sided, density):
    """Lambertian points drawn uniformly on every face of every facet."""
    faces = np.concatenate([np.arange(len(rows)), np.flatnonzero(two_sided)])
    sign = np.where(np.arange(faces.size) < len(rows), 1.0, -1.0)
    length = np.hypot(rows[faces, 2] - rows[faces, 0], rows[faces, 3] - rows[faces, 1])
    area = length * (rows[faces, 5] - rows[faces, 4])
    count = rng.poisson(density * area)
    idx = np.repeat(np.arange(faces.size), count)
    f = faces[idx]
    uv = rng.random((idx.size, 2))
    pos = np.column_stack(
        (
            rows[f, 0] + uv[:, 0] * (rows[f, 2] - rows[f, 0]),
            rows[f, 1] + uv[:, 0] * (rows[f, 3] - rows[f, 1]),
            rows[f, 4] + uv[:, 1] * (rows[f, 5] - rows[f, 4]),
        )
    )
    rcs = (sigma0[faces] * area / np.maximum(count, 1))[idx]
    normal = np.column_stack((normals[f] * sign[idx, None], np.zeros(idx.size)))
    phase = rng.uniform(0.0, 2.0 * np.pi, idx.size)
    return _block(pos, rcs, phase, LAMBERTIAN, host=f, normal=normal)


def _clutter(rng, ground, objects):
    """Lambertian ground points on `ground.extent` outside object footprints."""
    x0, x1, y0, y1 = ground.get("extent", (0.0, 0.0, 0.0, 0.0))
    area = (x1 - x0) * (y1 - y0)
    count = rng.poisson(ground.get("clutter_density", 0.0) * area)
    xy = rng.uniform((x0, y0), (x1, y1), (count, 2))
    phase = rng.uniform(0.0, 2.0 * np.pi, count)
    keep = ~_inside_footprints(xy, objects)
    rcs = material(ground["material"])["sigma0"] * area / max(count, 1)
    pos = np.column_stack((xy[keep], np.zeros(keep.sum())))
    return _block(pos, rcs, phase[keep], LAMBERTIAN, normal=(0.0, 0.0, 1.0))


def _discrete(objects, facet_ids, lam):
    """Reflectors, points and cylinder glints."""
    blocks = []
    for k, obj in enumerate(objects):
        if obj["type"] == "reflector":
            b = math.radians(obj.get("boresight", -90.0))
            par = (math.cos(b), math.sin(b), pattern_exponent(obj.get("beamwidth", 40)))
            blocks.append(_block(obj["pos"], obj["rcs"], 0.0, TRIHEDRAL, par))
        elif obj["type"] == "point":
            blocks.append(_block(obj["pos"], obj["rcs"], 0.0, ISOTROPIC))
        elif obj["type"] == "cylinder":
            z0, z1 = obj["z"]
            rcs = 2.0 * np.pi * obj["radius"] * (z1 - z0) ** 2 / lam
            pos = (*obj["center"], 0.5 * (z0 + z1))
            par = (obj["radius"], 0.0, 0.0)
            blocks.append(_block(pos, rcs, 0.0, GLINT, par, facet_ids.index(k)))
        elif obj["type"] not in ("wall", "box"):
            raise ValueError(f"unknown object type {obj['type']!r}")
    return blocks


def compile_scene(scene, lam):
    """Compile a scene dict (or JSON path) to `Geometry` at wavelength `lam`."""
    scene = load_scene(scene)
    rng = np.random.default_rng(scene.get("seed", 0))
    objects = scene.get("objects", [])
    ground = scene.get("ground", {"material": "soil"})
    rows, ids, mirror, mats, two_sided = _facets(objects)
    d = rows[:, 2:4] - rows[:, 0:2]
    normals = np.column_stack((d[:, 1], -d[:, 0])) / np.hypot(*d.T)[:, None]
    sigma0 = np.array([m["sigma0"] for m in mats], dtype=np.float64)
    density = scene.get("diffuse_density", max(4.0 / lam**2 * 0.05, 4.0))
    blocks = [
        *_discrete(objects, ids, lam),
        _diffuse(rng, rows, normals, sigma0, two_sided, density),
        _clutter(rng, ground, objects),
    ]
    sc = {k: np.concatenate([b[k] for b in blocks]) for k in blocks[0]}
    gmat = material(ground["material"])
    return Geometry(
        scene=scene,
        lam=float(lam),
        pol=scene.get("polarization", "h"),
        ground_eps=gmat["eps_r"],
        ground_mirror=gmat["smooth"],
        facet_p0=np.ascontiguousarray(rows[:, 0:2]),
        facet_p1=np.ascontiguousarray(rows[:, 2:4]),
        facet_z=np.ascontiguousarray(rows[:, 4:6]),
        facet_normal=np.ascontiguousarray(normals),
        facet_eps=np.array([m["eps_r"] for m in mats], dtype=np.complex128),
        facet_sigma0=sigma0,
        facet_mirror=np.array(mirror, dtype=bool),
        facet_obj=np.array(ids, dtype=np.int64),
        scat_pos=np.ascontiguousarray(sc["pos"]),
        scat_amp=sc["amp"],
        scat_kind=sc["kind"],
        scat_par=np.ascontiguousarray(sc["par"]),
        scat_host=sc["host"],
        scat_normal=np.ascontiguousarray(sc["normal"]),
    )

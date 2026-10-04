"""Propagation paths against closed-form geometry and the radar equation."""

import dataclasses
import importlib.util
import math

import numba
import numpy as np
import pytest

from qmrdk.constants import C
from qmrdk.sim import propagation
from qmrdk.sim.scene import LAMBERTIAN, builtin_scene, compile_scene

LAM = 0.12
G0 = 10.0
Q = math.log(0.5) / (2 * math.log(math.cos(math.radians(30.0))))
K1 = LAM / (4 * np.pi) ** 1.5
K2 = LAM / (4 * np.pi)
TX = np.array([-0.06, 0.02, 1.0])
RX = np.array([0.06, 0.02, 1.1])
RXH = np.array([0.06, 0.02, 1.0])
WIDE = 170.0
QW = math.log(0.5) / (2 * math.log(math.cos(math.radians(WIDE / 2))))
SOIL, CONCRETE = 10.0 - 2.0j, 6.0 - 0.6j


@pytest.fixture(name="prop", scope="module", params=["jit", "python"])
def _prop(request):
    """The propagation module, compiled or as plain Python (same source)."""
    if request.param == "jit":
        return propagation
    numba.config.DISABLE_JIT = True
    try:
        spec = importlib.util.spec_from_file_location(
            "qmrdk.sim._propagation_python", propagation.__file__
        )
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        numba.config.DISABLE_JIT = False
    return mod


def field(a, p, bore=(0.0, 1.0, 0.0), q=Q):
    """Antenna (boresight `bore`, exponent q) field gain from a to p."""
    d = np.asarray(p, float) - a
    c = d @ bore / np.linalg.norm(d)
    return math.sqrt(G0) * max(c, 0.0) ** q


def hx(P, ta, ra):
    """h-pol induced-dipole factor: cosine between the horizontal directions
    from P to the (image) antennas."""
    a, b = (np.asarray(ta) - P)[:2], (np.asarray(ra) - P)[:2]
    return a @ b / (np.linalg.norm(a) * np.linalg.norm(b))


def te(eps, c):
    root = np.sqrt(eps - 1 + c * c)
    return (c - root) / (c + root)


def tm(eps, c):
    root = np.sqrt(eps - 1 + c * c)
    return (root - eps * c) / (root + eps * c)


def flip(p, axis, plane=0.0):
    """Mirror image of p in the plane coordinate[axis] = plane."""
    q = np.array(p, float)
    q[axis] = 2 * plane - q[axis]
    return q


def geometry(objects, ground="grass", **kw):
    scene = {"seed": 5, "ground": {"material": ground}, "objects": objects}
    return compile_scene({"diffuse_density": 0.0, **scene, **kw}, LAM)


def wall(x0, x1, y, z1=4.0, material="concrete"):
    return {
        "type": "wall",
        "p0": [x0, y],
        "p1": [x1, y],
        "z": [0, z1],
        "material": material,
    }


def point(p, rcs=0.5):
    return {"type": "point", "pos": list(p), "rcs": rcs}


def assert_paths(p, want):
    """Paths match the non-zero (path length, amplitude, kind) triples in any
    order."""
    want = sorted((w for w in want if w[1] != 0), key=lambda w: w[0])
    order = np.argsort(p.delay)
    assert len(p) == len(want)
    np.testing.assert_allclose(p.delay[order] * C, [w[0] for w in want], rtol=1e-12)
    np.testing.assert_allclose(
        p.amp[order], [w[1] for w in want], rtol=1e-9, atol=1e-18
    )
    np.testing.assert_array_equal(p.kind[order], [w[2] for w in want])
    np.testing.assert_array_equal(p.speed, 0.0)


def test_antenna_pattern():
    ant = propagation.Antenna()
    np.testing.assert_allclose(ant.boresight, (0, 1, 0), atol=1e-15)
    s30, c30 = math.sin(math.radians(30)), math.cos(math.radians(30))
    d = np.array([[0, 1, 0], [s30, c30, 0], [0, -1, 0], [0, 2, 2]])
    g = ant.field(d) ** 2 / G0
    np.testing.assert_allclose(g, [1, 0.5, 0, 0.5**Q], atol=1e-15)
    up = propagation.Antenna(yaw=0.0, elevation=90.0)
    np.testing.assert_allclose(up.boresight, (0, 0, 1), atol=1e-15)


def test_direct_isotropic(prop):
    P = np.array([1.0, 8.0, 1.3])
    g = geometry([point(P)])
    lt, lr = np.linalg.norm(P - TX), np.linalg.norm(P - RX)
    amp = K1 * math.sqrt(0.5) * field(TX, P) * field(RX, P) / (lt * lr)
    amp *= hx(P, TX, RX)
    assert_paths(prop.paths(g, TX, RX, LAM, prop.Antenna()), [(lt + lr, amp, 0)])


def test_ground_bounce(prop):
    P = np.array([0.5, 6.0, 0.7])
    g = geometry([point(P)], ground="soil")
    el = math.radians(-30.0)
    b = (0.0, math.cos(el), math.sin(el))
    legs = []
    for a in (TX, RX):
        L, Lg = np.linalg.norm(P - a), np.linalg.norm(P - flip(a, 2))
        gam = te(SOIL, (P[2] + a[2]) / Lg)
        legs.append([(L, field(a, P, b), 0), (Lg, gam * field(a, flip(P, 2), b), 1)])
    want = [
        (lt + lr, K1 * math.sqrt(0.5) * et * er * hx(P, TX, RX) / (lt * lr), kt | kr)
        for lt, et, kt in legs[0]
        for lr, er, kr in legs[1]
    ]
    rxi = flip(RX, 2)
    L = np.linalg.norm(TX - rxi)
    R = TX + (rxi - TX) * TX[2] / (TX[2] + RX[2])
    gam = te(SOIL, (TX[2] + RX[2]) / L)
    want.append((L, K2 * gam * field(TX, R, b) * field(RX, R, b) / L, 2))
    assert all(w[1] != 0 for w in want)
    p = prop.paths(g, TX, RX, LAM, prop.Antenna(elevation=-30.0))
    assert_paths(p, want)


def test_wall_ghost(prop):
    P = np.array([1.0, 8.0, 1.0])
    g = geometry([wall(-5, 5, 12.0), point(P)])
    legs = []
    for a in (TX, RXH):
        L, Lw = np.linalg.norm(P - a), np.linalg.norm(P - flip(a, 1, 12))
        gam = -tm(CONCRETE, (24 - a[1] - P[1]) / Lw)
        legs.append(
            [
                (L, field(a, P), 0, a),
                (Lw, gam * field(a, flip(P, 1, 12)), 1, flip(a, 1, 12)),
            ]
        )
    want = [
        (lt + lr, K1 * math.sqrt(0.5) * et * er * hx(P, ta, ra) / (lt * lr), kt | kr)
        for lt, et, kt, ta in legs[0]
        for lr, er, kr, ra in legs[1]
    ]
    L = np.linalg.norm(TX - flip(RXH, 1, 12))
    gam = -tm(CONCRETE, (24 - TX[1] - RXH[1]) / L)
    amp = K2 * gam * field(TX, flip(RXH, 1, 12)) * field(RXH, flip(TX, 1, 12)) / L
    want.append((L, amp, 2))
    assert_paths(prop.paths(g, TX, RXH, LAM), want)


@pytest.mark.parametrize(
    "objs,n",
    [
        ([wall(2, 6, 12.0), point((1, 8, 1))], 1),
        ([wall(-5, 5, 12.0, z1=0.5), point((0.5, 8, 2.5))], 1),
        ([wall(-5, 5, 12.0, z1=3.0), point((0.5, 8, 2.5))], 5),
    ],
)
def test_mirror_bounds(prop, objs, n):
    p = prop.paths(geometry(objs), TX, RX, LAM)
    assert len(p) == n
    P = np.array(objs[1]["pos"])
    direct = np.linalg.norm(P - TX) + np.linalg.norm(P - RX)
    assert np.sum(np.isclose(p.delay * C, direct, rtol=1e-12)) == 1


def test_box_shadow(prop):
    P = np.array([0.0, 9.0, 1.0])
    box = {
        "type": "box",
        "center": [0, 6],
        "size": [2, 0.5],
        "z": [0, 2],
        "material": "metal",
    }
    g = geometry([box, point(P, 1.0)])
    p = prop.paths(g, TX, RX, LAM)
    assert np.all(p.kind == 2) and len(p) == 1
    L = np.linalg.norm(TX - flip(RX, 1, 5.75))
    amp = K2 * field(TX, flip(RX, 1, 5.75)) * field(RX, flip(TX, 1, 5.75)) / L
    assert_paths(p, [(L, amp, 2)])
    tx, rx = TX + (4, 0, 0), RX + (4, 0, 0)
    p = prop.paths(g, tx, rx, LAM)
    lt, lr = np.linalg.norm(P - tx), np.linalg.norm(P - rx)
    direct = np.isclose(p.delay * C, lt + lr, rtol=1e-12)
    assert direct.sum() == 1 and p.kind[direct][0] == 0
    want = K1 * field(tx, P) * field(rx, P) * hx(P, tx, rx) / (lt * lr)
    assert p.amp[direct][0] == pytest.approx(want, rel=1e-9)
    np.testing.assert_array_equal(
        prop.line_of_sight(g, TX, [P, P + (8, 0, 0)]), [False, True]
    )


def test_lambertian_back_face(prop):
    rough = {"eps_r": [4.0, 0.0], "sigma0": 0.1, "smooth": False}
    g = geometry([wall(-3, 3, 10.0, material=rough)])
    P = np.array([0.5, 10.0, 1.2])
    for ny, n in ((-1.0, 1), (1.0, 0)):
        nrm = np.array([0.0, ny, 0.0])
        h = dataclasses.replace(
            g,
            scat_pos=P[None],
            scat_amp=np.ones(1, complex),
            scat_kind=np.array([LAMBERTIAN]),
            scat_par=np.zeros((1, 3)),
            scat_host=np.zeros(1, np.int64),
            scat_normal=nrm[None],
        )
        p = prop.paths(h, TX, RX, LAM)
        assert len(p) == n
    lt, lr = np.linalg.norm(P - TX), np.linalg.norm(P - RX)
    F = math.sqrt((P[1] - TX[1]) / lt * (P[1] - RX[1]) / lr)
    h = dataclasses.replace(h, scat_normal=-nrm[None])
    want = K1 * F * field(TX, P) * field(RX, P) * hx(P, TX, RX) / (lt * lr)
    assert_paths(prop.paths(h, TX, RX, LAM), [(lt + lr, want, 0)])


def test_convex_box_visibility(prop):
    box = {
        "type": "box",
        "center": [0.5, 8],
        "size": [3, 2],
        "angle": 30,
        "z": [0, 1.5],
        "material": "metal",
    }
    g = geometry([box], diffuse_density=30.0)
    p = prop.paths(g, TX, RX, LAM)
    nrm, pos = g.scat_normal, g.scat_pos
    seen = (np.sum(nrm * (TX - pos), 1) > 0) & (np.sum(nrm * (RX - pos), 1) > 0)
    assert 0 < seen.sum() < g.n_scatterers
    assert np.sum(p.kind == 0) == seen.sum()


def test_wall_glint(prop):
    a = np.array([0.0, 0.0, 1.0])
    p = prop.paths(geometry([wall(-3, 3, 10.0)]), a, a, LAM)
    gam = -(1 - np.sqrt(CONCRETE)) / (1 + np.sqrt(CONCRETE))
    assert_paths(p, [(20.0, K2 * gam * G0 / 20.0, 2)])
    assert len(prop.paths(geometry([wall(1, 5, 10.0)]), a, a, LAM)) == 0


def test_dihedral(prop):
    g = geometry([wall(-3, 3, 10.0, z1=3.0, material="metal")], ground="soil")
    a = np.array([0.0, 0.0, 1.0])
    L = 2 * math.sqrt(101.0)
    corner = np.array([0.0, 10.0, 0.0])
    dihedral = K2 * te(SOIL, 2 / L) * field(a, corner) ** 2 / L
    want = [(20.0, K2 * G0 / 20.0, 2), (L, dihedral, 2)]
    assert_paths(prop.paths(g, a, a, LAM), want)
    rx = a + (0, 0, 0.2)
    r2 = np.array([0.0, 20.0, -1.2])
    L = np.linalg.norm(r2 - a)
    p1 = a + (r2 - a) / 2.2
    p2 = p1 + (np.array([0, 20, 1.2]) - p1) * (10 - p1[1]) / (20 - p1[1])
    p = prop.paths(g, a, rx, LAM)
    hit = np.isclose(p.delay * C, L, rtol=1e-12)
    assert hit.sum() == 1 and len(p) == 2
    want = K2 * te(SOIL, 2.2 / L) * field(a, p1) * field(rx, p2) / L
    assert p.amp[hit][0] == pytest.approx(want, rel=1e-9)


def effective(p, tx, rx, spec, ant):
    """Effective scalar coefficient of the specular path via point `spec`
    (0 if the path is absent, as exactly zero paths are dropped)."""
    L = np.linalg.norm(spec - tx) + np.linalg.norm(rx - spec)
    hit = np.isclose(p.delay * C, L, rtol=1e-12) & (p.kind == 2)
    assert hit.sum() <= 1
    if not hit.any():
        return 0.0
    et = field(tx, spec, ant.boresight, ant.q)
    er = field(rx, spec, ant.boresight, ant.q)
    return p.amp[hit][0] * L / (K2 * et * er)


@pytest.mark.parametrize("d,x", [(5.0, 10.0), (0.02, 10.0), (5.0, 3.0), (5.0, 0.0)])
def test_wall_reflection_hpol(prop, d, x):
    """Horizontal plane of incidence on a wall with h-pol: TM, -Gamma_p in the
    scalar convention (Brewster zero at tan = sqrt(eps), -1 at grazing)."""
    eps = 4.0
    mat = {"eps_r": [eps, 0.0], "sigma0": 0.01, "smooth": True}
    g = geometry([wall(-30, 30, d, z1=3.0, material=mat)])
    tx, rx = np.array([-x, 0.0, 1.0]), np.array([x, 0.0, 1.0])
    ant = prop.Antenna(beamwidth=WIDE)
    gam = effective(prop.paths(g, tx, rx, LAM, ant), tx, rx, np.array([0, d, 1.0]), ant)
    c = d / math.hypot(x, d)
    assert gam == pytest.approx(-tm(eps, c), abs=1e-12)
    if x == 2 * d:
        assert abs(gam) < 1e-12
    if d < 0.1:
        assert abs(gam + 1) < 0.02
    if x == 0:
        assert gam == pytest.approx(-(1 - math.sqrt(eps)) / (1 + math.sqrt(eps)))


@pytest.mark.parametrize("pol", ["h", "v"])
@pytest.mark.parametrize("h,x", [(1.0, 2.0), (0.02, 20.0), (1.0, 0.5)])
def test_ground_two_ray(prop, pol, h, x):
    """Ground bounce: h-pol is TE (Gamma_s), v-pol TM (-Gamma_p, zero at the
    Brewster grazing angle atan(1 / sqrt(eps))); both -1 at grazing."""
    eps = 4.0
    ground = {"eps_r": [eps, 0.0], "sigma0": 0.01, "smooth": True}
    g = geometry([], ground=ground, polarization=pol)
    tx, rx = np.array([-x, 0.0, h]), np.array([x, 0.0, h])
    ant = prop.Antenna(beamwidth=WIDE, elevation=-90.0)
    gam = effective(prop.paths(g, tx, rx, LAM, ant), tx, rx, np.zeros(3), ant)
    c = h / math.hypot(x, h)
    assert gam == pytest.approx(te(eps, c) if pol == "h" else -tm(eps, c), abs=1e-12)
    if pol == "v" and x == 2 * h:
        assert abs(gam) < 1e-12
    if h < 0.1:
        assert abs(gam + 1) < 0.02


@pytest.mark.parametrize("pol,sign", [("h", 1.0), ("v", -1.0)])
def test_plate_point_dihedral_signs(prop, pol, sign):
    """A PEC plate at normal incidence returns with the sign of a point
    scatterer; a PEC dihedral with its seam along the h field the opposite."""
    a = np.array([0.0, 0.0, 1.0])
    pec = {"eps_r": None, "sigma0": 0.01, "smooth": True}
    plate = geometry([wall(-3, 3, 10.0, material="metal")], polarization=pol)
    assert_paths(prop.paths(plate, a, a, LAM), [(20.0, sign * K2 * G0 / 20.0, 2)])
    pt = geometry([point((0, 10, 1), 1.0)], polarization=pol)
    assert_paths(prop.paths(pt, a, a, LAM), [(20.0, sign * K1 * G0 / 100.0, 0)])
    if pol == "h":
        both = geometry([wall(-3, 3, 10.0, material="metal")], ground=pec)
        L = 2 * math.sqrt(101.0)
        e2 = field(a, (0, 10, 0)) ** 2
        assert_paths(
            prop.paths(both, a, a, LAM),
            [(20.0, K2 * G0 / 20.0, 2), (L, -K2 * e2 / L, 2)],
        )


def test_trihedral_pattern(prop):
    a = np.array([0.0, 0.0, 1.0])
    q = math.log(0.5) / (2 * math.log(math.cos(math.radians(20))))
    for alpha in (0.0, 15.0, 100.0):
        refl = {
            "type": "reflector",
            "pos": [0, 8, 1],
            "rcs": 10.0,
            "boresight": -90 + alpha,
            "beamwidth": 40,
        }
        p = prop.paths(geometry([refl]), a, a, LAM)
        if alpha > 90:
            assert len(p) == 0
            continue
        want = K1 * math.sqrt(10) * math.cos(math.radians(alpha)) ** q * G0 / 64
        assert_paths(p, [(16.0, want, 0)])


def test_cylinder_glint(prop):
    a = np.array([0.0, 0.0, 1.0])
    c, r = np.array([1.0, 8.0, 1.0]), 0.3
    cyl = {
        "type": "cylinder",
        "center": [1, 8],
        "radius": r,
        "z": [0, 2],
        "material": "metal",
    }
    P = c + r * (a - c) / np.linalg.norm(a - c)
    L = np.linalg.norm(a - P)
    amp = K1 * math.sqrt(2 * np.pi * r * 4 / LAM) * field(a, P) ** 2 / L**2
    assert_paths(prop.paths(geometry([cyl]), a, a, LAM), [(2 * L, amp, 0)])
    p = prop.paths(geometry([cyl], ground="soil"), a, a, LAM)
    Lg = np.linalg.norm(P - flip(a, 2))
    np.testing.assert_allclose(
        np.sort(p.delay[p.kind < 2]) * C, [2 * L, L + Lg, L + Lg, 2 * Lg]
    )
    blocked = geometry([cyl, wall(-5, 5, 5.0)])
    assert np.all(prop.paths(blocked, a, a, LAM).kind == 2)


def test_line_of_sight(prop):
    g = geometry([wall(-5, 5, 10.0, z1=2.0)])
    pts = [(0, 15, 1), (0, 15, 5), (8, 15, 1), (0, 5, 1), (0, 10, 1)]
    np.testing.assert_array_equal(
        prop.line_of_sight(g, (0, 0, 1), pts), [False, True, True, True, True]
    )


def test_yard():
    g = compile_scene(builtin_scene("yard"), LAM)
    tx, rx = np.array([-0.06, 0.02, 1.0]), np.array([0.06, 0.02, 1.0])
    p = propagation.paths(g, tx, rx, LAM)
    assert set(np.unique(p.kind)) == {0, 1, 2}
    refl = np.array([o["pos"] for o in g.scene["objects"] if o["type"] == "reflector"])
    hidden = np.array([6.5, 16.0, 0.8])
    for x in np.linspace(-0.75, 0.75, 3):
        los = propagation.line_of_sight(g, (x, 0.02, 1.0), refl)
        assert np.sum(~los) == 1 and np.all(refl[~los] == hidden)
    for r in refl:
        direct = np.linalg.norm(r - tx) + np.linalg.norm(r - rx)
        found = np.any(np.isclose(p.delay * C, direct, rtol=1e-13) & (p.kind == 0))
        assert found != np.all(r == hidden)

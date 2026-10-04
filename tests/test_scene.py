"""Scene loading, materials, Fresnel coefficients and compilation."""

import json
import math

import numpy as np
import pytest

from qmrdk.sim import scene as sc

LAM = 0.12


def test_fresnel_normal_incidence():
    eps = np.array([6.0 - 0.6j, 10.0 - 2.0j, 2.0 + 0.0j])
    want = (1 - np.sqrt(eps)) / (1 + np.sqrt(eps))
    np.testing.assert_allclose(sc.fresnel(eps, 1.0, "h"), want, rtol=1e-12)
    np.testing.assert_allclose(sc.fresnel(eps, 1.0, "v"), want, rtol=1e-12)


def test_fresnel_grazing_and_brewster():
    eps = np.array([6.0 - 0.6j, 4.5 - 0.4j])
    np.testing.assert_allclose(sc.fresnel(eps, 0.0, "h"), -1.0, atol=1e-12)
    np.testing.assert_allclose(sc.fresnel(eps, 0.0, "v"), 1.0, atol=1e-12)
    np.testing.assert_allclose(sc.fresnel(eps, 1e-6, "h"), -1.0, atol=1e-5)
    er = 4.0
    assert abs(sc.fresnel(er, 1.0 / math.sqrt(1.0 + er), "v")) < 1e-12


def test_fresnel_closed_form_and_pec():
    c = np.linspace(0.05, 1.0, 9)
    eps = 6.0 - 0.6j
    root = np.sqrt(eps - np.sin(np.arccos(c)) ** 2)
    np.testing.assert_allclose(sc.fresnel(eps, c, "h"), (c - root) / (c + root))
    np.testing.assert_allclose(
        sc.fresnel(eps, c, "v"), (root - eps * c) / (root + eps * c)
    )
    for pol in "hv":
        np.testing.assert_array_equal(sc.fresnel(sc.PEC, c, pol), -1.0)
        assert sc.gamma(sc.PEC, 0.3, pol == "v") == -1.0
        assert (
            abs(sc.gamma(eps, 1.0, pol == "v") - (1 - eps**0.5) / (1 + eps**0.5))
            < 1e-12
        )
    assert np.all(np.abs(sc.fresnel(eps, c, "v")) <= 1.0)
    with pytest.raises(ValueError):
        sc.fresnel(eps, c, "x")


def test_scalar_gamma_python():
    for tm in (False, True):
        assert sc.gamma.py_func(sc.PEC, 0.5, tm) == -1.0
        assert sc.gamma.py_func(9.0 + 0j, 1.0, tm) == pytest.approx(-0.5)


def test_pattern_exponent():
    for bw in (20.0, 40.0, 60.0, 90.0):
        q = sc.pattern_exponent(bw)
        assert math.cos(math.radians(bw / 2)) ** (2 * q) == pytest.approx(0.5)


def test_materials_and_inline():
    assert sc.material("metal")["eps_r"] == sc.PEC
    m = sc.material({"eps_r": [3.0, -0.1], "sigma0": 0.2, "smooth": False})
    assert m == {"eps_r": 3.0 - 0.1j, "sigma0": 0.2, "smooth": False}
    assert sc.material({"eps_r": None, "sigma0": 0.1})["eps_r"] == sc.PEC
    with pytest.raises(KeyError):
        sc.material("unobtainium")


def test_load_scene_path_and_builtins(tmp_path):
    d = {"name": "t", "objects": []}
    path = tmp_path / "s.json"
    path.write_text(json.dumps(d))
    assert sc.load_scene(path) == d
    assert sc.load_scene(d) is d
    yard = sc.builtin_scene("yard")
    kinds = [o["type"] for o in yard["objects"]]
    assert kinds.count("reflector") >= 4 and "wall" in kinds and "box" in kinds
    assert kinds.count("cylinder") >= 2
    for o in yard["objects"]:
        xy = o.get("pos", o.get("center", o.get("p0")))
        assert -12 <= xy[0] <= 12 and 3 <= xy[1] <= 28


def test_single_scene():
    g = sc.compile_scene(sc.builtin_scene("single"), LAM)
    assert g.n_facets == 0 and g.n_scatterers == 1
    assert g.scat_kind[0] == sc.TRIHEDRAL
    np.testing.assert_allclose(g.scat_pos[0], (0.3, 8.0, 1.0))
    assert g.scat_amp[0] == pytest.approx(math.sqrt(10.0))
    np.testing.assert_allclose(g.scat_par[0, :2], (0.0, -1.0), atol=1e-15)
    assert g.ground_mirror


def _scene(objects, **kw):
    return {"seed": 3, "ground": {"material": "grass"}, "objects": objects, **kw}


def test_facets_geometry():
    objs = [
        {"type": "wall", "p0": [-2, 5], "p1": [2, 5], "z": [0, 3], "material": "brick"},
        {
            "type": "box",
            "center": [3, 9],
            "size": [2, 1],
            "angle": 30,
            "z": [0, 1.2],
            "material": "metal",
        },
        {
            "type": "cylinder",
            "center": [-4, 12],
            "radius": 0.5,
            "z": [0, 3],
            "material": "metal",
            "n_facets": 8,
        },
    ]
    g = sc.compile_scene(_scene(objs, diffuse_density=0.0), LAM)
    assert g.n_facets == 1 + 4 + 8
    np.testing.assert_array_equal(g.facet_obj, [0] + [1] * 4 + [2] * 8)
    np.testing.assert_allclose(g.facet_normal[0], (0, -1))
    np.testing.assert_array_equal(g.facet_mirror, [True] * 5 + [False] * 8)
    assert np.isinf(g.facet_eps[1].real) and g.facet_eps[0] == 4.5 - 0.4j
    mid = 0.5 * (g.facet_p0 + g.facet_p1)
    for k, centre in ((1, (3, 9)), (2, (-4, 12))):
        sel = g.facet_obj == k
        out = np.sum((mid[sel] - centre) * g.facet_normal[sel], axis=1)
        assert np.all(out > 0)
    length = np.hypot(*(g.facet_p1 - g.facet_p0)[1:5].T)
    np.testing.assert_allclose(np.sort(length), [1, 1, 2, 2])
    np.testing.assert_allclose(np.hypot(*(g.facet_p0[5:] - (-4, 12)).T), 0.5)
    glint = np.flatnonzero(g.scat_kind == sc.GLINT)
    assert glint.size == 1 and g.scat_host[glint[0]] == 5
    assert abs(g.scat_amp[glint[0]]) ** 2 == pytest.approx(2 * np.pi * 0.5 * 9 / LAM)
    np.testing.assert_allclose(g.scat_pos[glint[0]], (-4, 12, 1.5))


def test_diffuse_density_and_rcs():
    density = 40.0
    objs = [
        {"type": "wall", "p0": [-2, 5], "p1": [2, 5], "z": [0, 3], "material": "brick"},
        {
            "type": "box",
            "center": [3, 9],
            "size": [2, 1],
            "z": [0, 2],
            "material": "wood",
        },
    ]
    g = sc.compile_scene(_scene(objs, diffuse_density=density), LAM)
    diffuse = g.scat_kind == sc.LAMBERTIAN
    host = g.scat_host[diffuse]
    pos, nrm = g.scat_pos[diffuse], g.scat_normal[diffuse]
    wall_area, box_area = 2 * 12.0, 6.0 * 2.0
    for sel, area, s0 in ((host == 0, wall_area, 0.08), (host > 0, box_area, 0.03)):
        mean = density * area
        assert abs(sel.sum() - mean) < 5 * math.sqrt(mean)
        assert np.sum(np.abs(g.scat_amp[diffuse][sel]) ** 2) == pytest.approx(s0 * area)
    wall = host == 0
    assert np.all(pos[wall, 1] == 5) and np.all(np.abs(pos[wall, 0]) <= 2)
    assert np.all((pos[:, 2] >= 0) & (pos[:, 2] <= 3))
    assert abs(np.sum(nrm[wall, 1] < 0) - np.sum(nrm[wall, 1] > 0)) < 5 * math.sqrt(
        density * wall_area
    )
    np.testing.assert_allclose(nrm[~wall, :2], g.facet_normal[host[~wall]], atol=1e-15)


def test_default_diffuse_density():
    objs = [
        {"type": "wall", "p0": [0, 5], "p1": [10, 5], "z": [0, 10], "material": "brick"}
    ]
    for lam, density in ((0.12, 4 / 0.12**2 * 0.05), (0.5, 4.0)):
        g = sc.compile_scene(_scene(objs), lam)
        mean = density * 200.0
        assert abs(g.n_scatterers - mean) < 5 * math.sqrt(mean)


def test_clutter_excludes_footprints():
    objs = [
        {
            "type": "box",
            "center": [0, 5],
            "size": [4, 2],
            "angle": 45,
            "z": [0, 1],
            "material": "metal",
        },
        {
            "type": "cylinder",
            "center": [5, 5],
            "radius": 1.5,
            "z": [0, 1],
            "material": "wood",
        },
    ]
    ground = {"material": "soil", "clutter_density": 30.0, "extent": [-5, 10, 0, 10]}
    g = sc.compile_scene(
        {"seed": 2, "ground": ground, "objects": objs, "diffuse_density": 0.0}, LAM
    )
    cl = g.scat_host == -1
    cl &= g.scat_kind == sc.LAMBERTIAN
    pos = g.scat_pos[cl]
    assert np.all(pos[:, 2] == 0) and np.all(g.scat_normal[cl] == (0, 0, 1))
    mean = 30.0 * (150.0 - 8.0 - np.pi * 1.5**2)
    assert abs(cl.sum() - mean) < 5 * math.sqrt(mean)
    assert np.all(np.hypot(pos[:, 0] - 5, pos[:, 1] - 5) >= 1.5)
    d = pos[:, :2] - (0, 5)
    u, v = d @ (np.sqrt(0.5), np.sqrt(0.5)), d @ (-np.sqrt(0.5), np.sqrt(0.5))
    assert not np.any((np.abs(u) < 2) & (np.abs(v) < 1))
    rcs = np.abs(g.scat_amp[cl]) ** 2
    np.testing.assert_allclose(rcs, rcs[0])
    assert rcs[0] == pytest.approx(0.002 / 30.0, rel=5 / math.sqrt(30 * 150))


def test_compile_deterministic():
    s = sc.builtin_scene("yard")
    a, b = sc.compile_scene(s, LAM), sc.compile_scene(s, LAM)
    for f in ("scat_pos", "scat_amp", "scat_normal", "facet_p0", "scat_host"):
        np.testing.assert_array_equal(getattr(a, f), getattr(b, f))
    c = sc.compile_scene({**s, "seed": 2}, LAM)
    assert a.n_facets == c.n_facets
    assert a.n_scatterers != c.n_scatterers or not np.array_equal(
        a.scat_pos, c.scat_pos
    )


def test_compile_from_path_and_reflectors(tmp_path):
    objs = [
        {
            "type": "reflector",
            "pos": [1, 2, 3],
            "rcs": 4.0,
            "boresight": 0,
            "beamwidth": 30,
        },
        {"type": "point", "pos": [0, 9, 1], "rcs": 0.25},
    ]
    path = tmp_path / "r.json"
    path.write_text(json.dumps(_scene(objs)))
    g = sc.compile_scene(path, LAM)
    np.testing.assert_allclose(g.scat_par[0], (1, 0, sc.pattern_exponent(30)))
    np.testing.assert_allclose(g.scat_amp, (2.0, 0.5))
    np.testing.assert_array_equal(g.scat_kind, (sc.TRIHEDRAL, sc.ISOTROPIC))
    np.testing.assert_array_equal(g.scat_host, (-1, -1))
    assert g.pol == "h" and not g.ground_mirror


def test_unknown_object():
    with pytest.raises(ValueError):
        sc.compile_scene(_scene([{"type": "tree"}]), LAM)

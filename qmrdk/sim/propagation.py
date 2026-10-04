"""Propagation paths between antennas and a compiled scene (docs/simulation.md §3)."""

import dataclasses
import math

import numba
import numpy as np

from qmrdk.constants import C
from qmrdk.sim.scene import GLINT, LAMBERTIAN, TRIHEDRAL, gamma, pattern_exponent

_jit = numba.njit(cache=True, error_model="numpy")
_EPS = 1e-9


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

    @property
    def boresight(self):
        """Unit boresight vector."""
        yaw, el = math.radians(self.yaw), math.radians(self.elevation)
        return np.array(
            [math.cos(el) * math.cos(yaw), math.cos(el) * math.sin(yaw), math.sin(el)]
        )

    @property
    def q(self):
        return pattern_exponent(self.beamwidth)

    def field(self, directions):
        """Field gain sqrt(g0) * pattern along directions [..., 3] (any length)."""
        d = np.asarray(directions, dtype=np.float64)
        c = d @ self.boresight / np.linalg.norm(d, axis=-1)
        return math.sqrt(self.g0) * np.maximum(c, 0.0) ** self.q


@dataclasses.dataclass(frozen=True)
class Paths:
    """Propagation paths for one tx/rx placement.

    Attributes:
        delay: total path length / c, s.
        amp: complex field amplitude for 1 W transmitted, sqrt(W).
        speed: rate of change of total path length / 2 (negative approaching), m/s.
        kind: 0 scatterer without bounce, 1 scatterer with a bounce, 2 specular.
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


def facet_table(geom):
    """Facet rows (p0x, p0y, ex, ey, length, nx, ny, z0, z1, n.p0), e the unit
    tangent."""
    d = geom.facet_p1 - geom.facet_p0
    length = np.hypot(d[:, 0], d[:, 1])
    off = np.sum(geom.facet_normal * geom.facet_p0, axis=1)
    cols = (geom.facet_p0, d / length[:, None], length, geom.facet_normal)
    return np.ascontiguousarray(
        np.column_stack(cols + (geom.facet_z, off)).reshape(-1, 10)
    )


def mirror_table(geom):
    """Mirror planes: the ground first if smooth, then the mirror facets.

    Returns:
        plane (M, 4) normal and offset, facet index (-1 ground), permittivity,
        True for v (else h) antenna polarisation.
    """
    idx = np.flatnonzero(geom.facet_mirror)
    n = geom.facet_normal[idx]
    off = np.sum(n * geom.facet_p0[idx], axis=1)
    plane = np.column_stack((n, np.zeros(idx.size), off))
    eps = geom.facet_eps[idx]
    if geom.ground_mirror:
        plane = np.vstack(([0.0, 0.0, 1.0, 0.0], plane))
        idx = np.concatenate(([-1], idx))
        eps = np.concatenate(([geom.ground_eps], eps))
    return (
        np.ascontiguousarray(plane.reshape(-1, 4)),
        idx.astype(np.int64),
        eps.astype(np.complex128),
        geom.pol == "v",
    )


@_jit
def _blocked(a, b, ft, fobj, xf1, xf2, xo):
    """Segment a -> b crosses a facet strictly between its endpoints, ignoring
    facets xf1, xf2 and the facets of object xo."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    for f in range(ft.shape[0]):
        ha = ft[f, 5] * a[0] + ft[f, 6] * a[1] - ft[f, 9]
        hb = ft[f, 5] * b[0] + ft[f, 6] * b[1] - ft[f, 9]
        if ha * hb >= 0.0:
            continue
        t = ha / (ha - hb)
        s = (a[0] + t * dx - ft[f, 0]) * ft[f, 2] + (a[1] + t * dy - ft[f, 1]) * ft[
            f, 3
        ]
        z = a[2] + t * dz
        if (
            (s >= 0.0)
            & (s <= ft[f, 4])
            & (z >= ft[f, 7])
            & (z <= ft[f, 8])
            & (f != xf1)
            & (f != xf2)
            & (fobj[f] != xo)
        ):
            return True
    return False


@_jit
def _inside(x, y, z, f, ft):
    """A point on a mirror plane lies within facet f (always for the ground),
    edges included to rounding tolerance."""
    if f < 0:
        return True
    s = (x - ft[f, 0]) * ft[f, 2] + (y - ft[f, 1]) * ft[f, 3]
    return (
        (s >= -_EPS)
        & (s <= ft[f, 4] + _EPS)
        & (z >= ft[f, 7] - _EPS)
        & (z <= ft[f, 8] + _EPS)
    )


@_jit
def _field(dx, dy, dz, ant):
    """Antenna field gain along (dx, dy, dz) from the antenna."""
    bore, q, sg = ant
    c = (dx * bore[0] + dy * bore[1] + dz * bore[2]) / math.sqrt(
        dx * dx + dy * dy + dz * dz
    )
    return sg * c**q if c > 0.0 else 0.0


@_jit
def _height(m, p, plane):
    return plane[m, 0] * p[0] + plane[m, 1] * p[1] + plane[m, 2] * p[2] - plane[m, 3]


@_jit
def _image(a, o, plane):
    """Position a as seen via option o (0 direct, m + 1 mirror m)."""
    if o == 0:
        return a[0], a[1], a[2]
    h = 2.0 * _height(o - 1, a, plane)
    n = plane[o - 1]
    return a[0] - h * n[0], a[1] - h * n[1], a[2] - h * n[2]


@_jit
def _dist(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


@_jit
def _towards(a, b, t):
    return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])


@_jit
def _basis(d, vpol):
    """Unit polarisation vector of a ray along unit d: h = z x d normalised,
    v = component of z perpendicular to d (x for a vertical ray)."""
    if vpol:
        x, y, z = -d[2] * d[0], -d[2] * d[1], 1.0 - d[2] * d[2]
    else:
        x, y, z = -d[1], d[0], 0.0
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return 1.0, 0.0, 0.0
    return x / n, y / n, z / n


@_jit
def _bounce(e, d, m, mir):
    """Reflect field vector e (complex) of a ray along unit d in mirror m.

    Returns:
        reflected field vector and direction.
    """
    plane, _, meps, _ = mir
    n = plane[m]
    c = d[0] * n[0] + d[1] * n[1] + d[2] * n[2]
    s = (
        d[1] * n[2] - d[2] * n[1],
        d[2] * n[0] - d[0] * n[2],
        d[0] * n[1] - d[1] * n[0],
    )
    ns = math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2)
    s = _basis(d, False) if ns < 1e-12 else (s[0] / ns, s[1] / ns, s[2] / ns)
    p = (
        s[1] * d[2] - s[2] * d[1],
        s[2] * d[0] - s[0] * d[2],
        s[0] * d[1] - s[1] * d[0],
    )
    es = gamma(meps[m], abs(c), False) * (e[0] * s[0] + e[1] * s[1] + e[2] * s[2])
    ep = gamma(meps[m], abs(c), True) * (e[0] * p[0] + e[1] * p[1] + e[2] * p[2])
    v = (es * s[0] + ep * p[0], es * s[1] + ep * p[1], es * s[2] + ep * p[2])
    vn = 2.0 * (v[0] * n[0] + v[1] * n[1] + v[2] * n[2])
    out = (v[0] - vn * n[0], v[1] - vn * n[1], v[2] - vn * n[2])
    return out, (d[0] - 2.0 * c * n[0], d[1] - 2.0 * c * n[1], d[2] - 2.0 * c * n[2])


@_jit
def _reflection(a, b, ms, mir):
    """Scalar coefficient of the ray a -> b (first segment) through mirrors
    ms[0], ms[1] (-1 none): the reflected field projected on the basis."""
    dist = _dist(a, b)
    d = ((b[0] - a[0]) / dist, (b[1] - a[1]) / dist, (b[2] - a[2]) / dist)
    e0 = _basis(d, mir[3])
    e = (e0[0] + 0j, e0[1] + 0j, e0[2] + 0j)
    for m in ms:
        if m >= 0:
            e, d = _bounce(e, d, m, mir)
    b = _basis(d, mir[3])
    return e[0] * b[0] + e[1] * b[1] + e[2] * b[2]


@_jit
def _dipole(ut, ur, vpol):
    """Polarisation factor of a scatterer re-radiating as an induced dipole:
    -(arriving basis vector) . (leaving basis vector), ut and ur unit
    directions from the scatterer towards where the rays come from and go."""
    a = _basis((-ut[0], -ut[1], -ut[2]), vpol)
    b = _basis((ur[0], ur[1], ur[2]), vpol)
    return -(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


@_jit
def _leg(p, nrm, hf, ho, o, a, ant, mir, ft, fobj, occl, inbound):
    """Leg between scatterer point p (normal nrm, on host facet hf or object
    ho) and antenna a via option o, travelling a -> p if `inbound`, with
    occlusion tests if `occl`.

    Returns:
        valid (and non-zero), unfolded length, Gamma E / length, unit
        direction (3) from p.
    """
    plane, mf = mir[0], mir[1]
    img = _image(a, o, plane)
    L = _dist(p, img)
    u = ((img[0] - p[0]) / L, (img[1] - p[1]) / L, (img[2] - p[2]) / L)
    cn = nrm[0] * u[0] + nrm[1] * u[1] + nrm[2] * u[2]
    ok = (L > 0.0) & ((cn > 0.0) | (nrm[0] ** 2 + nrm[1] ** 2 + nrm[2] ** 2 == 0.0))
    r, e, f, g = (a[0], a[1], a[2]), (p[0], p[1], p[2]), -1, 1.0 + 0.0j
    if o > 0:
        hs, ha = _height(o - 1, p, plane), _height(o - 1, a, plane)
        ok = ok and hs * ha > 0.0
        if ok:
            r = _towards(p, img, hs / (hs + ha))
            e, f = r, mf[o - 1]
            ok = _inside(r[0], r[1], r[2], f, ft)
            g = _reflection((a[0], a[1], a[2]) if inbound else p, r, (o - 1, -1), mir)
    coef = g * _field(e[0] - a[0], e[1] - a[1], e[2] - a[2], ant) / L if ok else 0j
    ok = ok and coef != 0.0
    if ok and occl:
        ok = not (
            _blocked(p, r, ft, fobj, hf, f, ho)
            or (o > 0 and _blocked(r, a, ft, fobj, f, -1, -1))
        )
    return ok, L, coef, u


@_jit
def _pattern(kind, par, nrm, ut, ur):
    """Scatterer field pattern F for unit directions ut (to tx), ur (to rx)."""
    if kind == LAMBERTIAN:
        ct = nrm[0] * ut[0] + nrm[1] * ut[1] + nrm[2] * ut[2]
        cr = nrm[0] * ur[0] + nrm[1] * ur[1] + nrm[2] * ur[2]
        return math.sqrt(max(ct, 0.0) * max(cr, 0.0))
    if kind == TRIHEDRAL:
        bx, by = ut[0] + ur[0], ut[1] + ur[1]
        nb = math.hypot(bx, by)
        c = (bx * par[0] + by * par[1]) / nb if nb > 0.0 else 0.0
        return c ** par[2] if c > 0.0 else 0.0
    return 1.0


@_jit
def _glint(s, sc, ho, tx, rx, ant, mir, ft, fobj, fill, k0, out):
    """Paths of a cylinder glint, whose point follows the bisector of the
    directions from the axis to the (image) antennas of each leg pair."""
    pos, scamp, par = sc[0], sc[1], sc[3]
    c = (pos[s, 0], pos[s, 1], pos[s, 2])
    no = mir[0].shape[0] + 1
    zero = np.zeros(3)
    n = 0
    for i in range(no):
        ti = _image(tx, i, mir[0])
        nt = _dist(c, ti)
        for j in range(no):
            rj = _image(rx, j, mir[0])
            nr = _dist(c, rj)
            bx = (ti[0] - c[0]) / nt + (rj[0] - c[0]) / nr
            by = (ti[1] - c[1]) / nt + (rj[1] - c[1]) / nr
            nb = math.hypot(bx, by)
            if nb == 0.0:
                continue
            p = (c[0] + par[s, 0] * bx / nb, c[1] + par[s, 0] * by / nb, c[2])
            okt, lt, ct, ut = _leg(
                p, zero, -1, ho, i, tx, ant, mir, ft, fobj, True, True
            )
            if not okt:
                continue
            okr, lr, cr, ur = _leg(
                p, zero, -1, ho, j, rx, ant, mir, ft, fobj, True, False
            )
            if not okr:
                continue
            if fill:
                out[0][k0 + n] = lt + lr
                out[1][k0 + n] = scamp[s] * ct * cr * _dipole(ut, ur, mir[3])
                out[2][k0 + n] = 0 if i + j == 0 else 1
            n += 1
    return n


@_jit
def _scatterer(s, sc, tx, rx, ant, mir, ft, fobj, vt, vr, fill, k0, out):
    """Paths of scatterer s: the count pass tests legs with occlusion and
    records their validity in vt, vr; the fill pass writes from index k0."""
    pos, scamp, kind, par, host, nrm = sc
    if kind[s] == GLINT:
        return _glint(s, sc, fobj[host[s]], tx, rx, ant, mir, ft, fobj, fill, k0, out)
    no = mir[0].shape[0] + 1
    ll = np.empty((2, no))
    cc = np.empty((2, no), dtype=np.complex128)
    uu = np.empty((2, no, 3))
    p = (pos[s, 0], pos[s, 1], pos[s, 2])
    for side in range(2):
        a, v = (tx, vt) if side == 0 else (rx, vr)
        for o in range(no):
            if fill and not v[s, o]:
                continue
            ok, ll[side, o], cc[side, o], u = _leg(
                p, nrm[s], host[s], -1, o, a, ant, mir, ft, fobj, not fill, side == 0
            )
            uu[side, o, 0], uu[side, o, 1], uu[side, o, 2] = u
            v[s, o] = ok
    n = 0
    for i in range(no):
        for j in range(no):
            if not (vt[s, i] and vr[s, j]):
                continue
            F = _pattern(kind[s], par[s], nrm[s], uu[0, i], uu[1, j])
            F *= _dipole(uu[0, i], uu[1, j], mir[3])
            if F == 0.0:
                continue
            if fill:
                out[0][k0 + n] = ll[0, i] + ll[1, j]
                out[1][k0 + n] = scamp[s] * F * cc[0, i] * cc[1, j]
                out[2][k0 + n] = 0 if i + j == 0 else 1
            n += 1
    return n


@numba.njit(parallel=True, cache=True, error_model="numpy")
def _scatter_kernel(sc, tx, rx, ant, mir, ft, fobj, vt, vr, start, fill, out):
    """Scatterer paths, parallel over scatterers: the count pass (fill False)
    returns the number of paths per scatterer, the fill pass writes them."""
    count = np.zeros(sc[0].shape[0], dtype=np.int64)
    for s in numba.prange(sc[0].shape[0]):  # pylint: disable=not-an-iterable
        count[s] = _scatterer(
            s, sc, tx, rx, ant, mir, ft, fobj, vt, vr, fill, start[s], out
        )
    return count


@_jit
def _specular(tx, rx, ant, mir, ft, fobj):
    """Specular paths tx -> m1 [-> m2] -> rx, dense over (m1, m2 + 1). A path
    through the line where m1 meets m2 is kept once, for m2 < m1.

    Returns:
        valid, unfolded length, Gamma1 Gamma2 E_tx E_rx / length.
    """
    plane, mf = mir[0], mir[1]
    nm = plane.shape[0]
    valid = np.zeros((nm, nm + 1), dtype=np.bool_)
    length = np.zeros((nm, nm + 1))
    amp = np.zeros((nm, nm + 1), dtype=np.complex128)
    for m1 in range(nm):
        for k in range(nm + 1):
            if k == m1 + 1:
                continue
            r1 = _image(rx, k, plane)
            r2 = _image(r1, m1 + 1, plane)
            h1t, h1i = _height(m1, tx, plane), _height(m1, r2, plane)
            if h1t * h1i >= 0.0:
                continue
            p1 = _towards(tx, r2, h1t / (h1t - h1i))
            if not _inside(p1[0], p1[1], p1[2], mf[m1], ft):
                continue
            L = _dist(tx, r2)
            p2, f2 = p1, -1
            if k > 0:
                f2 = mf[k - 1]
                h2p, h2i = _height(k - 1, p1, plane), _height(k - 1, r1, plane)
                corner = abs(h2p) <= _EPS
                if (corner and k - 1 > m1) or (not corner and h2p * h2i >= 0.0):
                    continue
                p2 = p1 if corner else _towards(p1, r1, h2p / (h2p - h2i))
                if not _inside(p2[0], p2[1], p2[2], mf[k - 1], ft) or _blocked(
                    p1, p2, ft, fobj, mf[m1], f2, -1
                ):
                    continue
            if _blocked(tx, p1, ft, fobj, mf[m1], -1, -1) or _blocked(
                p2, rx, ft, fobj, f2, -1, -1
            ):
                continue
            g = _reflection(tx, p1, (m1, k - 1), mir)
            et = _field(p1[0] - tx[0], p1[1] - tx[1], p1[2] - tx[2], ant)
            er = _field(p2[0] - rx[0], p2[1] - rx[1], p2[2] - rx[2], ant)
            valid[m1, k], length[m1, k], amp[m1, k] = True, L, g * et * er / L
    return valid, length, amp


@numba.njit(parallel=True, cache=True, error_model="numpy")
def _los_kernel(src, pts, ft, fobj):
    out = np.empty(pts.shape[0], dtype=np.bool_)
    for i in numba.prange(pts.shape[0]):  # pylint: disable=not-an-iterable
        out[i] = not _blocked(src, pts[i], ft, fobj, -1, -1, -1)
    return out


def line_of_sight(geom, src, points):
    """True where the straight segment src -> point [M, 3] crosses no facet."""
    pts = np.ascontiguousarray(np.atleast_2d(points), dtype=np.float64)
    src = np.asarray(src, dtype=np.float64)
    return _los_kernel(src, pts, facet_table(geom), geom.facet_obj)


def paths(geom, tx, rx, lam, antenna=Antenna()):
    """All scatterer and specular paths from tx to rx through `geom`."""
    tx = np.asarray(tx, dtype=np.float64)
    rx = np.asarray(rx, dtype=np.float64)
    ft, fobj, mir = facet_table(geom), geom.facet_obj, mirror_table(geom)
    ant = (antenna.boresight, float(antenna.q), math.sqrt(antenna.g0))
    sc = (
        geom.scat_pos,
        geom.scat_amp,
        geom.scat_kind,
        geom.scat_par,
        geom.scat_host,
        geom.scat_normal,
    )
    ns, no = geom.n_scatterers, mir[0].shape[0] + 1
    vt = np.zeros((ns, no), dtype=np.bool_)
    vr = np.zeros((ns, no), dtype=np.bool_)
    out = (np.zeros(0), np.zeros(0, np.complex128), np.zeros(0, np.int8))
    args = (sc, tx, rx, ant, mir, ft, fobj, vt, vr)
    count = _scatter_kernel(*args, np.zeros(ns, np.int64), False, out)
    total = int(count.sum())
    out = (np.empty(total), np.empty(total, np.complex128), np.empty(total, np.int8))
    _scatter_kernel(*args, np.cumsum(count) - count, True, out)
    valid, length, amp = _specular(tx, rx, ant, mir, ft, fobj)
    valid &= amp != 0.0
    delay = np.concatenate((out[0], length[valid])) / C
    return Paths(
        delay,
        np.concatenate(
            (
                out[1] * (lam / (4.0 * np.pi) ** 1.5),
                amp[valid] * (lam / (4.0 * np.pi)),
            )
        ),
        np.zeros(delay.size),
        np.concatenate((out[2], np.full(int(valid.sum()), 2, dtype=np.int8))),
    )

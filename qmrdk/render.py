"""Ground-truth scene drawing, SAR image display and the scan animation."""

import dataclasses

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.markers import MarkerStyle
from matplotlib.patches import Circle, Patch, Polygon, Rectangle
from matplotlib.path import Path
from matplotlib.transforms import Affine2D
from PIL import Image
from scipy.signal import get_window
from tqdm import tqdm

from qmrdk.dsp.sar import (
    SarImage,
    backproject,
    form_image,
    phase_history,
    sharpness,
)
from qmrdk.sim.propagation import Antenna, facet_table, line_of_sight, mirror_table
from qmrdk.sim.scene import GLINT, ISOTROPIC, TRIHEDRAL

COLOURS = {
    "metal": "#4f5d6b",
    "concrete": "#8c8c8c",
    "brick": "#a9472f",
    "wood": "#8b5a2b",
    "glass": "#6cb8d8",
    "foliage": "#3f8f3f",
    "soil": "#c8b48a",
    "grass": "#9cc46a",
}
OTHER = "#b0b0b0"
RAIL = "#222222"
USED = "#e8322b"
CMAP = "inferno"
PIXEL = (0.05, 0.25)
_NUDGE = 1e-6


_TARGET = {"ls": "", "color": "#1f77b4", "mec": "k", "zorder": 5}
_GHOST = {"ls": "", "ms": 9, "mfc": "none", "mec": "#1f77b4", "mew": 1.5, "zorder": 5}
_WEDGE = Path([(-1.0, 0.5), (0.0, -0.5), (1.0, 0.5), (-1.0, 0.5)], closed=True)


def _reflector(boresight):
    """Wedge marker with its corner away from and its opening towards `boresight`
    (deg)."""
    return MarkerStyle(_WEDGE, transform=Affine2D().rotate_deg(boresight - 90.0))


def _colour(mat):
    return COLOURS.get(mat, OTHER) if isinstance(mat, str) else OTHER


def rail_centre(geometry, x_pos):
    """Sled reference point at the middle of the scanned rail span."""
    x_pos = np.asarray(x_pos, dtype=np.float64)
    return np.array([0.5 * (x_pos.min() + x_pos.max()), 0.0, geometry.height])


def scene_extent(scene, x_pos, margin=0.5):
    """Panel extent `(x0, x1, y0, y1)` covering the ground extent and the rail."""
    x_pos = np.asarray(x_pos, dtype=np.float64)
    x0, x1, y0, y1 = scene.get("ground", {}).get("extent", (-5.0, 5.0, 1.0, 15.0))
    return (
        float(min(x0, x_pos.min() - margin)),
        float(max(x1, x_pos.max() + margin)),
        float(min(y0, -margin)),
        float(y1),
    )


def grid(extent, pixel=PIXEL):
    """Image axes `(gx, gy)` over `extent` at spacing `pixel = (dx, dy)`."""
    x0, x1, y0, y1 = extent
    return (
        x0 + pixel[0] * np.arange(int(np.floor((x1 - x0) / pixel[0])) + 1),
        y0 + pixel[1] * np.arange(int(np.floor((y1 - y0) / pixel[1])) + 1),
    )


def reflector_height(geom, default):
    """Mean reflector height, `default` without reflectors."""
    sel = geom.scat_kind == TRIHEDRAL
    return float(geom.scat_pos[sel, 2].mean()) if sel.any() else float(default)


def shadow_mask(geom, src, gx, gy, z):
    """Grid `[ny, nx]` of points at height `z` hidden from `src` by a facet."""
    xx, yy = np.meshgrid(gx, gy)
    pts = np.column_stack((xx.ravel(), yy.ravel(), np.full(xx.size, float(z))))
    return ~line_of_sight(geom, src, pts).reshape(xx.shape)


def ghost_positions(geom, src, antenna=Antenna()):
    """Apparent positions `[G, 3]` of the reflectors, points and cylinder
    glints seen from `src` via the same mirror facet on both legs: the mirror
    image of the scattering point, for every (scatterer, facet) pair whose
    reflection point lies on the facet in front of the antenna, within the
    reflector's pattern, and unoccluded on both legs. A cylinder's glint
    points give one ghost per mirror (the lowest valid point)."""
    plane, fidx, _, _ = mirror_table(geom)
    plane, fidx = plane[fidx >= 0], fidx[fidx >= 0]
    kind = geom.scat_kind
    sel = np.flatnonzero(np.isin(kind, (TRIHEDRAL, ISOTROPIC, GLINT)))
    kind, par = kind[sel], geom.scat_par[sel]
    a = np.asarray(src, dtype=np.float64)
    n, off = plane[:, :3], plane[:, 3]
    ha = a @ n.T - off
    img_a = a - 2.0 * ha[:, None] * n
    p = np.repeat(geom.scat_pos[sel, None], len(n), axis=1)
    d = img_a[None, :, :2] - p[..., :2]
    glint = kind == GLINT
    p[glint, :, :2] += (
        par[glint, 0, None, None]
        * d[glint]
        / np.linalg.norm(d[glint], axis=-1)[..., None]
    )
    hs = np.einsum("smk,mk->sm", p, n) - off
    u = img_a - p
    with np.errstate(divide="ignore", invalid="ignore"):
        r = p + (hs / (hs + ha))[..., None] * u
    ft = facet_table(geom)[fidx]
    s = (r[..., 0] - ft[:, 0]) * ft[:, 2] + (r[..., 1] - ft[:, 1]) * ft[:, 3]
    ok = (
        (hs * ha > 0.0)
        & (s >= 0.0)
        & (s <= ft[:, 4])
        & (r[..., 2] >= ft[:, 7])
        & (r[..., 2] <= ft[:, 8])
        & (
            (kind != TRIHEDRAL)[:, None]
            | (np.einsum("smk,sk->sm", u[..., :2], par[:, :2]) > 0.0)
        )
    )
    ok[ok] = antenna.field(r[ok] - a) > 0.0
    si, mi = np.nonzero(ok)
    rp, pp = r[si, mi], p[si, mi]
    vis = line_of_sight(geom, a, rp + _NUDGE * (a - rp))
    vis &= [
        line_of_sight(geom, q, (x + _NUDGE * (q - x))[None])[0] for q, x in zip(pp, rp)
    ]
    host = geom.facet_obj[np.maximum(geom.scat_host[sel], 0)]
    group = np.where(glint, -1 - host, np.arange(sel.size))
    keys = np.column_stack((group[si], mi))[vis]
    first = np.unique(keys, axis=0, return_index=True)[1]
    return (pp - 2.0 * hs[si, mi, None] * n[mi])[vis][np.sort(first)]


def draw_scene(
    ax,
    geom,
    geometry,
    x_pos,
    shadow=True,
    ghosts=True,
    extent=None,
    pixel=0.1,
    legend=True,
):
    """Top-down ground truth of compiled scene `geom` for a scan at rail
    positions `x_pos`: objects by material, rail, radar shadow at reflector
    height from the rail centre and predicted multipath ghosts."""
    scene = geom.scene
    x_pos = np.asarray(x_pos, dtype=np.float64)
    extent = scene_extent(scene, x_pos) if extent is None else extent
    ground = scene.get("ground", {"material": "soil"})
    gx0, gx1, gy0, gy1 = ground.get("extent", extent)
    gc = _colour(ground.get("material"))
    ax.add_patch(Rectangle((gx0, gy0), gx1 - gx0, gy1 - gy0, fc=gc, alpha=0.3, lw=0))
    src = rail_centre(geometry, x_pos)
    handles = [Patch(fc=gc, alpha=0.3, label=f"ground ({ground.get('material')})")]
    if shadow:
        sx, sy = grid(extent, (pixel, pixel))
        mask = shadow_mask(geom, src, sx, sy, reflector_height(geom, geometry.height))
        ax.imshow(
            np.ma.masked_where(~mask, mask),
            extent=_edges(sx, sy),
            origin="lower",
            cmap=ListedColormap(["#000000"]),
            alpha=0.3,
            interpolation="nearest",
            zorder=1,
        )
        handles.append(Patch(fc="#000000", alpha=0.3, label="radar shadow"))
    mats = set()
    objects = scene.get("objects", [])
    for k, obj in enumerate(objects):
        kind, mat = obj["type"], obj.get("material")
        c = _colour(mat)
        if kind == "box":
            verts = geom.facet_p0[geom.facet_obj == k]
            ax.add_patch(Polygon(verts, fc=c, ec="k", lw=0.5, zorder=3))
        elif kind == "cylinder":
            ax.add_patch(
                Circle(obj["center"], obj["radius"], fc=c, ec="k", lw=0.5, zorder=3)
            )
        if kind in ("wall", "box", "cylinder"):
            mats.add(mat if isinstance(mat, str) else "other")
    walls = [k for k, o in enumerate(objects) if o["type"] == "wall"]
    wf = np.isin(geom.facet_obj, walls)
    ax.add_collection(
        LineCollection(
            np.stack((geom.facet_p0[wf], geom.facet_p1[wf]), axis=1),
            colors=[_colour(objects[k].get("material")) for k in geom.facet_obj[wf]],
            linewidths=3,
            zorder=3,
        )
    )
    handles += [Patch(fc=_colour(m), ec="k", lw=0.5, label=m) for m in sorted(mats)]
    sel = geom.scat_kind == TRIHEDRAL
    bore = np.degrees(np.arctan2(geom.scat_par[sel, 1], geom.scat_par[sel, 0]))
    for (x, y, _), b in zip(geom.scat_pos[sel], bore):
        ax.plot(x, y, marker=_reflector(b), ms=12, **_TARGET)
    pts = geom.scat_pos[geom.scat_kind == ISOTROPIC]
    ax.plot(pts[:, 0], pts[:, 1], "o", ms=6, **_TARGET)
    handles += [
        Line2D([], [], marker=_reflector(90.0), label="reflector (open to boresight)",
               ms=12, **_TARGET),
        Line2D([], [], marker="o", label="point", ms=6, **_TARGET),
    ]  # fmt: skip
    if ghosts:
        g = ghost_positions(geom, src)
        ax.plot(g[:, 0], g[:, 1], "o", **_GHOST)
        handles.append(Line2D([], [], marker="o", label="multipath ghost", **_GHOST))
    ax.plot(x_pos[[0, -1]], [0.0, 0.0], color=RAIL, lw=3, solid_capstyle="butt")
    ax.plot(x_pos, np.zeros_like(x_pos), "|", color=RAIL, ms=4)
    handles.append(Line2D([], [], color=RAIL, lw=3, marker="|", label="rail"))
    ax.set(xlim=extent[:2], ylim=extent[2:], aspect="equal", xlabel="x (m)")
    ax.set_ylabel("y (m)")
    if legend:
        ax.legend(
            handles=handles,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.1),
            ncols=3,
            fontsize="x-small",
            frameon=False,
        )
    return handles


def _edges(gx, gy):
    def span(g):
        h = 0.5 * (g[1] - g[0]) if g.size > 1 else 0.5
        return g[0] - h, g[-1] + h

    return (*span(np.asarray(gx)), *span(np.asarray(gy)))


def to_db(image, dynamic_range):
    """Magnitude in dB relative to the maximum, floored at `-dynamic_range`."""
    a = np.abs(image)
    ref = max(float(a.max()), np.finfo(float).tiny)
    return 20.0 * np.log10(np.maximum(a / ref, 10.0 ** (-dynamic_range / 20.0)))


def draw_sar(ax, img: SarImage, dynamic_range=40):
    """Image magnitude in dB relative to its maximum on the grid axes."""
    im = ax.imshow(
        to_db(img.image, dynamic_range),
        extent=_edges(img.gx, img.gy),
        origin="lower",
        cmap=CMAP,
        vmin=-dynamic_range,
        vmax=0.0,
        aspect="equal",
    )
    ax.set_xlabel("x (m)")
    return im


def truth_targets(geom, src):
    """Plan positions `[T, 2]` and kinds ("target" or "ghost") of the discrete
    scatterers (reflectors, points, cylinder axes) and their predicted ghosts."""
    objs = geom.scene.get("objects", [])
    pts = [o["pos"][:2] for o in objs if o["type"] in ("reflector", "point")]
    pts += [o["center"] for o in objs if o["type"] == "cylinder"]
    ghosts = ghost_positions(geom, src)[:, :2]
    xy = np.vstack([np.reshape(pts, (-1, 2)), ghosts])
    return xy, ["target"] * len(pts) + ["ghost"] * len(ghosts)


def box_peaks(img: SarImage, xy, half):
    """Image peak in dB relative to the image maximum inside the square of
    half-width `half` around each of `xy`; -inf where the box misses the grid."""
    db = 20.0 * np.log10(np.abs(img.image) / np.abs(img.image).max())
    inx = np.abs(img.gx[None, :] - xy[:, :1]) <= half
    iny = np.abs(img.gy[None, :] - xy[:, 1:]) <= half
    sel = iny[:, :, None] & inx[:, None, :]
    return np.where(
        sel.any(axis=(1, 2)), np.where(sel, db, -np.inf).max(axis=(1, 2)), -np.inf
    )


def overlay_truth(ax, geom, src, img: SarImage | None = None, half=0.75):
    """Truth boxes on the SAR panel: solid for scatterers, dashed for predicted
    ghosts, walls as lines; labelled with the image peak in each box when
    `img` is given."""
    xy, kinds = truth_targets(geom, src)
    peaks = None if img is None else box_peaks(img, xy, half)
    for i, ((x, y), k) in enumerate(zip(xy, kinds)):
        ls = "-" if k == "target" else "--"
        ax.add_patch(
            Rectangle((x - half, y - half), 2 * half, 2 * half, fill=False,
                      ec="#00e5ff", ls=ls, lw=0.8, zorder=6)
        )  # fmt: skip
        if peaks is not None and np.isfinite(peaks[i]):
            ax.text(x + half, y + half, f"{peaks[i]:.0f}", color="#00e5ff",
                    fontsize="xx-small", va="bottom", zorder=6, clip_on=True)  # fmt: skip
    walls = [
        k for k, o in enumerate(geom.scene.get("objects", [])) if o["type"] == "wall"
    ]
    wf = np.isin(geom.facet_obj, walls)
    ax.add_collection(
        LineCollection(np.stack((geom.facet_p0[wf], geom.facet_p1[wf]), axis=1),
                       colors="#00e5ff", linewidths=0.8, linestyles=":", zorder=6)
    )  # fmt: skip
    return peaks


def sar_title(lam, length):
    res = f"{lam / (2.0 * length):.3f} R" if length > 0 else "inf"
    return f"SAR  L = {length:.2f} m,  cross-range res. = {res} m"


def _figure(geom, geometry, x_pos, lam, img, dynamic_range, dpi, extent=None):
    """Figure with the truth panel (when `geom` is given) and the SAR panel
    sharing axes; the SAR title needs `x_pos` and `lam`."""
    if extent is None:
        extent = _edges(img.gx, img.gy)
    width = extent[1] - extent[0]
    height = extent[3] - extent[2]
    panels = 1 if geom is None else 2
    fig = Figure(
        figsize=(1.0 + panels * 5.0, 1.6 + 4.4 * height / width),
        dpi=dpi,
        layout="constrained",
    )
    FigureCanvasAgg(fig)
    axes = fig.subplots(1, panels, sharex=True, sharey=True, squeeze=False)[0]
    if geom is not None:
        draw_scene(axes[0], geom, geometry, x_pos, extent=extent)
        axes[0].set_title(geom.scene.get("name", "scene"))
    im = draw_sar(axes[-1], img, dynamic_range)
    if geom is not None:
        overlay_truth(axes[-1], geom, rail_centre(geometry, x_pos),
                      img if np.any(img.image) else None)  # fmt: skip
    axes[-1].set(xlim=extent[:2], ylim=extent[2:])
    if x_pos is not None and lam is not None:
        axes[-1].set_title(sar_title(lam, float(np.ptp(x_pos))))
    fig.colorbar(im, ax=axes[-1], label="dB", shrink=0.7)
    return fig, axes, im


def save_image(
    path,
    geom,
    img: SarImage,
    geometry=None,
    x_pos=None,
    lam=None,
    dynamic_range=40,
    dpi=120,
    extent=None,
):
    """Static PNG: truth | SAR, or SAR only when `geom` is None. The truth
    panel needs `geometry` and `x_pos`; the SAR title `x_pos` and `lam`."""
    _figure(geom, geometry, x_pos, lam, img, dynamic_range, dpi, extent)[0].savefig(
        path
    )


def write_apng(path, frames, durations):
    """Animated PNG of RGB `frames` on one shared adaptive palette."""
    stack = Image.fromarray(np.concatenate(frames, axis=0))
    pal = stack.quantize(256, method=Image.Quantize.MEDIANCUT)
    ims = [
        Image.fromarray(f).quantize(palette=pal, dither=Image.Dither.NONE)
        for f in frames
    ]
    ims[0].save(
        path,
        format="PNG",
        save_all=True,
        append_images=ims[1:],
        duration=list(durations),
        loop=0,
        optimize=True,
    )


def animate(
    path,
    geom,
    rec,
    cal,
    sweep,
    geometry,
    gx,
    gy,
    z=None,
    frames=24,
    dpi=80,
    dynamic_range=40,
    frame_ms=250,
    hold_ms=3000,
    extent=None,
    background="mean",
    aperture_window=None,
):
    """APNG of truth | SAR while the aperture grows: frame k shows the image
    of the first ceil(k P / frames) positions, accumulated chunk by chunk.
    `background` and `aperture_window` are as for `form_image`.

    Returns the full-aperture `SarImage`.
    """
    x_pos = np.asarray(rec.x_pos, dtype=np.float64)
    z = geometry.height if z is None else float(z)
    up = cal.first_up
    if up is None:
        up = form_image(
            rec.codes, x_pos, sweep, cal, geometry, gx, gy, z,
            background=background, aperture_window=aperture_window,
        ).first_up  # fmt: skip
    ph = phase_history(rec.codes, sweep, cal, up, background)
    tx, rx = geometry.antenna_positions(x_pos, cal)
    w = (
        np.ones(x_pos.size)
        if aperture_window is None
        else get_window(aperture_window, x_pos.size, fftbins=False)
    )
    acc = np.zeros((len(gy), len(gx)), dtype=np.complex128)
    img = SarImage(acc, np.asarray(gx), np.asarray(gy), z, up, 0.0)
    fig, axes, im = _figure(
        geom, geometry, x_pos, sweep.lam, img, dynamic_range, dpi, extent
    )
    marks = [
        (
            ax.plot([], [], color=USED, lw=4, solid_capstyle="butt", zorder=6)[0],
            ax.plot([], [], "s", color=USED, mec="k", ms=7, zorder=7)[0],
        )
        for ax in axes
    ]
    bounds = np.unique(np.ceil(np.arange(1, frames + 1) * x_pos.size / frames))
    rgb, lo = [], 0
    for hi in tqdm(bounds.astype(int), desc="frames", unit="frame"):
        sub = dataclasses.replace(ph, profiles=ph.profiles[lo:hi])
        backproject(sub, tx[lo:hi], rx[lo:hi], gx, gy, z, cal.r_cal, w[lo:hi], acc)
        lo = hi
        im.set_data(to_db(acc, dynamic_range))
        for used, sled in marks:
            used.set_data(x_pos[[0, hi - 1]], [0.0, 0.0])
            sled.set_data([x_pos[hi - 1]], [0.0])
        axes[-1].set_title(sar_title(sweep.lam, x_pos[hi - 1] - x_pos[0]))
        fig.canvas.draw()
        rgb.append(np.array(fig.canvas.buffer_rgba())[..., :3])
    write_apng(path, rgb, [frame_ms] * (len(rgb) - 1) + [hold_ms])
    return dataclasses.replace(img, sharpness=sharpness(acc))

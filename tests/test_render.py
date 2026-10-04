"""Scene drawing, SAR display and the scan animation."""

import dataclasses

import numpy as np
import pytest
from matplotlib.figure import Figure
from PIL import Image
from sarscene import GEOMETRY, HW, SWEEP, scan, tiny

from qmrdk.dsp.sar import SarImage, form_image
from qmrdk.render import (
    animate,
    draw_sar,
    draw_scene,
    ghost_positions,
    grid,
    rail_centre,
    reflector_height,
    save_image,
    scene_extent,
    shadow_mask,
    to_db,
)
from qmrdk.sim.propagation import line_of_sight
from qmrdk.sim.scene import compile_scene

SRC = np.array([0.15, 0.0, 1.0])


@pytest.fixture(name="geom", scope="module")
def fixture_geom():
    return compile_scene(tiny(), SWEEP.lam)


@pytest.fixture(name="scanned", scope="module")
def fixture_scanned():
    return scan()


def test_extent_and_grid():
    ext = scene_extent(tiny(), [0.0, 0.3])
    assert ext == (-4.0, 4.0, -0.5, 12.0)
    assert scene_extent({}, [0.0, 0.3], margin=0.0) == (-5.0, 5.0, 0.0, 15.0)
    gx, gy = grid(ext, (0.5, 2.0))
    assert gx[0] == -4.0 and gx[-1] == pytest.approx(4.0) and gx.size == 17
    assert np.allclose(np.diff(gy), 2.0) and gy[-1] <= 12.0
    np.testing.assert_allclose(rail_centre(GEOMETRY, [0.0, 0.3]), SRC)


def test_reflector_height(geom):
    assert reflector_height(geom, 7.0) == 1.0
    empty = compile_scene({"ground": {"material": "soil"}}, SWEEP.lam)
    assert reflector_height(empty, 7.0) == 7.0


def test_shadow_mask_matches_line_of_sight(geom):
    gx, gy = np.linspace(-4, 4, 9), np.linspace(2, 12, 11)
    mask = shadow_mask(geom, SRC, gx, gy, 1.0)
    xx, yy = np.meshgrid(gx, gy)
    pts = np.column_stack((xx.ravel(), yy.ravel(), np.ones(xx.size)))
    np.testing.assert_array_equal(mask.ravel(), ~line_of_sight(geom, SRC, pts))
    assert mask[gy == 11, np.abs(gx) <= 2].all() and not mask[gy == 11, 0].any()
    assert not mask[gy == 3, gx == 0].any()
    assert mask[gy == 8, gx == 3].all()


def _glint_ghost(centre, radius, z, wall_y):
    img_a = np.array([SRC[0], 2 * wall_y - SRC[1]])
    d = img_a - centre
    p = centre + radius * d / np.linalg.norm(d)
    return np.array([p[0], 2 * wall_y - p[1], z])


def test_ghost_positions_closed_form(geom):
    got = ghost_positions(geom, SRC)
    want = np.array(
        [
            _glint_ghost(np.array([-2.5, 4.0]), 0.05, 0.5, 10.0),
            [-1.0, 13.0, 1.0],
            [-1.5, 17.0, 1.0],
        ]
    )
    got, want = got[:, :2], want[:, :2]
    key = np.lexsort(got.T[::-1])
    np.testing.assert_allclose(got[key], want[np.lexsort(want.T[::-1])], atol=1e-12)


def test_ghost_occlusion_and_finite_mirror():
    scene = tiny()
    scene["objects"][0]["p1"] = [-0.8, 10]
    blocker = {"type": "box", "center": [-1.354, 4.5], "size": [0.3, 0.2],
               "z": [0, 2], "material": "wood"}  # fmt: skip
    got = ghost_positions(compile_scene(scene, SWEEP.lam), SRC)
    np.testing.assert_allclose(np.sort(got[:, 0]), [-2.5, -1.5], atol=0.05)
    scene["objects"].append(blocker)
    got = ghost_positions(compile_scene(scene, SWEEP.lam), SRC)
    assert len(got) == 1 and got[0, 0] < -2.0


def _axes():
    return Figure().add_subplot()


def test_draw_scene_artists(geom):
    ax = _axes()
    handles = draw_scene(ax, geom, GEOMETRY, [0.0, 0.3])
    labels = [h.get_label() for h in handles]
    for want in ("radar shadow", "multipath ghost", "concrete", "metal", "rail"):
        assert want in labels
    assert len(ax.patches) == 3
    assert len(ax.images) == 1 and len(ax.collections) == 1
    ghost = [ln for ln in ax.lines if ln.get_mfc() == "none"]
    assert len(ghost) == 1 and len(ghost[0].get_xdata()) == 3
    assert ax.get_xlim() == (-4.0, 4.0) and ax.get_legend() is not None
    bare = _axes()
    handles = draw_scene(bare, geom, GEOMETRY, [0.0, 0.3], False, False, legend=False)
    assert "radar shadow" not in [h.get_label() for h in handles]
    assert not bare.images and bare.get_legend() is None


def test_draw_sar_db():
    gx, gy = np.arange(5.0), np.arange(3.0) * 2
    img = np.zeros((3, 5), complex)
    img[1, 2], img[0, 0] = 10.0, 1.0
    im = draw_sar(_axes(), SarImage(img, gx, gy, 1.0, True, 0.0), 30)
    data = im.get_array()
    assert data[1, 2] == 0.0 and data[0, 0] == pytest.approx(-20.0)
    assert data.min() == -30.0
    assert im.get_extent() == [-0.5, 4.5, -1.0, 5.0]
    assert np.all(to_db(np.zeros((2, 2)), 10) == -10.0)


def test_save_image(tmp_path, geom):
    img = SarImage(np.ones((2, 3), complex), np.arange(3.0), np.arange(2.0), 1, True, 0)
    save_image(tmp_path / "a.png", None, img)
    save_image(tmp_path / "b.png", geom, img, GEOMETRY, [0.0, 0.3], SWEEP.lam)
    a, b = Image.open(tmp_path / "a.png"), Image.open(tmp_path / "b.png")
    assert b.size[0] > a.size[0]


@pytest.mark.parametrize(
    "first_up, window, background", [(True, None, None), (None, "hann", "mean")]
)
def test_animate_accumulates(tmp_path, scanned, first_up, window, background):
    geom, rec = scanned
    cal = dataclasses.replace(HW.calibration(SWEEP), first_up=first_up)
    gx, gy = grid((-2.0, 2.0, 2.0, 9.0), (0.2, 0.5))
    path = tmp_path / "anim.png"
    img = animate(
        path, geom, rec, cal, SWEEP, GEOMETRY, gx, gy, frames=4, dpi=40,
        frame_ms=100, hold_ms=900, aperture_window=window, background=background,
    )  # fmt: skip
    ref = form_image(
        rec.codes, rec.x_pos, SWEEP, cal, GEOMETRY, gx, gy,
        background=background, aperture_window=window,
    )  # fmt: skip
    assert img.first_up == ref.first_up
    np.testing.assert_allclose(img.image, ref.image, rtol=1e-10, atol=0)
    assert img.sharpness == pytest.approx(ref.sharpness)
    with Image.open(path) as im:
        assert im.format == "PNG" and im.n_frames == 4 and im.info["loop"] == 0
        size = im.size
        durations = []
        for k in range(im.n_frames):
            im.seek(k)
            durations.append(im.info["duration"])
            assert im.size == size
    assert durations == [100, 100, 100, 900]
    if background is None:
        iy, ix = divmod(int(np.abs(img.image).argmax()), gx.size)
        assert abs(gx[ix]) <= 0.2 and abs(gy[iy] - 5.0) <= 0.5


def test_animate_more_frames_than_positions(tmp_path, scanned):
    geom, rec = scanned
    gx, gy = grid((-1.0, 1.0, 4.0, 6.0), (0.5, 1.0))
    path = tmp_path / "anim.png"
    animate(path, geom, rec, HW.calibration(SWEEP), SWEEP, GEOMETRY, gx, gy,
            frames=4 * rec.x_pos.size, dpi=30)  # fmt: skip
    with Image.open(path) as im:
        assert im.n_frames == rec.x_pos.size

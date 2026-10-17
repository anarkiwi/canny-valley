"""LINZ aerial photo basemaps and SAR overlays, against a fake tile server."""

import io
import math
import re

import numpy as np
import pytest
from PIL import Image
from sarscene import scan

from qmrdk import basemap, cli
from qmrdk.dsp.sar import SarImage

LAT, LON = -41.1, 175.1
KEY = "0123abcd"
URL = (
    "https://tiles.example/services;key="
    + KEY
    + "/tiles/v4/layer={n},{{style}}/{{TileMatrixSet}}/{{TileMatrix}}/{{TileCol}}/{{TileRow}}.png"
)


def _layer(n, title, box=(174.9, -41.2, 175.2, -41.0), sets=("EPSG:3857",)):
    links = "".join(
        f"<TileMatrixSetLink><TileMatrixSet>{s}</TileMatrixSet></TileMatrixSetLink>"
        for s in sets
    )
    return (
        f"<Layer><ows:Title>{title}</ows:Title>"
        f"<ows:WGS84BoundingBox><ows:LowerCorner>{box[0]} {box[1]}</ows:LowerCorner>"
        f"<ows:UpperCorner>{box[2]} {box[3]}</ows:UpperCorner></ows:WGS84BoundingBox>"
        f'{links}<ResourceURL format="image/png" resourceType="tile" '
        f'template="{URL.format(n=n)}"/></Layer>'
    )


CAPS = (
    '<Capabilities xmlns="http://www.opengis.net/wmts/1.0" '
    'xmlns:ows="http://www.opengis.net/ows/1.1"><Contents>'
    + _layer(1, "Town 0.075m Urban Aerial Photos (2025)")
    + _layer(2, "Town 0.10m Urban Aerial Photos (2017)")
    + _layer(3, "Region 0.3m Rural Aerial Photos (2016-17)")
    + _layer(4, "Region 0.2m Rural Aerial Photos (2025-2026)")
    + _layer(5, "Region 0.3m Rural Aerial Photos (2021)")
    + _layer(6, "Town LiDAR 1m DSM (2021)")
    + _layer(
        7, "Taupo 0.05m Urban Aerial Photos (2024)", box=(175.9, -38.8, 176.2, -38.6)
    )
    + _layer(8, "Region 0.05m Urban Aerial Photos (2024)", sets=("EPSG:2193",))
    + "</Contents></Capabilities>"
)


def _png(rgba):
    out = io.BytesIO()
    Image.fromarray(np.full((256, 256, 4), rgba, np.uint8)).save(out, "PNG")
    return out.getvalue()


class Server:
    """Layer 1 transparent, 2 black no-data, others tiles coloured by their
    column and row."""

    def __init__(self):
        self.urls = []

    def __call__(self, url):
        self.urls.append(url)
        if url.endswith("WMTSCapabilities.xml"):
            return CAPS.encode()
        n, col, row = map(
            int, re.search(r"layer=(\d+).*/(\d+)/(\d+)\.png$", url).groups()
        )
        return _png(
            {1: (0, 0, 0, 0), 2: (0, 0, 0, 255)}.get(n, (col % 256, row % 256, n, 255))
        )


@pytest.fixture(name="server")
def fixture_server(monkeypatch):
    server = Server()
    monkeypatch.setattr(basemap, "_get", server)
    return server


def test_parse_layers():
    layers = {lay.title.split(" (")[0]: lay for lay in basemap.parse_layers(CAPS)}
    assert "Town LiDAR 1m DSM" not in layers
    assert not any(t.startswith("Region 0.05m") for t in layers)
    years = sorted((lay.gsd, lay.year) for lay in basemap.parse_layers(CAPS))
    assert years == [
        (0.05, 2024),
        (0.075, 2025),
        (0.1, 2017),
        (0.2, 2026),
        (0.3, 2017),
        (0.3, 2021),
    ]
    order = basemap.candidates(basemap.parse_layers(CAPS), LAT, LON)
    assert [(lay.gsd, lay.year) for lay in order] == [
        (0.075, 2025),
        (0.1, 2017),
        (0.2, 2026),
        (0.3, 2021),
        (0.3, 2017),
    ]
    assert not any(lay.title.startswith("Taupo") for lay in order)


def test_tiles_match_slippy_map():
    z = 20
    col = (LON + 180) / 360 * 2**z
    row = (1 - math.asinh(math.tan(math.radians(LAT))) / math.pi) / 2 * 2**z
    (c0, c1, r0, r1), size = basemap.tile_range(LAT, LON, 0.0, z)
    assert (c0, c1, r0, r1) == (int(col),) * 2 + (int(row),) * 2
    assert size == pytest.approx(2 * basemap.HALF_CIRCUMFERENCE / 2**z)
    (c0, c1, r0, r1), _ = basemap.tile_range(
        LAT, LON, 3 * size * math.cos(math.radians(LAT)), z
    )
    assert c1 - c0 in (6, 7) and r1 - r0 in (6, 7)


@pytest.mark.parametrize("gsd", [0.075, 0.2, 1.0])
def test_zoom_for(gsd):
    z = basemap.zoom_for(gsd, LAT)
    pixel = 2 * basemap.HALF_CIRCUMFERENCE * math.cos(math.radians(LAT)) / 256
    assert pixel / 2**z <= gsd < pixel / 2 ** (z - 1)


def test_find_photo_skips_empty_layers(server, tmp_path):
    xml = basemap.capabilities(KEY, tmp_path)
    assert KEY not in (tmp_path / "capabilities.xml").read_text()
    assert basemap.capabilities(KEY, tmp_path) == xml and len(server.urls) == 1
    layer, z, rgba, extent = basemap.find_photo(
        basemap.parse_layers(xml), LAT, LON, 10.0, cache=tmp_path
    )
    assert layer.title.startswith("Region 0.2m") and z == basemap.zoom_for(0.2, LAT)
    (c0, c1, r0, r1), size = basemap.tile_range(LAT, LON, 10.0, z)
    assert rgba.shape == (256 * (r1 - r0 + 1), 256 * (c1 - c0 + 1), 4)
    assert tuple(rgba[0, 0, :3]) == (c0 % 256, r0 % 256, 4)
    assert tuple(rgba[-1, -1, :3]) == (c1 % 256, r1 % 256, 4)
    k = math.cos(math.radians(LAT))
    assert extent[1] - extent[0] == pytest.approx(k * size * (c1 - c0 + 1))
    assert extent[0] < -10 and extent[1] > 10 and extent[2] < -10 and extent[3] > 10
    fetched = len(server.urls)
    basemap.find_photo(basemap.parse_layers(xml), LAT, LON, 10.0, cache=tmp_path)
    assert len(server.urls) == fetched


def test_find_photo_none(server):
    with pytest.raises(LookupError):
        basemap.find_photo(basemap.parse_layers(CAPS), 0.0, 0.0, 10.0)
    assert not server.urls


@pytest.mark.parametrize("flip, east", [(False, 1.0), (True, -1.0)])
def test_rail_transform(flip, east):
    t = basemap.rail_transform(2.0, 90.0, flip)
    np.testing.assert_allclose(
        t.transform([[2.0, 5.0], [3.0, 0.0]]), [[5.0, 0.0], [0.0, -east]], atol=1e-12
    )
    np.testing.assert_allclose(
        basemap.rail_transform(0.0, 30.0).transform([[0.0, 2.0]]), [[1.0, math.sqrt(3)]]
    )


def test_sar_rgba():
    img = SarImage(
        np.array([[1.0, 0.1, 1e-6]]), np.arange(3.0), np.zeros(1), 0.0, True, 0.0
    )
    rgba = basemap.sar_rgba(img, 40.0, 0.5)
    np.testing.assert_allclose(rgba[0, :, 3], [0.5, 0.25, 0.0])


def test_overlay_cli(server, tmp_path, monkeypatch, capsys):
    path = tmp_path / "scan.npz"
    scan()[1].save(path)
    out = tmp_path / "o.png"
    argv = [
        "sar",
        "overlay",
        str(path),
        "--out",
        str(out),
        "--lat",
        str(LAT),
        "--lon",
        str(LON),
        "--bearing",
        "50",
        "--radius",
        "8",
        "--pixel",
        "0.2",
        "0.2",
    ]
    monkeypatch.delenv("LINZ_API_KEY", raising=False)
    assert cli.main(argv) == 1 and "LINZ_API_KEY" in capsys.readouterr().err
    monkeypatch.setenv("LINZ_API_KEY", KEY)
    assert cli.main(argv + ["--cache", str(tmp_path / "c"), "--r-cal", "0.5"]) == 0
    assert "Region 0.2m" in capsys.readouterr().out and Image.open(out).size[0] > 0
    assert any("layer=4," in u and KEY in u for u in server.urls)
    assert cli.main(argv + ["--min-range", "9"]) == 1

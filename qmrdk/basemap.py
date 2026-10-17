"""LINZ Data Service aerial photo basemaps and SAR images placed on them.

Tiles come from the LDS WMTS (EPSG:3857, an API key is required). The map
is drawn in local east/north metres about the radar: over a few hundred
metres Web Mercator is a uniform scale `cos(lat)` from ground metres.
"""

import io
import math
import pathlib
import re
import urllib.request
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass

import numpy as np
from matplotlib import colormaps
from matplotlib.cm import ScalarMappable
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.colors import Normalize
from matplotlib.figure import Figure
from matplotlib.transforms import Affine2D
from PIL import Image
from tqdm import tqdm

from qmrdk.render import CMAP, _edges, to_db

CAPABILITIES = (
    "https://data.linz.govt.nz/services;key={key}/wmts/1.0.0/WMTSCapabilities.xml"
)
ATTRIBUTION = "Aerial imagery: {title}, LINZ, CC BY 4.0"
HALF_CIRCUMFERENCE = math.pi * 6378137.0
_NS = {"w": "http://www.opengis.net/wmts/1.0", "ows": "http://www.opengis.net/ows/1.1"}
_PHOTO = re.compile(r"([\d.]+)\s*m .*Aerial Photos.*\((\d{4})(?:-(\d{2,4}))?\)")


@dataclass(frozen=True)
class Layer:
    """An aerial photo layer: ground sample distance (m), last capture year,
    WGS84 bounds `(lon0, lat0, lon1, lat1)` and tile URL template."""

    title: str
    gsd: float
    year: int
    bounds: tuple
    template: str

    def covers(self, lat, lon):
        lon0, lat0, lon1, lat1 = self.bounds
        return lon0 <= lon <= lon1 and lat0 <= lat <= lat1


def _year(first, last):
    if last is None:
        return int(first)
    return int(last) if len(last) == 4 else int(first[:2] + last)


def parse_layers(xml):
    """Aerial photo layers with an EPSG:3857 tile set in WMTS capabilities."""
    out = []
    for el in ET.fromstring(xml).iter(f"{{{_NS['w']}}}Layer"):
        title = el.findtext("ows:Title", "", _NS)
        m = _PHOTO.search(title)
        sets = {t.text for t in el.iterfind("w:TileMatrixSetLink/w:TileMatrixSet", _NS)}
        url = el.find("w:ResourceURL", _NS)
        box = el.find("ows:WGS84BoundingBox", _NS)
        if m is None or "EPSG:3857" not in sets or url is None or box is None:
            continue
        lo = [float(v) for v in box.findtext("ows:LowerCorner", "", _NS).split()]
        hi = [float(v) for v in box.findtext("ows:UpperCorner", "", _NS).split()]
        out.append(
            Layer(
                title,
                float(m.group(1)),
                _year(m.group(2), m.group(3)),
                (*lo, *hi),
                url.get("template"),
            )
        )
    return out


def candidates(layers, lat, lon):
    """Layers whose bounds contain `(lat, lon)`, finest then most recent
    first (a layer's photos need not fill its bounds)."""
    found = [lay for lay in layers if lay.covers(lat, lon)]
    return sorted(found, key=lambda lay: (lay.gsd, -lay.year))


def _get(url):
    with urllib.request.urlopen(url, timeout=60) as r:
        return r.read()


def capabilities(key, cache=None):
    """Capabilities XML, cached under `cache` with the key removed."""
    path = None if cache is None else pathlib.Path(cache) / "capabilities.xml"
    if path is not None and path.is_file():
        return path.read_text().replace("{key}", key)
    xml = _get(CAPABILITIES.format(key=key)).decode()
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(xml.replace(key, "{key}"))
    return xml


def mercator(lat, lon):
    """EPSG:3857 metres."""
    x = HALF_CIRCUMFERENCE * lon / 180.0
    y = HALF_CIRCUMFERENCE / math.pi * math.asinh(math.tan(math.radians(lat)))
    return x, y


def zoom_for(gsd, lat):
    """Smallest zoom whose 256-pixel tiles are at least as fine as `gsd`."""
    return math.ceil(
        math.log2(2 * HALF_CIRCUMFERENCE * math.cos(math.radians(lat)) / (256 * gsd))
    )


def tile_range(lat, lon, half, z):
    """Tile columns and rows `(c0, c1, r0, r1)` (inclusive) covering ground
    metres `half` about `(lat, lon)`, and the mercator metres per tile."""
    x, y = mercator(lat, lon)
    size = 2 * HALF_CIRCUMFERENCE / 2**z
    h = half / math.cos(math.radians(lat))
    c0, c1, r0, r1 = np.floor(
        (np.array([x - h, x + h, -y - h, -y + h]) + HALF_CIRCUMFERENCE) / size
    ).astype(int)
    return (int(c0), int(c1), int(r0), int(r1)), size


def mosaic(template, lat, lon, half, z, cache=None, fetch=None, workers=8):
    """RGBA mosaic covering ground metres `half` about `(lat, lon)` at zoom
    `z`, and its extent `(e0, e1, n0, n1)` in east/north metres from it."""
    (c0, c1, r0, r1), size = tile_range(lat, lon, half, z)
    keys = [(c, r) for r in range(r0, r1 + 1) for c in range(c0, c1 + 1)]
    root = (
        None
        if cache is None
        else pathlib.Path(cache) / re.sub(r"\W+", "_", template.split("/")[-5])
    )

    def tile(cr):
        path = None if root is None else root / f"{z}_{cr[0]}_{cr[1]}.png"
        if path is not None and path.is_file():
            data = path.read_bytes()
        else:
            url = template.format(
                style="style=auto",
                TileMatrixSet="EPSG:3857",
                TileMatrix=z,
                TileCol=cr[0],
                TileRow=cr[1],
            )
            data = (fetch or _get)(url)
            if path is not None:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
        return np.asarray(Image.open(io.BytesIO(data)).convert("RGBA"))

    with ThreadPoolExecutor(workers) as ex:
        tiles = list(
            tqdm(ex.map(tile, keys), total=len(keys), desc="tiles", unit="tile")
        )
    ncol = c1 - c0 + 1
    rows = [
        np.concatenate(tiles[i : i + ncol], axis=1) for i in range(0, len(tiles), ncol)
    ]
    x, y = mercator(lat, lon)
    k = math.cos(math.radians(lat))
    left = c0 * size - HALF_CIRCUMFERENCE
    top = HALF_CIRCUMFERENCE - r0 * size
    extent = (
        k * (left - x),
        k * (left + (c1 - c0 + 1) * size - x),
        k * (top - (r1 - r0 + 1) * size - y),
        k * (top - y),
    )
    return np.concatenate(rows, axis=0), extent


def find_photo(layers, lat, lon, half, zoom=None, cache=None, fetch=None, cover=0.9):
    """First of `candidates` whose mosaic about `(lat, lon)` is at least
    `cover` photo (opaque and not the black no-data fill), at `zoom` or the
    layer's resolution: `(layer, zoom, rgba, extent)` as for `mosaic`."""
    for layer in candidates(layers, lat, lon):
        z = zoom or zoom_for(layer.gsd, lat)
        rgba, extent = mosaic(layer.template, lat, lon, half, z, cache, fetch)
        if np.mean((rgba[..., 3] > 0) & rgba[..., :3].any(-1)) >= cover:
            return layer, z, rgba, extent
    raise LookupError(f"no aerial photos cover {lat}, {lon}")


def rail_transform(xc, bearing, flip=False):
    """Affine map from image `(x, y)` (rail, range) to east/north metres:
    rail centre `xc` at the origin, range along `bearing` (degrees true),
    +x to the right of boresight (left with `flip`)."""
    return (
        Affine2D()
        .translate(-xc, 0.0)
        .scale(-1.0 if flip else 1.0, 1.0)
        .rotate_deg(-bearing)
    )


def sar_rgba(img, dynamic_range, alpha=0.8):
    """Colour-mapped image whose opacity rises from 0 at the floor to `alpha`
    at the peak."""
    db = to_db(img.image, dynamic_range)
    rgba = colormaps[CMAP](Normalize(-dynamic_range, 0.0)(db))
    rgba[..., 3] = alpha * (1.0 + db / dynamic_range)
    return rgba


def save_overlay(
    path,
    photo,
    extent,
    img,
    xc,
    bearing,
    view=None,
    dynamic_range=40,
    alpha=0.8,
    flip=False,
    attribution="",
    title="",
    dpi=150,
):
    """PNG of the SAR image on the aerial photo, in east/north metres, limited
    to `view` `(e0, e1, n0, n1)` (default the photo)."""
    view = extent if view is None else view
    fig = Figure(figsize=(8.0, 8.0), dpi=dpi, layout="constrained")
    FigureCanvasAgg(fig)
    ax = fig.add_subplot()
    ax.imshow(photo, extent=extent, origin="upper", interpolation="lanczos")
    ax.imshow(
        sar_rgba(img, dynamic_range, alpha),
        extent=_edges(img.gx, img.gy),
        origin="lower",
        transform=rail_transform(xc, bearing, flip) + ax.transData,
    )
    ax.plot(0.0, 0.0, "c^", ms=8, mec="k")
    fig.colorbar(
        ScalarMappable(Normalize(-dynamic_range, 0.0), CMAP),
        ax=ax,
        shrink=0.7,
        label="dB",
    )
    ax.set(
        xlim=view[:2],
        ylim=view[2:],
        xlabel="east (m)",
        ylabel="north (m)",
        aspect="equal",
        title=title,
    )
    ax.text(
        0.01,
        0.01,
        attribution,
        transform=ax.transAxes,
        fontsize=7,
        color="w",
        va="bottom",
    )
    fig.savefig(path)

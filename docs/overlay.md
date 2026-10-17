# Aerial photo overlay

`qmrdk sar overlay` (`qmrdk/basemap.py`) forms the SAR image of a scan
recording on a square `±radius` about the rail centre and draws it on a
LINZ aerial photo of the site.

```
export LINZ_API_KEY=...        # LINZ Data Service key (data.linz.govt.nz)
qmrdk sar overlay artifacts/scan.npz --lat LAT --lon LON --bearing DEG --out map.png
```

| Option | Meaning |
|--------|---------|
| `--lat`, `--lon` | rail centre, WGS84 degrees |
| `--bearing` | boresight (range axis), degrees true |
| `--snap R` | move `--bearing` to the nearest wall axis of buildings within `R` m |
| `--flip` | `+x` (away from home) is left of boresight; default right |
| `--image`, `--image-origin COL ROW`, `--image-scale` | use a north-up image instead of the LINZ photo: rail-centre pixel and pixels per metre (no key needed) |
| `--r-cal` | override the calibration's `r_cal` (m) |
| `--radius` | map half-width and maximum imaged range, m |
| `--zoom` | tile zoom; default the finest at or below the layer's GSD |
| `--cache` | tile and capabilities cache (default `artifacts/basemap`) |

## Imagery

The LINZ Data Service WMTS capabilities list every aerial photo layer with
its title, WGS84 bounds and an EPSG:3857 tile template. Layers whose title
gives a ground sample distance and capture year (`… 0.2m Rural Aerial Photos
(2025-2026)`) are candidates when their bounds contain the site, tried
finest then most recent first. A layer's bounds are a rectangle around an
irregular footprint, so outside its photos it serves transparent or black
no-data tiles; the first candidate whose mosaic is at least 90 % photo is
used. Tiles and the capabilities (with the key removed) are cached.

Imagery is © LINZ, CC BY 4.0; the plot carries the layer title as
attribution.

## Geometry

Over the plotted area Web Mercator is a uniform scale `cos(lat)` from ground
metres, so the photo is drawn in east/north metres from the site. The SAR
grid `(x, y)` maps to the same plane by translating the rail centre to the
origin and rotating so `+y` points along `--bearing`, with `+x` to its right
(left with `--flip`). The image is colour-mapped in dB with opacity rising
from zero at the dynamic-range floor.

`--snap` finds the dominant pair of perpendicular edge directions in the
photo (`dominant_axis`): the angle of the squared-gradient-weighted mean of
`exp(4jθ)` over Gaussian-derivative gradient directions `θ`, which is
invariant to 90° turns, so both walls of a rectilinear building vote for
the same axis. The bearing moves to the nearest multiple of 90° from that
axis; the coherence (0 to 1) reports how strongly the edges agree.

The rail is drawn as a solid line, the boresight dotted, and the radar as an
arrowhead pointing along the boresight.

Without a calibrated `r_cal` the range axis includes the board's internal
delay and returns plot beyond their true position; the title states which.

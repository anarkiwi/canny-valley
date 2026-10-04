# Simulation model

The simulator produces raw ADC frames that the device would return for a
given scene, radar position and board. Its purpose is to test the signal
processing, SAR imaging and calibration without hardware. It has three
parts with one interface each:

| Part | Module | Input → output |
|------|--------|----------------|
| Scene | `qmrdk/sim/scene.py` | scene file → compiled geometry arrays |
| Propagation | `qmrdk/sim/propagation.py` | geometry, tx and rx phase centres → propagation paths (delay, complex amplitude, radial speed) |
| Board | `qmrdk/sim/hardware.py` | paths, sweep, board model → uint16 codes |

`qmrdk/sim/devices.py` wraps these behind the `Radar` and `Sled` interfaces
of `qmrdk/radar.py`.

## 1. Coordinates

Right-handed, metres. `x` along the rail, `y` towards the scene, `z` up.
The ground is the plane `z = 0`. The sled reference point at rail position
`x` is `(x, 0, height)` (`ScanGeometry`); antenna phase centres are the
reference plus the board's `tx_offset` and `rx_offset`.

## 2. Scene

A scene is a JSON file (examples in `qmrdk/scenes/`):

```json
{
  "name": "yard",
  "seed": 1,
  "ground": {"material": "soil", "clutter_density": 2.0, "extent": [-15, 15, 1, 30]},
  "objects": [
    {"type": "wall", "p0": [-8, 20], "p1": [8, 20], "z": [0, 3], "material": "concrete"},
    {"type": "box", "center": [3, 10], "size": [2, 1], "angle": 30, "z": [0, 1.2], "material": "metal"},
    {"type": "cylinder", "center": [-4, 12], "radius": 0.15, "z": [0, 3], "material": "metal"},
    {"type": "reflector", "pos": [0, 15, 1.0], "rcs": 10.0, "boresight": -90, "beamwidth": 40},
    {"type": "point", "pos": [5, 18, 0.5], "rcs": 0.5}
  ]
}
```

Angles are in degrees in the `xy` plane from `+x`. A reflector's boresight
is the direction it faces (−90 faces the rail).

### 2.1 Materials

| Name | Relative permittivity | `sigma0` (diffuse backscatter, m²/m²) | Smooth |
|------|------------------------|------|--------|
| `metal` | perfect conductor (reflection coefficient −1) | 0.01 | yes |
| `concrete` | 6.0 − 0.6j | 0.05 | yes |
| `brick` | 4.5 − 0.4j | 0.08 | yes |
| `wood` | 2.0 − 0.2j | 0.03 | yes |
| `glass` | 6.5 − 0.1j | 0.002 | yes |
| `foliage` | 1.5 − 0.3j | 0.2 | no |
| `soil` | 10.0 − 2.0j | 0.002 | yes |
| `grass` | 8.0 − 2.5j | 0.01 | no |

A material can also be given inline as
`{"eps_r": [re, im], "sigma0": s, "smooth": true}`. Smooth surfaces act as
mirrors (§3.2); every surface also scatters diffusely with `sigma0`.

Reflection coefficients are Fresnel coefficients at the local grazing
angle for the polarisation of the antennas (`"polarization": "h"` or `"v"`
in the scene, default `"h"`; horizontal relative to the ground). For
vertical facets the field is treated as the complementary polarisation, so
an `h`-polarised radar sees TM reflection from walls.

### 2.2 Compilation

`compile_scene(scene, lam) -> Geometry` produces flat float64 / int64 arrays
suitable for numba:

* **Facets**: vertical rectangles, one per wall, four per box, `n_facets`
  per cylinder (default 16). Each has endpoints `p0, p1` (xy), `z0, z1`,
  unit normal (xy), material permittivity, `sigma0`, a `mirror` flag
  (smooth and not part of a cylinder) and an object id. All facets occlude.
* **Scatterers**: points with position, complex amplitude `sqrt(rcs)·e^{jφ}`,
  a pattern kind and pattern parameters, a host facet id (−1 for none) and
  a normal.
  * Diffuse points are drawn uniformly on every facet with density
    `max(4 / lam**2 * 0.05, 4)` per m² unless the scene sets
    `"diffuse_density"`, each with `rcs = sigma0 * area / count`, random
    phase, Lambertian pattern (§3.3), host facet set.
  * Ground clutter points are drawn on `ground.extent` (`[x0, x1, y0, y1]`)
    with `clutter_density` per m² and `rcs = sigma0 * area / count`,
    excluding points inside object footprints, normal `+z`.
  * Reflectors: trihedral pattern centred on `boresight` with half-power
    `beamwidth`.
  * Points: isotropic.
  * Cylinders: a specular glint whose position follows the aspect (§3.3),
    `rcs = 2*pi*radius*h**2 / lam` with `h = z1 - z0`, at mid height.
* All random draws use `numpy.random.default_rng(scene["seed"])`, so a
  scene compiles to the same geometry every time.

## 3. Propagation

`paths(geom, tx, rx, lam, antenna) -> Paths` with fields `delay` (s, one-way
total path length / c), `amp` (complex, field amplitude in √W for 1 W
transmitted) and `speed` (m/s, all zero for static scenes). The antenna
model holds gain `g0` (linear), boresight (default `+y`, level) and
half-power beamwidth; its field pattern is `cos(theta)**q` for
`theta < 90°` and 0 behind, with `q` chosen so the power pattern is −3 dB at
half the beamwidth.

### 3.1 Path classes

Mirrors are the ground plane and every facet with the `mirror` flag. A
**leg** joins an antenna to a scatterer, directly or via exactly one mirror.

| Class | Route | Amplitude |
|-------|-------|-----------|
| Scatterer | tx → (≤ 1 mirror) → scatterer → (≤ 1 mirror) → rx | `Γt Γr · sqrt(σ) F_s · E_tx E_rx · lam / ((4π)^1.5 · Lt · Lr)` |
| Specular | tx → m1 → rx, or tx → m1 → m2 → rx (m1 ≠ m2) | `Γ1 (Γ2) · E_tx E_rx · lam / (4π · L)` |

`Lt`, `Lr`, `L` are unfolded path lengths, `E` the antenna field gains
`sqrt(g0) · pattern(direction)` along the departing and arriving rays,
`Γ` the Fresnel coefficients at each bounce (1 for no bounce), `σ` and
`F_s` the scatterer's cross-section and pattern for the incoming and
outgoing directions. The specular class produces the glint of a flat
surface seen at normal incidence and the wall–ground dihedral.

Reflection points are found with the image method: reflect the far
endpoint in the mirror plane, intersect the straight line with the plane.
A reflection is valid only if the point lies inside the mirror (within the
facet's segment and height range; ground points anywhere) and both
endpoints are on the same side of the mirror.

### 3.2 Occlusion

Every straight segment of every path is tested against every facet: it is
blocked if it crosses the facet rectangle strictly between its endpoints.
The facet that hosts an endpoint (the mirror being reflected from, or the
host facet of the scatterer) is excluded from that segment's test. The
ground does not occlude (antennas and scatterers are above it). A
scatterer on a facet is visible on a leg only from the side its normal
faces; ground clutter only from above.

Occlusion is what produces radar shadows behind objects; image-method
reflection is what produces multipath ghosts (a reflector seen via a wall
appears at its mirror image) and the bright wall–ground line.

### 3.3 Scatterer patterns

With `u_in` the unit vector from the scatterer towards where the incoming
ray came from and `u_out` towards where the outgoing ray goes:

* Isotropic: `F = 1`.
* Lambertian (diffuse): `F = sqrt(max(n·u_in, 0) · max(n·u_out, 0))`.
* Trihedral: `F = cos(a)**q` with `a` the angle between the bisector of
  `u_in`, `u_out` (projected on `xy`) and the boresight, `q` from the
  beamwidth as for the antenna, 0 beyond 90°.
* Cylinder glint: the point on the circle in the direction of the bisector
  of `u_in`, `u_out` from the axis; `F = 1`.

### 3.4 Implementation

Legs are computed once per scatterer and mirror option (direct, ground,
each mirror facet) for tx and for rx in one numba kernel parallel over
scatterers; scatterer paths are then all valid (tx leg, rx leg) pairs. The
cost is `scatterers × (2 + mirrors) × facets` segment tests per antenna.

## 4. Board

### 4.1 Model

`Hardware` (dataclass) holds the true values of everything the calibration
procedure measures, plus the analogue chain:

| Field | Meaning | Default |
|-------|---------|---------|
| `fs` | true ADC rate relative to the sweep clock | 21 977 Hz |
| `t_start` | time from frame start to the first commanded turnaround | 2.3 ms |
| `t_reset` | time from frame start at which the sweep is restarted (≤ `t_start`); before it the synthesiser sits at `f_prev` | 0.4 ms |
| `f_prev` | frequency before the restart | `f1` |
| `first_up` | the ramp starting at the first turnaround sweeps up | True |
| `pll_fn`, `pll_zeta` | closed-loop natural frequency and damping of the synthesiser (type 2) | 4 kHz, 0.7 |
| `hp_fc`, `hp_order` | IF high-pass corner and order (Butterworth) | 40 Hz, 1 |
| `lp_fc`, `lp_order` | IF low-pass corner and order (Butterworth) | 9 kHz, 4 |
| `r_cal` | fixed extra delay as range (`delay = 2 r_cal / c` added to every path) | 0.35 m |
| `tx_offset`, `rx_offset` | true antenna phase centres from the sled reference | (−0.06, 0.02, 0), (0.06, 0.02, 0) |
| `antenna` | antenna model (§3) | `g0` 10 (10 dBi), beamwidth 60° |
| `leak_amp`, `leak_range` | direct tx→rx coupling: field amplitude and equivalent range | 1e-3 √W, 0.1 m |
| `pt` | transmit power | 10 mW |
| `gain` | IF volts per √W of received field amplitude | 2.0e4 |
| `noise` | additive white noise at the ADC, V rms | 2e-4 |
| `dc` | ADC offset, V | 0.0 |
| `oversample` | analogue simulation rate / `fs` | 8 |

### 4.2 Synthesis

For a frame of `n` samples at `t_k = k / fs`:

1. Simulation grid `t_m = m / (oversample · fs)` from `t_reset − 4 ms` to the
   last sample.
2. Commanded frequency `f_cmd(t)`: `f_prev` before `t_reset`, the start
   frequency (`f0` if `first_up` else `f1`) until `t_start`, then the
   triangle with turnarounds every `ramp_time`. CW: constant `f0`.
3. Synthesiser output `f_tx = H_pll * f_cmd` with
   `H_pll(s) = (2ζω s + ω²) / (s² + 2ζω s + ω²)`, discretised with the
   first-order hold (exact for the piecewise-linear `f_cmd`) and started in
   steady state at `f_prev`.
4. IF before filtering, summed over paths (plus the leakage path) with
   delay `tau_p + 2 r_cal / c`:

   ```
   s(t_m) = Re sum_p A_p exp( j 2π f_tx(t_m) (tau_p - 2 v_p t_m / c) )
   ```
   Static paths are summed with a type-3 non-uniform FFT
   (`finufft.nufft1d3`, sources `tau_p`, targets `2π f_tx(t_m)`, tolerance
   1e-12); paths with `v_p ≠ 0` are summed directly.
5. Multiply by `sqrt(pt) · gain`, filter with the IF high-pass and low-pass
   (analogue Butterworth prototypes, bilinear transform with pre-warping at
   the simulation rate, started in steady state for the first input value).
6. Take the samples at `t_k` (the grid contains them exactly), add
   Gaussian noise and `dc`, quantise: `code = clip(round((v + 2.5) * 65535 / 5), 0, 65535)`.

`Hardware` also exposes the derived truths the calibration tests compare
against: `nr = ramp_time * fs`, the commanded turnaround positions
`t_start * fs + k * nr`, and the IF filter group delay at a given frequency.

## 5. Devices

* `SimSled(sigma=0.0, bias=0.0, seed)`: `move_to(x)` sets the true
  position `x + bias + N(0, sigma)`; `position()` reports the commanded
  value.
* `SimRadar(scene_geometry, hardware, sweep, sled, scan_geometry, seed)`:
  `capture(n)` computes paths at the sled's true position and synthesises
  one frame. Path computation is cached per true position.

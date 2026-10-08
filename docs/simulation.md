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
is the direction it faces (default −90, facing the rail); its `beamwidth`
defaults to 40. A box's `size` is its length along `angle` and its width;
a cylinder may set `n_facets`.

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
`{"eps_r": [re, im], "sigma0": s, "smooth": true}` (`"eps_r": null` for a
perfect conductor). Smooth surfaces act as
mirrors (§3.2); every surface also scatters diffusely with `sigma0`.

The antennas' polarisation is `"polarization": "h"` or `"v"` in the scene
(default `"h"`). Polarisation enters only through the basis vector of a ray
along unit direction `d`: `h`: `e(d) = normalize(z × d)`; `v`:
`e(d) = normalize(z − (z·d) d)`. A ray's scalar amplitude `a` stands for
the field vector `a · e(d)`.

`fresnel(eps_r, cos_incidence, pol)` gives the Fresnel coefficients
relative to the mirror image of the incident field (`pol` `"h"` for TE,
`"v"` for TM; −1 for a perfect conductor at every angle):

```
root = sqrt(eps_r - sin(theta)**2)
Γs (TE): (cos(theta) - root) / (cos(theta) + root)
Γp (TM): (root - eps_r cos(theta)) / (root + eps_r cos(theta))
```

A bounce at a mirror with unit normal `n` reflects the field vector
exactly: with `s = normalize(d × n)` (any unit vector ⟂ `d` at normal
incidence) and `p = s × d`, the reflected direction is
`d' = d − 2 (d·n) n` and the reflected field
`E' = M [Γs (E·s) s + Γp (E·p) p]`, `M v = v − 2 (v·n) n`. The field is
carried as a vector through every bounce of a leg or specular path, starting
from `e(d)` of the first segment, and projected on `e` of the last segment;
that projection is the path's reflection coefficient `Γ`. Consequences:
grazing reflection from a dielectric gives −1 for both polarisations
(direct and reflected rays cancel); for `h` the ground gives `Γs`, a wall
in a horizontal plane of incidence `−Γp` (zero at Brewster,
`tan(theta) = √eps_r`), a perfect-conductor plate at normal incidence `+1`
and an `h` dihedral with its seam along the field `−1`; for `v` the ground
gives `−Γp`.

A scatterer re-radiates as an induced dipole: its path is multiplied by
`−e(d_arrive) · e(d_leave)` of the rays arriving at and leaving it, so a
monostatic point and a perfect-conductor plate at normal incidence return
with the same sign in both polarisations.

### 2.2 Compilation

`compile_scene(scene, lam) -> Geometry` produces flat float64 / int64 arrays
suitable for numba:

* **Facets**: vertical rectangles, one per wall, four per box, `n_facets`
  per cylinder (default 16). Each has endpoints `p0, p1` (xy), `z0, z1`,
  unit normal (xy, to the right of `p0 → p1`, so outward for boxes and
  cylinders whose vertices run counter-clockwise), material permittivity
  (infinite for metal), `sigma0`, a `mirror` flag (smooth and not part of a
  cylinder) and an object id. All facets occlude; mirrors reflect on both
  sides.
* **Scatterers**: points with position, complex amplitude `sqrt(rcs)·e^{jφ}`,
  a pattern kind and pattern parameters, a host facet id (−1 for none) and
  a normal.
  * Diffuse points are drawn uniformly on every face with a Poisson count
    of mean `density * area`, density `max(4 / lam**2 * 0.05, 4)` per m²
    unless the scene sets `"diffuse_density"`, each with
    `rcs = sigma0 * area / count`, random phase, Lambertian pattern (§3.3),
    host facet set, normal the facet normal. Walls have two faces (normals
    ±), boxes and cylinders one per facet (outward).
  * Ground clutter points are drawn on `ground.extent` (`[x0, x1, y0, y1]`)
    with a Poisson count of mean `clutter_density * area` and
    `rcs = sigma0 * area / count`, then points inside box and cylinder
    footprints are removed; normal `+z`, Lambertian.
  * Reflectors: trihedral pattern centred on `boresight` with half-power
    `beamwidth`; phase 0.
  * Points: isotropic; phase 0.
  * Smooth cylinders: a specular glint whose position follows the aspect
    (§3.3), `rcs = 2*pi*radius*h**2 / lam * |Γ0|**2` with `h = z1 - z0` and
    `Γ0` the normal-incidence reflection coefficient, as a vertical line of
    in-phase points at spacing at most `lam / 4` whose coherent broadside
    sum is that cross-section (so ground lobing integrates over height), on
    the axis, phase 0, host facet the cylinder's first facet.
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
`Γ` the reflection coefficients of §2.1 for each leg or path (1 for no
bounce), `σ` the scatterer's cross-section and `F_s` its pattern for the
incoming and outgoing directions times the dipole polarisation factor of
§2.1. The specular class produces the glint of a flat
surface seen at normal incidence and the wall–ground dihedral.

Reflection points are found with the image method: reflect the far
endpoint in the mirror plane, intersect the straight line with the plane.
A reflection is valid only if the point lies inside the mirror (within the
facet's segment and height range, edges included to rounding tolerance;
ground points anywhere) and both endpoints are strictly on the same side of
the mirror. A double-bounce path through the line where two mirrors meet
(the monostatic wall–ground dihedral, when tx and rx are at equal height)
is valid in both bounce orders; it is kept once, for the order whose second
mirror has the lower index (the ground first, then mirror facets in facet
order). Paths whose amplitude is exactly zero (pattern nulls) are dropped.

### 3.2 Occlusion

Every straight segment of every path is tested against every facet: it is
blocked if it crosses the facet rectangle strictly between its endpoints.
The facet that hosts an endpoint (the mirror being reflected from, or the
host facet of the scatterer) is excluded from that segment's test; for a
cylinder glint, whose point lies on the circle outside the facet polygon,
all facets of its cylinder are excluded. The
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
scatterers; scatterer paths are then all valid (tx leg, rx leg) pairs with
non-zero pattern. The kernel runs twice: a count pass tests every leg with
occlusion and records its validity, then, after a prefix sum of the
per-scatterer counts, a fill pass recomputes only the valid legs' geometry
(no occlusion) and writes the pairs into exact-size arrays. A glint's point
depends on the leg pair, so its legs are tested per pair in both passes.
The cost is `scatterers × (2 + mirrors) × facets` segment tests per
antenna. Specular paths (at most `mirrors²`) are enumerated densely.
`line_of_sight(geom, src, points)` applies the same segment test to draw
shadows.

## 4. Board

### 4.1 Model

`Hardware` (dataclass) holds the true values of everything the calibration
procedure measures, plus the analogue chain:

| Field | Meaning | Default |
|-------|---------|---------|
| `fs` | true ADC rate relative to the sweep clock | 21 977 Hz |
| `t_start` | time from frame start to the first commanded turnaround | 2.3 ms |
| `t_reset` | time from frame start at which the sweep is restarted (≤ `t_start`); before it the synthesiser sits at `f_prev` | 0.4 ms |
| `f_prev` | frequency before the restart (`None`: `f1`) | `f1` |
| `first_up` | the ramp starting at the first turnaround sweeps up | True |
| `pll_fn`, `pll_zeta` | closed-loop natural frequency and damping of the synthesiser (type 2) | 4 kHz, 0.7 |
| `hp_fc`, `hp_order` | IF high-pass corner and order (Butterworth; order 0 removes it) | 40 Hz, 1 |
| `lp_fc`, `lp_order` | IF low-pass corner and order (Butterworth; order 0 removes it) | 9 kHz, 4 |
| `r_cal` | fixed extra delay as range at 25 °C (`delay = 2 r_cal / c` added to every path) | 0.35 m |
| `delay_tc`, `temperature` | extra delay drift as range per °C above 25 °C, and board temperature (`r_extra = r_cal + delay_tc (temperature − 25)`) | 0 m/°C, 25 °C |
| `tx_offset`, `rx_offset` | true antenna phase centres from the sled reference | (−0.06, 0.02, 0), (0.06, 0.02, 0) |
| `antenna` | antenna model (§3) | `g0` 10 (10 dBi), beamwidth 60° |
| `leak_amp`, `leak_range` | direct tx→rx coupling: field amplitude and equivalent range | 1e-3 √W, 0.1 m |
| `pt` | transmit power | 10 mW |
| `gain` | IF volts per √W of received field amplitude | 2.0e3 |
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
   `H_pll(s) = (2ζω s + ω²) / (s² + 2ζω s + ω²)`, started in steady state at
   `f_prev`. `f_cmd` is `f_prev` plus steps and slope changes at its
   breakpoints, so `f_tx = f_cmd − e` with the tracking error `e` the exact
   sum, over breakpoints, of the closed-form step and ramp responses of
   `1 − H_pll` (poles `−ζω ± ω sqrt(ζ² − 1)`). This equals the first-order-hold
   discretisation on any grid containing the breakpoints, but is exact at
   arbitrary times (`f_tx(sweep, hw, t)`), including breakpoints off the
   simulation grid, and is evaluated as a deviation from `f_cmd` that decays
   to zero.
4. IF before filtering, summed over paths (plus the leakage path) with
   delay `tau_p + 2 r_cal / c`:

   ```
   s(t_m) = Re sum_p A_p exp( j 2π f_tx(t_m) (tau_p + 2 v_p t_m / c) )
   ```
   with `v_p` = `Paths.speed` (half the rate of change of path length,
   negative approaching) and `tau_p` the delay at frame start. The leakage
   path has delay `2 leak_range / c` and amplitude `leak_amp`.
   Static paths are summed with a type-3 non-uniform FFT
   (`finufft.nufft1d3`, sources `tau_p`, targets `2π f_tx(t_m)`, tolerance
   1e-12); paths with `v_p ≠ 0` are summed directly.
5. Multiply by `sqrt(pt) · gain`, filter with the IF high-pass and low-pass
   (analogue Butterworth prototypes, bilinear transform at the simulation
   rate with each filter pre-warped at its own corner, as one SOS cascade,
   started in steady state for the first input value).
   `synthesize_volts` returns the result at `t_k` (noiseless, unquantised).
6. Take the samples at `t_k` (the grid contains them exactly), add
   Gaussian noise and `dc`, quantise: `code = clip(round((v + 2.5) * 65535 / 5), 0, 65535)`.

`Hardware` also exposes the derived truths the calibration tests compare
against: `nr(sweep) = ramp_time * fs`, the commanded turnaround positions
`turnarounds(sweep, n) = t_start * fs + k * nr` inside the frame, and
`if_group_delay(f)`, the exact group delay of the simulated (pre-warped
bilinear) IF cascade. `calibration(sweep, f_ref=fs/4)` returns the ideal
`Calibration`: `n0 = (t_start + if_group_delay(f_ref)) * fs`, i.e. the
commanded first turnaround delayed by the IF group delay at the centre of
the beat band (the type-2 PLL adds no delay: its ramps meet at the commanded
corner); `r_cal`, `first_up` (None for CW) and the antenna offsets are the
true values.

## 5. Devices

* `SimSled(sigma=0.0, bias=0.0, seed)`: `move_to(x)` sets the true
  position `x + bias + N(0, sigma)`; `position()` reports the commanded
  value.
  The true position is the attribute `true_position`.
* `SimRadar(scene_geometry, hardware, sweep, sled, scan_geometry, seed)`:
  `capture(n)` computes paths with `propagation.paths` for phase centres at
  the sled's true position (its reported position for sleds without
  `true_position`) plus `hardware.tx_offset`/`rx_offset`, at `sweep.lam`
  with `hardware.antenna`, and synthesises one frame with the radar's
  random generator. Path computation is cached per true position.

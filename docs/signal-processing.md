# QM-RDK signal processing specification

Defines how captured ADC frames are turned into range, Doppler and
time-history products. It replaces the processing of the vendor Windows GUI
and MATLAB scripts. Acquisition is defined in [protocol.md](protocol.md).

## 1. Symbols

| Symbol | Meaning | Value / unit |
|--------|---------|--------------|
| `c` | speed of light | 299 792 458 m/s |
| `fs` | ADC sample rate | 20 000 Hz |
| `N` | samples per capture | 1 – 4096 |
| `f0`, `f1` | sweep start / stop frequency (read back from the device) | Hz |
| `B` | swept bandwidth `f1 - f0` | Hz, ≤ 100e6 |
| `fc` | centre frequency `(f0 + f1) / 2`; in CW type `fc = f0` | Hz |
| `lam` | wavelength `c / fc` | m |
| `T` | one-way ramp time | s |
| `Nr` | samples per ramp `T * fs` | integer when `T` is a whole number of ms |
| `mu` | sweep slope `B / T` | Hz/s |
| `R` | target range | m |
| `v` | target radial speed, positive approaching | m/s |

## 2. Input

A capture is `N` unsigned 16-bit codes plus the read-back sweep parameters
and a host timestamp. Conversion to volts:

```
x[n] = code[n] * 5 / 65535 - 2.5        n = 0 .. N-1
```

Full scale is a sine of amplitude `A_fs = 2.5 V`. All level outputs are in
dBFS relative to `A_fs`. (The vendor plots label the same quantity "dBm";
the board is not power-calibrated and no dBm output is defined.)

All processing uses float64 NumPy arrays; batched products operate on
`[captures, samples]` arrays without Python-level loops.

## 3. Signal model

The receiver mixes the echo with the transmitted signal (homodyne, single
real channel). For a scatterer with round-trip delay `tau = 2R / c` the
intermediate-frequency output is

```
s(t) = a * cos(2*pi * f_tx(t) * tau + theta)
```

where `f_tx(t)` is the instantaneous transmit frequency. Consequences that
the rest of this document relies on:

1. **Static scene: `s` depends on time only through `f_tx(t)`.** A triangle
   sweep therefore produces an IF signal that is periodic with period `2T`
   and mirror-symmetric about every turnaround.
2. **Within a ramp** `f_tx = f0 + mu*t`, so a static scatterer produces a
   tone at the beat frequency

   ```
   f_b = mu * tau = 2 * R * B / (c * T)        R = c * T * f_b / (2 * B)
   ```
3. **A moving scatterer** adds the Doppler shift `f_d = 2 * v / lam`:

   ```
   up ramp:    f_up = | f_b - f_d |
   down ramp:  f_dn = | f_b + f_d |
   CW:         f    = | f_d |
   ```
4. The channel is real, so the sign of a frequency is not observable: CW
   mode cannot tell approaching from receding.

(The vendor manual's Doppler formula omits the two-way factor of 2; the
relations above are the ones used.)

Derived limits:

| Quantity | Expression | With `B` = 100 MHz, `T` = 16 ms, `N` = 4096 |
|----------|------------|------|
| Range resolution | `c / (2B)` | 1.5 m |
| Unambiguous range (Nyquist) | `(Nr / 2) * c / (2B)` | 240 m |
| Beat frequency per metre | `2B / (cT)` | 41.7 Hz/m |
| CW speed resolution | `(fs / N) * lam / 2` | 0.31 m/s |
| CW maximum speed | `(fs / 2) * lam / 2` | 625 m/s |
| Ramps per capture | `N / Nr` | 12.8 |
| Range-Doppler unambiguous speed (§8) | `± lam / (8T)` | ± 0.96 m/s |

## 4. Sweep segmentation

Range processing operates on single ramps. The device provides no sync
channel, so the position of the ramps inside a capture must be established.

### 4.1 Segmentation source

In order of preference:

1. **Firmware timing**, if bring-up shows that acquisition starts at a fixed
   offset from the sweep (protocol open question 4). The offset is then a
   constant and §4.2 is used only as a self-check.
2. **Mirror-symmetry estimator** (§4.2) for the free-running triangle type
   (AUTO), which is the type the vendor GUI uses by default.

Single-shot types (RAMP, TRI) are processed only with source 1.

### 4.2 Mirror-symmetry estimator (triangle sweeps)

By §3 item 1, the capture is mirror-symmetric about each turnaround, and
turnarounds repeat every `Nr` samples. Requires `N >= 2 * Nr`.

1. Remove the mean of `x`.
2. Autoconvolution, via one real FFT of length `L >= 2N - 1`:

   ```
   a[m] = sum_n x[n] * x[m - n]        m = 0 .. 2N-2
        = irfft( rfft(x, L)^2 )
   ```
   `a[m]` correlates the capture with its own reflection about the
   half-sample position `m / 2`.
3. Normalise by the energy of the overlapping samples (prefix sum of `x^2`):

   ```
   lo[m] = max(0, m - N + 1),  hi[m] = min(m, N - 1)
   rho[m] = a[m] / sum_{n = lo..hi} x[n]^2        in [-1, 1]
   ```
4. Keep only lags whose overlap `hi - lo + 1 >= Nr`, and fold them onto one
   turnaround period:

   ```
   J[p] = mean_j rho[p + 2*Nr*j]        p = 0 .. 2*Nr - 1
   ```
5. `p_hat = argmax J`, refined to sub-sample precision by a three-point
   parabolic fit through `J[p_hat - 1 .. p_hat + 1]` (indices modulo `2*Nr`).
   Turnarounds lie at `n_k = p_hat / 2 + k * Nr`.
6. Output: turnaround positions and the quality figure `J[p_hat]` (1 for a
   noiseless static scene). The figure is stored with every product.

Properties and limits:

* The estimate is biased by the group delay of the IF filter; the bias is
  common to all ramps and is absorbed by the guard interval (§4.3).
* Which turnaround is the top and which the bottom of the triangle is not
  observable from a static scene. Ramps are therefore labelled as two
  alternating sets, `E` (even) and `O` (odd), not as up/down, unless source 1
  is available. Range magnitude products do not depend on the labelling.
* Validity rests on static clutter dominating the capture, which is the
  normal case; a capture dominated by movers lowers `J[p_hat]`.

### 4.3 Ramp extraction

For each pair of adjacent turnarounds, extract the samples between them,
discarding a guard of `Ng` samples at each end to remove the turnaround
transient of the PLL and IF filter. `Ng` is derived from the measured IF
filter settling time (implementation plan, phase 0); it is a property of the
board, not a per-scene setting. Usable ramp length `Nu = Nr - 2*Ng`.

Fractional turnaround positions are applied as a fractional delay
(linear phase in the frequency domain) so that all ramps of all captures of a
recording are sampled on the same sweep-frequency grid.

Ramps of one set are time-reversed so both sets share the same
frequency-versus-index direction. Output: `ramps[set, ramp, Nu]`.

## 5. Range profile

Per ramp `r[n]`, `n = 0 .. Nu-1`:

1. Subtract the ramp mean.
2. Multiply by window `w[n]` (default Hann; any `scipy.signal.windows`
   window selectable).
3. Real FFT of length `Nfft = Nu * z` (zero-padding factor `z`, default 4,
   rounded up to a fast length).
4. Level:

   ```
   P[k] = 20 * log10( 2 * |X[k]| / (sum(w) * A_fs) )        dBFS
   ```
5. Range axis:

   ```
   R[k] = k * (fs / Nfft) * c * T / (2 * B) - R_cal        k = 0 .. Nfft/2
   ```
   `R_cal` is the fixed delay of cables, antennas and IF filter expressed as
   range, obtained once from a target at a surveyed distance and stored in
   the user configuration (default 0).

Combination within a capture:

* **Coherent** (default): mean of the complex spectra of the ramps of one
  set. Gains `10*log10(M)` dB in signal-to-noise for static targets.
* **Noncoherent**: mean of `|X|^2` over all ramps of both sets.

Optional range compensation multiplies `|X[k]|` by `R[k]^2`, the two-way
spreading of a point target.

### 5.1 Vendor-equivalent spectrum

For comparison with data and plots from the vendor software, the unsegmented
product is also provided: FFT of the whole capture, rectangular window,
`Nfft = 7N`, level `20*log10(|X| / Nfft)`, same range axis expression with
`R_cal = 0`. It mixes both ramp directions and the turnaround transients and
is not used for any other product.

## 6. Clutter suppression

| Method | Definition | Requirement |
|--------|------------|-------------|
| Two-pulse canceller | `d_i = ramp_i - ramp_(i-1)` between consecutive ramps of the same set, before §5 | same capture: exact; across captures: needs §4.3 alignment |
| Background subtraction | subtract the mean ramp of a reference recording of the empty scene | reference recorded with identical sweep parameters |
| Mean removal | subtract the per-recording mean ramp (over slow time) | at least 2 ramps |

The canceller between same-set ramps of one capture has pulse interval `2T`
and amplitude response `2 * |sin(2*pi * f_d * T)|` for a target with Doppler
`f_d`: nulls at `v = k * lam / (4T)`.

Cancellation across captures is limited by residual alignment error; a
residual of `delta` seconds leaves `2 * |sin(pi * f_b * delta)|` of a
static return at beat frequency `f_b`. Where the residual is unacceptable,
the noncoherent variant subtracts ramp spectrum magnitudes instead.

## 7. Doppler (CW type)

1. Subtract the capture mean.
2. Window (default Hann), real FFT of length `N * z`.
3. Level as §5 step 4.
4. Speed axis `v[k] = k * (fs / Nfft) * lam / 2`, unsigned.

Doppler versus time inside a capture uses `scipy.signal.ShortTimeFFT` with
segment length `Ns` and hop `Ns / 2`: speed resolution
`(fs / Ns) * lam / 2`, time resolution `Ns / fs`.

## 8. Range–Doppler map (triangle type)

Uses the `M` ramps of one set in a capture; needs `M >= 2`.

1. Fast time: complex range spectra per ramp (§5 steps 1–3), giving
   `X[m, k]`.
2. Slow time: window across `m` and FFT of length `M * z`, shifted so zero
   Doppler is centred.
3. Axes: range as §5; speed
   `v[j] = j * lam / (2 * 2T * Mfft)` for `j = -Mfft/2 .. Mfft/2 - 1`.

Unambiguous speed `± lam / (8T)`, speed resolution `lam / (4 * T * M)`,
range extent `(Nu / 2) * c / (2B)`. Short ramps trade range extent for
speed span at constant range resolution. The speed sign is defined only when
ramp direction is known (§4.2).

Single-target range and speed from one triangle (requires known ramp
direction): with peak beat frequencies `f_up`, `f_dn` of the same target,
`f_b = (f_up + f_dn) / 2` and `f_d = (f_dn - f_up) / 2`.

## 9. Time histories

A recording is a sequence of captures with host timestamps; captures are
not contiguous (protocol §3.4), so the time axis is the timestamp vector,
not a uniform grid.

| Product | Rows | Columns |
|---------|------|---------|
| Raw versus time | capture | sample |
| Range–time intensity | capture | range profile (§5), optionally after §6 |
| Doppler–time intensity | capture, or STFT segment within capture | Doppler spectrum (§7) |

Display scaling: levels relative to the recording maximum with a selectable
dynamic range, colour map linear in dB.

## 10. Detection

Optional peak extraction on a range profile or Doppler spectrum in the
power domain:

1. Cell-averaging CFAR with `Nc` reference cells split either side of the
   cell under test and `Ngc` guard cells. For a design false-alarm
   probability `Pfa` the threshold multiplier is

   ```
   alpha = Nc * (Pfa^(-1/Nc) - 1)
   ```
   `Ngc` equals the main-lobe half-width of the chosen window in bins
   (window bandwidth × `z`).
2. Local maxima above threshold are refined by a three-point parabolic fit
   in dB and reported as (range or speed, level).

## 11. Synthetic aperture imaging

The radar is moved along a straight rail (aperture length `L`, 1.5 m for the
kit sled) and a two-dimensional image of the scene is formed from captures
taken at known positions. Rail axis `x`, ground range `y`, antenna height
ignored unless configured.

### 11.1 Acquisition

* Stop-and-go: move, settle, capture one frame, repeat. One frame per
  position is sufficient; no streaming is required.
* Triangle sweep (AUTO). All positions use identical sweep parameters.
* Position spacing `dx <= lam_min / (4 * sin(theta_max))`, where
  `lam_min = c / f1` and `theta_max` is the largest off-boresight angle from
  which significant energy is received; `theta_max = 90 deg` gives
  `dx <= lam_min / 4` (30 mm), which is always alias-free.
* The scene must not change during the scan. Position error must be small
  against `lam / 8` (15 mm): millimetre repeatability is required of the
  sled.
* A recording stores the commanded and, if the sled reports it, the measured
  position of every capture (`x_pos`).

### 11.2 Per-position phase history

1. Segment and align ramps (§4), with fractional alignment, so every ramp of
   every position is sampled on the same sweep-frequency grid starting at
   `f_a = f0 + mu * Ng / fs`.
2. Coherent mean of all ramps of the capture (both sets after reversal).
3. Optional background subtraction: subtract the mean over positions (removes
   antenna leakage and returns that do not vary along the rail), or a
   reference scan of the empty scene.
4. Window, zero-pad, FFT, keep the positive-frequency half: complex range
   profile `p_n[k]` for position `n`, with range axis as §5.

For a point scatterer at range `R` the profile peaks at `R` with phase
`4 * pi * f_a * R / c`. This holds only if the ramps are indexed in the
direction of increasing frequency; indexing them the other way conjugates
the phase and references it to the other band edge. Ramp direction comes
from firmware timing when available (§4.1). Otherwise images are formed
under both hypotheses and the one with the higher sharpness
`sum |I|^4 / (sum |I|^2)^2` is kept; the choice is binary and is recorded
with the image.

### 11.3 Image formation: backprojection

For every pixel `(x, y)` on a user-defined grid:

```
R_n(x, y) = ( |pixel - tx_n| + |pixel - rx_n| ) / 2 + R_cal
I(x, y)   = sum_n  p_n( R_n ) * exp( -j * 4 * pi * f_a * R_n / c )
```

`tx_n`, `rx_n` are the transmit and receive antenna phase centres at
position `n` (the two cantennas are side by side; their offsets from the
sled reference are configuration). `p_n(R_n)` is linearly interpolated from
the zero-padded profile. An optional aperture window weights the sum over
`n`.

Backprojection is chosen over the range-migration algorithm of the MIT
reference script because it is exact in the near field, accepts unequal
position spacing and bistatic antenna offsets directly, and its cost
(`positions × pixels`) is negligible at this aperture size. It is
implemented as one numba kernel parallel over pixels.

### 11.4 Performance

| Quantity | Expression | `L` = 1.5 m, `B` = 100 MHz |
|----------|------------|------|
| Down-range resolution | `c / (2B)` | 1.5 m |
| Cross-range resolution at range `R` | `lam * R / (2L)` | 0.041 × `R` (0.4 m at 10 m) |
| Angular resolution | `lam / (2L)` | 2.3° |
| Positions at `dx = lam_min / 4` | `L / dx + 1` | 51 |
| Phase error from ramp misalignment `delta` | `2 * pi * f_b * delta` | 0.03 rad at 48 m for 0.05 sample |

Pixel spacing defaults to half the resolution in each axis. Output: complex
image, grid axes, and level in dB relative to the image maximum.

## 12. Verification

A synthetic source generates captures from the model of §3: a list of
scatterers (position or range, speed, amplitude), radar position, triangle or CW `f_tx(t)` with arbitrary
sweep phase and optional additive Gaussian noise, quantised to 16-bit codes
exactly as the device returns them. The same source backs the simulated
device used by the protocol tests.

| Property tested | Expected result |
|-----------------|-----------------|
| Code-to-volt conversion | code 0 → -2.5 V, 65535 → +2.5 V |
| Level calibration | full-scale bin-centred sine → 0 dBFS for every supported window |
| Sweep segmentation | recovered turnaround within 0.5 sample of the generated sweep phase for every phase offset in `[0, 2*Nr)`; `J = 1` for noiseless static scenes |
| Range axis | single static scatterer at `R` → interpolated peak within one padded bin of `R` |
| Resolution | two equal scatterers separated by `2 * c / (2B)` resolved, separated by `0.5 * c / (2B)` not resolved (rectangular window) |
| Two-pulse canceller (same capture) | static scatterer suppressed to the quantisation floor; mover at `v = lam / (8T)` passes with gain 2 |
| CW Doppler | scatterer at speed `v` → peak at `2v / lam` |
| Range–Doppler | scatterer at (`R`, `v`), `|v| < lam / (8T)` → peak at the corresponding cell |
| CFAR | on noise-only input the measured false-alarm rate matches `Pfa` within the binomial confidence interval of the trial count |
| Vendor-equivalent spectrum | matches a direct evaluation of the vendor formula on the same input |
| SAR point response | synthetic scatterer at `(x, y)` scanned over `L` → image peak within half a resolution cell; -3 dB widths equal `c / (2B)` and `lam * R / (2L)` times the broadening factor of the windows used |
| SAR ramp direction | sharpness test selects the generated direction for every sweep phase offset |
| SAR sampling | grating lobes absent at `dx = lam_min / 4`, present at the predicted angle for `dx = lam` |

Recordings from the real board (gitignored `artifacts/`) are used for the
hardware-dependent items: `Ng`, `R_cal`, confirmation of the segmentation
source, and a SAR scan of a single strong reflector at a surveyed position.

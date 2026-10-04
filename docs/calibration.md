# Calibration procedure

Determines the board and installation constants that the signal
processing ([signal-processing.md](signal-processing.md)) and SAR imaging
need and that no document supplies. The result is a `Calibration`
(`qmrdk/config.py`) stored as JSON and passed to every processing command
with `--cal`.

| Constant | Meaning | Step |
|----------|---------|------|
| `fs` | ADC rate relative to the sweep clock (`Nr = T * fs`) | 1 |
| `n0` | apparent position of the first turnaround in a frame | 1 |
| `ng` | turnaround guard | 2 |
| `first_up` | direction of the ramp starting at `n0` | 3 |
| `r_cal` | fixed delay as range | 3 |
| `tx_offset`, `rx_offset` | antenna phase centres from the sled reference | 3 |
| `sled_sigma` | sled position repeatability | 4 |

Every step runs against the simulated board with `--sim`, and the test
suite checks that each estimator recovers the simulator's true values
(`Hardware`, [simulation.md](simulation.md) §4).

`n0` and `ng` are properties of the board's timing and IF chain. They are
re-measured whenever the ramp time or sample count changes. `r_cal` and the
offsets are properties of the board, cables and antenna mount. They are
re-measured whenever any of these changes.

## 0. Preparation

* Let the board warm up until `SYST:TEMP?` is stable to 1 °C.
* Use the sweep settings that will be used for imaging (start and stop
  frequency, ramp time, AUTO type, 4096 samples).
* Mount the antennas on the sled and fix the rail. Mark the rail origin
  (`x = 0`, sled reference point) and survey all positions relative to it:
  `x` along the rail, `y` perpendicular towards the scene, `z` up from the
  ground. A tape measure good to 1 cm is sufficient for steps 1–3.
* Measure the horizontal separation of the two antenna apertures with a
  ruler. This is the tx–rx baseline used in step 3.

## 1. Sweep period and frame timing (`fs`, `n0`)

Setup: a static scene with a strong return, e.g. the corner reflector of
step 3 at 3–10 m. Nothing in view should move. Use the shortest ramp time
the sweep settings allow for this step only: precision improves with the
number of turnarounds per frame.

Acquisition: `qmrdk calib timing --frames K` takes `K` frames (default 32)
at a fixed sled position.

Estimation (`qmrdk.calib.estimate_timing`):

1. The frames are sweep-synchronous (protocol §3.4), so their mean `x̄`
   keeps the signal and reduces the noise by `sqrt(K)`. The frame-to-frame
   spread gives the per-sample noise variance `σ²`.
2. Exclude the restart transient: samples before the first turnaround
   found by the mirror-symmetry estimator (§4.2) on the nominal `Nr`.
3. Each interior turnaround is a local maximum of the normalised mirror
   correlation `rho[m]` of `x̄` (§4.2 steps 1–3) at `m = 2 n_k`. Refine each
   maximum to sub-sample precision with a three-point parabolic fit.
4. Fit `n_k = n0 + k * Nr` by least squares. The slope gives `Nr` and
   `fs = Nr / T`, the intercept gives `n0`. The residual RMS and the
   standard errors of both parameters are reported. A residual above 0.1
   sample means the ADC and sweep clocks are not locked over a frame, and
   the procedure stops.
5. Stability: repeat steps 3–4 on each frame on its own. The spread of the
   per-frame `n0` is the frame-start jitter, and the spread of `Nr` over
   repeated runs is the clock-ratio drift (protocol open question 10).

`n0` for the imaging ramp time is measured with the same estimator on
frames taken at that ramp time, holding `fs` fixed.

## 2. Turnaround guard (`ng`)

Same data as step 1, at the imaging ramp time.

In a static scene, the IF after a turnaround should be the mirror image of
the IF before it (signal-processing §3, item 1). Two effects break this:

* The **turnaround transient** (synthesiser overshoot and IF filter memory)
  is confined to the samples just after the turnaround and decays. This is
  what the guard removes.
* **Dispersion**: the IF filter's phase is not linear in frequency, so the
  steady-state tones of the up and down ramps acquire opposite phase
  errors. This is an LTI effect that does not decay.

Estimation (`qmrdk.calib.estimate_guard`):

1. For each interior turnaround `n_k`, resample `x̄` at `a_k[d] = x̄(n_k - d)`
   (before) and `b_k[d] = x̄(n_k + d)` (after), for `d = 0 .. D-1` with
   `D = floor(Nr / 2)`, using the fractional interpolator of §4.3.
2. Dispersion equaliser: fit by least squares a short two-sided FIR `g`
   (length `2L + 1`, `L = 8`) such that `b_k ≈ g * a_k` over the settled
   half `d ∈ [D/2, D)`, jointly over all turnarounds of one kind. Fit
   separately for even and odd turnarounds (top and bottom).
3. Transient: `e_k[d] = b_k[d] - (g * a_k)[d]`, mean over turnarounds of a
   kind: `e[d]`.
4. Criterion: the guard must push the transient below the window's own
   sidelobes. For a candidate guard `G`, form the range spectrum (§5, the
   imaging window) of the ramp `b[d]` for `d ≥ G` and of the transient
   `e[d]` for `d ≥ G` on the same window. `ng` is the smallest `G` for
   which the largest transient spectral level, relative to the ramp
   spectrum's peak, is below the window's peak sidelobe level. The PSL is
   computed from the window itself (−31.5 dB for Hann).
5. The noise must not mask the criterion. The noise contribution to
   `e[d]`, `σ² / K` per sample, has to sit below the same level. If it does
   not, more frames are required and the procedure reports how many.

## 3. Reflector scan (`first_up`, `r_cal`, antenna offsets)

Setup: one trihedral corner reflector (RCS ≥ 1 m² at 2.45 GHz, e.g. a
30 cm triangular trihedral) at about 5–10 m and 20–30° off boresight. Put
it at antenna height, facing the rail, with nothing else within 2 m of it
or of the line of sight. Survey its phase centre (the inner corner) to
1 cm. Being off boresight makes the range history slope along the rail,
which is what makes the along-rail offset and the sled errors observable.

Acquisition: `qmrdk calib reflector --target X Y Z` runs a full SAR scan
(`dx ≤ lam_min / 4`) across the aperture and stores it with the surveyed
position.

Estimation (`qmrdk.calib.estimate_reflector`):

1. Phase history with the guard from step 2, for both ramp-direction
   hypotheses, without background subtraction.
2. For each position, find the profile peak nearest the predicted range
   and interpolate it (parabolic in dB) to get the magnitude range `R̂_n`.
   Read the complex profile at that range to get the phase `φ_n`.
3. **`r_cal`**: the median over positions of `R̂_n` minus the geometric
   range `(|P - tx_n| + |P - rx_n|) / 2`. The nominal offsets are used in
   the first pass and the fit of step 5 in the second.
4. **`first_up`**: for each hypothesis, unwrap `φ_n` along the rail and
   compare it with the model `4 π f_m R_n / c`. Under the right hypothesis
   the residual is a constant plus small errors. Under the wrong one, the
   conjugated phase leaves twice the range variation. Keep the hypothesis
   with the smaller residual RMS after removing the constant.
5. **Offsets**: write `R_n(o)` for the bistatic range with the midpoint
   phase-centre offset `o = (ox, oy, oz)` and the measured baseline. Fit
   `o` and the phase constant by nonlinear least squares
   (`scipy.optimize.least_squares`) to the unwrapped phase:
   `φ_n = 4 π f_m R_n(o) / c + φ_c`. Only the midpoint is observable to
   first order. `tx_offset` and `rx_offset` are the midpoint ∓ half the
   baseline along `x`. `oz` is weakly observable with the reflector at
   antenna height. It is held at its mechanical value unless the reflector
   is placed at a different height.
6. Report the parameter standard errors (from the Jacobian and the
   residual variance) and the phase residual RMS converted to range,
   `σ_R = c σ_φ / (4 π f_m)`.
7. Check: form an image of the scan with the new calibration. The peak
   must lie within half a resolution cell of the surveyed position, and
   its −3 dB widths must match signal-processing §11.4.

The fitted offsets are in the frame defined by the surveyed reflector
position, so a survey error moves the image as a whole by the same amount.
This is why the survey uses the same rail origin as imaging.

## 4. Sled repeatability (`sled_sigma`)

Repeat the reflector scan `J` times (default 5) without moving the
reflector: `qmrdk calib repeat --scans J`. For each pair of scans, the
phase difference at each position, divided by the local slope of the range
history `dR/dx = ∂R_n/∂x`, is a position difference:

```
Δx_n = c Δφ_n / (4 π f_m) / (dR/dx)_n
```

Positions with `|dR/dx|` below half its maximum are excluded.
`sled_sigma` is the standard deviation of `Δx_n` over all positions and
pairs, divided by `sqrt(2)`. SAR focusing needs `sled_sigma` well below
`lam / 8`; the report states the predicted loss of peak gain,
`exp(-(4 π sled_sigma / lam)²)`.

## 5. Optional checks

* **Ambient**: with the transmit antenna replaced by a 50 Ω termination,
  record frames and confirm that the range profile is at the noise floor
  with no lines. Wi-Fi and Bluetooth in the band show up as non-static
  lines.
* **Background reference**: a scan of the empty scene with the imaging
  settings, used for background subtraction (signal-processing §11.2).

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
(`Hardware`, [simulation.md](simulation.md) §4) within its reported
standard errors.

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

Setup: the corner reflector of step 3 in place, nothing in view moving.
Steps 1 and 2 use the imaging sweep settings: with `K = 32` frames the
standard errors are about 0.01 Hz on `fs` and 1e-3 sample on `n0`, far below
what segmentation needs, so no separate short-ramp acquisition is made.

Acquisition: `qmrdk calib timing --frames K` takes `K` frames at the sled
home position and `K` more one step `dx` further on, with `dx` chosen so the
reflector's range changes by `lam / 4` (`calib.pair_step`). The estimators
work on the difference of the two sets. Antenna leakage, the specular ground
return and the frame-start restart transient do not change with the sled
position and cancel. They would otherwise dominate the low-range spectrum
and, through the IF high-pass, the mirror symmetry. The reflector's return
remains, mirror-symmetric about every turnaround. A single frame set
`[K, N]` is also accepted.

Estimation (`qmrdk.calib.estimate_timing`):

1. Mean frame `x̄` over the `K` differences. Their spread gives the
   per-sample noise variance `σ²` of one capture.
2. Coarse turnaround: the mirror-symmetry estimator (signal-processing §4.2)
   on the nominal `Nr`, applied to `x̄` after a zero-phase high-pass
   (forward-backward Butterworth, order 4) at the window's main-lobe
   frequency `fs / Nu`. A zero-phase filter keeps every mirror point.
3. Ramps centred between the turnarounds `n0 + j Nr` (fractional
   interpolator, §4.3) from the second turnaround on. The first ramp carries
   the restart transient. They are taken forward in time, with a guard of
   the interpolator half-width.
4. Range spectrum of each ramp (§5, imaging window). The strongest line
   beyond the main lobe of zero range is the reflector, at `ω` (parabolic
   in dB). With the phase referenced to the ramp centre, the up and down
   ramps either side of turnaround `k` carry phases `±Φ + arg H + ω ε`,
   where `ε` is the error of the assumed turnaround. Hence

   ```
   n_k = n_k(assumed) - arg(z_(k-1) z_k) / (2 ω)
   ```

   which is the commanded turnaround delayed by the IF *phase* delay at the
   reflector's beat frequency. That is the turnaround at which both ramp
   sets are coherent at that range (§11.2), and it is not the group delay.
   The phase fixes `n_k` modulo `π / ω`. Among the branches within one
   interpolator half-width, the one with the largest broadband mirror
   correlation of the high-passed frame is kept.
5. Fit `n_k = n0 + k Nr` by least squares and iterate steps 3–5 from the
   new `n0`, `Nr`. The slope gives `fs = Nr / T`. The residual RMS and the
   standard errors (residual variance, Student-t with `turnarounds - 2`
   degrees of freedom) are reported. A residual above 0.1 sample means the
   ADC and sweep clocks are not locked over a frame (`locked` false).
6. Stability: steps 4–5 on each frame with `Nr` fixed. The spread of the
   per-frame `n0` is the frame-start jitter.

For a close reflector (low beat frequency) the decaying turnaround response
of the IF high-pass leaks into the reflector's line. That shifts the phase
equally for both ramp sets, so the estimate is still the coherent
turnaround for that range, but it differs from the stationary-tone phase
delay. Beyond about 20 m in the simulator the two agree within the standard
error.

## 2. Turnaround guard (`ng`)

Same frame pairs as step 1.

In a static scene, the IF after a turnaround should be the mirror image of
the IF before it (signal-processing §3, item 1). Two effects break this:

* The **turnaround transient** (synthesiser overshoot and IF filter memory)
  is confined to the samples just after the turnaround and decays. This is
  what the guard removes.
* **Dispersion**: the IF filter's phase is not linear in frequency, so the
  steady-state tones of the up and down ramps acquire opposite phase
  errors. This is an LTI effect that does not decay.

Estimation (`qmrdk.calib.estimate_guard`):

1. High-pass `x̄` as in step 1. For each interior turnaround `n_k` (the
   first is skipped), resample `a_k[d] = x̄(n_k - d)` and
   `b_k[d] = x̄(n_k + d)` for `d = 0 .. D-1`, `D = floor(Nr / 2)` (§4.3).
2. Dispersion equaliser: `b_k ≈ (δ + g) * a_k` with a two-sided FIR
   correction `g` (`2L + 1` taps, `L = 8`) fitted by least squares over the
   settled half `d ∈ [D/2, D)`, jointly over the turnarounds of one kind
   (even and odd separately). The tones of a static scene are narrowband,
   so this fit is ill-posed. It is solved by truncated SVD that keeps only
   singular values above the window's PSL relative to the largest:
   directions carrying less than sidelobe-level energy are not equalised.
3. Transient: `e_k[d] = b_k[d] - ((δ + g) * a_k)[d]`, mean over turnarounds
   of a kind: `e[d]`.
4. Criterion: for a candidate guard `G`, form the range spectrum (§5, the
   imaging window) of the ramps extracted with guard `G` and of the
   transient `e[d]`, `d ≥ G`, placed at the start of a ramp of the same
   length. Bins inside the window main lobe of zero range are excluded:
   leakage occupies them and no guard changes that. The transient level is
   the largest transient bin relative to the largest ramp bin. `ng` is the
   smallest `G` for which, at `G` and every larger guard up to `D / 2`, the
   transient level plus the noise peak (step 5) is below the window's peak
   sidelobe level. The PSL is computed from the window (−31.5 dB for Hann).
   If no guard qualifies, `ng` is reported as `null` and the calibration
   stops.
5. Noise: `e[d]` carries noise of variance
   `σ² (1 + |δ + g|²) / (K · turnarounds)` per sample. Its spectral peak is
   that level times `sqrt(ln(bins / 1e-3))`, the level a Rayleigh
   magnitude exceeds in any bin with probability 1e-3. If it is not below
   the PSL, the report gives the number of frames needed.

Verified against the simulator by comparing extracted ramps with the same
samples of a frame whose sweep crosses the ramp's band far from any
turnaround (`tests/test_calib.py`). On the default board model the
reflector's transient is already below the Hann PSL at `G = 0`.

## 3. Reflector scan (`first_up`, `r_cal`, antenna offsets)

Setup: one trihedral corner reflector (RCS ≥ 1 m² at 2.45 GHz, e.g. a
30 cm triangular trihedral) at about 8 m from the aperture centre and 25°
off boresight (`calib.default_target`), at antenna height and facing the
rail, with nothing else within 2 m of it or of the line of sight. Survey
its phase centre (the inner corner) to 1 cm. Being off boresight makes the
range history slope along the rail, which makes the along-rail offset and
the sled errors observable. It also makes the cross-track offset `oy`
observable through the variation of the line-of-sight angle; near
broadside `oy` is only weakly determined and is traded against `r_cal`.

The ground is a mirror. A reflector at antenna height over it returns four
paths of almost equal length: direct, two single bounces and a double
bounce. Their excess lengths are far below the range resolution, and at
low grazing they nearly cancel, so the reflector sits close to a two-ray
null. Fitting a single path to this return is wrong by up to a resolution
cell. The estimator therefore models the ground image paths.

Acquisition: `qmrdk calib reflector --target X Y Z` runs a full SAR scan
(`dx ≤ lam_min / 4`) across the aperture and stores it with the surveyed
position.

Estimation (`qmrdk.calib.estimate_reflector`):

1. Phase history (§11.2) without background subtraction, without zero
   padding, for both ramp-direction hypotheses. Keep the bins within the
   window main lobe of the profile peak nearest the predicted range. Whiten
   them with the known covariance of windowed white noise between bins.
2. Model per position `n`: the four paths tx→P→rx with the transmit and/or
   receive leg mirrored in `z = 0`. Each path is a unit tone through the
   same window and ramp-mean removal as the data (closed form: shifted
   window transform). Its phase is `2π f τ` with
   `τ = (L + 2 r_cal) / c`, its amplitude is scaled by the spreading
   `L_t L_r / (L_t' L_r')`, and each bounced leg is scaled by
   `ρ0 + ρ1 (sin e - mean)`, with `e` the bounce-ray elevation (ground
   reflection and antenna elevation pattern to first order). A global
   phase, a real amplitude per position (azimuth patterns) and a complex
   static background per bin (leakage) are projected out (variable
   projection).
3. **`first_up`**: for each hypothesis, the best linear start (free complex
   coefficients of the direct, single- and double-bounce classes on an
   `r_cal` grid over one range cell). The wrong hypothesis conjugates the
   phase history and fits far worse. The ratio of the two costs is
   reported.
4. **`r_cal` and offsets**: nonlinear least squares
   (`scipy.optimize.least_squares`) over `r_cal`, the midpoint offset
   `(ox, oy)`, the phase and `ρ0`, `ρ1`. It is solved in
   `r_cal + ∇R · o`, the combination the envelope measures. `oz` is held at
   its mechanical value. `tx_offset` and `rx_offset` are the midpoint ∓
   half the measured baseline along `x`.
5. Covariance of `(r_cal, ox, oy)`: cluster-robust (sandwich) estimate with
   positions as clusters. Sled position errors and any per-position model
   error are correlated across a position's bins. The report also gives the
   phase-residual RMS as range, `σ_R = c σ_φ / (4π f_m)`, and `chi2`, the
   residual variance over the noise variance expected from step 1 (1 for an
   adequate model).
6. Check: image the scan with the new calibration and image the fitted
   model scan the same way. The data image peak must lie within half a
   resolution cell of the model image peak; the ground images displace both
   peaks equally from the surveyed position (`model_offset`). The −3 dB
   widths are compared with signal-processing §11.4, using the aperture
   projected on the line of sight.

Limits found in simulation:

* A sled repeatability of 0.5 mm makes the offsets uncertain at the
  centimetre level, because `oy` and `r_cal` rest on the curvature of the
  range history. The reported covariance reflects this. Averaging the
  repeated scans of step 4 would reduce it.
* The reflector's own turnaround response through the IF high-pass is not
  in the model. With a perfect sled it biases `r_cal` and the offsets by a
  few millimetres and shows as `chi2 > 1`.

The fitted offsets are in the frame defined by the surveyed reflector
position, so a survey error moves the image as a whole by the same amount.
This is why the survey uses the same rail origin as imaging.

## 4. Sled repeatability (`sled_sigma`)

Repeat the reflector scan `J` times (default 5) without moving the
reflector: `qmrdk calib repeat --scans J`.

Estimation (`qmrdk.calib.estimate_repeatability`):

1. Fit step 3 to the first scan, or reuse its result.
2. For each scan and position, project the whitened profile bins (less the
   fitted background) onto the model and its quadrature. This gives the
   phase `φ_jn` relative to the model.
3. The model's phase slope along the rail `(dΦ/dx)_n` is the derivative
   of the fitted model, ground images included. Positions with
   `|dΦ/dx|` below half its maximum are excluded.
4. `Δx_jn = (φ_jn - mean_j φ_jn) / (dΦ/dx)_n`. Its pooled variance about
   the per-position mean, less the noise variance, is `sled_sigma²`. The
   noise variance comes from the profile components orthogonal to the model
   and its quadrature, which a position error does not reach.
5. The report gives the standard error and the predicted peak-gain factor
   `exp(-(4 π sled_sigma / lam)²)`. SAR focusing needs `sled_sigma` well
   below `lam / 8`.

## Simulation

`qmrdk.calib.calibrate_sim(hw, sweep)` (`qmrdk calib sim`) runs steps 1–4
in order on `builtin_scene("single")` with the reflector moved to
`default_target`. It starts from the nominal constants and the true
baseline (the ruler measurement), feeds each step's estimates into the next,
and returns the `Calibration` with a JSON report. Every command also runs
against the simulated board with `--sim`.

## 5. Optional checks

* **Ambient**: with the transmit antenna replaced by a 50 Ω termination,
  record frames and confirm that the range profile is at the noise floor
  with no lines. Wi-Fi and Bluetooth in the band show up as non-static
  lines.
* **Background reference**: a scan of the empty scene with the imaging
  settings, used for background subtraction (signal-processing §11.2).

#!/usr/bin/env python3
"""Compare ramp times on the rail: per ramp time, a sled-made timing pair,
static frames for the per-bin noise floor and line SNR, and a scan imaged
with that timing. Results go to a report directory."""

import argparse
import dataclasses
import json
import pathlib

import numpy as np

from qmrdk.calib import estimate_timing, pair_phase
from qmrdk.config import Calibration, ScanGeometry, Sweep
from qmrdk.dsp.sar import form_image, phase_history
from qmrdk.radar import HardwareSled, UsbRadar
from qmrdk.render import grid, save_image
from qmrdk.scan import frames, run_scan, scan_positions


def noise_and_lines(codes, sweep, cal, ranges):
    """Per-frame noise power per bin (from frame differences, insensitive
    to drift) and the static line SNR near each of `ranges` (m)."""
    ph = phase_history(codes, sweep, cal, True, None)
    p, r = ph.profiles, ph.r_axis
    noise = np.var(np.diff(p, axis=0), axis=0) / 2.0
    snr = np.abs(p.mean(axis=0)) ** 2 / noise
    out = {"noise_db": {}, "snr_db": {}}
    for rr in ranges:
        k = int(np.argmin(np.abs(r - rr)))
        out["noise_db"][rr] = float(10 * np.log10(noise[k]))
        win = np.abs(r - rr) < 1.0
        out["snr_db"][rr] = float(10 * np.log10(snr[win].max()))
    return out


def timing_pair(radar, sled, n, count, steps):
    """Static sets at rail 0 and at the first step in `steps` whose line
    phase change lies in 90..180 deg; returns the timing estimate."""
    sled.move_to(0.0)
    first = frames(radar, n, count, "timing")
    for dx in steps:
        sled.move_to(dx)
        phi = pair_phase(first, frames(radar, n, 8, "check"), radar.sweep)
        if 90.0 <= abs(phi) <= 180.0:
            second = frames(radar, n, count, "timing +dx")
            return estimate_timing(np.stack([first, second]), radar.sweep)
    raise RuntimeError("no sled step gave a usable pair phase")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True, help="report directory")
    ap.add_argument("--ramps", type=float, nargs="+", default=[16.0, 8.0, 4.0])
    ap.add_argument("--sled", help="sled serial URL")
    ap.add_argument("--frames", type=int, default=32)
    ap.add_argument("--noise-frames", type=int, default=64)
    ap.add_argument("--length", type=float, default=0.85)
    ap.add_argument("--steps", type=float, nargs="+", default=[0.05, 0.1, 0.15, 0.2])
    ap.add_argument("--ranges", type=float, nargs="+", default=[5.0, 8.2, 12.0, 20.0])
    ap.add_argument("--extent", type=float, nargs=4, default=[-8.0, 8.0, 3.0, 20.0])
    ap.add_argument("--n", type=int, default=4096)
    args = ap.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    sled, geometry, report = HardwareSled(args.sled), ScanGeometry(), {}
    gx, gy = grid(args.extent, (0.05, 0.1))
    for ramp in args.ramps:
        tag = f"{ramp:g}ms"
        with UsbRadar(sweep=Sweep(ramp_time=ramp * 1e-3)) as radar:
            sweep = radar.sweep
            t = timing_pair(radar, sled, args.n, args.frames, args.steps)
            cal = t.apply(Calibration(first_up=None))
            sled.move_to(0.0)
            static = frames(radar, args.n, args.noise_frames, "noise")
            positions = scan_positions(sweep, args.length)
            rec = run_scan(radar, sled, positions, args.n, geometry, desc=tag)
        rec.save(out / f"scan_{tag}.npz")
        np.save(out / f"static_{tag}.npy", static)
        cal.save(out / f"cal_{tag}.json")
        img = form_image(rec.codes, rec.x_pos, sweep, cal, geometry, gx, gy)
        save_image(
            out / f"scan_{tag}.png",
            None,
            img,
            geometry,
            rec.x_pos,
            sweep.lam,
            25,
            extent=args.extent,
        )
        cal = dataclasses.replace(cal, first_up=img.first_up)
        report[tag] = {
            "timing": t.summary(),
            "first_up": img.first_up,
            "sharpness": img.sharpness,
            **noise_and_lines(static, sweep, cal, args.ranges),
        }
        print(json.dumps({tag: report[tag]}, indent=1, default=str), flush=True)
    sled.close()
    (out / "report.json").write_text(json.dumps(report, indent=2, default=str))


if __name__ == "__main__":
    main()

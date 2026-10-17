#!/usr/bin/env python3
"""Transmitting checks of a board running CV firmware, antennas (or a 50 Ω
load) fitted (docs/firmware.md, bring-up checklist): IF level with RF off
and on, range profile, IF spurs, sweep periodicity and RF auto-mute. Writes
a JSON report and prints it. RF is off when it exits.
"""

import argparse
import json
import pathlib
import time

import numpy as np
from scipy.signal import find_peaks, welch
from tqdm import tqdm

from bench import periodicity
from qmrdk.config import Calibration, Sweep
from qmrdk.device import Device
from qmrdk.dsp.range import range_profile

SWEEP = Sweep(2.3e9, 2.5e9, 16e-3)
CW = Sweep(2.4e9, 2.4e9, 16e-3, kind="cw")
RATES = (25_000.0, 40_000.0)


def stats(codes: np.ndarray) -> dict:
    return {
        "mean": float(codes.mean()),
        "std": float(codes.std()),
        "min": int(codes.min()),
        "max": int(codes.max()),
    }


def run(dev: Device, sweep: Sweep, dark: bool) -> Sweep:
    dev.dark = dark
    return dev.configure(sweep)


def if_level(dev: Device, seconds: float) -> dict:
    """Code statistics and sweep periodicity of a stream, RF off then on."""
    out = {}
    for dark in (True, False):
        sweep = run(dev, SWEEP, dark)
        fs = dev.timing().fs
        codes = dev.capture(round(seconds * fs))
        out["rf_off" if dark else "rf_on"] = stats(codes) | periodicity(
            codes.astype(float), round(2 * sweep.ramp_time * fs)
        )
    return out


def profile(dev: Device, frames: int, r_max: float, peaks: int) -> dict:
    """Coherent range profile with RF on against the RF-off floor; strongest
    lines below `r_max` (no `if_delay` or `r_cal` correction)."""
    out = {}
    for dark in (True, False):
        sweep = run(dev, SWEEP, dark)
        cal = Calibration().with_timing(**vars(dev.timing()))
        codes, _, _ = dev.capture_many(4096, frames, desc="frames")
        r, level = range_profile(codes, sweep, cal)
        level = 10 * np.log10(np.mean(10 ** (level / 10), axis=0))
        keep = (r > 0.2) & (r < r_max)
        out["rf_off" if dark else "rf_on"] = (r[keep], level[keep])
    r, on = out["rf_on"]
    floor = float(np.median(out["rf_off"][1]))
    idx, _ = find_peaks(on, prominence=6.0)
    idx = idx[np.argsort(on[idx])[::-1][:peaks]]
    return {
        "floor_rf_off_dbfs": floor,
        "median_rf_on_dbfs": float(np.median(on)),
        "peaks": [
            {
                "range_m": float(r[i]),
                "dbfs": float(on[i]),
                "snr_db": float(on[i] - floor),
            }
            for i in sorted(idx)
        ],
    }


def spurs(dev: Device, seconds: float, lines: int) -> dict:
    """IF spectrum lines of a CW at each sample rate, RF off and on: the
    strongest Welch bins above 1 Hz, in dB over the spectrum median."""
    out = {}
    for rate in RATES:
        dev.scpi(f"CAPT:RATE {rate}")
        for dark in (True, False):
            run(dev, CW, dark)
            fs = dev.timing().fs
            x = dev.capture(round(seconds * fs)).astype(float)
            f, p = welch(x - x.mean(), fs, nperseg=4096)
            db = 10 * np.log10(p / np.median(p))
            idx, _ = find_peaks(np.where(f > 1.0, db, 0.0), height=10.0)
            idx = idx[np.argsort(db[idx])[::-1][:lines]]
            out[f"{fs:.2f}_{'rf_off' if dark else 'rf_on'}"] = [
                {"hz": float(f[i]), "db_over_median": float(db[i])} for i in idx
            ]
    dev.scpi(f"CAPT:RATE {RATES[0]}")
    return out


def auto_mute(dev: Device, wait: float) -> dict:
    """RF on, no host traffic for `wait` s, then `POWE:RF?`."""
    run(dev, SWEEP, False)
    timeout = int(dev.transport.query("POWE:RF:TIME?"))
    before = dev.transport.query("POWE:RF?")
    for _ in tqdm(range(int(wait)), desc="silent", unit="s"):
        time.sleep(1.0)
    return {
        "timeout_s": timeout,
        "rf_before": before,
        "rf_after": dev.transport.query("POWE:RF?"),
        "silent_s": wait,
    }


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial")
    p.add_argument("--seconds", type=float, default=4.0, help="per stream")
    p.add_argument("--frames", type=int, default=16, help="profile frames")
    p.add_argument("--r-max", type=float, default=15.0, help="profile range, m")
    p.add_argument("--out", type=pathlib.Path, default=pathlib.Path("artifacts/bench"))
    a = p.parse_args()
    with Device(serial=a.serial) as dev:
        if not dev.cv:
            raise SystemExit(f"{dev.idn.firmware}: not CV firmware")
        report = {"idn": vars(dev.idn), "temperature": dev.temperature()}
        checks = {
            "if_level": (if_level, dev, a.seconds),
            "profile": (profile, dev, a.frames, a.r_max, 8),
            "spurs": (spurs, dev, a.seconds, 6),
            "auto_mute": (auto_mute, dev, 35.0),
        }
        try:
            for name, (fn, *args) in checks.items():
                report[name] = fn(*args)
        finally:
            dev.stop()
            report["rf_final"] = dev.transport.query("POWE:RF?")
            report["temperature_final"] = dev.temperature()
            report["errors"] = dev.errors()
            a.out.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%dT%H%M%S")
            path = a.out / f"rftest-{dev.idn.firmware}-{stamp}.json"
            path.write_text(json.dumps(report, indent=2))
            print(json.dumps(report, indent=2))
            print(path)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Antenna-free bench checks of a board running CV firmware (docs/firmware.md,
bring-up checklist). Every sweep runs dark (`Device.dark`): the RF is switched
off in the same message that starts it, so nothing is transmitted into an
unterminated port. Writes a JSON report and prints a summary.
"""

import argparse
import json
import pathlib
import time

import numpy as np
from tqdm import tqdm

from qmrdk.config import Sweep
from qmrdk.device import Device, LockError
from qmrdk.transport import DeviceError, DeviceTimeout

RATES = (25_000.0, 40_000.0)
REJECT = ("SWEEP:FREQSTAR 2.2", "SWEEP:FREQSTOP 2.6", "SWEEP:RAMPTIME 0")
SWEEPS = (
    Sweep(2.3e9, 2.5e9, 16e-3),
    Sweep(2.4e9, 2.5e9, 1e-3),
    Sweep(2.4e9, 2.5e9, 40e-3),
    Sweep(2.3e9, 2.5e9, 16e-3, kind="saw", dwell=1e-3),
)


def rejected(dev: Device) -> dict:
    """Error codes queued by each out-of-range setter; settings unchanged."""
    before = dev.settings().sweep
    out = {}
    for cmd in REJECT:
        dev.transport.write(cmd)
        out[cmd] = [code for code, _ in dev.transport.errors()]
    out["unchanged"] = dev.settings().sweep == before
    return out


def lock_scan(dev: Device, step: float) -> dict:
    """CW lock and read-back frequency across the device's frequency range."""
    freqs = np.arange(dev.limits.f_min, dev.limits.f_max + step / 2, step)
    unlocked, worst = [], 0.0
    for f in tqdm(freqs, desc="cw lock", unit="freq"):
        try:
            got = dev.configure(Sweep(f, f, 16e-3, kind="cw")).f0
            worst = max(worst, abs(got - f))
        except LockError:
            unlocked.append(f)
    return {
        "points": len(freqs),
        "unlocked_hz": unlocked,
        "max_readback_error_hz": worst,
    }


def sweep_locks(dev: Device) -> list:
    """Lock and read-back of representative triangle and sawtooth sweeps."""
    out = []
    for s in SWEEPS:
        try:
            got = dev.configure(s)
            out.append(
                {
                    "requested": vars(s),
                    "readback": vars(got),
                    "timing": vars(dev.timing()),
                }
            )
        except LockError as err:
            out.append({"requested": vars(s), "error": str(err)})
    return out


def periodicity(x: np.ndarray, period: int) -> dict:
    """Sweep-synchronous content of a stream: the autocorrelation peak near
    the sweep period and the share of variance in the period-folded mean."""
    m = x.size // period * period
    y = x[:m] - x[:m].mean()
    folded = y.reshape(-1, period)
    lags = np.arange(period - period // 8, period + period // 8 + 1)
    spec = np.fft.rfft(y, 2 * m)
    acf = np.fft.irfft(spec * spec.conj())[: lags[-1] + 1] / np.dot(y, y)
    return {
        "period": period,
        "acf_peak_lag": int(lags[np.argmax(acf[lags])]),
        "acf_at_period": float(acf[period]),
        "periodic_variance_share": float(folded.mean(0).var() / y.var()),
    }


def stream(dev: Device, rate: float, seconds: float) -> dict:
    """Stream `seconds` of samples at `rate` Hz on a dark 16 ms triangle;
    host-clock sample rate, code statistics and sweep periodicity."""
    dev.scpi(f"CAPT:RATE {rate}")
    dev.settings()
    sweep = dev.configure(Sweep(2.4e9, 2.5e9, 16e-3))
    t = dev.timing()
    n = round(seconds * t.fs)
    t0 = time.monotonic()
    try:
        codes = dev.capture(n)
    except DeviceError as err:
        return {"rate_requested": rate, "fs": t.fs, "error": str(err)}
    elapsed = time.monotonic() - t0
    bits = np.bitwise_or.reduce(codes ^ codes[0])
    return {
        "rate_requested": rate,
        "fs": t.fs,
        "samples": n,
        "host_rate": n / elapsed,
        "elapsed_s": elapsed,
        "code_mean": float(codes.mean()),
        "code_std": float(codes.std()),
        "code_min": int(codes.min()),
        "code_max": int(codes.max()),
        "toggling_bits": int(bits).bit_count(),
        **periodicity(codes.astype(float), round(2 * sweep.ramp_time * t.fs)),
    }


def timeout_recovery(dev: Device, rounds: int = 4) -> list:
    """Block reads that time out, each followed by `CAPT:ABOR` and a fresh
    query; the queries between rounds walk the USBTMC bTag through 1-255, so
    the host's abort runs with tags on both sides of 128."""
    dev.configure(Sweep(2.4e9, 2.5e9, 16e-3))
    out = []
    for _ in range(rounds):
        for _ in range(50):
            dev.transport.query("*IDN?")
        dev.transport.write("CAPT:CONT 1000000")
        try:
            dev.transport.query_block("CAPT:BLOC?", timeout=1)
            timed_out = False
        except DeviceTimeout:
            timed_out = True
        dev.transport.write("CAPT:ABOR")
        dev.transport.flush()
        out.append(
            {
                "timed_out": timed_out,
                "idn_after": dev.transport.query("*IDN?"),
                "errors": dev.errors(),
            }
        )
    return out


def temperature(dev: Device, reads: int = 200) -> dict:
    """`SYST:TEMP?` at idle and after each of `reads` short captures: the
    readings' spread exposes a corrupted sensor read on the shared SPI1."""
    dev.configure(Sweep(2.4e9, 2.4e9, 16e-3, kind="cw"))
    out = {}
    for name, before in (("idle", None), ("after_capture", 256)):
        t = np.empty(reads)
        for i in tqdm(range(reads), desc=f"temperature {name}", unit="read"):
            if before:
                dev.capture(before)
            t[i] = dev.temperature()
        med = float(np.median(t))
        out[name] = {
            "median": med,
            "min": float(t.min()),
            "max": float(t.max()),
            "outliers": int(np.sum(np.abs(t - med) > 1.0)),
        }
    return out


def checked(fn, *args):
    """Result of one check, or the error that ended it."""
    try:
        return fn(*args)
    except (DeviceError, ValueError) as err:
        return {"error": f"{type(err).__name__}: {err}"}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--serial")
    p.add_argument("--seconds", type=float, default=30.0, help="per stream")
    p.add_argument("--step", type=float, default=5e6, help="CW scan step, Hz")
    p.add_argument("--out", type=pathlib.Path, default=pathlib.Path("artifacts/bench"))
    a = p.parse_args()
    with Device(serial=a.serial) as dev:
        if not dev.cv:
            raise SystemExit(f"{dev.idn.firmware}: not CV firmware")
        dev.dark = True
        report = {
            "idn": vars(dev.idn),
            "firm": dev.transport.query("SYST:FIRM?"),
            "boot_errors": dev.boot_errors,
            "rf_idle": dev.transport.query("POWE:RF?"),
            "temperature": dev.temperature(),
            "limits": vars(dev.limits),
        }
        checks = {
            "rejected": (rejected, dev),
            "cw_lock": (lock_scan, dev, a.step),
            "sweeps": (sweep_locks, dev),
            **{f"stream_{r:.0f}": (stream, dev, r, a.seconds) for r in RATES},
            "timeout_recovery": (timeout_recovery, dev),
            "temperature": (temperature, dev),
        }
        try:
            for name, (fn, *args) in checks.items():
                report[name] = checked(fn, *args)
            dev.scpi(f"CAPT:RATE {RATES[0]}")
            dev.stop()
            report["rf_final"] = dev.transport.query("POWE:RF?")
            report["errors_final"] = dev.errors()
        finally:
            write_report(report, a.out)


def write_report(report: dict, out: pathlib.Path) -> None:
    """JSON report named by firmware and time; also printed."""
    out.mkdir(parents=True, exist_ok=True)
    path = (
        out / f"bench-{report['idn']['firmware']}-{time.strftime('%Y%m%dT%H%M%S')}.json"
    )
    path.write_text(json.dumps(report, indent=2, default=str))
    print(json.dumps(report, indent=2, default=str))
    print(path)


if __name__ == "__main__":
    main()

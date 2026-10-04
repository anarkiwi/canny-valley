#!/usr/bin/env python3
"""Bring-up probe for the QM-RDK: records raw protocol behaviour, capture
timing and capture sets (optionally alongside an audio recording of the IF
tap) into a report directory for offline analysis.

Sends only documented commands, never saves instrument state, and restores
the sweep settings found at start.
"""

import argparse
import json
import pathlib
import subprocess
import time

import numpy as np
import pyvisa
from tqdm import tqdm

VID = 0x2012
CHUNK = 31
FS = 21977.0
QUERIES = (
    "*IDN?",
    "SYST:IDEN?",
    "SYST:SERNUM?",
    "SYST:MODNUM?",
    "SYST:FIRM?",
    "SYST:VERS?",
    "*OPT?",
    "*ESE?",
    "*SRE?",
    "*STB?",
    "*ESR?",
    "*OPC?",
    "SWEEP:TYPE?",
    "SWEEP:FREQSTAR?",
    "SWEEP:FREQSTOP?",
    "SWEEP:RAMPTIME?",
    "FREQ:REF:DIV?",
    "FREQ:LOCK?",
    "POWE:RF?",
    "SYST:TEMP?",
    "SYST:STAT?",
    "CAPT:STRE?",
)
SETTINGS = ("SWEEP:TYPE", "SWEEP:FREQSTAR", "SWEEP:FREQSTOP", "SWEEP:RAMPTIME")


class Rdk:
    """Minimal raw session; every exchange is appended to a log."""

    def __init__(self, timeout_ms=2000):
        manager = pyvisa.ResourceManager("@py")
        names = [r for r in manager.list_resources() if f"::{VID}::" in r]
        if not names:
            raise SystemExit("no QM-RDK found")
        self.dev = manager.open_resource(names[0])
        self.dev.timeout = timeout_ms
        self.dev.read_termination = "\n"
        self.dev.write_termination = "\n"
        self.resource = names[0]
        self.log = []

    def write(self, cmd):
        """Send a command."""
        start = time.time()
        self.dev.write(cmd)
        self.log.append({"t": start, "dt": time.time() - start, "w": cmd})

    def query(self, cmd):
        """Send a query; returns None and clears the device on timeout."""
        start = time.time()
        try:
            resp = self.dev.query(cmd)
        except pyvisa.VisaIOError as err:
            resp = None
            self.log.append({"t": start, "q": cmd, "err": str(err)})
            self.dev.clear()
        else:
            self.log.append(
                {"t": start, "dt": time.time() - start, "q": cmd, "r": resp}
            )
        return resp

    def errors(self):
        """Drain the error queue."""
        out = []
        while True:
            resp = self.query("SYST:ERR?")
            if resp is None or resp.startswith(("0,", "-0,")) or len(out) > 12:
                return out
            out.append(resp)

    def capture(self, count, after_arm=None):
        """One frame; returns codes, timing and the raw non-data responses."""
        info = {"n": count, "not_ready": 0, "odd": []}
        info["t_arm"] = time.time()
        self.dev.write(f"CAPT:FRAM {count}")
        info["dt_arm"] = time.time() - info["t_arm"]
        if after_arm:
            self.dev.write(after_arm)
        codes = []
        chunk_dt = []
        deadline = time.time() + count / FS + 5
        while len(codes) < count and time.time() < deadline:
            start = time.time()
            resp = self.dev.query("CAPT:FRAM?")
            chunk_dt.append(time.time() - start)
            if resp.strip() == "Not Ready":
                info["not_ready"] += 1
                continue
            try:
                vals = [int(resp[i : i + 4], 16) for i in range(0, len(resp), 4)]
            except ValueError:
                info["odd"].append(resp)
                continue
            if len(resp) % 4 or not vals:
                info["odd"].append(resp)
                continue
            if not codes:
                info["dt_first_data"] = time.time() - info["t_arm"]
            codes.extend(vals)
        info["dt_total"] = time.time() - info["t_arm"]
        info["chunk_dt_median"] = float(np.median(chunk_dt))
        info["chunks"] = len(chunk_dt)
        return np.array(codes[:count], dtype=np.uint16), info


def cmd_protocol(rdk, out):
    """Query formats, setter validation and capture timing."""
    report = {"resource": rdk.resource}
    report["initial_errors"] = rdk.errors()
    report["queries"] = {q: rdk.query(q) for q in tqdm(QUERIES, desc="queries")}
    ramp = rdk.query("SWEEP:RAMPTIME?")
    checks = {}
    for cmd in (
        "SWEEP:RAMPTIME 8",
        "SWEEP:RAMPTIME 8.5",
        "SWEEP:RAMPTIME 0",
        "SWEEP:FREQSTAR 3.0",
        "SWEEP:TYPE AUTO",
        "BOGUS:CMD 1",
    ):
        rdk.write(cmd)
        checks[cmd] = {
            "errors": rdk.errors(),
            "readback": (
                rdk.query(cmd.split()[0] + "?") if not cmd.startswith("BOGUS") else None
            ),
            "rf": rdk.query("POWE:RF?"),
        }
    rdk.write(f"SWEEP:RAMPTIME {int(float(ramp))}")
    rdk.write("SWEEP:START")
    report["setter_checks"] = checks
    timing = []
    for count in tqdm((1, 31, 32, 1024, 4096), desc="capture timing"):
        _, info = rdk.capture(count)
        info["after_end"] = [rdk.query("CAPT:FRAM?") for _ in range(2)]
        info["errors"] = rdk.errors()
        timing.append(info)
    report["capture_timing"] = timing
    report["capture_before_ready"] = {}
    rdk.write("CAPT:FRAM 4096")
    report["capture_before_ready"]["immediate"] = rdk.query("CAPT:FRAM?")
    time.sleep(0.5)
    rdk.capture(31)
    (out / "protocol.json").write_text(json.dumps(report, indent=1))


def cmd_captures(rdk, out, frames):
    """Capture sets across sweep types; arrays saved per configuration."""
    configs = [
        ("auto_16ms", ["SWEEP:TYPE 2", "SWEEP:RAMPTIME 16", "SWEEP:START"], {}),
        ("auto_4ms", ["SWEEP:TYPE 2", "SWEEP:RAMPTIME 4", "SWEEP:START"], {}),
        ("auto_40ms", ["SWEEP:TYPE 2", "SWEEP:RAMPTIME 40", "SWEEP:START"], {}),
        ("cw", ["SWEEP:TYPE 3", "SWEEP:START"], {}),
        ("tri_once", ["SWEEP:TYPE 1", "SWEEP:RAMPTIME 50", "SWEEP:START"], {}),
        (
            "tri_start_before",
            ["SWEEP:TYPE 1", "SWEEP:RAMPTIME 50"],
            {"before": "SWEEP:START"},
        ),
        (
            "tri_start_after",
            ["SWEEP:TYPE 1", "SWEEP:RAMPTIME 50"],
            {"after": "SWEEP:START"},
        ),
        (
            "tri_trg_after",
            ["SWEEP:TYPE 1", "SWEEP:RAMPTIME 50", "SWEEP:START"],
            {"after": "*TRG"},
        ),
        (
            "ramp_start_after",
            ["SWEEP:TYPE 0", "SWEEP:RAMPTIME 50"],
            {"after": "SWEEP:START"},
        ),
        ("ramp_no_trigger", ["SWEEP:TYPE 0", "SWEEP:RAMPTIME 50"], {}),
    ]
    meta = {}
    for name, setup, opts in tqdm(configs, desc="configs"):
        for cmd in setup:
            rdk.write(cmd)
        time.sleep(0.5)
        entry = {
            "setup": setup,
            "opts": opts,
            "errors_setup": rdk.errors(),
            "state": {q: rdk.query(q) for q in ("POWE:RF?", "FREQ:LOCK?")},
            "t_begin": time.time(),
            "frames": [],
        }
        codes = []
        for _ in tqdm(range(frames), desc=name, leave=False):
            if "before" in opts:
                rdk.write(opts["before"])
            data, info = rdk.capture(4096, after_arm=opts.get("after"))
            codes.append(data)
            entry["frames"].append(info)
        entry["errors"] = rdk.errors()
        entry["t_end"] = time.time()
        np.save(out / f"cap_{name}.npy", np.stack(codes))
        meta[name] = entry
        time.sleep(1.0)
    (out / "captures.json").write_text(json.dumps(meta, indent=1))


def main():
    """Entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("what", nargs="+", choices=("protocol", "captures"))
    parser.add_argument("--out", default="artifacts/probe")
    parser.add_argument("--frames", type=int, default=6)
    parser.add_argument("--audio", help="ALSA capture device, e.g. hw:HD")
    parser.add_argument("--rate", type=int, default=48000)
    args = parser.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rdk = Rdk()
    saved = {s: rdk.query(s + "?") for s in SETTINGS}
    rf_on = rdk.query("POWE:RF?")
    recorder = None
    if args.audio:
        wav = out / "audio.wav"
        recorder = subprocess.Popen(  # pylint: disable=consider-using-with
            ["arecord", "-q", "-D", args.audio, "-f", "S24_3LE", "-c", "2"]
            + ["-r", str(args.rate), str(wav)]
        )
        time.sleep(1.0)
    session = {"t_start": time.time(), "saved": saved, "rf_on": rf_on}
    try:
        if "protocol" in args.what:
            cmd_protocol(rdk, out)
        if "captures" in args.what:
            cmd_captures(rdk, out, args.frames)
    finally:
        for name, value in saved.items():
            if value is not None:
                if name == "SWEEP:RAMPTIME":
                    value = int(float(value))
                rdk.write(f"{name} {value}")
        rdk.write("SWEEP:START" if rf_on == "1" else "SWEEP:STOP")
        session["restore_errors"] = rdk.errors()
        session["final"] = {s: rdk.query(s + "?") for s in SETTINGS + ("POWE:RF",)}
        session["t_end"] = time.time()
        if recorder:
            time.sleep(1.0)
            recorder.terminate()
            recorder.wait()
        session["log"] = rdk.log
        (out / "session.json").write_text(json.dumps(session, indent=1))
        subprocess.run(["chmod", "-R", "a+rwX", str(out)], check=False)


if __name__ == "__main__":
    main()

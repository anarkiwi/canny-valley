"""`qmrdk` command line."""

import argparse
import dataclasses
import json
import pathlib
import sys

import numpy as np

from qmrdk.config import Calibration, ScanGeometry, Sweep
from qmrdk.constants import ADC_MAX, FS_NOMINAL
from qmrdk.device import SWEEP_TYPES, Device, DeviceError
from qmrdk.dsp.sar import form_image
from qmrdk.radar import HardwareSled, ManualSled, UsbRadar
from qmrdk.recording import Recording
from qmrdk.render import (
    animate,
    grid,
    reflector_height,
    save_image,
    scene_extent,
)
from qmrdk.scan import run_scan, scan_positions
from qmrdk.sim.devices import SimRadar, SimSled
from qmrdk.sim.hardware import Hardware
from qmrdk.sim.scene import builtin_scene, compile_scene, load_scene
from qmrdk.sim.scpi import default_manager
from qmrdk.transport import resources


def _scene(spec):
    """Scene dict from a JSON path or a built-in scene name."""
    return load_scene(spec) if pathlib.Path(spec).is_file() else builtin_scene(spec)


def _board(args):
    """Simulated board, with the IF gain overridden by --gain."""
    hw = Hardware()
    return hw if args.gain is None else dataclasses.replace(hw, gain=args.gain)


def _clipping(codes, name):
    """Warn when ADC codes sit at either rail."""
    frac = float(np.mean((codes == 0) | (codes == ADC_MAX)))
    if frac > 0.0:
        print(f"{name}: warning: {frac:.1%} of ADC samples clipped", file=sys.stderr)
    return frac


def _sim_scan(scene, sweep, geometry, args, hw):
    """Simulated scan of `scene` on board `hw`."""
    geom = compile_scene(scene, sweep.lam)
    sled = SimSled(sigma=args.sled_sigma, seed=args.seed)
    radar = SimRadar(geom, hw, sweep, sled, geometry, seed=args.seed)
    extra = {
        "scene": scene.get("name"),
        "extent": scene.get("ground", {}).get("extent"),
    }
    positions = scan_positions(sweep, args.length, args.dx)
    return geom, run_scan(radar, sled, positions, args.n, geometry, extra)


def _scan(args):
    sweep, geometry = _sweep(args), ScanGeometry(height=args.height)
    if args.sim:
        _, rec = _sim_scan(_scene(args.scene), sweep, geometry, args, _board(args))
    else:
        sled = ManualSled() if args.manual else HardwareSled()
        positions = scan_positions(sweep, args.length, args.dx)
        with UsbRadar(sweep=sweep) as radar:
            rec = run_scan(radar, sled, positions, args.n, geometry)
    _clipping(rec.codes, args.out)
    rec.save(args.out)
    print(f"{args.out}: {rec.codes.shape[0]} positions")
    return 0


def _extent(args, scene, rec):
    """Image extent from --extent, the truth scene, the recording or the rail."""
    if args.extent is not None:
        return tuple(args.extent)
    if scene is None and rec.extra.get("extent") is not None:
        scene = {"ground": {"extent": rec.extra["extent"]}}
    if scene is None:
        mid = 0.5 * (rec.x_pos.min() + rec.x_pos.max())
        scene = {"ground": {"extent": [mid - 10.0, mid + 10.0, 1.0, 20.0]}}
    return scene_extent(scene, rec.x_pos)


def _image(args):
    rec = Recording.load(args.recording)
    if rec.x_pos is None:
        print(f"{args.recording}: not a SAR scan (no x_pos)", file=sys.stderr)
        return 1
    _clipping(rec.codes, args.recording)
    cal = Calibration() if args.cal is None else Calibration.load(args.cal)
    geometry = rec.geometry or ScanGeometry()
    scene = None if args.scene is None else _scene(args.scene)
    extent = _extent(args, scene, rec)
    gx, gy = grid(extent, args.pixel)
    img = form_image(
        rec.codes,
        rec.x_pos,
        rec.sweep,
        cal,
        geometry,
        gx,
        gy,
        background=_background(args),
        aperture_window=args.aperture_window,
    )
    geom = None if scene is None else compile_scene(scene, rec.sweep.lam)
    save_image(
        args.out,
        geom,
        img,
        geometry,
        rec.x_pos,
        rec.sweep.lam,
        args.dynamic_range,
        extent=extent,
    )
    print(f"{args.out}: first_up={img.first_up} sharpness={img.sharpness:.3g}")
    return 0


def _background(args):
    return None if args.background == "none" else args.background


def _json_default(o):
    return o.tolist() if hasattr(o, "tolist") else str(o)


def _demo(args):
    hw, sweep, geometry = _board(args), _sweep(args), ScanGeometry(height=args.height)
    scene = _scene(args.scene)
    geom, rec = _sim_scan(scene, sweep, geometry, args, hw)
    clipped = _clipping(rec.codes, args.scene)
    report = None
    if args.cal == "truth":
        cal = hw.calibration(sweep)
    elif args.cal == "procedure":
        from qmrdk.calib import calibrate_sim  # pylint: disable=C0415,E0401,E0611

        cal, report = calibrate_sim(hw, sweep, geometry, args.seed)
    else:
        cal = Calibration.load(args.cal)
    extent = scene_extent(scene, rec.x_pos)
    gx, gy = grid(extent, args.pixel)
    z = reflector_height(geom, geometry.height)
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    img = animate(
        out, geom, rec, cal, sweep, geometry, gx, gy, z, args.frames,
        dynamic_range=args.dynamic_range, extent=extent,
        background=_background(args), aperture_window=args.aperture_window,
    )  # fmt: skip
    final = out.with_name(f"{out.stem}_final.png")
    save_image(
        final, geom, img, geometry, rec.x_pos, sweep.lam, args.dynamic_range,
        extent=extent,
    )  # fmt: skip
    meta = out.with_name(f"{out.stem}_cal.json")
    meta.write_text(
        json.dumps(
            {
                "source": args.cal,
                "gain": hw.gain,
                "clipped": clipped,
                "calibration": dataclasses.asdict(cal),
                "report": report,
            },
            indent=2,
            default=_json_default,
        ),
        encoding="utf-8",
    )
    print(
        f"{out} {final} {meta}: first_up={img.first_up} sharpness={img.sharpness:.3g}"
    )
    return 0


def _drift(args):
    from qmrdk.drift import run  # pylint: disable=C0415

    return run(args)


def _calib_unavailable(args):
    print(f"calib commands unavailable: {args.reason}", file=sys.stderr)
    return 1


def _add_calib(sub):
    try:
        from qmrdk.calib import add_commands  # pylint: disable=C0415
    except ImportError as e:
        calib = sub.add_parser("calib", help="board calibration (docs/calibration.md)")
        calib.set_defaults(func=_calib_unavailable, reason=str(e))
        return
    add_commands(sub)


def _manager(args):
    return default_manager() if args.sim else None


def _device(args):
    return Device(args.resource, args.serial, _manager(args))


def _dump(obj):
    print(json.dumps(obj, indent=2, default=_json_default))


def _list(args):
    manager = _manager(args)
    names = [args.resource] if args.resource else resources(manager, args.serial)
    for name in names:
        try:
            with Device(name, args.serial, manager) as dev:
                print(f"{name}\t{dev.idn.serial}\t{dev.idn.firmware}")
        except DeviceError as e:
            print(f"{name}\t{e}", file=sys.stderr)
    return 0 if names else 1


def _settings(dev):
    s = dev.settings()
    return {
        "sweep": dataclasses.asdict(s.sweep),
        "ref_div": s.ref_div,
        "rf": s.rf,
        "locked": s.locked,
    }


def _info(args):
    with _device(args) as dev:
        _dump(
            {
                "resource": dev.transport.name,
                "idn": dataclasses.asdict(dev.idn),
                **_settings(dev),
                "temperature": dev.temperature(),
                "status": dataclasses.asdict(dev.status()),
                "errors": dev.boot_errors + dev.errors(),
            }
        )
    return 0


def _set(args):
    with _device(args) as dev:
        dev.configure(_sweep(args))
        dev.leave_rf_on = True
        _dump(_settings(dev))
    return 0


def _rf(args):
    with _device(args) as dev:
        dev.rf(args.state == "on")
        dev.leave_rf_on = args.state == "on"
        _dump(_settings(dev))
    return 0


def _scpi(args):
    with _device(args) as dev:
        resp = dev.scpi(args.message, force=args.force)
    if resp is not None:
        print(resp)
    return 0


def _capture(args):
    with _device(args) as dev:
        sweep = dev.configure(_sweep(args))
        codes, t_host, temp = dev.capture_many(
            args.n, args.frames, interval=args.interval, temperature=args.temperature
        )
        extra = {
            "idn": dataclasses.asdict(dev.idn),
            "resource": dev.transport.name,
            "ref_div": dev.settings().ref_div,
            "fs": FS_NOMINAL,
        }
    Recording(codes, sweep, t_host, extra=extra, temperature=temp).save(args.out)
    _clipping(codes, args.out)
    print(f"{args.out}: {codes.shape[0]} frames of {codes.shape[1]} samples")
    return 0


def _hw_options(p, sweep=False):
    p.add_argument(
        "--sim", action="store_true", help="simulated board (qmrdk.sim.scpi)"
    )
    p.add_argument("--resource", help="VISA resource name")
    p.add_argument("--serial", help="board serial number")
    if sweep:
        p.add_argument("--f0", type=float, default=2.4, help="sweep start, GHz")
        p.add_argument("--f1", type=float, default=2.5, help="sweep stop, GHz")
        p.add_argument("--ramp-time", type=float, default=16.0, help="ramp time, ms")
        p.add_argument("--type", choices=list(SWEEP_TYPES), default="triangle")


def _add_hardware(sub):
    p = sub.add_parser("list", help="boards on USB and their *IDN?")
    _hw_options(p)
    p.set_defaults(func=_list)
    p = sub.add_parser(
        "info", help="identity, settings, lock, temperature, status, errors"
    )
    _hw_options(p)
    p.set_defaults(func=_info)
    p = sub.add_parser("set", help="configure and start the sweep (RF left on)")
    _hw_options(p, sweep=True)
    p.set_defaults(func=_set)
    p = sub.add_parser("rf", help="start (on, left on) or stop (off) the sweep")
    p.add_argument("state", choices=("on", "off"))
    _hw_options(p)
    p.set_defaults(func=_rf)
    p = sub.add_parser(
        "scpi", help="raw command or query (undocumented commands refused)"
    )
    p.add_argument("message")
    p.add_argument("--force", action="store_true", help="allow *SAV 0 and SYST:REST")
    _hw_options(p)
    p.set_defaults(func=_scpi)
    p = sub.add_parser("capture", help="frames into a recording")
    p.add_argument("--frames", type=int, default=1)
    p.add_argument("--n", type=int, default=4096, help="samples per frame")
    p.add_argument("--out", required=True, help="recording .npz")
    p.add_argument(
        "--interval", type=float, default=0.0, help="minimum s between frame starts"
    )
    p.add_argument(
        "--temperature",
        action="store_true",
        help="read the board temperature before each frame",
    )
    _hw_options(p, sweep=True)
    p.set_defaults(func=_capture)


def _sweep(args):
    return Sweep(
        f0=args.f0 * 1e9,
        f1=args.f1 * 1e9,
        ramp_time=args.ramp_time * 1e-3,
        kind=getattr(args, "type", "triangle"),
    )


def _sim_options(p, seed=0):
    p.add_argument("--f0", type=float, default=2.4, help="sweep start, GHz")
    p.add_argument("--f1", type=float, default=2.5, help="sweep stop, GHz")
    p.add_argument("--ramp-time", type=float, default=16.0, help="ramp time, ms")
    p.add_argument("--length", type=float, default=1.5, help="rail span, m")
    p.add_argument("--dx", type=float, help="position spacing, m (lam_min / 4)")
    p.add_argument("--n", type=int, default=4096, help="samples per capture")
    p.add_argument("--height", type=float, default=1.0, help="rail height, m")
    p.add_argument("--seed", type=int, default=seed)
    p.add_argument("--sled-sigma", type=float, default=0.0, help="sled error, m")
    p.add_argument("--gain", type=float, help="simulated IF gain, V/sqrt(W)")


def _imaging_options(p, background, window):
    p.add_argument(
        "--pixel", type=float, nargs=2, default=(0.05, 0.25), metavar=("DX", "DY")
    )
    p.add_argument("--dynamic-range", type=float, default=40.0, help="dB")
    p.add_argument(
        "--aperture-window", default=window, help="scipy window over positions"
    )
    p.add_argument(
        "--background",
        choices=("mean", "none"),
        default=background,
        help="subtract the mean over positions (removes returns constant along the rail)",
    )


def parser():
    ap = argparse.ArgumentParser(prog="qmrdk", description=__doc__)
    sub = ap.add_subparsers(dest="command", required=True)
    sar = sub.add_parser("sar", help="rail SAR").add_subparsers(
        dest="sar_command", required=True
    )
    p = sar.add_parser("scan", help="scan the rail into a recording")
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true", help="simulated radar and sled")
    mode.add_argument(
        "--manual", action="store_true", help="move the radar by hand when prompted"
    )
    p.add_argument("--scene", default="yard", help="built-in scene name or JSON")
    p.add_argument("--out", required=True, help="recording .npz")
    _sim_options(p, seed=None)
    p.set_defaults(func=_scan)
    p = sar.add_parser("image", help="form an image from a scan recording")
    p.add_argument("recording")
    p.add_argument("--out", required=True, help="PNG")
    p.add_argument("--cal", help="calibration JSON")
    p.add_argument("--scene", help="scene (name or JSON) for the truth panel")
    p.add_argument("--extent", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"))
    _imaging_options(p, "mean", None)
    p.set_defaults(func=_image)
    p = sar.add_parser("demo", help="simulated scan, calibration and animation")
    p.add_argument("--scene", default="yard", help="built-in scene name or JSON")
    p.add_argument("--out", default="artifacts/yard.png", help="animated PNG")
    p.add_argument("--cal", default="procedure", help="truth, procedure or JSON")
    p.add_argument("--frames", type=int, default=24)
    _sim_options(p)
    _imaging_options(p, "none", "hann")
    p.set_defaults(func=_demo)
    _add_hardware(sub)
    p = sub.add_parser("drift", help="phase drift of a static capture vs time, temp")
    p.add_argument("recording", help="static capture .npz (qmrdk capture)")
    p.add_argument("--cal", required=True, help="calibration JSON")
    p.add_argument("--out", help="plot PNG")
    p.add_argument("--report", help="write the JSON report here")
    p.add_argument("--lines", type=int, default=3, help="strongest lines to track")
    p.set_defaults(func=_drift)
    _add_calib(sub)
    return ap


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        return args.func(args)
    except (NotImplementedError, DeviceError) as e:
        print(f"qmrdk: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

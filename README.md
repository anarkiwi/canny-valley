# canny-valley

Linux/USB/Python control and signal processing for the Quonset Microwave
QM-RDK 2.4 GHz FMCW/CW radar demonstration kit, replacing the vendor's
Windows GUI and MATLAB scripts and adding rail SAR imaging.

Control and data are over USB only; the board's Bluetooth link is not used.
USB capture is implemented and `calib timing`/`calib guard` run on the
board; the sled driver is not implemented, so scans and calibration steps 3
and 4 run against the simulated board and scene with `--sim`.

## Install

```
pip install -e .[dev]
```

or build `docker/Dockerfile`.

## Usage

```
qmrdk sar demo --scene yard --out artifacts/yard.png          # simulated scan, calibration, animated truth vs SAR
qmrdk sar scan --sim --scene yard --out artifacts/scan.npz    # simulated scan recording
qmrdk sar image artifacts/scan.npz --cal cal.json --scene yard --out artifacts/scan.png
qmrdk calib sim --cal cal.json                                # full calibration procedure on the simulated board
qmrdk calib {timing,guard,reflector,repeat} --sim --cal cal.json
qmrdk calib timing --frames-file A.npz                        # on the board, first position
qmrdk calib timing --frames-file B.npz --pair A.npz --cal cal.json   # after moving the radar ~3 cm
```

Sweep options `--f0`, `--f1` (GHz) and `--ramp-time` (ms) select the
simulated sweep. Scenes are JSON files; built-in scenes are in
`qmrdk/scenes/`.

## Documents

* [Implementation plan](docs/implementation-plan.md)
* [USB control protocol](docs/protocol.md)
* [Signal processing](docs/signal-processing.md)
* [Simulation model](docs/simulation.md)
* [Calibration procedure](docs/calibration.md)
* [Board notes](docs/hardware.md)

## License

MIT, see [LICENSE](LICENSE).

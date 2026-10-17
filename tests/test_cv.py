"""CV firmware (docs/protocol.md §6): simulated board, transport and device."""

# pylint: disable=protected-access

import dataclasses

import numpy as np
import pytest
from pyvisa import errors

from qmrdk import device, transport
from qmrdk.config import Calibration, Sweep, Timing
from qmrdk.device import (
    BLOCK,
    BLOCK_FIELDS,
    END,
    F_REF,
    OVERFLOW,
    ConfigError,
    Device,
    DeviceError,
    ForbiddenCommand,
    FrameError,
    ScpiError,
)
from qmrdk.dsp.segment import mirror_correlation
from qmrdk.sim import scpi
from qmrdk.sim.scpi import SimBoard, SimManager

pytestmark = pytest.mark.timeout(120)

CV = "CV1.0.0"
FS = 25_000.0
Q = F_REF / 2**24


def random_source(seed=0):
    """CV source of random codes; keeps every call's arguments and codes."""
    rng = np.random.default_rng(seed)
    served = []

    def source(n, sweep, timing):
        served.append((sweep, timing, rng.integers(0, 65536, n, dtype=np.uint16)))
        return served[-1][2]

    source.served = served
    return source


def realised(f0, f1, ramp, r=800):
    """Read-back (f0, f1, ramp) of the §6 quantisation, Hz and s."""
    nr = device.nearest(ramp * F_REF / r)
    n0q, n1q = device.nearest(f0 / Q), device.nearest(f1 / Q)
    step = device.nearest((n1q - n0q) / (nr * r))
    return n0q * Q, (n0q + step * nr * r) * Q, nr * r / F_REF


@pytest.fixture(name="board")
def fixture_board():
    return SimBoard(firmware=CV, source=random_source())


@pytest.fixture(name="res")
def fixture_res(board):
    return SimManager([board]).open_resource(SimManager.name(board))


@pytest.fixture(name="dev")
def fixture_dev(board):
    with Device(manager=SimManager([board])) as dev:
        yield dev


@pytest.fixture(name="running")
def fixture_running(dev):
    dev.configure(Sweep(ramp_time=8e-3))
    return dev


@pytest.fixture(autouse=True)
def _fast(monkeypatch):
    monkeypatch.setattr(device, "LOCK_TIMEOUT", 0.05)
    monkeypatch.setattr(device, "LOCK_POLL", 0.01)
    monkeypatch.setattr(transport, "POLL_S", 0.01)


def errs(res):
    out = []
    for _ in range(12):
        code = int(res.query("SYST:ERR?").split(",")[0])
        if code == 0:
            return out
        out.append(code)
    raise AssertionError("error queue does not drain")


def test_power_up(res, board):
    assert res.query("*IDN?") == f"Quonset Microwave,QM4004,0042,{CV}"
    replies = {
        "SWEEP:FREQSTAR?": "2.400000000",
        "SWEEP:FREQSTOP?": "2.499945068",
        "SWEEP:RAMPTIME?": "16.000000",
        "SWEEP:TYPE?": "2",
        "SWEEP:DWEL?": "1.000000",
        "SWEEP:ORIG?": "0.018125,1",
        "SWEEP:LIM?": "2.300000000,2.500000000,23841857.910156,0.040000,0.160000",
        "CAPT:RATE?": "25000.000000",
        "POWE:RF?": "0",
        "POWE:RF:TIME?": "30",
        "FREQ:LOCK?": "0",
        "FREQ:REF:DIV?": "1",
        "CAPT:FRAM?": "Not Ready",
        "SYST:FIRM?": CV,
    }
    assert {q: res.query(q) for q in replies} == replies
    res.write("CAPT:BLOC?")
    assert res.read_raw() == b"#6000000\n"
    assert errs(res) == [-500]
    assert not board.rf


@pytest.mark.parametrize(
    "cmd, code, query, value",
    [
        ("SWEEP:FREQSTAR 2.3", None, "SWEEP:FREQSTAR?", "2.300000000"),
        ("SWEEP:FREQSTAR 2.2", 201, "SWEEP:FREQSTAR?", "2.400000000"),
        ("SWEEP:FREQSTOP 2.51", 201, "SWEEP:FREQSTOP?", "2.499945068"),
        ("SWEEP:RAMPTIME 8.3", None, "SWEEP:RAMPTIME?", "8.320000"),
        ("SWEEP:RAMPTIME 0.1", 201, "SWEEP:RAMPTIME?", "16.000000"),
        ("SWEEP:RAMPTIME 0", 201, "SWEEP:RAMPTIME?", "16.000000"),
        ("SWEEP:TYPE 4", None, "SWEEP:TYPE?", "4"),
        ("SWEEP:TYPE 5", 201, "SWEEP:TYPE?", "2"),
        ("SWEEP:DWEL 0.1", 201, "SWEEP:DWEL?", "1.000000"),
        ("SWEEP:DWEL 2.51", None, "SWEEP:DWEL?", "2.520000"),
        ("CAPT:RATE 40000", None, "SWEEP:ORIG?", "0.028770,1"),
        ("CAPT:RATE 50000", 201, "CAPT:RATE?", "25000.000000"),
        ("CAPT:RATE 1000", 201, "CAPT:RATE?", "25000.000000"),
        ("POWE:RF:TIME 0", None, "POWE:RF:TIME?", "0"),
        ("POWE:RF:TIME 3601", 201, "POWE:RF:TIME?", "30"),
        ("FREQ:REF:DIV 7", None, "FREQ:REF:DIV?", "7"),
        ("*SAV 1", -113, "SWEEP:TYPE?", "2"),
        ("CAPT:FRAM 4097", 201, "CAPT:FRAM?", "Not Ready"),
    ],
)
def test_settings(res, cmd, code, query, value):
    errs(res)
    res.write(cmd)
    assert errs(res) == ([] if code is None else [code])
    assert res.query(query) == value


def test_rate_limits(res):
    errs(res)
    res.write("CAPT:RATE 40000")
    assert res.query("CAPT:RATE?") == f"{F_REF / 504:.6f}"
    assert res.query("SWEEP:LIM?").endswith(",0.025200,0.100800")
    res.write("CAPT:RATE 1222")
    assert res.query("CAPT:RATE?") == f"{F_REF / 16368:.6f}"
    res.write("CAPT:RATE 1221")
    assert errs(res) == [201]


@pytest.mark.parametrize(
    "f0, f1, ramp",
    [(2.4e9, 2.5e9, 16e-3), (2.3e9, 2.36e9, 7.3e-3), (2.47e9, 2.3e9, 1.1e-3)],
)
def test_realised_sweep(res, board, f0, f1, ramp):
    res.write(f"SWEEP:FREQSTAR {f0 / 1e9:.9f};SWEEP:FREQSTOP {f1 / 1e9:.9f}")
    res.write(f"SWEEP:RAMPTIME {ramp * 1e3:.6f}")
    r0, r1, t = realised(f0, f1, ramp)
    assert float(res.query("SWEEP:FREQSTAR?")) == pytest.approx(r0 / 1e9, abs=1e-9)
    assert float(res.query("SWEEP:FREQSTOP?")) == pytest.approx(r1 / 1e9, abs=1e-9)
    assert float(res.query("SWEEP:RAMPTIME?")) == pytest.approx(t * 1e3, abs=1e-9)
    assert board.sweep == Sweep(r0, r1, t)
    assert res.query("SWEEP:ORIG?") == f"0.018125,{int(f1 > f0)}"


def test_start_lock_and_types(res, board):
    res.write("SWEEP:START")
    assert res.query("FREQ:LOCK?") == "1" and board.rf
    res.write("SWEEP:TYPE 3")
    assert res.query("SWEEP:ORIG?") == "0.000000,1" and res.query("FREQ:LOCK?") == "0"
    res.write("SWEEP:STOP")
    assert not board.rf
    errs(res)
    res.write("SWEEP:TYPE 2;SWEEP:FREQSTOP 2.4001;SWEEP:RAMPTIME 60000;SWEEP:START")
    assert errs(res) == [201] and not board.rf and res.query("FREQ:LOCK?") == "0"
    res.write("SWEEP:TYPE 3;SWEEP:START")
    assert res.query("FREQ:LOCK?") == "1"
    res.write("*RST")
    assert board.boot == 1 and not board.rf


def test_dark_sweeps_and_captures_with_rf_off(dev, board):
    dev.dark = True
    dev.configure(Sweep(ramp_time=8e-3))
    assert dev.running and not board.rf and dev.transport.query("POWE:RF?") == "0"
    assert dev.capture(3000).shape == (3000,) and not board.rf


def read_block(res, timeout=10_000):
    res.timeout = timeout
    res.write("CAPT:BLOC?")
    raw = res.read_raw()
    head, payload = raw[:8], raw[8:-1]
    assert raw[-1:] == b"\n" and int(head[2:]) == len(payload)
    if not payload:
        return None
    seq, first, count, flags = BLOCK_FIELDS.unpack_from(payload)
    data = np.frombuffer(payload, "<u2", offset=BLOCK_FIELDS.size)
    assert data.size == count
    return seq, first, flags, data


@pytest.mark.parametrize("kind, n", [(2, 2500), (4, 1024), (0, 3), (3, 1500)])
def test_stream_blocks(res, board, kind, n):
    res.write(f"SWEEP:TYPE {kind};SWEEP:START;CAPT:CONT {n}")
    sweep, timing, codes = board.source.served[-1]
    assert sweep == board.sweep and timing == board.timing and codes.size == n
    blocks = [read_block(res) for _ in range(-(-n // BLOCK))]
    assert [b[:3] for b in blocks] == [
        (i, i * BLOCK, END * (i == len(blocks) - 1)) for i in range(len(blocks))
    ]
    np.testing.assert_array_equal(np.concatenate([b[3] for b in blocks]), codes)
    assert read_block(res) is None


def test_stream_until_abort(res, board):
    res.write("SWEEP:START;CAPT:CONT 0")
    blocks = [read_block(res) for _ in range(9)]
    assert [b[1] for b in blocks] == [i * BLOCK for i in range(9)]
    assert not any(b[2] for b in blocks)
    period = 2 * round(board.sweep.ramp_time * FS)
    assert all(s[2].size % period == 0 for s in board.source.served)
    res.write("CAPT:ABOR")
    assert read_block(res) is None


def test_stopped_stream_is_mid_scale(res, board):
    res.write("CAPT:CONT 10")
    assert not board.source.served
    np.testing.assert_array_equal(read_block(res)[3], scpi.MID_SCALE)


def test_frame_hex_and_blocks(res, board):
    res.write("SWEEP:START;CAPT:FRAM 40")
    codes = board.source.served[-1][2]
    chunk = res.query("CAPT:FRAM?")
    assert chunk == codes[:31].astype(">u2").tobytes().hex().upper()
    seq, first, flags, data = read_block(res)
    assert (seq, first, flags) == (1, 31, END)
    np.testing.assert_array_equal(data, codes[31:])


def test_block_timeout(res):
    res.write("SWEEP:START;SWEEP:RAMPTIME 1000;CAPT:CONT 10")
    with pytest.raises(errors.VisaIOError):
        read_block(res, timeout=1000)
    assert read_block(res, timeout=10)[1] == 0


def test_resource_reads(res):
    res.write("*IDN?")
    assert res.read_bytes(3) == b"Quo"
    with pytest.raises(errors.VisaIOError):
        res.read_bytes(1000)
    res.write("*IDN?")
    assert res.read_raw(5) == b"Quons" and res.read().startswith("et Microwave")
    with pytest.raises(errors.VisaIOError):
        res.read_raw()


def test_allowlist(dev, attach):
    assert dev.cv and dev.transport.allowed_extra == transport.CV_ONLY
    assert dev.scpi("SWEEP:DWEL?") == "1.000000"
    assert dev.scpi("POWE:RF:TIME 10") is None
    with pytest.raises(ForbiddenCommand, match="binary"):
        dev.scpi("CAPT:BLOC?")
    stock = attach(SimBoard("0007"))
    with Device(serial="7") as other:
        assert not other.cv and not other.transport.allowed_extra
        sent = len(stock.log)
        for cmd in ("SWEEP:DWEL?", "CAPT:CONT 10", "CAPT:ABOR"):
            with pytest.raises(ForbiddenCommand):
                other.scpi(cmd)
        assert len(stock.log) == sent
        assert other.limits is None and other.timing() is None


def test_query_block(dev, board, monkeypatch):
    t = dev.transport
    assert t.query_block("CAPT:BLOC?") == b""
    res = t.res
    monkeypatch.setattr(res, "chunk_size", 5)
    dev.configure(Sweep(ramp_time=8e-3))
    t.write("CAPT:CONT 100")
    payload = t.query_block("CAPT:BLOC?")
    np.testing.assert_array_equal(
        device.decode_block(payload, 0, 0, 100), board.source.served[-1][2]
    )
    board.inject("nonhex")
    t.write("CAPT:CONT 100")
    with pytest.raises(FrameError, match="header"):
        t.query_block("CAPT:BLOC?")
    with pytest.raises(FrameError, match="header"):
        t.query_block("*IDN?")
    monkeypatch.setattr(res, "chunk_size", 20)
    monkeypatch.setattr(res, "read_bytes", lambda count, *a: b"x" * count)
    t.write("CAPT:CONT 100")
    with pytest.raises(FrameError, match="bytes"):
        t.query_block("CAPT:BLOC?")


def block(seq, first, data, flags=0, count=None):
    data = np.asarray(data, np.uint16)
    count = data.size if count is None else count
    return BLOCK_FIELDS.pack(seq, first, count, flags) + data.astype("<u2").tobytes()


@pytest.mark.parametrize(
    "payload, match",
    [
        (b"", "no block"),
        (block(0, 0, [1, 2], END, count=3), "bytes for"),
        (block(0, 0, [1, 2], END | OVERFLOW), "dropped"),
        (block(1, 0, [1, 2], END), "expected 0 at 0"),
        (block(0, 5, [1, 2], END), "expected 0 at 0"),
        (block(0, 0, [1, 2]), "END False"),
        (block(0, 0, [1], END), "END True"),
        (block(0, 0, [1, 2, 3], END), "3 samples"),
        (block(0, 0, [], END), "0 samples"),
    ],
)
def test_decode_block_rejects(payload, match):
    with pytest.raises(FrameError, match=match):
        device.decode_block(payload, 0, 0, 2)


def test_decode_block():
    out = device.decode_block(block(2, 2048, [0, 65535, 258], END), 2, 2048, 2051)
    np.testing.assert_array_equal(out, [0, 65535, 258])
    assert out.dtype == np.uint16


def test_connect(board):
    board.message("SWEEP:START;CAPT:CONT 0", 1.0)
    with Device(manager=SimManager([board])) as dev:
        assert board.acq is None and dev.boot_errors == [(-500, "Power on")]
        assert board.log[2:5] == ["CAPT:ABOR", "CAPT:FRAM?", "SYST:ERR?"]
        assert dev.fs == FS and dev.limits.f_min == 2.3e9
        assert dev.limits.t_quantum == pytest.approx(1 / FS)
        assert dev.sweep.ramp_time == 16e-3 and not dev.running
        s = dev.settings()
        assert (s.ref_div, s.rf, s.fs, s.n0, s.first_up) == (
            1,
            True,
            FS,
            0.018125,
            True,
        )
        assert s.timing == Timing(FS, 0.018125, True) == dev.timing()
    assert not board.rf


@pytest.mark.parametrize(
    "sweep, match",
    [
        (Sweep(kind="auto"), "kind"),
        (Sweep(f0=2.2e9), "outside 2.3..2.5"),
        (Sweep(f1=2.51e9, kind="cw"), "outside 2.3..2.5"),
        (Sweep(ramp_time=0.1e-3), "outside 0.16..65536"),
        (Sweep(ramp_time=70.0), "outside 0.16..65536"),
        (Sweep(f0=2.5e9, f1=2.4e9), "above start"),
        (Sweep(f1=2.4001e9, ramp_time=8.0), "slope"),
        (Sweep(kind="saw", dwell=0.1e-3), "dwell 0.1 ms below 0.16"),
        (Sweep(dwell=1e-3), "sawtooth sweeps only"),
    ],
)
def test_validate_rejects(sweep, match, dev, board):
    with pytest.raises(ConfigError, match=match):
        device.validate(sweep, dev.limits, dev.fs)
    sent = len(board.log)
    with pytest.raises(ConfigError):
        dev.configure(sweep)
    assert len(board.log) == sent


def test_validate_stock_refuses_saw():
    with pytest.raises(ConfigError, match="need CV firmware"):
        device.validate(Sweep(kind="saw"))
    with pytest.raises(ConfigError, match="sawtooth sweeps only"):
        device.validate(Sweep(dwell=1e-3))


@pytest.mark.parametrize(
    "sweep",
    [
        Sweep(ramp_time=8e-3),
        Sweep(f0=2.3e9, f1=2.36e9, ramp_time=7.3e-3, kind="tri"),
        Sweep(f0=2.3e9, kind="cw"),
        Sweep(ramp_time=0.16e-3, kind="ramp"),
        Sweep(f1=2.4001e9, ramp_time=4e-3),
    ],
)
def test_configure_reads_back(sweep, dev, board):
    got = dev.configure(sweep)
    f0, f1, t = realised(sweep.f0, sweep.f1, sweep.ramp_time)
    f1 = got.f1 if sweep.kind == "cw" else f1
    assert got == Sweep(round(f0), round(f1), t, sweep.kind) == dev.sweep
    assert dev.running and board.rf and board.locked()
    assert not any(m.startswith("FREQ:REF:DIV") for m in board.log)
    assert f"SWEEP:RAMPTIME {sweep.ramp_time * 1e3:.6f}" in board.log


def test_configure_saw(dev, board):
    got = dev.configure(Sweep(ramp_time=8e-3, kind="saw"))
    assert got.dwell == 1e-3 and not any("DWEL " in m for m in board.log)
    got = dev.configure(Sweep(ramp_time=8e-3, kind="saw", dwell=2.51e-3))
    assert got.kind == "saw" and got.dwell == 2.52e-3 and board.sweep.dwell == 2.52e-3
    assert "SWEEP:DWEL 2.510000" in board.log
    assert dev.timing() == Timing(FS, 0.018125, True)


def test_firmware_rejection_leaves_rf_off(dev, board, monkeypatch):
    monkeypatch.setattr(device, "_validate_cv", lambda *a: (2, 1.0, None))
    with pytest.raises(ScpiError) as err:
        dev.configure(Sweep(f1=2.6e9))
    assert err.value.code == 201 and err.value.cmd == "SWEEP:FREQSTOP 2.600000000"
    with pytest.raises(ScpiError, match="SWEEP:START"):
        dev.configure(Sweep(f1=2.4e9))
    assert not board.rf and not dev.running


@pytest.mark.parametrize("n", [1, BLOCK, BLOCK + 1, 4096, 10_000])
def test_capture_streams(n, running, board):
    np.testing.assert_array_equal(running.capture(n), board.source.served[-1][2])
    blocks = -(-n // BLOCK)
    assert board.log[-1 - blocks :] == [f"CAPT:CONT {n}"] + ["CAPT:BLOC?"] * blocks


def test_capture_long_progress(running, board, capsys):
    codes = running.capture(device.PROGRESS_BLOCKS * BLOCK + 1)
    np.testing.assert_array_equal(codes, board.source.served[-1][2])
    assert "stream" in capsys.readouterr().err


@pytest.mark.parametrize(
    "sweep", [Sweep(ramp_time=500e-3), Sweep(ramp_time=900e-3, kind="saw", dwell=0.6)]
)
def test_capture_waits_one_period(sweep, board):
    with Device(manager=SimManager([board]), timeout=50) as dev:
        dev.configure(sweep)
        np.testing.assert_array_equal(dev.capture(3000), board.source.served[-1][2])
        assert dev.transport.res.timeout == 50


@pytest.mark.parametrize(
    "fault", ["nonhex", "truncate", "not_ready", "stall", "overflow"]
)
def test_capture_recovers(fault, running, board):
    board.inject(fault)
    np.testing.assert_array_equal(running.capture(3000), board.source.served[-1][2])
    assert len(board.source.served) == 2 and board.log.count("CAPT:CONT 3000") == 2
    assert "CAPT:ABOR" in board.log and not board.errs and board.rf


def test_capture_gives_up_with_rf_off(running, board):
    board.inject(*["overflow"] * device.RETRIES)
    with pytest.raises(DeviceError, match="capture failed: block 1") as err:
        running.capture(3000)
    assert isinstance(err.value.__cause__, FrameError)
    assert not board.rf and not running.running


def test_capture_after_disconnect(running, board):
    board.message("CAPT:RATE 20000", 1.0)
    sweep = running.configure(Sweep(ramp_time=8e-3, kind="saw", dwell=2e-3))
    board.inject("disconnect")
    np.testing.assert_array_equal(running.capture(64), board.source.served[-1][2])
    assert board.boot == 1 and board.rf and running.fs == FS
    assert board.sweep.kind == "saw" and board.sweep.dwell == sweep.dwell


def test_capture_limits(running):
    with pytest.raises(ConfigError, match="outside 1..2147483647"):
        running.capture(0)


def test_capture_many(running, board):
    codes, t_host, _ = running.capture_many(5000, 2)
    np.testing.assert_array_equal(codes, np.stack([s[2] for s in board.source.served]))
    assert np.all(np.diff(t_host) >= 0)


def test_reset(board):
    with Device(manager=SimManager([board])) as dev:
        dev.configure(Sweep(ramp_time=8e-3))
        dev.reset()
        assert board.boot == 1 and not board.rf and dev.sweep.ramp_time == 16e-3


def test_malformed_replies(dev, monkeypatch):
    with monkeypatch.context() as m:
        m.setattr(dev.transport, "query", lambda cmd, timeout=None: "25000")
        with pytest.raises(DeviceError, match="SWEEP:LIM"):
            dev.settings()
        with pytest.raises(DeviceError, match="SWEEP:ORIG"):
            dev.timing()


@pytest.mark.parametrize("kind", ["triangle", "saw"])
def test_scene_source_continuous(kind):
    """Frames of a running sweep repeat sample for sample from sample 0."""
    board = SimBoard(firmware=CV)
    board.message(f"SWEEP:TYPE {4 if kind == 'saw' else 2};SWEEP:RAMPTIME 8", 1.0)
    board.message("SWEEP:START;CAPT:CONT 4000", 1.0)
    codes = board.acq["codes"].astype(float)
    period = 400 if kind == "triangle" else 225
    assert np.abs(codes[period:] - codes[:-period]).max() < 40
    nr = 200
    j = mirror_correlation(codes, np.arange(nr, 2 * nr), np.arange(16, nr // 2))
    assert (j.max() > 0.9) == (kind == "triangle")


def test_scene_source_single_ramp():
    board = SimBoard(firmware=CV)
    board.message("SWEEP:TYPE 0;SWEEP:RAMPTIME 40;SWEEP:START;CAPT:CONT 9000", 1.0)
    codes = board.acq["codes"]
    end = int(np.ceil(0.018125 + 1000))
    assert np.all(codes[end:] == scpi.MID_SCALE) and np.ptp(codes[:end]) > 100


def test_calibration_with_timing_and_period(tmp_path):
    cal = Calibration(fs=21_977.0, n0=3.0, if_delay=40e-6, ng=7)
    cv = cal.with_timing(**dataclasses.asdict(Timing(FS, 0.018125, False)))
    assert (cv.fs, cv.first_up, cv.ng) == (FS, False, 7)
    assert cv.n0 == pytest.approx(0.018125 + 40e-6 * FS)
    cv.save(tmp_path / "cal.json")
    assert Calibration.load(tmp_path / "cal.json") == cv
    saw = Sweep(ramp_time=8e-3, kind="saw", dwell=1e-3)
    assert cv.period(saw) == pytest.approx(225.0) and cv.period(Sweep()) is None
    assert cv.nr(saw) == pytest.approx(200.0)


def test_nearest_rounds_halves_away_from_zero():
    assert [device.nearest(x) for x in (182.5, -2.5, 2.4999, -0.4, 3.5)] == [
        183,
        -3,
        2,
        0,
        4,
    ]

"""Device layer against the simulated board (qmrdk.sim.scpi)."""

# pylint: disable=protected-access

import signal

import numpy as np
import pytest

from qmrdk import device, transport
from qmrdk.config import Sweep
from qmrdk.device import (
    ConfigError,
    Device,
    DeviceError,
    ForbiddenCommand,
    FrameError,
    LockError,
    ScpiError,
)
from qmrdk.sim.scpi import SimBoard, SimManager

LONG = Sweep(ramp_time=10.0)
CHUNK = transport.CHUNK


def random_source(seed=0):
    """Source of uniformly random codes; keeps every frame it served."""
    rng = np.random.default_rng(seed)
    served = []

    def source(n, sweep):
        del sweep
        served.append(rng.integers(0, 65536, n, dtype=np.uint16))
        return served[-1]

    source.served = served
    return source


@pytest.fixture(name="board")
def fixture_board():
    return SimBoard(source=random_source())


@pytest.fixture(name="dev")
def fixture_dev(board):
    with Device(manager=SimManager([board])) as dev:
        yield dev


@pytest.fixture(name="running")
def fixture_running(dev):
    dev.configure(Sweep())
    return dev


@pytest.fixture(autouse=True)
def _fast(monkeypatch):
    monkeypatch.setattr(device, "LOCK_TIMEOUT", 0.05)
    monkeypatch.setattr(device, "LOCK_POLL", 0.01)
    monkeypatch.setattr(transport, "POLL_S", 0.01)


def test_ref_divider():
    assert device.ref_divider(Sweep()) == 1
    assert device.ref_divider(Sweep(kind="cw", ramp_time=10.0)) == 1
    div = device.ref_divider(LONG)
    assert div == 2
    slope_min = device.F_REF**2 / 2**25
    assert slope_min / div <= LONG.slope < slope_min / (div - 1)


@pytest.mark.parametrize(
    "sweep, match",
    [
        (Sweep(kind="auto"), "kind"),
        (Sweep(f1=2.6e9), "outside 2.4..2.5"),
        (Sweep(f0=2.3e9, kind="cw"), "outside 2.4..2.5"),
        (Sweep(ramp_time=16.5e-3), "not an integer"),
        (Sweep(ramp_time=0.0), "outside 1..65536"),
        (Sweep(ramp_time=65.537), "outside 1..65536"),
        (Sweep(f0=2.5e9, f1=2.4e9), "above start"),
        (Sweep(f1=2.401e9, ramp_time=30.0), "above T_max 21475 ms"),
    ],
)
def test_validate_rejects(sweep, match, dev, board):
    with pytest.raises(ConfigError, match=match):
        device.validate(sweep)
    sent = len(board.log)
    with pytest.raises(ConfigError):
        dev.configure(sweep)
    assert len(board.log) == sent


def test_validate():
    assert device.validate(Sweep(ramp_time=7e-3, kind="tri")) == (1, 7, 1)
    assert device.validate(LONG) == (2, 10000, 2)
    assert device.validate(Sweep(f0=2.45e9, f1=2.41e9, kind="cw")) == (3, 16, 1)
    t_max = Sweep(f1=2.401e9).bandwidth * 256 * 2**25 / device.F_REF**2
    assert device.validate(Sweep(f1=2.401e9, ramp_time=int(t_max * 1e3) * 1e-3))


def test_connect(board):
    board.message("CAPT:FRAM 64", 1.0)
    board.message("SWEEP:TYPE AUTO", 1.0)
    with Device(manager=SimManager([board])) as dev:
        assert dev.boot_errors == [(-500, "Power on"), (-102, "Syntax error")]
        assert dev.idn == dev.identify() and dev.idn.model == "QM4004"
        assert not dev.errors() and board.cursor >= len(board.frame)
        assert dev.sweep == Sweep() and not dev.running
        s = dev.settings()
        assert (s.ref_div, s.rf, s.locked) == (1, True, True)
        assert dev.temperature() == board.temperature
        assert dev.status() == device.Status(0, "Operational")
        assert signal.getsignal(signal.SIGINT) is device._on_signal
    assert not board.rf and board.log[-1] == "POWE:RF?"
    assert signal.getsignal(signal.SIGINT) is not device._on_signal
    dev.close()


@pytest.mark.parametrize(
    "sweep",
    [
        Sweep(),
        LONG,
        Sweep(f0=2.45e9, kind="cw"),
        Sweep(f0=2.41e9, f1=2.47e9, ramp_time=50e-3, kind="tri"),
        Sweep(ramp_time=3e-3, kind="ramp"),
    ],
)
def test_configure_reads_back(sweep, dev, board):
    assert dev.configure(sweep) == sweep == dev.sweep
    assert dev.running and board.rf and board.locked()
    assert board.state["FREQ:REF:DIV"] == device.ref_divider(sweep)
    assert dev.settings().sweep == sweep


def test_divider_written_only_on_change(dev, board):
    dev.configure(Sweep())
    dev.configure(Sweep(ramp_time=20e-3))
    assert not any(m.startswith("FREQ:REF:DIV ") for m in board.log)
    dev.configure(LONG)
    assert board.log.count("FREQ:REF:DIV 2") == 1


def test_firmware_rejection_leaves_rf_off(dev, board, monkeypatch):
    monkeypatch.setattr(device, "validate", lambda sweep: (2, 16, 1))
    with pytest.raises(ScpiError) as err:
        dev.configure(Sweep(f1=2.6e9))
    assert err.value.code == -222 and err.value.cmd == "SWEEP:FREQSTOP 2.600000"
    assert not board.rf and not dev.running


def test_lock(dev, board, monkeypatch):
    board.stuck_unlocked = True
    with pytest.raises(LockError):
        dev.configure(Sweep())
    assert not board.rf and not dev.running
    board.stuck_unlocked = False
    monkeypatch.setattr(device, "ref_divider", lambda sweep: 1)
    with pytest.raises(LockError):
        dev.configure(LONG)
    assert not board.rf


def test_rf(dev, board):
    dev.rf(True)
    assert board.rf and board.sweeping and dev.running
    dev.rf(False)
    assert not board.rf and not dev.running
    board._run = lambda on: None
    board.rf = True
    with pytest.raises(DeviceError, match="RF still on"):
        dev.rf(False)
    board.rf = False


@pytest.mark.parametrize("n", [1, CHUNK, CHUNK + 1, 4096])
def test_capture_pages(n, running, board):
    np.testing.assert_array_equal(running.capture(n), board.source.served[-1])
    assert board.log[-1 - -(-n // CHUNK) :] == [f"CAPT:FRAM {n}"] + ["CAPT:FRAM?"] * -(
        -n // CHUNK
    )


def test_capture_extends_first_timeout(board):
    with Device(manager=SimManager([board]), timeout=50) as dev:
        dev.configure(Sweep())
        np.testing.assert_array_equal(dev.capture(4096), board.source.served[-1])
        assert dev.transport.res.timeout == 50


@pytest.mark.parametrize("n", [0, 4097])
def test_capture_length(n, running):
    with pytest.raises(ConfigError, match="outside"):
        running.capture(n)


def test_capture_needs_running_sweep(dev):
    with pytest.raises(DeviceError, match="not started"):
        dev.capture(10)


@pytest.mark.parametrize("fault", ["nonhex", "truncate", "not_ready", "stall"])
def test_capture_recovers(fault, running, board):
    board.inject(fault)
    np.testing.assert_array_equal(running.capture(100), board.source.served[-1])
    assert len(board.source.served) == 2 and board.log.count("CAPT:FRAM 100") == 2
    assert not board.errs and board.rf


def test_capture_gives_up_with_rf_off(running, board):
    board.inject(*["nonhex"] * device.RETRIES)
    with pytest.raises(DeviceError, match="capture failed: malformed") as err:
        running.capture(100)
    assert isinstance(err.value.__cause__, FrameError)
    assert not board.rf and not running.running
    with pytest.raises(DeviceError, match="not started"):
        running.capture(100)


def test_capture_after_disconnect(running, board):
    sweep = running.configure(LONG)
    board.inject("disconnect")
    np.testing.assert_array_equal(running.capture(64), board.source.served[-1])
    assert board.boot == 1 and board.sweep == sweep and board.rf
    assert running.boot_errors == [(-500, "Power on")]


def test_capture_after_unplug(running, board):
    running.transport.reopen_s = 0.05
    board.inject("unplug")
    with pytest.raises(DeviceError, match="did not re-enumerate"):
        running.capture(64)
    running.close()
    assert not board.rf


def test_close_after_reboot(running, board):
    board.reboot()
    assert board.rf
    running.close()
    assert not board.rf and board.boot == 1


def test_leave_rf_on(board):
    with Device(manager=SimManager([board])) as dev:
        dev.configure(Sweep())
        dev.leave_rf_on = True
    assert board.rf and "SWEEP:STOP" not in board.log


def test_capture_many(running, board):
    codes, t_host = running.capture_many(40, 3)
    np.testing.assert_array_equal(codes, np.stack(board.source.served))
    assert codes.shape == (3, 40) and np.all(np.diff(t_host) >= 0)


def test_reset(board):
    board.reboot_s = 0.03
    with Device(manager=SimManager([board])) as dev:
        dev.configure(LONG)
        dev.reset()
        assert board.boot == 1 and not board.rf and not dev.running
        assert dev.boot_errors == [(-500, "Power on")] and dev.sweep == Sweep()
        assert dev.idn.serial == board.serial


def test_scpi(dev, board):
    assert dev.scpi("SYST:IDEN?") == "QM4004"
    assert dev.scpi("SWEEP:RAMPTIME 20") is None
    assert board.state["SWEEP:RAMPTIME"] == 20
    with pytest.raises(ScpiError, match="-222"):
        dev.scpi("SWEEP:FREQSTAR 2.6")
    with pytest.raises(ScpiError, match="-113"):
        dev.scpi("BOGUS?")
    board.inject("stall")
    dev.transport.write("CAPT:FRAM 10")
    with pytest.raises(transport.DeviceTimeout):
        dev.scpi("CAPT:FRAM?")
    sent = len(board.log)
    for msg in ("FACT:BIASGATE 1", "CAPT:STRE 1", "*RST", "*SAV 0", "SYST:REST"):
        with pytest.raises(ForbiddenCommand):
            dev.scpi(msg)
    with pytest.raises(ForbiddenCommand):
        dev.scpi(":system:restore")
    assert len(board.log) == sent


def test_memory(dev, board):
    dev.scpi("SWEEP:RAMPTIME 20")
    dev.save(3)
    with pytest.raises(ForbiddenCommand):
        dev.save(0)
    dev.scpi("SWEEP:RAMPTIME 30")
    dev.recall(3)
    assert board.state["SWEEP:RAMPTIME"] == 20
    dev.save(0, force=True)
    assert board.memory[0]["SWEEP:RAMPTIME"] == 20
    with pytest.raises(ForbiddenCommand):
        dev.restore_factory()
    dev.restore_factory(force=True)
    assert board.memory[0]["SWEEP:RAMPTIME"] == 16


def test_signal_stops_rf(board):
    previous = signal.signal(signal.SIGTERM, signal.SIG_DFL)
    try:
        with Device(manager=SimManager([board])) as dev:
            dev.configure(Sweep())
            with pytest.raises(SystemExit) as stop:
                device._on_signal(signal.SIGTERM, None)
            assert stop.value.code == 128 + signal.SIGTERM and not board.rf
            dev.configure(Sweep())
            with pytest.raises(KeyboardInterrupt):
                device._on_signal(signal.SIGINT, None)
            assert not board.rf
        assert signal.getsignal(signal.SIGTERM) == signal.SIG_DFL
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        with Device(manager=SimManager([board])) as dev:
            dev.configure(Sweep())
            assert device._on_signal(signal.SIGTERM, None) is None
            assert not board.rf
        assert signal.getsignal(signal.SIGTERM) == signal.SIG_IGN
    finally:
        signal.signal(signal.SIGTERM, previous)


def test_atexit_closes(board):
    dev = Device(manager=SimManager([board]))
    dev.configure(Sweep())
    device._close_all()
    assert not board.rf and dev not in device._OPEN and dev.transport.res is None


@pytest.mark.parametrize(
    "resp", ["", "12G4", "123", "0" * (4 * CHUNK + 4), "Not Ready", " 0000"]
)
def test_decode_chunk_rejects(resp):
    with pytest.raises(FrameError, match="malformed"):
        device.decode_chunk(resp)


def test_decode_chunk():
    np.testing.assert_array_equal(
        device.decode_chunk("0000FFFF01020a0b"), [0, 65535, 258, 2571]
    )
    with pytest.raises(FrameError):
        device.decode_chunk("0000FFFF", 3)


def test_open_failure_stops_rf(board):
    board.message("CAPT:FRAM 64", 1.0)
    board._chunk = lambda timeout: "0000"
    with pytest.raises(DeviceError, match="does not drain"):
        Device(manager=SimManager([board]))
    assert not board.rf and not device._OPEN

"""USB device layer against the fake board (tests/fakeboard.py)."""

import numpy as np
import pytest
from fakeboard import FakeBoard, install

from qmrdk import radar
from qmrdk.config import Sweep
from qmrdk.radar import DeviceError, UsbRadar

LONG = Sweep(ramp_time=10.0)


@pytest.fixture(name="fast_lock")
def fixture_fast_lock(monkeypatch):
    monkeypatch.setattr(radar, "LOCK_TIMEOUT", 0.1)


def test_ref_divider():
    assert radar.ref_divider(Sweep()) == 1
    assert radar.ref_divider(Sweep(kind="cw", ramp_time=10.0)) == 1
    div = radar.ref_divider(LONG)
    assert div == 2
    slope_min = radar.F_REF**2 / 2**25
    assert slope_min / div <= LONG.slope < slope_min / (div - 1)


@pytest.mark.parametrize("sweep", [Sweep(), LONG, Sweep(f0=2.45e9, kind="cw")])
def test_configure_reads_back(monkeypatch, sweep):
    board = install(monkeypatch, FakeBoard())
    with UsbRadar(sweep=sweep) as dev:
        assert dev.sweep == sweep
        assert dev.idn[2] == "0042"
        assert board.div == radar.ref_divider(sweep)
        assert board.read_termination == "\n" and board.timeout == 2000
    assert board.closed and board.log[-1] == "SWEEP:STOP"
    assert not board.sweeping


@pytest.mark.usefixtures("fast_lock")
def test_wrong_divider_never_locks(monkeypatch):
    board = install(monkeypatch, FakeBoard())
    monkeypatch.setattr(radar, "ref_divider", lambda sweep: 1)
    with pytest.raises(DeviceError, match="PLL not locked"):
        UsbRadar(sweep=LONG)
    assert board.closed and board.log[-1] == "SWEEP:STOP"


@pytest.mark.usefixtures("fast_lock")
def test_unlocked_pll(monkeypatch):
    install(monkeypatch, FakeBoard(lock=False))
    with pytest.raises(DeviceError, match="PLL not locked"):
        UsbRadar()


@pytest.mark.parametrize(
    "board, sweep, match",
    [
        (None, Sweep(), "no QM-RDK"),
        (FakeBoard(idn="Other,X,1,1"), Sweep(), "not a QM-RDK"),
        (FakeBoard(), Sweep(f1=2.6e9), "-222"),
        (FakeBoard(), Sweep(ramp_time=4e-4), "below 1 ms"),
    ],
)
def test_open_errors(monkeypatch, board, sweep, match):
    install(monkeypatch, board)
    with pytest.raises(DeviceError, match=match):
        UsbRadar(sweep=sweep)
    assert board is None or board.closed


def test_setter_error_queue(monkeypatch):
    board = install(monkeypatch, FakeBoard())
    dev = UsbRadar()
    with pytest.raises(DeviceError, match="-102"):
        dev.write("SWEEP:RAMPTIME 1.5")
    assert not dev.errors()
    board.errs = [f'-113,"x{i}"' for i in range(12)]
    assert len(dev.errors()) == 10


@pytest.mark.parametrize(
    "resp", ["", "12G4", "123", "0" * (4 * radar.CHUNK + 4), "Not Ready"]
)
def test_decode_chunk_rejects(resp):
    with pytest.raises(DeviceError, match="malformed"):
        radar.decode_chunk(resp)


def test_decode_chunk():
    np.testing.assert_array_equal(radar.decode_chunk("0000FFFF0102"), [0, 65535, 258])


@pytest.mark.parametrize("n", [1, radar.CHUNK, 4096])
def test_capture_pages(monkeypatch, n):
    codes = np.random.default_rng(n).integers(0, 65536, n, dtype=np.uint16)
    board = install(monkeypatch, FakeBoard(lambda m: codes[:m]))
    dev = UsbRadar()
    np.testing.assert_array_equal(dev.capture(n), codes)
    assert board.log.count("CAPT:FRAM?") == 1 + -(-n // radar.CHUNK)
    assert board.timeout == 2000


def test_capture_retries_after_bad_chunk(monkeypatch):
    board = install(monkeypatch, FakeBoard(bad=1))
    dev = UsbRadar()
    np.testing.assert_array_equal(dev.capture(100), np.arange(100))
    assert board.log.count("CAPT:FRAM 100") == 2


def test_capture_fails_after_retries(monkeypatch):
    board = install(monkeypatch, FakeBoard())
    dev = UsbRadar()
    board.bad = radar.RETRIES
    with pytest.raises(DeviceError, match="capture failed"):
        dev.capture(100)
    assert board.log.count("CAPT:FRAM 100") == radar.RETRIES


def test_capture_longer_frame_than_requested(monkeypatch):
    install(monkeypatch, FakeBoard(lambda n: np.zeros(n + 1, np.uint16)))
    with pytest.raises(DeviceError, match="longer than requested"):
        UsbRadar().capture(radar.CHUNK - 1)


@pytest.mark.parametrize("n", [0, 4097])
def test_capture_length(monkeypatch, n):
    install(monkeypatch, FakeBoard())
    with pytest.raises(DeviceError, match="outside"):
        UsbRadar().capture(n)


def test_flush_discards_pending(monkeypatch):
    board = install(monkeypatch, FakeBoard())
    dev = UsbRadar()
    board.write("CAPT:FRAM 4096")
    dev.flush()
    assert not board.pending.size
    board.pending = np.zeros(10**5, np.uint16)
    with pytest.raises(DeviceError, match="does not drain"):
        dev.flush()


def test_flush_survives_read_timeouts(monkeypatch):
    board = install(monkeypatch, FakeBoard())
    dev = UsbRadar()
    replies = iter([TimeoutError, "0000", "Not Ready"])

    def chunk():
        r = next(replies)
        if r is TimeoutError:
            raise r("read")
        return r

    monkeypatch.setattr(board, "_chunk", chunk)
    dev.flush()
    assert next(replies, None) is None

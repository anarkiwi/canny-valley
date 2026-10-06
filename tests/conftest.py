"""Pin thread pools before numerical libraries are imported, so parallel
test workers do not oversubscribe the host, and keep tests off real USB."""

import os

for _var in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_var, "2")
os.environ.setdefault("NUMBA_NUM_THREADS", "2")

# pylint: disable=wrong-import-position
import pytest

from qmrdk import transport
from qmrdk.sim import scpi


@pytest.fixture(autouse=True)
def _no_usb(monkeypatch):
    """No test reaches a real USB device: the bus is empty unless a test
    attaches simulated boards, and `--sim` starts from a fresh board."""
    monkeypatch.setattr(transport, "_manager", lambda: scpi.SimManager([]))
    scpi.default_manager.cache_clear()


@pytest.fixture(name="attach")
def fixture_attach(monkeypatch):
    """`attach(*boards)` puts simulated boards on the bus; returns the first."""

    def attach(*boards):
        monkeypatch.setattr(transport, "_manager", lambda: scpi.SimManager(boards))
        return boards[0] if boards else None

    return attach

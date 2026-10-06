"""Pin thread pools before numerical libraries are imported, so parallel
test workers do not oversubscribe the host, and keep tests off real USB."""

import os

for _var in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_var, "2")
os.environ.setdefault("NUMBA_NUM_THREADS", "2")

# pylint: disable=wrong-import-position
import pytest
from fakeboard import FakeManager

from qmrdk import radar


@pytest.fixture(autouse=True)
def _no_usb(monkeypatch):
    """No test reaches a real USB device: the board is absent unless a test
    installs a fake one."""
    monkeypatch.setattr(radar, "_manager", FakeManager)

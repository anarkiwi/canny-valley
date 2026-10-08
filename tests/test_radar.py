"""UsbRadar: the Radar protocol over the device layer; the manual sled."""

import io

import numpy as np
import pytest

from qmrdk import device, radar, transport
from qmrdk.config import Sweep
from qmrdk.radar import DeviceError, ManualSled, UsbRadar
from qmrdk.sim.scpi import SimBoard


def test_reexports():
    assert radar.DeviceError is device.DeviceError is transport.DeviceError
    assert radar.ref_divider is device.ref_divider
    assert radar.decode_chunk is device.decode_chunk


@pytest.mark.parametrize("sweep", [Sweep(), Sweep(ramp_time=10.0), Sweep(kind="cw")])
def test_configure_capture_close(attach, sweep):
    codes = np.arange(100, dtype=np.uint16)
    board = attach(SimBoard(source=lambda n, s: codes[:n]))
    with UsbRadar(sweep=sweep) as dev:
        assert dev.sweep == sweep == board.sweep and board.rf
        assert dev.idn.serial == "0042"
        np.testing.assert_array_equal(dev.capture(100), codes)
    assert not board.rf and dev.device.transport.res is None


def test_resource_and_serial(attach):
    attach(SimBoard("0001"), SimBoard("0002"))
    with UsbRadar(serial="2") as dev:
        assert dev.idn.serial == "0002"
    with UsbRadar(resource="USB0::8210::19::0001::0::INSTR") as dev:
        assert dev.idn.serial == "0001"
    assert not device._OPEN  # pylint: disable=protected-access


def test_open_errors(attach):
    with pytest.raises(DeviceError, match="no QM-RDK"):
        UsbRadar()
    board = attach(SimBoard())
    with pytest.raises(DeviceError, match="outside 2.4..2.5"):
        UsbRadar(sweep=Sweep(f1=2.6e9))
    assert not board.rf
    board.rf, board.stuck_unlocked = True, True
    with pytest.raises(DeviceError, match="not locked"):
        UsbRadar()
    assert not board.rf


def test_manual_sled():
    asked, out = [], io.StringIO()
    sled = ManualSled(prompt=lambda text: asked.append(out.getvalue()), out=out)
    assert sled.position() == 0.0
    sled.home()
    sled.move_to(0.0)
    sled.move_to(0.0305)
    assert sled.position() == 0.0305 and len(asked) == 2
    lines = out.getvalue().splitlines()
    assert "0.0000 m = 0.00 cm = 0.0 mm from home;" in lines[0]
    assert "0.0305 m = 3.05 cm = 30.5 mm from home, +30.5 mm from here" in lines[1]
    assert asked[1].endswith("Enter when placed\n")

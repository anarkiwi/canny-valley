"""Code-to-volt conversion and dBFS scaling (signal-processing §2)."""

import numpy as np

from qmrdk.dsp.convert import as_volts, codes_to_volts, dbfs


def test_codes_to_volts_end_points():
    v = codes_to_volts(np.array([0, 65535], dtype=np.uint16))
    np.testing.assert_array_equal(v, [-2.5, 2.5])


def test_codes_to_volts_is_affine():
    codes = np.arange(0, 65536, 4097)
    np.testing.assert_allclose(
        codes_to_volts(codes), codes * 5 / 65535 - 2.5, rtol=0, atol=1e-15
    )


def test_dbfs():
    np.testing.assert_allclose(
        dbfs([2.5, -2.5, 0.25, 2.5j]), [0.0, 0.0, -20.0, 0.0], atol=1e-12
    )
    assert dbfs(0.0) == -np.inf


def test_as_volts():
    codes = np.array([0, 32768, 65535], dtype=np.uint16)
    np.testing.assert_array_equal(as_volts(codes), codes_to_volts(codes))
    v = as_volts(np.array([0.5, -1.0], dtype=np.float32))
    assert v.dtype == np.float64 and v.tolist() == [0.5, -1.0]

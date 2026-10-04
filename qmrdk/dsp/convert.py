"""ADC code to volt conversion and level scaling (signal-processing §2)."""

import numpy as np

from qmrdk.constants import A_FS, ADC_MAX


def codes_to_volts(codes):
    """Unsigned 16-bit ADC codes to volts in [-A_FS, +A_FS]."""
    return np.asarray(codes, dtype=np.float64) * (2.0 * A_FS / ADC_MAX) - A_FS


def dbfs(amplitude):
    """Tone amplitude in volts to dB relative to the full-scale amplitude."""
    with np.errstate(divide="ignore"):
        return 20.0 * np.log10(np.abs(amplitude) / A_FS)

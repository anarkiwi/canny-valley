"""Pin thread pools before numerical libraries are imported, so parallel
test workers do not oversubscribe the host."""

import os

for _var in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_var, "2")
os.environ.setdefault("NUMBA_NUM_THREADS", "2")

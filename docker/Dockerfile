FROM python:3.12-slim AS base
ENV OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
    PIP_NO_CACHE_DIR=1 PYTHONDONTWRITEBYTECODE=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 alsa-utils && rm -rf /var/lib/apt/lists/*
COPY requirements.txt requirements-dev.txt /tmp/
RUN pip install -r /tmp/requirements.txt -r /tmp/requirements-dev.txt

FROM base
WORKDIR /work
COPY . /work
RUN pip install --no-deps -e .

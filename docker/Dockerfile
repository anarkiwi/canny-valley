FROM python:3.12-slim AS base
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 alsa-utils && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

FROM base
WORKDIR /work

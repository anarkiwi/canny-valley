# MPLAB IPE (ipecmd) for PICkit 5 access to the QM-RDK controller.
# Build context is the directory holding the Microchip Linux installer tar,
# which Microchip does not allow to be fetched by script:
#   docker build -f docker/mplab.Dockerfile -t canny-valley-mplab /scratch/dists/microchip
# Run with the live host USB bus, so the PICkit stays reachable when it
# re-enumerates during a firmware update:
#   docker run --rm -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' \
#     -v "$PWD/untracked/mplab:/out" canny-valley-mplab \
#     ipecmd.sh -TPPK5 -P24FJ256GB106 -GF/out/qmrdk-u4.hex
# --build-arg IDE=1 also installs the IDE, for the mdb command-line debugger
# (mdb.sh), tagged e.g. canny-valley-mdb.
FROM ubuntu:25.10
ARG MPLABX=6.35
ARG IDE=0
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 procps sudo ca-certificates && rm -rf /var/lib/apt/lists/*
COPY MPLABX-v${MPLABX}-linux-installer.tar /tmp/
RUN cd /tmp && tar xf MPLABX-v${MPLABX}-linux-installer.tar \
    && USER=root ./MPLABX-v${MPLABX}-linux-installer.sh -- --mode unattended \
       --ide ${IDE} --ipe 1 --8bitmcu 0 --16bitmcu 1 --32bitmcu 0 --othermcu 0 \
    ; rm -f /tmp/MPLABX-* \
    && ln -s /opt/microchip/mplabx/v${MPLABX}/mplab_platform/mplab_ipe/ipecmd.sh /usr/local/bin/ipecmd.sh \
    && if [ "${IDE}" = 1 ]; then ln -s /opt/microchip/mplabx/v${MPLABX}/mplab_platform/bin/mdb.sh /usr/local/bin/mdb.sh; fi
WORKDIR /out

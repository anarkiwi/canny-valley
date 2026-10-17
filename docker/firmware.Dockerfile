FROM ubuntu:24.04 AS xc16
ARG XC16=2.10 \
    XC16_SHA256=d64d5e7391ec862d4dd169361b6680b725f8960435121709b45eb1a4afe15dc8
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates && rm -rf /var/lib/apt/lists/*
RUN curl -fsSL -o /tmp/xc16.run \
    https://ww1.microchip.com/downloads/aemDocuments/documents/DEV/ProductDocuments/SoftwareTools/xc16-v${XC16}-full-install-linux64-installer.run \
    && echo "${XC16_SHA256}  /tmp/xc16.run" | sha256sum -c - \
    && chmod +x /tmp/xc16.run \
    && /tmp/xc16.run --mode unattended --netservername "" --prefix /opt/xc16 \
    && rm -rf /tmp/xc16.run /opt/xc16/docs /opt/xc16/src

FROM python:3.12-slim AS tools
ARG ARDUINO_CLI=1.5.1 AVR_CORE=1.8.6 ACCELSTEPPER=1.64.0 DOCTEST=2.5.2
ENV PIP_NO_CACHE_DIR=1 PYTHONDONTWRITEBYTECODE=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    g++ make curl ca-certificates && rm -rf /var/lib/apt/lists/*
COPY firmware/requirements.txt /tmp/
RUN pip install -r /tmp/requirements.txt
RUN curl -fsSL https://downloads.arduino.cc/arduino-cli/arduino-cli_${ARDUINO_CLI}_Linux_64bit.tar.gz \
    | tar -xz -C /usr/local/bin arduino-cli \
    && arduino-cli core update-index \
    && arduino-cli core install arduino:avr@${AVR_CORE} \
    && arduino-cli lib install AccelStepper@${ACCELSTEPPER}
RUN mkdir -p /usr/local/include/doctest && curl -fsSL -o /usr/local/include/doctest/doctest.h \
    https://raw.githubusercontent.com/doctest/doctest/v${DOCTEST}/doctest/doctest.h
COPY --from=xc16 /opt/xc16 /opt/xc16

FROM tools
WORKDIR /work/firmware
COPY firmware /work/firmware
CMD ["make", "all"]

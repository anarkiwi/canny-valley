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

FROM tools
WORKDIR /work/firmware
COPY firmware /work/firmware
CMD ["make", "all"]

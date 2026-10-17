#!/bin/sh
# Build the CV firmware at the checked-out commit and program it into U4 with
# a PICkit 5 on J1 (docs/programming.md). The image is kept as
# radar-<commit>.hex next to the stock dump; -OL releases the board from reset
# in the same session so it starts without a power cycle.
#   tools/flash.sh            build and program
#   tools/flash.sh <file.hex> program an existing image from the image dir
set -eu
root=$(git rev-parse --show-toplevel)
images=${IMAGES:-$(cd "$(git rev-parse --git-common-dir)/.." && pwd)/untracked/mplab}
if [ $# -eq 0 ]; then
  git -C "$root" submodule update --init --quiet firmware/radar/lib
  commit=$(git describe --always --dirty --abbrev=12 --exclude '*')
  hex=radar-$commit.hex
  timeout 1800 docker build -q -f "$root/docker/firmware.Dockerfile" \
    -t qmrdk-firmware "$root" >/dev/null
  timeout 900 docker run --rm -v "$images:/out" qmrdk-firmware sh -c \
    "make compile RADAR_COMMIT=$commit >build.log 2>&1 || { tail -40 build.log; exit 1; }
     cp build/radar/radar.hex /out/$hex"
else
  hex=$1
fi
timeout 300 docker run --rm -v /dev/bus/usb:/dev/bus/usb \
  --device-cgroup-rule='c 189:* rmw' -v "$images:/out" canny-valley-mplab \
  ipecmd.sh -TPPK5 -P24FJ256GB106 -M -F/out/"$hex" -Y -OL |
  grep -E "Succeeded|Failed|Error"
echo "$images/$hex"

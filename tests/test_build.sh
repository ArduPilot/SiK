#!/bin/bash
# build bootloader and firmware for every known board and check
# that each one produced an output

set -e

cd Firmware

BOARDS=$(ls include/rules_*.mk | sed 's|.*/rules_||;s|\.mk$||' | tr '\n' ' ')
echo "Building boards: $BOARDS"

make BOARDS="$BOARDS"
make BOARDS="$BOARDS" install

missing=0
for b in $BOARDS; do
    if [ ! -s "dst/radio~$b.ihx" ]; then
        echo "ERROR: missing firmware for $b"
        missing=1
    fi
    if ! ls dst/bootloader~$b~*.hex >/dev/null 2>&1; then
        echo "ERROR: missing bootloader for $b"
        missing=1
    fi
done
[ $missing -eq 0 ] || exit 1

echo "Built $(echo $BOARDS | wc -w) boards:"
ls -l dst

# the ATI8 counters (TDM_COUNTERS=1) only fit on boards with 8 KB of XRAM;
# install them elsewhere so the release firmware in dst stays as it is
COUNTER_BOARDS="mro900 rfd900p rfd900pe rfd900u rfd900ue"
echo "Building boards with TDM_COUNTERS=1: $COUNTER_BOARDS"
for b in $COUNTER_BOARDS; do
    make clean~radio~$b
    make TDM_COUNTERS=1 DSTROOT=$PWD/dst-counters install~radio~$b
    make clean~radio~$b
done

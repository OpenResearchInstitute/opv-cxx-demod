#!/bin/bash
# Run sync word test using same Pluto setup as main script

PLUTO_IP="${1:-ip:192.168.3.1}"
FREQ="${2:-905050000}"
COUNT="${3:-100}"

echo "Configuring PlutoSDR at $PLUTO_IP..."
iio_attr -u "$PLUTO_IP" -c ad9361-phy altvoltage1 frequency "$FREQ"
iio_attr -u "$PLUTO_IP" -c ad9361-phy -o voltage0 sampling_frequency 2168000
iio_attr -u "$PLUTO_IP" -c ad9361-phy -o voltage0 hardwaregain -0

echo ""
echo "Sending $COUNT sync words with dummy payload..."
./opv-sync-test -c "$COUNT" | iio_rwdev -u "$PLUTO_IP" -b 346880 -w cf-ad9361-dds-core-lpc

echo "Done. Check receiver for sync hit count."

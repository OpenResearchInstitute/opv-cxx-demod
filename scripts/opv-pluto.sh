#!/bin/bash
#
# opv-pluto.sh - OPV PlutoSDR Transceiver
#
# Copyright 2026 Open Research Institute, Inc.
# SPDX-License-Identifier: MIT
#
# Usage:
#   ./opv-pluto.sh                    # Default: 435 MHz simplex
#   ./opv-pluto.sh -f 435000000       # Specify frequency
#   ./opv-pluto.sh --tx-freq 435000000 --rx-freq 440000000  # Split
#
# Runs full-duplex: Interlocutor ↔ PlutoSDR ↔ RF
#

set -e

# =============================================================================
# CONFIGURATION
# =============================================================================

PLUTO_URI="ip:192.168.2.1"
TX_FREQ=435000000
RX_FREQ=435000000
SAMPLE_RATE=2168000
TX_GAIN=-20
RX_GAIN=40
BUFFER_SIZE=346880
TX_PORT=57372                        # Listen for frames from Interlocutor
RX_PORT=57373                        # Send decoded frames to Interlocutor
VERBOSE=0

OPV_MODEM="bin/opv-modem"

# PIDs for cleanup
TX_PID=""
RX_PID=""
IIO_TX_PID=""
IIO_RX_PID=""
TX_FIFO=""

# =============================================================================
# USAGE
# =============================================================================

usage() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

OPV PlutoSDR Transceiver - Full duplex Interlocutor integration.

Frequency Options:
  -f, --frequency HZ     Simplex frequency (default: 435 MHz)
  --tx-freq HZ           TX frequency (for split operation)
  --rx-freq HZ           RX frequency (for split operation)

Radio Options:
  --tx-gain DB           TX gain in dB (default: $TX_GAIN)
  --rx-gain DB           RX gain in dB (default: $RX_GAIN)
  -u, --uri URI          PlutoSDR URI (default: $PLUTO_URI)

Network Options:
  --tx-port PORT         UDP port to receive from Interlocutor (default: $TX_PORT)
  --rx-port PORT         UDP port to send to Interlocutor (default: $RX_PORT)

Other:
  -v, --verbose          Verbose output
  -h, --help             Show this help

Examples:
  $(basename "$0")                                    # 435 MHz simplex
  $(basename "$0") -f 905050000                       # 905.05 MHz simplex
  $(basename "$0") --tx-freq 435000000 --rx-freq 440000000  # Split
  $(basename "$0") -f 144390000 --tx-gain -10 --rx-gain 50  # 2m with custom gains

This script runs both TX and RX simultaneously:
  - Receives OPV frames from Interlocutor (UDP $TX_PORT) → transmits via Pluto
  - Receives from Pluto → sends decoded frames to Interlocutor (UDP $RX_PORT)

Press Ctrl+C to stop.

EOF
    exit 1
}

# =============================================================================
# PARSE ARGUMENTS
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        -f|--frequency)
            TX_FREQ="$2"
            RX_FREQ="$2"
            shift 2
            ;;
        --tx-freq)
            TX_FREQ="$2"
            shift 2
            ;;
        --rx-freq)
            RX_FREQ="$2"
            shift 2
            ;;
        --tx-gain)
            TX_GAIN="$2"
            shift 2
            ;;
        --rx-gain)
            RX_GAIN="$2"
            shift 2
            ;;
        --tx-port)
            TX_PORT="$2"
            shift 2
            ;;
        --rx-port)
            RX_PORT="$2"
            shift 2
            ;;
        -u|--uri)
            PLUTO_URI="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE=1
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

# =============================================================================
# VALIDATION
# =============================================================================

if [[ ! -x "$OPV_MODEM" ]]; then
    if [[ -x "./bin/opv-modem" ]]; then
        OPV_MODEM="./bin/opv-modem"
    elif [[ -x "../bin/opv-modem" ]]; then
        OPV_MODEM="../bin/opv-modem"
    else
        echo "Error: Cannot find opv-modem executable"
        echo "Build with: make"
        exit 1
    fi
fi

if ! command -v iio_attr &> /dev/null; then
    echo "Error: iio_attr not found. Install libiio."
    exit 1
fi

# Detect iio tools
if command -v iio_rwdev &> /dev/null; then
    IIO_WRITE_CMD="iio_rwdev -w"
    IIO_READ_CMD="iio_rwdev"
elif command -v iio_writedev &> /dev/null; then
    IIO_WRITE_CMD="iio_writedev"
    IIO_READ_CMD="iio_readdev"
else
    echo "Error: iio read/write tools not found."
    exit 1
fi

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

format_freq() {
    local freq=$1
    if [[ $freq -ge 1000000000 ]]; then
        printf "%.3f GHz" "$(echo "scale=3; $freq/1000000000" | bc)"
    elif [[ $freq -ge 1000000 ]]; then
        printf "%.3f MHz" "$(echo "scale=3; $freq/1000000" | bc)"
    else
        printf "%d Hz" "$freq"
    fi
}

cleanup() {
    echo ""
    echo "Shutting down..."
    
    # Kill all child processes
    [[ -n "$TX_PID" ]] && kill $TX_PID 2>/dev/null
    [[ -n "$RX_PID" ]] && kill $RX_PID 2>/dev/null
    [[ -n "$IIO_TX_PID" ]] && kill $IIO_TX_PID 2>/dev/null
    [[ -n "$IIO_RX_PID" ]] && kill $IIO_RX_PID 2>/dev/null
    
    # Kill any remaining children
    jobs -p | xargs -r kill 2>/dev/null || true
    
    # Clean up FIFO
    [[ -n "$TX_FIFO" ]] && rm -f "$TX_FIFO"
    
    wait 2>/dev/null
    echo "✓ Stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# =============================================================================
# CHECK PLUTO CONNECTION
# =============================================================================

echo "╔═══════════════════════════════════════════════════════════════════╗"
echo "║              OPV PlutoSDR Transceiver                             ║"
echo "╚═══════════════════════════════════════════════════════════════════╝"
echo ""
echo "Connecting to PlutoSDR at $PLUTO_URI..."

if ! iio_info -u "$PLUTO_URI" &> /dev/null; then
    echo "Error: Cannot connect to PlutoSDR at $PLUTO_URI"
    exit 1
fi

echo "✓ PlutoSDR connected"
echo ""

# =============================================================================
# CONFIGURE PLUTO
# =============================================================================

echo "Configuring radio..."

# TX settings
iio_attr -u "$PLUTO_URI" -c ad9361-phy altvoltage1 frequency "$TX_FREQ" > /dev/null
iio_attr -u "$PLUTO_URI" -c ad9361-phy -o voltage0 sampling_frequency "$SAMPLE_RATE" > /dev/null
iio_attr -u "$PLUTO_URI" -c ad9361-phy -o voltage0 hardwaregain "$TX_GAIN" > /dev/null 2>&1 || true

# Disable DDS
iio_attr -u "$PLUTO_URI" -c cf-ad9361-dds-core-lpc voltage0 raw 0 > /dev/null 2>&1 || true
iio_attr -u "$PLUTO_URI" -c cf-ad9361-dds-core-lpc voltage1 raw 0 > /dev/null 2>&1 || true

# RX settings
iio_attr -u "$PLUTO_URI" -c ad9361-phy altvoltage0 frequency "$RX_FREQ" > /dev/null
iio_attr -u "$PLUTO_URI" -c ad9361-phy -i voltage0 sampling_frequency "$SAMPLE_RATE" > /dev/null
iio_attr -u "$PLUTO_URI" -c ad9361-phy -i voltage0 rf_bandwidth "$SAMPLE_RATE" > /dev/null 2>&1 || true
iio_attr -u "$PLUTO_URI" -c ad9361-phy -i voltage0 hardwaregain "$RX_GAIN" > /dev/null 2>&1 || true

echo "✓ Radio configured"
echo ""

# =============================================================================
# DISPLAY STATUS
# =============================================================================

if [[ "$TX_FREQ" == "$RX_FREQ" ]]; then
    echo "  Frequency:  $(format_freq $TX_FREQ) (simplex)"
else
    echo "  TX Freq:    $(format_freq $TX_FREQ)"
    echo "  RX Freq:    $(format_freq $RX_FREQ)"
fi
echo "  TX Gain:    $TX_GAIN dB"
echo "  RX Gain:    $RX_GAIN dB"
echo ""
echo "  Interlocutor TX → UDP:$TX_PORT → Pluto TX"
echo "  Pluto RX → UDP:$RX_PORT → Interlocutor RX"
echo ""

# =============================================================================
# START TX PATH
# =============================================================================

# Create named pipe for TX
TX_FIFO="/tmp/opv_tx_$$.fifo"
mkfifo "$TX_FIFO"

# Start iio_writedev reading from FIFO
$IIO_WRITE_CMD -u "$PLUTO_URI" -b "$BUFFER_SIZE" cf-ad9361-dds-core-lpc < "$TX_FIFO" &
IIO_TX_PID=$!

# Start opv-modem TX mode writing to FIFO
MODEM_TX_OPTS="-t -p $TX_PORT"
[[ $VERBOSE -eq 1 ]] && MODEM_TX_OPTS="$MODEM_TX_OPTS -v"
$OPV_MODEM $MODEM_TX_OPTS > "$TX_FIFO" 2>&1 &
TX_PID=$!

# =============================================================================
# START RX PATH
# =============================================================================

# Start RX pipeline: iio_readdev | opv-modem -R
MODEM_RX_OPTS="-R -r $RX_PORT"
[[ $VERBOSE -eq 1 ]] && MODEM_RX_OPTS="$MODEM_RX_OPTS -v"

$IIO_READ_CMD -u "$PLUTO_URI" -b "$BUFFER_SIZE" cf-ad9361-lpc | $OPV_MODEM $MODEM_RX_OPTS 2>&1 &
RX_PID=$!

# =============================================================================
# RUN
# =============================================================================

echo "═══════════════════════════════════════════════════════════════════"
echo "  ✓ Transceiver running - Press Ctrl+C to stop"
echo "═══════════════════════════════════════════════════════════════════"
echo ""

# Wait for any child to exit (or Ctrl+C)
wait

# If we get here without Ctrl+C, clean up
cleanup

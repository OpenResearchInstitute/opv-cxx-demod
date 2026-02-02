#!/bin/bash
#
# opv-pluto-rx.sh - Receive and decode OPV frames via PlutoSDR
#
# Copyright 2026 Open Research Institute, Inc.
# SPDX-License-Identifier: MIT
#
# Usage:
#   ./opv-pluto-rx.sh                          # Receive until Ctrl+C
#   ./opv-pluto-rx.sh -t 10                    # Receive for 10 seconds
#   ./opv-pluto-rx.sh -n 21680000              # Receive 21.68M samples (10 sec)
#   ./opv-pluto-rx.sh -o capture.iq            # Save raw IQ for debugging
#   ./opv-pluto-rx.sh -f 144390000 -g 40       # 144.39 MHz, 40dB gain
#

set -e

# =============================================================================
# DEFAULT CONFIGURATION
# =============================================================================

PLUTO_URI="ip:192.168.2.1"          # PlutoSDR address (USB or IP)
RX_FREQ=435000000                    # RX frequency in Hz (435 MHz)
SAMPLE_RATE=2168000                  # Must match OPV spec (2.168 MSPS)
RX_GAIN=40                           # RX gain in dB (RF front-end only)
BUFFER_SIZE=346880                   # One frame: 2168 bits * 40 samp/bit * 4 bytes/samp
DURATION=0                           # Capture duration in seconds (0 = continuous)
NUM_SAMPLES=0                        # Number of samples (0 = use duration or continuous)
IQ_FILE=""                           # Save raw IQ to file (empty = don't save)
VERBOSE=0                            # Verbose output

# Path to opv-demod (adjust if needed)
OPV_DEMOD="./opv-demod-full"

# =============================================================================
# USAGE
# =============================================================================

usage() {
    cat << USAGE_EOF
Usage: $(basename "$0") [OPTIONS]

Receive and decode OPV frames via PlutoSDR.

Options:
  -f, --frequency HZ     RX frequency in Hz (default: $RX_FREQ)
  -g, --gain DB          RX gain in dB (default: $RX_GAIN)
  -t, --time SECONDS     Capture duration in seconds (default: continuous)
  -n, --samples COUNT    Number of samples to capture (overrides -t)
  -o, --output FILE      Save raw IQ samples to file (for debugging)
  -u, --uri URI          PlutoSDR URI (default: $PLUTO_URI)
  -v, --verbose          Verbose output
  -h, --help             Show this help

Examples:
  $(basename "$0")                             # Receive at 435 MHz until Ctrl+C
  $(basename "$0") -t 10                       # Receive for 10 seconds
  $(basename "$0") -t 30 -o capture.iq         # 30 sec capture, save IQ
  $(basename "$0") -f 144390000 -g 50          # 144.39 MHz with 50dB gain

Decoded frames are written to stdout. Status messages go to stderr.
To save decoded output:
  $(basename "$0") -t 10 > frames.txt 2>status.txt

USAGE_EOF
    exit 1
}

# =============================================================================
# PARSE ARGUMENTS
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        -f|--frequency)
            RX_FREQ="$2"
            shift 2
            ;;
        -g|--gain)
            RX_GAIN="$2"
            shift 2
            ;;
        -t|--time)
            DURATION="$2"
            shift 2
            ;;
        -n|--samples)
            NUM_SAMPLES="$2"
            shift 2
            ;;
        -o|--output)
            IQ_FILE="$2"
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
            echo "Unknown option: $1" >&2
            usage
            ;;
    esac
done

# =============================================================================
# VALIDATION
# =============================================================================

# Find the demodulator binary
if [[ ! -x "$OPV_DEMOD" ]]; then
    if [[ -x "./opv-demod-full" ]]; then
        OPV_DEMOD="./opv-demod-full"
    elif [[ -x "./opv-demod-fixed" ]]; then
        OPV_DEMOD="./opv-demod-fixed"
    elif [[ -x "./apps/opv-demod-full" ]]; then
        OPV_DEMOD="./apps/opv-demod-full"
    elif [[ -x "./apps/opv-demod" ]]; then
        OPV_DEMOD="./apps/opv-demod"
    else
        echo "Error: Cannot find opv-demod-full executable" >&2
        echo "Make sure you have built it and are in the right directory" >&2
        echo "Build with: g++ -std=c++17 -O2 -o opv-demod-full opv-demod-full.cpp -lm" >&2
        exit 1
    fi
fi

# Check for iio_attr
if ! command -v iio_attr &> /dev/null; then
    echo "Error: iio_attr not found. Install libiio:" >&2
    echo "  macOS:  brew install libiio" >&2
    echo "  Ubuntu: sudo apt install libiio-utils" >&2
    exit 1
fi

# Check for iio tools - newer libiio uses iio_rwdev, older uses iio_readdev
if command -v iio_rwdev &> /dev/null; then
    IIO_READ_CMD="iio_rwdev"
    IIO_READ_OPTS=""
elif command -v iio_readdev &> /dev/null; then
    IIO_READ_CMD="iio_readdev"
    IIO_READ_OPTS=""
else
    echo "Error: Neither iio_rwdev nor iio_readdev found. Install libiio:" >&2
    echo "  macOS:  Build from source (https://github.com/analogdevicesinc/libiio)" >&2
    echo "  Ubuntu: sudo apt install libiio-utils" >&2
    exit 1
fi

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

log() {
    if [[ $VERBOSE -eq 1 ]]; then
        echo "[INFO] $*" >&2
    fi
}

error() {
    echo "[ERROR] $*" >&2
}

format_freq() {
    local freq=$1
    if [[ $freq -ge 1000000000 ]]; then
        printf "%.3f GHz" "$(echo "scale=3; $freq/1000000000" | bc)"
    elif [[ $freq -ge 1000000 ]]; then
        printf "%.3f MHz" "$(echo "scale=3; $freq/1000000" | bc)"
    elif [[ $freq -ge 1000 ]]; then
        printf "%.3f kHz" "$(echo "scale=3; $freq/1000" | bc)"
    else
        printf "%d Hz" "$freq"
    fi
}

format_samples() {
    local samples=$1
    if [[ $samples -ge 1000000000 ]]; then
        printf "%.2fG" "$(echo "scale=2; $samples/1000000000" | bc)"
    elif [[ $samples -ge 1000000 ]]; then
        printf "%.2fM" "$(echo "scale=2; $samples/1000000" | bc)"
    elif [[ $samples -ge 1000 ]]; then
        printf "%.2fk" "$(echo "scale=2; $samples/1000" | bc)"
    else
        printf "%d" "$samples"
    fi
}

cleanup() {
    echo "" >&2
    echo "Interrupted. Cleaning up..." >&2
    jobs -p | xargs -r kill 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

# =============================================================================
# CHECK PLUTO CONNECTION
# =============================================================================

echo "==============================================" >&2
echo "OPV PlutoSDR Receiver" >&2
echo "==============================================" >&2
echo "" >&2
echo "Checking PlutoSDR connection at $PLUTO_URI..." >&2

if ! iio_info -u "$PLUTO_URI" &> /dev/null; then
    error "Cannot connect to PlutoSDR at $PLUTO_URI"
    echo "" >&2
    echo "Troubleshooting:" >&2
    echo "  1. Check USB connection" >&2
    echo "  2. Try: iio_info -u usb:" >&2
    echo "  3. Try: iio_info -u ip:192.168.2.1" >&2
    echo "  4. Check if PlutoSDR is powered on" >&2
    exit 1
fi

echo "PlutoSDR connected" >&2

# =============================================================================
# CONFIGURE PLUTO
# =============================================================================

echo "" >&2
echo "Configuring PlutoSDR..." >&2

# Set RX LO frequency (altvoltage0 is RX_LO, altvoltage1 is TX_LO)
log "Setting RX frequency to $(format_freq $RX_FREQ)"
iio_attr -u "$PLUTO_URI" -c ad9361-phy altvoltage0 frequency "$RX_FREQ" > /dev/null
echo "  RX Frequency: $(format_freq $RX_FREQ)" >&2

# Set sample rate on ad9361-phy (input channel for RX)
log "Setting sample rate to $SAMPLE_RATE SPS"
iio_attr -u "$PLUTO_URI" -c ad9361-phy -i voltage0 sampling_frequency "$SAMPLE_RATE" > /dev/null
echo "  Sample Rate:  $SAMPLE_RATE SPS (2.168 MSPS)" >&2

# Set RF bandwidth
log "Setting RF bandwidth to $SAMPLE_RATE Hz"
iio_attr -u "$PLUTO_URI" -c ad9361-phy -i voltage0 rf_bandwidth "$SAMPLE_RATE" > /dev/null 2>&1 || true
echo "  RF Bandwidth: $SAMPLE_RATE Hz" >&2

# Set RX hardware gain
log "Setting RX gain to $RX_GAIN dB"
iio_attr -u "$PLUTO_URI" -c ad9361-phy -i voltage0 hardwaregain "$RX_GAIN" > /dev/null 2>&1 || true
echo "  RX Gain:      $RX_GAIN dB" >&2

echo "" >&2
echo "PlutoSDR configured" >&2

# =============================================================================
# BUILD COMMAND
# =============================================================================

# Calculate number of samples if duration specified
if [[ $NUM_SAMPLES -eq 0 && $DURATION -gt 0 ]]; then
    NUM_SAMPLES=$((DURATION * SAMPLE_RATE))
fi

# Build iio read command
IIO_CMD="$IIO_READ_CMD -u $PLUTO_URI -b $BUFFER_SIZE"

if [[ $NUM_SAMPLES -gt 0 ]]; then
    IIO_CMD="$IIO_CMD -s $NUM_SAMPLES"
    MODE="Timed capture: $(format_samples $NUM_SAMPLES) samples"
    if [[ $DURATION -gt 0 ]]; then
        MODE="$MODE (~${DURATION}s)"
    fi
else
    MODE="Continuous (Ctrl+C to stop)"
fi

IIO_CMD="$IIO_CMD $IIO_READ_OPTS cf-ad9361-lpc"

# =============================================================================
# RECEIVE
# =============================================================================

echo "" >&2
echo "==============================================" >&2
echo "Receiving..." >&2
echo "==============================================" >&2
echo "  Mode:       $MODE" >&2
echo "  Frequency:  $(format_freq $RX_FREQ)" >&2
echo "  Gain:       $RX_GAIN dB" >&2
echo "  Demod:      $OPV_DEMOD" >&2
if [[ -n "$IQ_FILE" ]]; then
    echo "  IQ Output:  $IQ_FILE" >&2
fi
echo "" >&2
echo "Command: $IIO_CMD | $OPV_DEMOD" >&2
echo "" >&2

if [[ $NUM_SAMPLES -gt 0 ]]; then
    echo "Receiving $(format_samples $NUM_SAMPLES) samples..." >&2
else
    echo "Receiving (Ctrl+C to stop)..." >&2
fi
echo "" >&2

# Run the pipeline
if [[ -n "$IQ_FILE" ]]; then
    # Save IQ and decode (tee to file)
    $IIO_CMD | tee "$IQ_FILE" | "$OPV_DEMOD"
    echo "" >&2
    echo "Raw IQ saved to: $IQ_FILE" >&2
    # Get file size (works on both Linux and macOS)
    if [[ -f "$IQ_FILE" ]]; then
        FILESIZE=$(wc -c < "$IQ_FILE" | tr -d ' ')
        FILESAMPLES=$((FILESIZE / 4))
        echo "  Size: $FILESIZE bytes" >&2
        echo "  Samples: $FILESAMPLES" >&2
    fi
else
    # Direct decode
    $IIO_CMD | "$OPV_DEMOD"
fi

echo "" >&2
echo "Reception complete" >&2

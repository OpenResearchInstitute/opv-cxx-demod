#!/bin/bash
#
# opv-pluto-tx.sh - Transmit OPV frames via PlutoSDR
#
# Copyright 2026 Open Research Institute, Inc.
# SPDX-License-Identifier: MIT
#
# Usage:
#   ./opv-pluto-tx.sh -S W5NYV -B 10           # Send 10 BERT frames
#   ./opv-pluto-tx.sh -S W5NYV                 # Voice mode (audio from stdin)
#   ./opv-pluto-tx.sh -S W5NYV -f 435000000    # Use 435 MHz
#

set -e

# =============================================================================
# DEFAULT CONFIGURATION
# =============================================================================

PLUTO_URI="ip:192.168.2.1"          # PlutoSDR address (USB or IP)
TX_FREQ=435000000                    # TX frequency in Hz (435 MHz)
SAMPLE_RATE=2168000                  # Must match opv-mod output (2.168 MSPS)
TX_GAIN=-20                          # TX gain in dB (negative = attenuation)
BUFFER_SIZE=400000                   # IIO buffer size for smooth streaming
CALLSIGN=""                          # Your callsign (required)
BERT_FRAMES=0                        # Number of BERT frames (0 = voice mode)
CONTINUOUS=0                         # Continuous transmission mode
VERBOSE=0                            # Verbose output

# Path to opv-mod (adjust if needed)
OPV_MOD="./opv-mod"

# =============================================================================
# USAGE
# =============================================================================

usage() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

Transmit OPV frames via PlutoSDR.

Required:
  -S, --callsign CALL    Your callsign (e.g., W5NYV)

Options:
  -f, --frequency HZ     TX frequency in Hz (default: $TX_FREQ)
  -g, --gain DB          TX gain in dB, negative for attenuation (default: $TX_GAIN)
  -B, --bert FRAMES      Send BERT frames instead of voice (default: voice mode)
  -c, --continuous       Continuous transmission (loop BERT frames, Ctrl+C to stop)
  -u, --uri URI          PlutoSDR URI (default: $PLUTO_URI)
  -v, --verbose          Verbose output
  -h, --help             Show this help

Examples:
  $(basename "$0") -S W5NYV -B 10                # Send 10 BERT frames at 435 MHz
  $(basename "$0") -S W5NYV -B 10 -c             # Continuous BERT at 435 MHz
  $(basename "$0") -S W5NYV -f 144390000 -B 5    # 5 BERT frames at 144.39 MHz
  $(basename "$0") -S W5NYV -g -10               # Voice mode with -10dB gain

Voice mode reads 48kHz 16-bit mono PCM from stdin:
  arecord -f S16_LE -r 48000 -c 1 | $(basename "$0") -S W5NYV

EOF
    exit 1
}

# =============================================================================
# PARSE ARGUMENTS
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        -S|--callsign)
            CALLSIGN="$2"
            shift 2
            ;;
        -f|--frequency)
            TX_FREQ="$2"
            shift 2
            ;;
        -g|--gain)
            TX_GAIN="$2"
            shift 2
            ;;
        -B|--bert)
            BERT_FRAMES="$2"
            shift 2
            ;;
        -c|--continuous)
            CONTINUOUS=1
            shift
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

if [[ -z "$CALLSIGN" ]]; then
    echo "Error: Callsign is required (-S option)"
    usage
fi

if [[ ! -x "$OPV_MOD" ]]; then
    # Try to find it in common locations
    if [[ -x "./apps/opv-mod" ]]; then
        OPV_MOD="./apps/opv-mod"
    elif [[ -x "../build/apps/opv-mod" ]]; then
        OPV_MOD="../build/apps/opv-mod"
    else
        echo "Error: Cannot find opv-mod executable"
        echo "Make sure you've built it and are in the right directory"
        exit 1
    fi
fi

# Check for iio_attr and iio_writedev
if ! command -v iio_attr &> /dev/null; then
    echo "Error: iio_attr not found. Install libiio:"
    echo "  macOS:  brew install libiio"
    echo "  Ubuntu: sudo apt install libiio-utils"
    exit 1
fi

# Check for iio tools - newer libiio uses iio_rwdev, older uses iio_writedev
if command -v iio_rwdev &> /dev/null; then
    IIO_WRITE_CMD="iio_rwdev"
    IIO_WRITE_OPTS="-w"
elif command -v iio_writedev &> /dev/null; then
    IIO_WRITE_CMD="iio_writedev"
    IIO_WRITE_OPTS=""
else
    echo "Error: Neither iio_rwdev nor iio_writedev found. Install libiio:"
    echo "  macOS:  Build from source (https://github.com/analogdevicesinc/libiio)"
    echo "  Ubuntu: sudo apt install libiio-utils"
    exit 1
fi

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

log() {
    if [[ $VERBOSE -eq 1 ]]; then
        echo "[INFO] $*"
    fi
}

error() {
    echo "[ERROR] $*" >&2
}

# Format frequency for display
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

# =============================================================================
# CHECK PLUTO CONNECTION
# =============================================================================

echo "=============================================="
echo "OPV PlutoSDR Transmitter"
echo "=============================================="
echo ""
echo "Checking PlutoSDR connection at $PLUTO_URI..."

if ! iio_info -u "$PLUTO_URI" &> /dev/null; then
    error "Cannot connect to PlutoSDR at $PLUTO_URI"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check USB connection"
    echo "  2. Try: iio_info -u usb:"
    echo "  3. Try: iio_info -u ip:192.168.2.1"
    echo "  4. Check if PlutoSDR is powered on"
    exit 1
fi

echo "✓ PlutoSDR connected"

# =============================================================================
# CONFIGURE PLUTO
# =============================================================================

echo ""
echo "Configuring PlutoSDR..."

# Set TX LO frequency
log "Setting TX frequency to $(format_freq $TX_FREQ)"
iio_attr -u "$PLUTO_URI" -c ad9361-phy altvoltage1 frequency "$TX_FREQ" > /dev/null
echo "  TX Frequency: $(format_freq $TX_FREQ)"

# Set sample rate on ad9361-phy (output channel for TX)
log "Setting sample rate to $SAMPLE_RATE SPS"
iio_attr -u "$PLUTO_URI" -c ad9361-phy -o voltage0 sampling_frequency "$SAMPLE_RATE" > /dev/null
echo "  Sample Rate:  $SAMPLE_RATE SPS (2.168 MSPS)"

# Set TX gain/attenuation on output channel
# PlutoSDR TX gain range: -89.75 to 0 dB (negative = attenuation)
log "Setting TX gain to $TX_GAIN dB"
iio_attr -u "$PLUTO_URI" -c ad9361-phy -o voltage0 hardwaregain "$TX_GAIN" > /dev/null 2>&1 || true
echo "  TX Gain:      $TX_GAIN dB"

# Disable DDS (we're providing our own samples)
log "Disabling internal DDS"
iio_attr -u "$PLUTO_URI" -c cf-ad9361-dds-core-lpc voltage0 raw 0 > /dev/null 2>&1 || true
iio_attr -u "$PLUTO_URI" -c cf-ad9361-dds-core-lpc voltage1 raw 0 > /dev/null 2>&1 || true

echo ""
echo "✓ PlutoSDR configured"

# =============================================================================
# BUILD COMMAND
# =============================================================================

OPV_CMD="$OPV_MOD -S $CALLSIGN"

if [[ $BERT_FRAMES -gt 0 ]]; then
    OPV_CMD="$OPV_CMD -B $BERT_FRAMES"
    if [[ $CONTINUOUS -eq 1 ]]; then
        OPV_CMD="$OPV_CMD -c"
        MODE="BERT continuous ($BERT_FRAMES frames/batch, Ctrl+C to stop)"
    else
        MODE="BERT ($BERT_FRAMES frames)"
    fi
else
    MODE="Voice (reading from stdin)"
fi

if [[ $VERBOSE -eq 1 ]]; then
    OPV_CMD="$OPV_CMD -v"
fi

IIO_CMD="$IIO_WRITE_CMD -u $PLUTO_URI -b $BUFFER_SIZE $IIO_WRITE_OPTS cf-ad9361-dds-core-lpc"

# =============================================================================
# TRANSMIT
# =============================================================================

echo ""
echo "=============================================="
echo "Transmitting..."
echo "=============================================="
echo "  Callsign:   $CALLSIGN"
echo "  Mode:       $MODE"
echo "  Frequency:  $(format_freq $TX_FREQ)"
echo "  Gain:       $TX_GAIN dB"
echo ""
echo "Command: $OPV_CMD | $IIO_CMD"
echo ""

if [[ $BERT_FRAMES -gt 0 ]]; then
    if [[ $CONTINUOUS -eq 1 ]]; then
        echo "Sending continuous BERT frames (Ctrl+C to stop)..."
    else
        echo "Sending $BERT_FRAMES BERT frames..."
    fi
else
    echo "Voice mode: Pipe audio to stdin (48kHz 16-bit mono PCM)"
    echo "Press Ctrl+C to stop."
fi

echo ""

# Run the pipeline
$OPV_CMD | $IIO_CMD

echo ""
echo "✓ Transmission complete"

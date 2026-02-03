# OPV Modem

**Opulent Voice Protocol** - A digital voice protocol for amateur radio.

Self-contained C++ implementations of the OPV modulator and demodulator, designed for use with PlutoSDR/LibreSDR hardware.

## Quick Start

```bash
# Build
make

# Loopback test
make test

# Receive from PlutoSDR (run from repo root)
scripts/opv-pluto-rx.sh

# Transmit to PlutoSDR (run from repo root)
scripts/opv-pluto-tx.sh -S W5NYV -B 10
```

## Signal Parameters

| Parameter | Value |
|-----------|-------|
| Modulation | MSK (Minimum Shift Keying) |
| Symbol Rate | 54,200 baud |
| Sample Rate | 2,168,000 SPS |
| Frequency Deviation | ±13,550 Hz |
| Samples/Symbol | 40 |

## Frame Structure

| Field | Size | Description |
|-------|------|-------------|
| Sync Word | 24 bits | 0x02B8DB |
| Encoded Payload | 2144 bits | Rate 1/2 convolutionally coded |
| **Total** | **2168 symbols** | ~40 ms per frame |

### Payload (134 bytes before encoding)

| Offset | Size | Field |
|--------|------|-------|
| 0-5 | 6 bytes | Station ID (Base-40 encoded) |
| 6-8 | 3 bytes | Token |
| 9-11 | 3 bytes | Reserved |
| 12-133 | 122 bytes | Voice/Data payload |

## Channel Coding

- **Convolutional Code**: Rate 1/2, K=7
  - G1 = 0x4F (171 octal)
  - G2 = 0x6D (133 octal)
- **Interleaver**: 67×32 block with bit reversal
- **Randomizer**: CCSDS 8-bit LFSR (polynomial x⁸+x⁷+x⁵+x³+1)

## Programs

### opv-mod - Modulator

```
Usage: bin/opv-mod -S CALLSIGN -B FRAMES [-t TOKEN] [-c] [-v]

Options:
  -S CALLSIGN   Station callsign (e.g., W5NYV, KB5MU)
  -B FRAMES     Number of frames to transmit
  -t TOKEN      24-bit token (default: 0xBBAADD)
  -c            Continuous mode (loop forever)
  -v            Verbose output

Output: 16-bit I/Q samples (little-endian) to stdout
```

### opv-demod - Demodulator

```
Usage: bin/opv-demod [-s] < input.iq

Options:
  -s            Streaming mode (real-time from radio)

Input: 16-bit I/Q samples (little-endian) from stdin

Features:
  - Automatic Frequency Control (AFC)
  - Symbol Timing Recovery (early-late gate TED)
  - Soft-decision Viterbi decoding
  - Sync tracking with flywheel
```

## PlutoSDR Scripts

Run these from the repository root directory.

### Receive: opv-pluto-rx.sh

```bash
scripts/opv-pluto-rx.sh                      # Receive until Ctrl+C
scripts/opv-pluto-rx.sh -t 10                # Receive for 10 seconds
scripts/opv-pluto-rx.sh -f 905036750 -g 50   # Custom frequency and gain
scripts/opv-pluto-rx.sh -o capture.iq        # Save raw IQ for debugging
```

### Transmit: opv-pluto-tx.sh

```bash
scripts/opv-pluto-tx.sh -S W5NYV -B 10       # Send 10 BERT frames
scripts/opv-pluto-tx.sh -S W5NYV -B 10 -c    # Continuous BERT (Ctrl+C to stop)
scripts/opv-pluto-tx.sh -S W5NYV -g -10      # Adjust TX gain
```

Requires `iio_attr` and `iio_rwdev` (libiio-utils).

## Directory Structure

```
opv-cxx-demod/
├── Makefile           # Build system
├── README.md          # This file
├── LICENSE            # CERN-OHL-S-2.0
├── bin/               # Built binaries (created by make)
│   ├── opv-mod
│   └── opv-demod
├── src/
│   ├── opv-mod.cpp    # Modulator (self-contained)
│   └── opv-demod.cpp  # Demodulator (self-contained)
├── scripts/
│   ├── opv-pluto-rx.sh
│   └── opv-pluto-tx.sh
└── docs/
    ├── numerology.ipynb    # Design calculations
    └── filter-taps.ipynb   # Filter design
```

## Interoperability

- **Demodulates LibreSDR HDL modum Locutus transmissions
- **Modulation To Be Tested with LibreSDR HDL modem Locutus receiving
- **Sample Format**: 16-bit signed I/Q, little-endian, interleaved

## Building

Requirements:
- C++17 compiler (g++ or clang++)
- No external dependencies (self-contained)

```bash
make        # Build both programs
make test   # Verify loopback works
make clean  # Remove binaries
```

## License

CERN Open Hardware License - Strongly Reciprocal (CERN-OHL-S-2.0)

## Credits

Open Research Institute, Inc.  
https://openresearch.institute

Developed as part of the Phase 4 Ground project for amateur radio digital communications.
## Thanks

Thanks to [Rob Riggs of Mobilinkd LLC](https://github.com/mobilinkd) for the M17 implementation that originally inspired this codebase.

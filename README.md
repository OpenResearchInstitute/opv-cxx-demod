# OPV Modem

**Opulent Voice Protocol** - A digital voice protocol for amateur radio.

Self-contained C++ implementations of the OPV modulator and demodulator, designed for use with PlutoSDR/LibreSDR hardware and Interlocutor.

## Quick Start

```bash
# Build
make

# Loopback test
make test

# Full transceiver with Interlocutor
./opv-pluto.sh -f 435000000 -v
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

### opv-pluto.sh - PlutoSDR Transceiver

Full-duplex transceiver for use with Interlocutor. One script, just like Dialogus.

```bash
./opv-pluto.sh                              # 435 MHz simplex (default)
./opv-pluto.sh -f 905050000                 # 905.05 MHz
./opv-pluto.sh -f 144390000 -v              # 2m band, verbose
./opv-pluto.sh --tx-freq 435000000 --rx-freq 440000000  # Split operation
./opv-pluto.sh -u ip:192.168.3.1            # Custom Pluto IP
```

**Workflow:**
1. Start `./opv-pluto.sh`
2. Start Interlocutor (TX to UDP 57372, listen on UDP 57373)
3. Use Interlocutor to send messages and make calls

**Port Configuration Note:**

Dialogus (running on the Pluto itself) uses port 57372 for both directions because Interlocutor and Dialogus are on different IP addresses. When running opv-modem on the same computer as Interlocutor, we need separate ports to avoid conflicts:

| Direction | Port | Description |
|-----------|------|-------------|
| Interlocutor → opv-modem | 57372 | Frames to transmit |
| opv-modem → Interlocutor | 57373 | Received frames |

Configure Interlocutor with: TX port = 57372, RX port = 57373

**Options:**
| Option | Description |
|--------|-------------|
| `-f, --frequency` | Simplex frequency in Hz |
| `--tx-freq` | TX frequency (split operation) |
| `--rx-freq` | RX frequency (split operation) |
| `--tx-gain` | TX gain in dB (default: -20) |
| `--rx-gain` | RX gain in dB (default: 40) |
| `--tx-port` | UDP port from Interlocutor (default: 57372) |
| `--rx-port` | UDP port to Interlocutor (default: 57373) |
| `-u, --uri` | PlutoSDR URI (default: ip:192.168.2.1) |
| `-v` | Verbose output |

### opv-modem - Modem Server

UDP server for Interlocutor integration. Used internally by opv-pluto.sh.

```
Usage: bin/opv-modem [OPTIONS]

Modes:
  -l          Loopback: UDP → mod → demod → UDP (testing)
  -t          TX mode: UDP → mod → stdout (to PlutoSDR)
  -R          RX mode: stdin → demod → UDP (from PlutoSDR)

Options:
  -p PORT     UDP port to listen on (default: 57372)
  -r PORT     UDP port to send to (default: 57373)
  -c CALL     Rewrite callsign (loopback repeater mode)
  -d PATH     Path to opv-demod (default: ./bin/opv-demod)
  -v          Verbose output
```

### opv-mod - Modulator

```
Usage: bin/opv-mod -S CALLSIGN -B FRAMES [-t TOKEN] [-c] [-v]

Options:
  -S CALLSIGN   Station callsign (e.g., W5NYV, KB5MU)
  -B FRAMES     Number of frames to transmit
  -R            Raw mode (read 134-byte frames from stdin)
  -t TOKEN      24-bit token (default: 0xBBAADD)
  -c            Continuous mode (loop forever)
  -v            Verbose output

Output: 16-bit I/Q samples (little-endian) to stdout
```

### opv-demod - Demodulator

```
Usage: bin/opv-demod [-s] [-r] < input.iq

Options:
  -s            Streaming mode (real-time from radio)
  -r            Raw output (134-byte frames to stdout)

Input: 16-bit I/Q samples (little-endian) from stdin

Features:
  - Automatic Frequency Control (AFC)
  - Symbol Timing Recovery (early-late gate timing error detector)
  - Soft-decision Viterbi decoding
  - Sync tracking with flywheel
```

## Standalone PlutoSDR Scripts

For use without Interlocutor (BERT testing, debugging).

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
├── Makefile              # Build system
├── README.md             # This file
├── LICENSE               # CERN-OHL-S-2.0
├── opv-pluto.sh          # Full transceiver script
├── bin/                  # Built binaries (created by make)
│   ├── opv-mod
│   ├── opv-demod
│   └── opv-modem
├── src/
│   ├── opv-mod.cpp       # Modulator (self-contained)
│   ├── opv-demod.cpp     # Demodulator (self-contained)
│   └── opv-modem.cpp     # Modem server (self-contained)
├── scripts/
│   ├── opv-pluto-rx.sh   # Standalone RX script
│   └── opv-pluto-tx.sh   # Standalone TX script
└── docs/
    ├── numerology.ipynb  # Design calculations
    └── filter-taps.ipynb # Filter design
```

## Interoperability

- **Interlocutor**: Full integration via UDP (text messages, voice calls)
- **Loopback**: Successfully modulates and demodulates to itself
- **Demodulates**: LibreSDR HDL modem Locutus transmissions
- **Modulation**: To Be Tested with LibreSDR HDL modem Locutus receiving
- **Sample Format**: 16-bit signed I/Q, little-endian, interleaved

## Building

Requirements:
- C++17 compiler (g++ or clang++)
- No external dependencies (self-contained)
- libiio-utils for PlutoSDR scripts

```bash
make            # Build all programs
make test       # Verify loopback works
make test-raw   # Test raw frame mode
make test-rx    # Test RX mode UDP output
make clean      # Remove binaries
```

## License

CERN Open Hardware License - Strongly Reciprocal (CERN-OHL-S-2.0)

## Credits

Open Research Institute, Inc.  
https://openresearch.institute

Developed as part of the Phase 4 Ground project for amateur radio digital communications.

## Thanks

Thanks to [Rob Riggs of Mobilinkd LLC](https://github.com/mobilinkd) for the M17 implementation that originally inspired this codebase.

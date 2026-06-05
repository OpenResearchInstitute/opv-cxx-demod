# OPV Modem

**Opulent Voice Protocol** - A digital voice protocol for amateur radio.

Self-contained C++ implementations of the OPV modulator and demodulator, designed for use with PlutoSDR/LibreSDR hardware and Interlocutor.

### Two ways to use this code

1. **As standalone programs** (the original and primary use) — build with `make`
   and run `opv-mod`, `opv-demod`, and `opv-modem` exactly as documented below.
   Nothing here has changed: same commands, same behaviour, same PlutoSDR and
   Interlocutor integration.

2. **As a header-only library** — the receive DSP lives in `src/opv_demod.hpp`,
   so another C++ design can embed the OPV demodulator directly. (For example, the
   Haifuraiya satellite payload runs many demodulator instances across the
   channelizer outputs instead of launching one program per channel.) See
   [Using OPV as a Library](#using-opv-as-a-library-header-only).

Both modes share the *exact same DSP code*, so the over-the-air format is
identical no matter which way you use it.

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
Usage: bin/opv-demod [options] < input.iq

Options:
  -s            Streaming mode (real-time from radio)
  -r            Raw output (134-byte frames to stdout)
  -c            Coherent mode (Costas loop, ~3dB SNR improvement)
  -p <hz>       PLL bandwidth in Hz (default: 50, coherent mode only)
  -a <bw>       AFC bandwidth alpha (default: 0.001)
  -o <hz>       Initial frequency offset in Hz (streaming mode)
  -q            Quiet mode (suppress all stderr output)

Input: 16-bit I/Q samples (little-endian) from stdin

Features:
  - Automatic Frequency Control (AFC)
  - Symbol Timing Recovery (early-late gate timing error detector)
  - Soft-decision Viterbi decoding
  - Sync tracking with flywheel
  - Optional coherent demodulation via 2nd-order Costas loop
```

**Minimum burst length:** The sync state machine requires two frames to acquire
lock (HUNTING → VERIFYING → LOCKED), so the first frame of any transmission is
always consumed by acquisition. A burst of N frames will produce N-1 decoded
frames at the receiver. For PTT-style operation, transmit at least 3 frames to
guarantee one decoded frame at the far end. The preamble frames sent by
`opv-modem` and `opv-pluto.sh` exist partly to allow the demodulator to acquire
symbol timing before the first data frame arrives, but frame sync still requires
two data frames to confirm.

**Coherent mode note:** The `-c` flag enables a 2nd-order Costas loop for
continuous carrier phase tracking, yielding a theoretical 3 dB SNR improvement
over non-coherent detection. In synthetic loopback tests both modes perform
identically due to the steep FEC waterfall masking the coherent gain. The
advantage becomes meaningful under real hardware conditions — oscillator drift,
phase noise, and dynamic Doppler on a live satellite pass. Use
`make test-doppler FREQ_MHZ=<band>` to characterize Doppler performance for
your operating frequency.

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

## Using OPV as a Library (header-only)

The OPV *receive* DSP — the MSK demodulators, symbol-lock detector, sync tracker,
Viterbi decoder, and frame decoder — lives in a single header, `src/opv_demod.hpp`.
The `opv-demod` program is now a thin shell that `#include`s that header and adds
the command-line/streaming harness around it. **The program behaves exactly as it
did before** (verified byte-for-byte against the previous single-file version);
the header simply makes the same DSP reusable by other designs.

Header-only means there is nothing extra to build or link — you include one file.
This keeps the repo's self-contained, dependency-free character intact.

### Adding it to your project

As a git submodule:

```bash
git submodule add https://github.com/OpenResearchInstitute/opv-cxx-demod.git extern/opv
```

Then put `src/` on your include path and include the header:

```cpp
#include "opv_demod.hpp"   // compile with -Iextern/opv/src
```

### The API

The header exposes the OPV receive chain as classes you instantiate and drive.
One set of these objects represents one receive channel; a multi-channel design
holds many independent instances and feeds each one blocks of complex baseband.

| Class | Role | Key methods / accessors |
|-------|------|--------------------------|
| **`ChannelReceiver`** | **High-level: one channel's full receive chain (demod + symbol lock + sync + FEC). Feed it IQ blocks, get decoded frames.** | **`process(samples, n, on_frame)`, `process(samples, n) → vector<Frame>`, `set_freq_offset()`, `estimate_offset()`, `sync_state()`, `total_symbols()`** |
| `MSKDemodulatorAFC` | Non-coherent MSK demod with AFC + early-late timing recovery | `demodulate(samples, n, soft_out)`, `set_freq_offset()`, `get_freq_offset()`, `set_tracking_enabled()`, `set_afc_bandwidth()` |
| `CoherentMSKDemodulator` | Coherent (Costas-loop) variant, ~3 dB SNR gain | same `demodulate()` shape |
| `SymbolLockDetector` | Gates frame-sync search until symbol timing converges | — |
| `SyncTracker` | Frame sync state machine (HUNTING → VERIFYING → LOCKED) | `get_state()` |
| `ViterbiDecoder` / `FrameDecoder` | Soft-decision FEC + frame extraction | — |
| `decode_base40()` | Decode a Base-40 Station ID field | free function |

Most consumers want **`ChannelReceiver`** — it owns the whole chain and hands
back decoded frames. The lower-level classes are exposed for hosts that need to
wire a custom orchestration.

The sample type is `sample_t` (`std::complex<double>`); IQ blocks are passed as a
`const sample_t*` plus length, and soft symbol decisions come back as a
`std::vector<double>`.

### Minimal example

A complete, compilable example lives in [`examples/use_as_library.cpp`](examples/use_as_library.cpp):

```cpp
#include "opv_demod.hpp"

ChannelReceiver rx;                 // one instance per channel
rx.set_freq_offset(0.0);           // or rx.estimate_offset(block, n) on first block

std::vector<sample_t> block(/* IQ samples */);
rx.process(block.data(), block.size(),
           [](const ChannelReceiver::Frame& f) {
               // delivered the instant a frame completes:
               // f.bytes (134 B), f.metric (0 = perfect), f.sync_quality
           });
```

Build it:

```bash
g++ -std=c++17 -O3 -Isrc examples/use_as_library.cpp -o use_as_library
```

### Note on orchestration

`ChannelReceiver` *is* the orchestration: it owns the demod → symbol lock → sync
tracker → Viterbi → frame decoder chain and the streaming bookkeeping, and emits
each decoded frame through your callback the moment it completes (or into a
`vector` via the convenience overload). A multi-channel host such as dogu holds
one `ChannelReceiver` per channel and schedules blocks across them; the I/O
source and cross-channel scheduling stay host-specific, while the per-channel
chain is shared.

Crucially, the standalone `opv-demod` program's streaming mode is built on the
very same `ChannelReceiver` — so the loopback, coherent, and Doppler tests in
this repo exercise the exact code a host reuses. (The callback form delivers
frames inline, byte-for-byte preserving the original streaming output.)

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
│   ├── opv-mod.cpp       # Modulator program (self-contained)
│   ├── opv-demod.cpp     # Demodulator program — thin shell over opv_demod.hpp
│   ├── opv_demod.hpp     # Demodulator DSP core (header-only library)
│   └── opv-modem.cpp     # Modem server (self-contained)
├── examples/
│   └── use_as_library.cpp  # Minimal header-only library usage example
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
make            # Build all programs for the host (x86)
make TARGET=pluto   # Cross-compile for PlutoSDR  (ARMv7-A Cortex-A9 + NEON)
make TARGET=a53     # Cross-compile for Haifuraiya (ZCU102 A53, aarch64)
make test       # Verify loopback works
make test-raw   # Test raw frame mode
make test-rx    # Test RX mode UDP output
make test-coherent                             # Verify coherent mode decodes as many frames as non-coherent
make test-coherent-compare                     # Compare coherent vs non-coherent, static offset + noise
make test-coherent-compare SNR=-5 OFFSET=300  # Custom conditions
make test-doppler                              # LEO Doppler stress test at 905 MHz (default)
make test-doppler FREQ_MHZ=433                # 70cm band (±11.3 kHz swing, 220 Hz/sec)
make test-doppler FREQ_MHZ=2400               # 2.4 GHz (±62.4 kHz swing, 1200 Hz/sec)
make test-doppler FREQ_MHZ=5000               # 5 GHz uplink (±130 kHz swing, 2535 Hz/sec)
make clean      # Remove binaries
```

Doppler rate scales with carrier frequency (`f · v²/c·h` at zenith for 400 km LEO).
LEO is the worst case: HEO only reaches LEO rates briefly at perigee, GEO drifts
a few Hz/sec. Note that synthetic Doppler tests may show equal coherent/non-coherent
performance due to the FEC waterfall masking the 3 dB coherent gain — true
validation of the coherent advantage requires over-the-air testing on a live
satellite pass with real oscillator drift.

## License

CERN Open Hardware License - Strongly Reciprocal (CERN-OHL-S-2.0)

## Credits

Open Research Institute, Inc.  
https://openresearch.institute

Developed as part of the Phase 4 Ground project for amateur radio digital communications.

## Thanks

Thanks to [Rob Riggs of Mobilinkd LLC](https://github.com/mobilinkd) for the M17 implementation that originally inspired this codebase.

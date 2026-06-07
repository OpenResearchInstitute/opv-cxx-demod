# OPV Modem

**Opulent Voice Protocol** — a digital voice protocol for amateur radio.

Self-contained C++ implementations of the OPV modulator and demodulator. The
same DSP runs three ways, all sharing the *exact* over-the-air format:

1. **Standalone programs on PlutoSDR/LibreSDR** (the original and primary use) —
   build with `make`, run `opv-mod` / `opv-demod` / `opv-modem`. See
   [Quick Start](#quick-start).
2. **As a header-only library / git submodule** — the receive DSP lives in one
   header, `src/opv_demod.hpp`, so another C++ design can embed it. See
   [Using OPV as a Library](#using-opv-as-a-library-header-only).
3. **Channelized, on the Haifuraiya satellite payload** — many demodulator
   instances run across the channelizer's per-channel outputs at the native
   channel rate, on the ZCU102 A53 cores under dogu. See
   [Running it in Haifuraiya (dogu)](#running-it-in-haifuraiya-dogu).

If you are bringing this to **another system or protocol**, the section
[Adapting to other waveforms](#adapting-to-other-waveforms-the-levers) lays out
the three layers you change — waveform, framing, FEC — and what you keep.

---

## Quick Start

```bash
make                                # build all binaries into bin/
make test-coherent                  # coherent loopback self-test
./opv-pluto.sh -f 435000000 -v      # full transceiver with Interlocutor (PlutoSDR)
```

`make` now builds four programs: `opv-mod`, `opv-demod`, `opv-modem`, and
`opv-resample`.

## Signal Parameters

| Parameter | Value | Lever |
|-----------|-------|-------|
| Modulation | MSK (Minimum Shift Keying, h = 0.5) | waveform |
| Symbol Rate | 54,200 baud | waveform |
| Frequency Deviation | ±13,550 Hz | waveform |
| Native Sample Rate | 2,168,000 SPS | waveform |
| Samples/Symbol (native) | 40 | waveform (see `-R` / `set_nominal_sps()` to run at any rate) |

## Frame Structure

| Field | Size | Description | Lever |
|-------|------|-------------|-------|
| Sync Word | 24 bits | 0x02B8DB | framing |
| Encoded Payload | 2144 bits | Rate 1/2 convolutionally coded | FEC |
| **Total** | **2168 symbols** | ~40 ms per frame | framing |

### Payload (134 bytes before encoding)

| Offset | Size | Field |
|--------|------|-------|
| 0–5 | 6 bytes | Station ID (Base-40 encoded) |
| 6–8 | 3 bytes | Token |
| 9–11 | 3 bytes | Reserved |
| 12–133 | 122 bytes | Voice/Data payload |

## Channel Coding

- **Convolutional Code**: Rate 1/2, K=7 (G1 = 0x4F, G2 = 0x6D)
- **Interleaver**: 67×32 block with bit reversal
- **Randomizer**: CCSDS 8-bit LFSR (x⁸+x⁷+x⁵+x³+1)

---

## Programs

### opv-pluto.sh — PlutoSDR Transceiver

Full-duplex transceiver for use with Interlocutor (one script, like Dialogus).

```bash
./opv-pluto.sh                              # 435 MHz simplex (default)
./opv-pluto.sh -f 905050000                 # 905.05 MHz
./opv-pluto.sh --tx-freq 435000000 --rx-freq 440000000  # split operation
./opv-pluto.sh -u ip:192.168.3.1            # custom Pluto IP
```

| Option | Description |
|--------|-------------|
| `-f, --frequency` | Simplex frequency in Hz |
| `--tx-freq` / `--rx-freq` | Split operation |
| `--tx-gain` / `--rx-gain` | dB (defaults −20 / 40) |
| `--tx-port` / `--rx-port` | UDP ports to/from Interlocutor (57372 / 57373) |
| `-u, --uri` | PlutoSDR URI (default ip:192.168.2.1) |
| `-v` | Verbose |

Interlocutor config: TX port = 57372, RX port = 57373.

### opv-modem — Modem Server

UDP server for Interlocutor (used internally by opv-pluto.sh).

```
-l   Loopback: UDP → mod → demod → UDP        -p PORT  listen port (57372)
-t   TX:       UDP → mod → stdout (to radio)  -r PORT  send port   (57373)
-R   RX:       stdin (from radio) → demod → UDP
```

### opv-mod — Modulator

```
Usage: bin/opv-mod -S CALLSIGN -B FRAMES [-P] [-t TOKEN] [-c] [-v]
  -S CALLSIGN   Station callsign           -P  prepend one preamble frame
  -B FRAMES     Number of BERT frames      -t  24-bit token (default 0xBBAADD)
  -R            Raw mode (134-byte frames from stdin)
  -c            Continuous (loop forever)  -v  verbose
Output: 16-bit I/Q (little-endian, interleaved) to stdout, at 2.168 Msps.
```

### opv-demod — Demodulator

```
Usage: bin/opv-demod [options] < input.iq
  -c            Coherent mode (Costas + 2T detector; ~6 dB through the FEC)
  -R <rate>     Channelized input rate in Hz -> fractional-timing front-end
                (e.g. -R 625000 for a 625 ksps channel; coherent only)
  -s            Streaming mode (real-time from radio; non-coherent today)
  -r            Raw output (134-byte frames to stdout)
  -p <hz>       PLL bandwidth (default 50, coherent only)
  -a <bw>       AFC bandwidth alpha (default 0.001, non-coherent)
  -o <hz>       Initial frequency offset (streaming)
  -q            Quiet
Input: 16-bit I/Q (little-endian, interleaved) from stdin.
```

Without `-R`, the coherent path processes at the native 2.168 Msps (40 sps,
fixed integer windows). With `-R <rate>` it sets the nominal samples/symbol to
`rate / 54200` and runs the fractional-timing front-end — this is how a single
demod handles whatever rate the radio actually delivers (e.g. a channelizer's
~11.53 sps) without resampling the receive path.

### opv-resample — Fractional-rate I/Q resampler

A standalone Unix pipe filter (its own program — not part of the modulator).
opv-mod is unchanged; you insert opv-resample between stages when you need a
different sample rate.

```
Usage: bin/opv-resample [Fin Fout [K]]      (defaults 2168000 625000 32)
  Fin, Fout   input/output sample rates in Hz
  K           windowed-sinc half-width in taps (default 32; raise for sharper
              anti-aliasing on large decimation ratios)
Reads/writes 16-bit I/Q (little-endian, interleaved) on stdin/stdout.
```

Two uses: (1) make channel-rate frames for a simulated loopback, and (2) as the
SIC reference-reconstruction step (bring a reconstructed signal to the working
rate before subtraction).

---

## Coherent demodulation (status and how to use it)

Coherent detection (the `-c` flag) uses a decision-switched Costas loop feeding
the Massey optimum 2T detector, with soft-differential decode. Measured through
the real rate-1/2 K=7 Viterbi/FEC in batch A/B testing, it gives roughly **6 dB**
over non-coherent — two stacked ~3 dB wins, coherent-vs-non-coherent and
antipodal-(2T)-vs-orthogonal-(per-symbol). The earlier "≈3 dB, masked in
loopback" note described a since-replaced simple per-symbol detector; the 2T
detector's gain is real and shows in batch comparison at the operating SNR.

Two honest caveats:

- **Batch vs live.** The `-c` coherent path is currently a *batch* (whole-capture)
  decode, including `-R` channelized operation. The *streaming* path (`-s`) is
  non-coherent today. Live coherent — running the coherent front-end causally
  inside `ChannelReceiver` — is in progress; the front-end is now causal-ready
  (forward timing loop, forward Costas, resolve-once parity/polarity), which was
  the blocker. The intent is for coherent to become the default and non-coherent
  to be retired once live coherent is proven on the bench.
- **Loop tuning is a bench activity.** The fractional timing loop functions in
  simulation and is bounded (anti-windup), but its bandwidth and lock thresholds
  are finalized against a real link, not synthetic drift.

### Channel-rate loopback (simulated)

Real FEC frames, resampled to a 625 ksps channel, decoded by the coherent
fractional front-end at ~11.53 samples/symbol:

```bash
opv-mod -S W5NYV -P -B 20 | opv-resample 2168000 625000 | opv-demod -c -R 625000
```

The same pipe at the native rate is just `opv-mod ... | opv-demod -c` (no resampler,
no `-R`).

---

## Adapting to other waveforms (the levers)

The receive chain is deliberately split so the **demodulator is protocol-agnostic
and reusable**. It contains no sync word and no frame knowledge — it turns samples
into soft symbols, emitting *both* differential-parity interpretations. Everything
protocol-specific lives downstream. To retarget another system you change one or
more of three layers and keep the rest:

**1. Waveform layer** — `SYMBOL_RATE`, `FREQ_DEV` (h = 0.5 MSK), and the running
sample rate. The rate is not baked in: `CoherentMSKDemodulator::set_nominal_sps()`
(or `-R` on the program) lets one demod run at any samples/symbol, integer or not,
via fractional timing. Change these for a different baud or deviation.

**2. Framing layer** — `SYNC_WORD`, `SYNC_BITS`, `FRAME_SYMBOLS`, and
`SyncTracker`. This is where sync detection and the four-fold parity/polarity
resolution live (`SyncTracker::resolve()`), entirely outside the demod. Swap the
sync correlator for a different preamble/framing and the demod is untouched.

**3. FEC layer** — the convolutional code, interleaver, and randomizer in
`FrameDecoder` (and the matching encoder in `opv-mod`). Swap these for a different
code and keep the demod and sync layers.

The boundary is enforced by the data flow: `demod → (dec0, dec1) →
SyncTracker::resolve → SyncTracker → FrameDecoder`. The demod never references the
sync word; the sync correlator owns the protocol.

---

## Using OPV as a Library (header-only)

The receive DSP — demodulators, symbol-lock detector, sync tracker, Viterbi, and
frame decoder — is a single header, `src/opv_demod.hpp`. Header-only: include one
file, nothing to link.

```bash
git submodule add https://github.com/OpenResearchInstitute/opv-cxx-demod.git extern/opv
# then compile your code with -Iextern/opv/src
```
```cpp
#include "opv_demod.hpp"
```

### The classes you drive

| Class | Role | Key methods |
|-------|------|-------------|
| `ChannelReceiver` | High-level **non-coherent** receive chain for one channel (demod + lock + sync + FEC). Feed IQ blocks, get frames. | `process(samples, n, on_frame)`, `set_freq_offset()`, `estimate_offset()` |
| `MSKDemodulatorAFC` | Non-coherent MSK demod, AFC + early-late timing | `demodulate(samples, n, soft_out)` |
| `CoherentMSKDemodulator` | Coherent (Costas + 2T) front-end, protocol-agnostic | `batch_correlations()` (native rate) / `track_correlations()` (fractional, set via `set_nominal_sps()`), then `combine(Y1,Y2,dec0,dec1)` → both parity streams |
| `SyncTracker` | Frame sync + parity/polarity resolution | `resolve(dec0,dec1)` → resolved soft; `process(soft, idx)`; `get_state()` |
| `ViterbiDecoder` / `FrameDecoder` | Soft-decision FEC + frame extraction | `decode(soft, out)` |
| `decode_base40()` | Decode a Base-40 Station ID | free function |

`sample_t` is `std::complex<double>`; IQ blocks are `const sample_t*` + length;
soft symbols come back as `std::vector<double>`.

**Coherent in a host:** instantiate `CoherentMSKDemodulator`, run
`track_correlations()` (or `batch_correlations()`), `combine()` to get the two
parity streams, `SyncTracker::resolve()` to pick parity/polarity, then drive
`SyncTracker::process()` and `FrameDecoder::decode()`. (This is exactly what the
program's `-c` path does — see `src/opv-demod.cpp`.) `ChannelReceiver` itself is
the non-coherent live chain today; the coherent live wrapper is the in-progress
work noted above.

A minimal non-coherent example is in
[`examples/use_as_library.cpp`](examples/use_as_library.cpp).

---

## Running it in Haifuraiya (dogu)

Haifuraiya's receiver is a 64-channel polyphase channelizer on a ZCU102. Each
channel can carry an Opulent Voice signal, and each needs its own demod+decode at
the **per-channel rate** — about **625 ksps**, i.e. ~11.53 samples/symbol (54200
baud does not divide the radio's 2.168 Msps cleanly, which is why the demod runs
fractional timing rather than resampling the receive path). One demod instance per
channel; up to 64 in parallel across the four A53 cores.

**Build for the A53:**

```bash
make TARGET=a53     # aarch64, cortex-a53
# PetaLinux SDK:  make TARGET=a53 CXX=aarch64-xilinx-linux-g++ SYSROOT=<sdk-sysroot>
```

**As a submodule in dogu:** add this repo as a submodule, put `src/` on the
include path, and hold one receiver per channel. Set the channel rate once:

```cpp
#include "opv_demod.hpp"
CoherentMSKDemodulator demod;
demod.set_nominal_sps(625000.0 / 54200.0);   // ~11.53 — the only rate lever
// per block: track_correlations -> combine -> SyncTracker::resolve -> SyncTracker -> FrameDecoder
```

opv-cxx-demod is dependency-light (libstdc++/libm/libc; complex baseband in, soft
symbols out — no Xilinx/ADI deps), so it drops into the dogu build the way dogu's
own artifacts do.

**dogu side (the radio).** dogu brings the channelizer up and exposes the DMA
path from the ADRV9002:

```sh
/home/root/bring-up.sh tes_0231_Haifuraiya_FDD_LVDS_20Msps_10MHz   # profile load, LO retune, calibrations, channelizer enable
/home/root/dma_listen -n 4096                                      # ARM userspace <-> channelizer DMA
```

`dma_listen` reads the channelizer DMA as 16-bit I/Q. Routing a given channel's
~625 ksps stream into a demod instance (program via `-R 625000`, or library via
`set_nominal_sps`) is the dogu-side integration and is finalized on the bench
against the live channelizer output. For a quick **simulated** check of the exact
channel-rate decode the payload will do, use the loopback pipe shown above.

---

## Directory Structure

```
opv-cxx-demod/
├── Makefile              # builds opv-mod, opv-demod, opv-modem, opv-resample
├── README.md
├── LICENSE               # CERN-OHL-S-2.0
├── opv-pluto.sh
├── bin/                  # built binaries (make)
├── src/
│   ├── opv-mod.cpp       # modulator (self-contained, unchanged)
│   ├── opv-demod.cpp     # demodulator program — thin shell over opv_demod.hpp
│   ├── opv_demod.hpp     # receive DSP core (header-only library)
│   ├── opv-modem.cpp     # modem server (self-contained)
│   └── opv-resample.cpp  # fractional-rate I/Q resampler (pipe filter)
├── examples/use_as_library.cpp
├── scripts/              # opv-pluto-rx.sh, opv-pluto-tx.sh
└── docs/
```

## Building

```bash
make                # host (x86), plus the test suite
make TARGET=pluto   # PlutoSDR   (ARMv7-A Cortex-A9 + NEON)
make TARGET=a53     # Haifuraiya (ZCU102 A53, aarch64)
make test           # loopback
make test-coherent  # coherent decodes as many frames as non-coherent
make clean
```

Requirements: a C++17 compiler, no external dependencies. The PlutoSDR scripts
need libiio-utils.

## Interoperability

- **Interlocutor**: full UDP integration (text, voice).
- **Demodulates**: LibreSDR HDL modem Locutus transmissions.
- **Sample format**: 16-bit signed I/Q, little-endian, interleaved.

## License

CERN Open Hardware License — Strongly Reciprocal (CERN-OHL-S-2.0)

## Credits

Open Research Institute, Inc. — https://openresearch.institute
Developed as part of the Phase 4 Ground project for amateur radio digital communications.

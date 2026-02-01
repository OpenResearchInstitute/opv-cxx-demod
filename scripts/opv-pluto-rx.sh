# OPV Software Modem

Software implementation of the Opulent Voice Protocol modulator and demodulator, designed to work with PlutoSDR and match the HDL reference implementation.

## Status

**Software loopback verified working with 0 bit errors.**

```bash
./opv-mod-hdl -S W5NYV -B 4 | ./opv-demod-full
# Result: 4 frames decoded, callsign "W5NYV" recovered correctly
```

## Files

### Core Components

| File | Description |
|------|-------------|
| `opv-mod-hdl.cpp` | Transmitter - HDL-accurate parallel-tone MSK modulator |
| `opv-demod-full.cpp` | Receiver - Full demodulator with Viterbi decoder |
| `opv-pluto-tx.sh` | PlutoSDR transmit script |
| `opv-pluto-rx.sh` | PlutoSDR receive script |
| `Makefile` | Build system |

### Legacy/Alternative Files

These files exist from earlier development and may be useful for comparison or debugging:

| File | Notes |
|------|-------|
| `opv-mod-fresh.cpp` | Earlier modulator version |
| `opv-mod-cpfsk.cpp` | Simple CPFSK modulator (single NCO) |

## Build

```bash
make
```

Or build individually:

```bash
g++ -std=c++17 -O2 -o opv-mod-hdl opv-mod-hdl.cpp -lm
g++ -std=c++17 -O2 -o opv-demod-full opv-demod-full.cpp -lm
```

## Usage

### Software Loopback Test

```bash
./opv-mod-hdl -S W5NYV -B 4 | ./opv-demod-full
```

### PlutoSDR Transmit

```bash
# Send 10 BERT frames at 435 MHz
./opv-pluto-tx.sh -S W5NYV -B 10

# Continuous transmission
./opv-pluto-tx.sh -S W5NYV -B 10 -c

# Different frequency
./opv-pluto-tx.sh -S W5NYV -B 5 -f 144390000
```

### PlutoSDR Receive

```bash
# Receive for 10 seconds at 435 MHz
./opv-pluto-rx.sh -t 10

# Continuous receive (Ctrl+C to stop)
./opv-pluto-rx.sh

# Save raw IQ for debugging
./opv-pluto-rx.sh -t 10 -o capture.iq

# Different frequency and gain
./opv-pluto-rx.sh -f 144390000 -g 50
```

## Signal Parameters

| Parameter | Value |
|-----------|-------|
| Sample Rate | 2.168 MSPS |
| Symbol Rate | 54.2 kbaud |
| Modulation | MSK (parallel-tone) |
| Frequency Deviation | ±13.55 kHz |
| Samples per Symbol | 40 |
| IQ Format | int16 I, int16 Q (4 bytes/sample) |

## Frame Format

| Field | Bytes | Description |
|-------|-------|-------------|
| Sync Word | 3 (24 bits) | 0x02B8DB |
| Payload | 134 | Encoded + interleaved data |

### Payload Structure (before encoding)

| Offset | Size | Field |
|--------|------|-------|
| 0-5 | 6 | Callsign (ASCII, null-padded) |
| 6-8 | 3 | Frame counter (24-bit big-endian) |
| 9-11 | 3 | Reserved |
| 12-133 | 122 | Data (BERT pattern in test mode) |

## Signal Processing Chain

### Transmit (opv-mod-hdl)

```
Payload (134 bytes)
    │
    ▼
LFSR Randomizer (seed 0xFF, poly x^8+x^7+x^5+x^3+1)
    │
    ▼
Convolutional Encoder (K=7, r=1/2, G1=0x79, G2=0x5B)
    │
    ▼
Bit Interleaver (67×32 matrix)
    │
    ▼
MSK Modulator (parallel-tone, ±13.55 kHz)
    │
    ▼
IQ Samples (int16)
```

### Receive (opv-demod-full)

```
IQ Samples (int16)
    │
    ▼
MSK Demodulator (dual correlators + CCLK compensation)
    │
    ▼
Differential Decoder
    │
    ▼
Sync Word Detection (0x02B8DB)
    │
    ▼
Bit Deinterleaver (67×32 matrix)
    │
    ▼
Viterbi Decoder (K=7, r=1/2)
    │
    ▼
LFSR Derandomizer
    │
    ▼
Payload (134 bytes)
```

## Technical Notes

### Viterbi Decoder

The Viterbi decoder state transitions must match the encoder exactly:

```cpp
// Encoder state update:
sr = ((sr << 1) | input_bit) & 0x3F;

// Decoder must use same transition:
next_state[sr][inp] = ((sr << 1) | inp) & 0x3F;
```

The decoder uses "best ending state" traceback (not state 0) since the protocol does not include tail bits.

### Byte/Bit Ordering

- Bytes are processed in reverse order (byte 133 first, byte 0 last)
- Bits within each byte are MSB first (bit 7 first, bit 0 last)
- Encoded bits are transmitted in linear order after interleaving

### Buffer Sizes

One frame = 2168 bits × 40 samples/bit × 4 bytes/sample = **346,880 bytes**

This is used as the IIO buffer size in both TX and RX scripts.

## Demodulator Output

The demodulator outputs decoded frames to stdout with formatted display:

```
┌─────────────────────────────────────────────────────────
│ FRAME 1
├─────────────────────────────────────────────────────────
│ Callsign: W5NYV
│ Counter:  0
│
│ Hex dump:
│   000: 57 35 4e 59 56 00 00 00 00 00 00 00 00 01 02 03  │W5NYV...........│
│   010: 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13  │................│
│   ...
└─────────────────────────────────────────────────────────
```

Status messages go to stderr, allowing redirection:

```bash
./opv-pluto-rx.sh -t 10 > frames.txt 2>status.txt
```

## Hardware Testing

### Prerequisites

- PlutoSDR with libiio tools installed
- Two PlutoSDRs for over-the-air testing, or cabled loopback with attenuation

### Test Procedure

1. **Build the tools:**
   ```bash
   make
   ```

2. **Verify software loopback:**
   ```bash
   ./opv-mod-hdl -S YOURCALL -B 4 | ./opv-demod-full
   ```

3. **Test PlutoSDR TX (terminal 1):**
   ```bash
   ./opv-pluto-tx.sh -S YOURCALL -B 10 -f 435000000
   ```

4. **Test PlutoSDR RX (terminal 2, different Pluto or cabled):**
   ```bash
   ./opv-pluto-rx.sh -t 15 -f 435000000 -o capture.iq
   ```

5. **If decode fails, analyze saved IQ:**
   ```bash
   ./opv-demod-full < capture.iq
   ```

## Troubleshooting

### No sync detected
- Check frequency match between TX and RX
- Verify signal level (adjust TX/RX gain)
- Confirm sample rate is exactly 2.168 MSPS

### Sync found but decode fails
- Save IQ with `-o capture.iq` and analyze offline
- Check for frequency offset (demod has no AFC)
- Verify frame counter is incrementing (rules out stuck TX)

### PlutoSDR connection issues
```bash
# Check connection
iio_info -u ip:192.168.2.1

# Try USB instead of network
iio_info -u usb:
```



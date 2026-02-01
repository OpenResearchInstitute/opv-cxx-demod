# OPV Fresh Transmitter

Clean reimplementation to test with your existing `opv-pluto-tx.sh` workflow.

## Two Versions to Test

| Executable | Modulation | Description |
|------------|------------|-------------|
| `opv-mod-fresh` | Parallel-Tone MSK | Matches HDL architecture (two NCOs, weighted sum) |
| `opv-mod-cpfsk` | Simple CPFSK | Single NCO switching frequencies |

## Build

```bash
make
```

## Test

These are drop-in replacements for `opv-mod-hdl-msk`. Edit your shell script or symlink:

```bash
# Option 1: Edit opv-pluto-tx.sh to point to new executable
OPV_MOD="./opv-mod-fresh"   # or ./opv-mod-cpfsk

# Option 2: Symlink
ln -sf opv-mod-fresh opv-mod-hdl-msk

# Option 3: Direct test
./opv-mod-fresh -S W5NYV -B 4 -v 2>/dev/null | iio_writedev -u ip:192.168.2.1 -b 346880 cf-ad9361-dds-core-lpc
```

## CLI Interface

Same as original `opv-mod-hdl-msk`:

```
./opv-mod-fresh -S CALLSIGN -B FRAMES [-c] [-v]

  -S CALL    Your callsign (required)
  -B FRAMES  Number of BERT frames (required)
  -c         Continuous mode (loop forever)
  -v         Verbose output to stderr
```

## What's Different

**Encoding chain** (same in both versions):
- LFSR randomizer: seed 0xFF, polynomial x^8+x^7+x^5+x^3+1
- Convolutional encoder: K=7, G1=0x4F, G2=0x6D
- Forward byte order (0→133), NO output reversal
- 67×32 bit interleaver

**Modulation difference:**

| Version | How it works |
|---------|--------------|
| `opv-mod-fresh` | Two NCOs at ±13.55 kHz, weighted by d_s1/d_s2, summed |
| `opv-mod-cpfsk` | Single NCO, frequency switches between ±13.55 kHz |

Both produce valid MSK spectrum, but the parallel-tone version matches the HDL's internal structure.

## Expected Results

Try both versions. Report:
1. How many sync hits out of how many frames?
2. Does it get worse with more frames?
3. What does decoded data look like (your actual payload, or LFSR sequence)?

If NEITHER works, the bug might be elsewhere (sample rate mismatch, RF issue, etc.)

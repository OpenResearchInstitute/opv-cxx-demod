# OPV Demod/Decode Split — Fabric ↔ A53 Interface (v0.1, draft)

ZCU102 deployment. The PL (fabric) demodulates; the A53 (PS) decodes. This note
fixes the seam so both sides can be built independently.

## Why this split

The 40 Msps of channelized samples (64 × 625 ksps) already live in fabric, where
the channelizer and power detector run. Demodulating in fabric keeps that
high-rate stream local and sends only the low-rate soft bits to the A53
(~108 kbit/s/channel × 64 ≈ 7 Mbit/s). Demodulating on the A53 would force
~160 MB/s of IQ across the PS/PL boundary — the wrong direction.

Measured per-channel A53 cost decides the partition:
- Demod front-end alone: ~78 ms/s/channel. For 64 channels on 4 cores the budget
  is 62.5 ms/s/channel, so demod-on-A53 busts the budget before any decode —
  it MUST be in fabric.
- Decode-only on A53 (`decode_bench`, measured): 1662 µs/decode → 24 channels/core
  → 96 across 4 cores. 64 channels fits at ~78 % utilization (decode 66 % + sync
  ~3 % + DMA/control ~10 %), leaving ~22 % headroom.

## Block diagram

```
   PL (fabric)                                   PS (A53)
   ┌─────────────┐   ┌──────────────┐   ┌──────────────────────────┐
   │ channelizer │──▶│ MSK demod    │──▶│ DecodeBank (demux)        │
   │ + power det │   │ (shared/     │   │  64× SingleStreamDecode-  │
   │  (existing) │   │  multiplexed)│   │  Receiver: sync + Viterbi │
   └─────────────┘   └──────────────┘   └──────────────────────────┘
       64 ch              soft stream         frames (134 B each)
```

The MSK demod is shared/time-multiplexed across channels (like the channelizer),
so it emits a channel-tagged stream rather than 64 parallel ports.

## The soft-bit contract (per symbol)

From `msk_demodulator.vhd` the fabric demod emits, per demodulated symbol:

| signal         | type             | meaning                                    |
|----------------|------------------|--------------------------------------------|
| `rx_data_soft` | `signed(15:0)`   | soft metric, **differentially decoded**    |
| `rx_dvalid`    | `std_logic`      | one strobe per valid symbol                |
| `rx_data`      | `std_logic`      | hard decision (back-up / hard mode)        |

Key properties:
- **Single stream.** Differential decode (line 294) resolves the 180° carrier /
  polarity ambiguity in fabric, so there is exactly one soft stream per channel —
  no parity/polarity hypotheses for the A53 to chase. (The coherent
  `CoherentChannelReceiver` carries a 4-fold ambiguity because it's a coherent 2T
  detector; the fabric path simply doesn't have it.)
- **Only valid symbols.** `rx_dvalid` gates; the A53 sees locked symbols only, so
  no symbol-lock detector is needed on the PS side.
- **Sign convention — VERIFY.** `FrameDecoder` quantizes with a leading minus
  (`n = (-soft/scale)*3.5 + 3.5`, clamp 0..7). Whether fabric `data_sum` sign
  matches must be confirmed against a real fabric vector; if it decodes inverted,
  negate at the source (or set a one-bit invert flag). This is the integration
  seam most likely to need a flip.

## Multiplexed PS/PL transport (provisional)

The shared demod emits channel-tagged samples. Provisional record (matches
`opv-decode -m N`):

```
struct Rec { uint8_t channel; int16_t soft; };   // packed, little-endian
```

Real transport is DMA fabric→A53 (the channelizer→A53 DMA path, now carrying soft
bits instead of raw IQ — far less data). Record packing / descriptor format is a
bench item once the demod is instantiated. `soft` may be narrowed (10–12 bits is
plenty; `FrameDecoder` rescales to 3-bit internally), TBD by DMA width.

## Frame structure (decoder inverts this; unchanged from OPV)

- `SYNC_WORD = 0x02B8DB`, `SYNC_BITS = 24`
- payload `ENCODED_BITS = 2144` → `FRAME_SYMBOLS = 2168`
- rate-1/2 K=7 convolutional (`G1=0x4F`, `G2=0x6D`), 67×32 block interleave,
  CCSDS randomizer, `FRAME_BYTES = 134`
- symbol rate 54200 /s/channel → 25 frames/s/channel

A53 side per channel: find sync (one 24-tap correlation, single hypothesis) →
collect 2144 soft → deinterleave → Viterbi → derandomize → 134-byte frame.

## A53 software (this repo)

- `SingleStreamDecodeReceiver` — one channel: `SyncTracker` + `FrameDecoder`, fed
  soft symbols via `push(soft, on_frame)`. No demod, no symbol-lock gate, no
  commit machinery.
- `DecodeBank(N)` — demuxes the channel-tagged stream into N receivers.
- `opv-decode` — harness/skeleton: `< soft.s16` (single) or `-m N` (multiplexed).
- `decode_bench` — pure `FrameDecoder` throughput → channel-capacity number.

Validated: round-trip (coherent demod `-X` soft tap → `opv-decode`) = 18/18
perfect; `DecodeBank -m 1` = 18/18.

## Open items / tests

1. **Sign/scale** of `rx_data_soft` vs `FrameDecoder` — verify on a real fabric
   vector; flip at source if inverted.
2. **Real integration vector** — capture `rx_data_soft` from the msk_demodulator
   HDL testbench (or GNU Radio model) and decode on A53.
3. **BER equivalence** — pluto_msk Costas+differential demod vs the coherent 2T
   baseline. Differential decode ~doubles error events → expect up to ~1 dB
   penalty; quantify for the link budget.
4. **Soft-bit DMA** cost fabric→A53 (~7 Mbit/s aggregate) — bench once demod is
   instantiated; refine the ~22 % headroom figure.
5. **Multiplexed demod scheduling** — confirm the shared demod's per-channel
   cadence and tag format; finalize the `Rec` layout / DMA descriptor.

---

## CONCRETE SEAM (found in pluto_msk — seam B is mostly already built)

pluto_msk `src/` already contains the full fabric decode chain, AXIS-connected:
`frame_sync_detector_soft.vhd`, `ov_frame_decoder_soft.vhd`,
`viterbi_decoder_k7_soft.vhd`, plus `axis_dma_adapter.vhd`. So seam B is a *tap*,
not a new build.

### The tap point: frame_sync_detector_soft.m_axis_soft_bit

`frame_sync_detector_soft` takes the demod's `signed(15:0)` soft
(`s_axis_soft_tdata` = rx_data_soft) and the hard bit, runs the sync-word
correlator (`SYNC_WORD=x"02B8DB"`, HUNTING/LOCKED thresholds + flywheel +
LOCK_FRAMES), quantizes to 3-bit, and emits:

```
m_axis_soft_bit_tdata  : std_logic_vector(2 DOWNTO 0)   -- SOFT_WIDTH=3, value 0..7
m_axis_soft_bit_tvalid / tready / tlast                 -- TLAST = end of frame
-- 2144 soft values per frame (PAYLOAD_BYTES=268 = 2144 bits)
```

That AXIS, routed through `axis_dma_adapter` → S2MM DMA, IS the seam-B interface:
one TLAST burst = one frame = 2144 three-bit soft values, interleaved (on-air)
order. For the multiplexed/shared demod, carry the channel on TID/TUSER (or one
DMA channel per demod slot); A53 demuxes by tag.

### Convention is already matched (no flip, no rescale)

fabric `viterbi_decoder_k7_soft.vhd` branch metric (lines 90-91):
`expected 0 -> metric = soft;  expected 1 -> metric = 7 - soft`.
A53 `FrameDecoder` Viterbi: `bm = (e ? SOFT_MAX - sg : sg)`, SOFT_MAX=7. Identical
(both NASA/CCSDS soft Viterbi). So the fabric's 3-bit bytes feed the A53 Viterbi
directly.

### A53 entry point: FrameDecoder::decode_soft3()

`int decode_soft3(const uint8_t* sg /*2144 values 0..7, interleaved*/,
                  std::array<uint8_t,FRAME_BYTES>& out);`
Skips the double->quantize step (fabric already quantized); does deinterleave +
Viterbi + derandomize. Validated bit-identical to decode() over 2000 frames.

### What this collapses the A53 side to

No SyncTracker, no demod, no streaming state: a pool of FrameDecoders, each fed a
DMA'd 2144-byte frame buffer on the TLAST/interrupt, calling decode_soft3. Pure
soft-real-time batch work (40 ms/frame slack), ~24 ch/core (decode_bench), 64 at
~66% + DMA/control -- comfortable headroom, vs seam A's tight 78%.

Open items 1 (sign/scale) and 2 (integration vector) are now mostly settled by
reading the source; the integration vector test just confirms it on real silicon.
Remaining: BER equivalence (link budget), DMA cost, multiplexed tag format.

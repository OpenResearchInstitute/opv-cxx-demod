# Coherent MSK demod — checkpoint (TRUE SEEING + de Buda)

Re-entry anchor for the coherent OPV-MSK detector on the C++ / A53 track.
The host-only DSP core `src/opv_demod.hpp` is the deliverable; `src/opv-demod.cpp`
is the driver. Both are saved whole here (the hpp lives nowhere durable else).

## State: coherent detector + carrier recovery DONE for batch; ready for capture→batch OTA
- Massey 2T detector + box-plus soft-differential + sync-resolved parity/polarity
- Decision-switched Costas carrier tracking (esign = -1, gains 0.01 / 2e-4)
- **de Buda coarse carrier acquisition (NEW)** — Welch-averaged squaring estimator,
  joint two-line scan, confidence gate. Retires the old `estimate_offset` stub.
- Non-coherent path untouched and still default.

## Measured through the REAL Viterbi/FEC (batch mode, B=200, repo SNR scale)
```
 perfect carrier:   ~6 dB gain (coherent 200/200 at -13..-15 where non-coh is dead)
 with de Buda, SNR=-10:  OFFSET 1000/2000/3000/4000 Hz all -> 200/200, est within ~20 Hz
 OFFSET=0 cliff (-12,-13): 200/200  (confidence gate => no harm at zero offset)
```
de Buda search range `foff_max = 5000 Hz` (named constant): reliable acquisition to
~4 kHz — covers a Keplerian-pre-corrected on-orbit residual + a bench LO offset.
Do NOT widen toward full Doppler without reason; a narrower scan is more robust at
low SNR (fewer false-lock chances). Widen only if running uncorrected at high carrier.

## Three de Buda bugs caught in the loop (not on the bench) — keep in mind
1. SEED SIGN: estimator reads +foff but the correlator needs -foff; +foff diverges.
2. FACTOR OF 2: the midpoint of the two squared lines is already 2*foff (needs /2, i.e. 0.25*sum not 0.5*sum).
3. SEARCH SPAN: fixed +/-2*dev windows were too narrow; joint scan over candidate foff fixes it.
Welch averaging (square whole capture, average |FFT|^2 over windows) is what makes it
reliable down at operating SNR; single-window failed below ~8 dB Eb/N0.

## A53 port — verified
- `make TARGET=a53 CXX=aarch64-linux-gnu-g++` cross-compiles clean (real aarch64 ELF).
  On the ZCU102 use `aarch64-xilinx-linux-g++` + the PetaLinux 2022.2 SDK sysroot.
- opv-cxx-demod is dependency-light (libstdc++/libm/libc + sockets, I/Q on stdin); no
  Xilinx/ADI deps. Drops into the MDT build the way dogu artifacts do.
- Runtime glue exists in dogu: `bring-up.sh <profile>` then
  `dma_listen | opv-modem -R -r <port>` (live, non-coherent) or
  `dma_listen > capture.iq` then `opv-demod -c < capture.iq` (offline, coherent).

## THE gating item for OTA (not code): sample rate
OPV is hardwired to 2.168 Msps (54200 x 40). dogu profiles are only 1.92 / 20 Msps.
Need a TES-generated ADRV9002 profile at 2.168 Msps (or a 271/240 resampler fallback).
Nothing decodes until the radio delivers 2.168 Msps.

## What's left
1. ADRV9002 2.168 Msps profile (TES, your side) -> unblocks capture→batch coherent OTA.
2. Live coherent (only if needed; non-coherent already covers a live link):
   - #1 route `-c` through ChannelReceiver (streaming path ignores it today)
   - #2 batch->streaming restructure: lock parity/polarity once at sync, run causally
3. Paul bench A/B on the step attenuator (prices the ~6 dB on real air).

## Suggested commit message
"Cast LOCATE OBJECT: de Buda coarse carrier acquisition for the coherent MSK demod"

## UPDATE — fractional-timing front-end for the A53 (native channelized rate)
Architecture decided with Michelle: OPV comes out of Haifuraiya per channel, up to
64 channels each carrying an OPV signal, each demod+decode at the channel rate
(~11.53 sps for a 625 ksps channel). NO resampler — adapt the demod to the native
rate via fractional symbol timing. Pluto modem stays at its 40 sps; A53 runs ~11.53.
(64 demods on 4 A53 cores => keep per-channel cost low; deferred optimization.)

NEW in CoherentMSKDemodulator (opv_demod.hpp): a fractional-timing front-end that
replaces the fixed k*SPS windows with a PI timing loop, feeding the UNCHANGED
Costas+Massey+parity back-end:
  - set_nominal_sps(double)         : 40 (Pluto) or ~11.53 (A53)
  - corr_at()                       : interpolating matched filter, MF_M=12 sub-samples,
                                      cubic (Catmull-Rom) interp, continuous-phase ref
  - track_correlations()            : ML-gradient TED err=Re{conj(y)*dy/dtau}/|y|^2,
                                      2nd-order PI loop (alpha_t_=0.06, beta_t_=0.0025)
  - set_timing_gains() / set_timing_bandwidth(BnT,zeta)
Validation harness: src/track_test.cpp (ports frac2.py; synth OPV at 11.53 sps,
0.37-sym offset + 25 ppm drift, perfect carrier, 4-way+shift BER scorer).

MEASURED (host, EbN0=8, cubic interp, a=0.06/b=0.0025, seeds 201-204):
  baseline (known timing) BER 4.84e-4;  loop BER 2.42e-4 / 4.35e-4 / 2.90e-4 / 0.352
  => 3/4 seeds lock to baseline (full coherent gain held at 11.53 sps, IN C++).
  1/4 slips. OPEN-LOOP SLIP RATE ~1/4 at 8 dB persists exactly as the model predicted.
Cross-compiles clean for cortex-a53; driver clean under -Wall.

## NEXT: sync-word data-aided anchoring (kills the slip)
The 24-bit sync word (0x02B8DB, 8:1 PSLR) arrives every FRAME_SYMBOLS=2168 symbols.
Plan: run SyncTracker's soft correlator on the dec stream; on a peak, snap the timing
phase to the correlator-interpolated peak and reset integrator windup -> re-anchor every
frame instead of tracking open-loop forever. Validate against a FRAMED synthetic signal
(insert sync words periodically) before hardware. Michelle's intent: build loop into C++
(DONE), exercise sync anchoring on hardware where frames arrive regularly.

## UPDATE 2 — parity/polarity relocation COMPLETE (demod now protocol-agnostic)
Decision (Michelle): demod stays self-contained/reusable; the sync-word soft
correlator owns parity/polarity resolution; keep it modular like the VHDL.

Done in opv_demod.hpp + opv-demod.cpp:
- CoherentMSKDemodulator::demodulate() REMOVED (it peeked at SYNC_WORD). Split into:
    * batch_correlations(samples,Y1,Y2) — fixed-window front-end (40 sps), no sync word
    * combine(Y1,Y2,dec0,dec1)         — Costas+Massey+softdiff, emits BOTH parity
                                          streams, no SYNC_WORD, no polarity decision
  track_correlations() (fractional, ~11.53 sps) is the channelized-rate front-end;
  both front-ends feed combine(). The demod contains ZERO references to SYNC_WORD now.
- SyncTracker::resolve(dec0,dec1) [static] — correlates both parity streams x both
  polarities against the sync word, returns the resolved single soft stream. Valid
  because parity & polarity are GLOBAL constants for the stream (FRAME_SYMBOLS even).
  The proven HUNTING/VERIFYING/LOCKED state machine runs UNCHANGED on the resolved stream.
- Driver coherent path rewired: estimate_offset -> batch_correlations -> combine ->
  SyncTracker::resolve -> tracker.process -> FrameDecoder (FrameDecoder UNCHANGED).

MEASURED end-to-end on REAL frames (opv-mod BERT | opv-demod, noiseless pipe, 40 sps):
  coherent  : 20 frames, 19 perfect (only the stream-TAIL frame imperfect, metric 2;
              pre-existing boundary effect, not a regression — soft stream is identical
              to the old path). Sync corr = 1.000 -> resolve() picked parity/pol correctly.
  non-coherent (default, untouched): 10/10 perfect.
Builds: driver -Wall clean; opv_demod.hpp cross-compiles clean for cortex-a53.

## VALIDATION MAP (so we know exactly what's proven vs pending)
- 11.53 sps fractional timing holds the coherent gain  : PROVEN (track_test, BER=baseline when locked)
- combine() emits both parity streams, one clean       : PROVEN (track_test: 2.4e-4 vs scrambled)
- resolve() + full chain decode REAL frames            : PROVEN at 40 sps (opv-mod|opv-demod -c)
- REAL frames end-to-end AT 11.53 sps (track->combine->resolve->FrameDecoder): NOT YET
  (needs channel-rate real frames: either resample opv-mod 2.168M->625k, or do it on the
  bench where the channelizer emits the real ~625 ksps channel). Natural next/bench step.
- loop hardening (bandwidth, lock thresholds)          : DEFERRED to hardware (Michelle's call)

## UPDATE 3 — channel-rate loopback CLOSED + resampler tool (also for SIC)
- NEW tool src/opv-resample.cpp: windowed-sinc (Blackman) fractional resampler,
  int16 I/Q in/out. `opv-resample [Fin Fout [K]]` (default 2168000 625000 32).
  Reusable as (1) channel-rate loopback source, (2) SIC reference reconstruction.
  K=32 taps needed for clean 3.47x decimation (K=16 aliased MSK sidelobes -> errors).
- opv-demod: new `-R <rate>` flag -> coherent path uses set_nominal_sps(rate/54200)
  + track_correlations (fractional) instead of batch_correlations; de Buda skipped
  (it is 2.168M-scaled), offset assumed 0 for the baseband loopback.
- BUGFIX (reusability): MF_M was a fixed 12 -> undersampled the matched filter at
  high sps (sps=40 gave 0/10). Now adaptive: M = max(12, round(sps_nom)). Fractional
  front-end is correct at ANY rate. (At the 11.53 target, M=12 as before -> validated
  results unchanged.)

MEASURED loopback (opv-mod -P -B 20 | opv-resample | opv-demod -c -R, noiseless):
  625 ksps / 11.53 sps : 17/18 perfect   <-- REAL frames end-to-end at the channel rate
  2168k   / 40 sps     : 17/18 perfect   (adaptive M)
  native batch coherent: 19/20 ; non-coherent: 20/20  (both untouched)
  (~2 frames lost to fractional cold-start acquisition; locks after preamble+1.)

VALIDATION MAP now: REAL frames end-to-end AT 11.53 sps = PROVEN (was the open seam).
Remaining = bench: loop bandwidth/lock-threshold tuning + real channelizer output;
sync-aided re-anchor wiring (hooks in place); de Buda rate-awareness if running with
carrier offset at the channel rate.

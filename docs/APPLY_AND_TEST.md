# Frame-sync detector: LEVEL -> PEAK detection (match VHDL)

File changed: `src/opv_demod.hpp` (SyncTracker only — demod/Costas/AFC untouched).

## What changed and why
`SyncTracker` HUNTING transitioned on a *level* test (`corr >= threshold`).
The authoritative `frame_sync_detector_soft.vhd` transitions on a *peak*:
`corr_prev >= HUNTING_THRESHOLD AND corr_v <= corr_prev`, and captures P(0)
on the transition clock. At 40 sps the peak is sharp so level==peak (bug
invisible); at the fractional channel rate the skirt crosses threshold early,
giving false locks + a one-symbol P(0) shift. Now matches the VHDL.

## Measured (this binary, after fix)
- Native 40 sps, `opv-demod -c`: 20/20 frames, frame 1 perfect, 0 spurious misses.
- Channel 11.53 sps, `-c -R 625000`: first lock at true peak (corr=1.000);
  leading garbage is front-end timing pull-in + faithful flywheel, not the FSM.
- End-of-transmission: flywheel emits exactly 2 garbage frames (= FLYWHEEL_TOLERANCE)
  then drops lock on miss 3. Expected; CRC rejects them downstream.

## Three exact hunks (apply to src/opv_demod.hpp)

### 1) New members (near `kSyncTrace`)
ADD:
    double prev_norm_corr_ = 0.0;
    double prev_raw_corr_  = 0.0;

### 2) HUNTING case body
REPLACE the level-detect block:
    if (raw_corr >= RAW_SYNC_HUNTING_THRESHOLD && norm_corr >= SOFT_SYNC_HUNTING_THRESHOLD) {
        state_ = SyncState::LOCKED;
        sync_quality_ = norm_corr;
        symbols_since_sync_ = 0;
        consecutive_misses_ = 0;
        collecting_payload_ = true;
        pending_frame_.clear();
        ...
    }
WITH the peak-detect block:
    bool prev_above = (prev_raw_corr_  >= RAW_SYNC_HUNTING_THRESHOLD &&
                       prev_norm_corr_ >= SOFT_SYNC_HUNTING_THRESHOLD);
    bool falling    = (raw_corr <= prev_raw_corr_);     // VHDL: corr_v <= corr_prev
    if (prev_above && falling) {
        state_ = SyncState::LOCKED;
        sync_quality_ = prev_norm_corr_;
        consecutive_misses_ = 0;
        collecting_payload_ = true;
        pending_frame_.clear();
        pending_frame_.push_back(soft_val);            // capture P(0) now
        symbols_since_sync_ = 1;                        // P(0) collected -> count 1
    }
    prev_norm_corr_ = norm_corr;                        // always update history
    prev_raw_corr_  = raw_corr;

### 3) Reset peak history on BOTH lost-lock paths
- In LOCKED->HUNTING (consecutive_misses_ >= SYNC_MISS_LIMIT): add
      prev_norm_corr_ = 0.0; prev_raw_corr_ = 0.0;
- In set_symbol_lock() lock-loss branch: add
      prev_norm_corr_ = 0.0; prev_raw_corr_ = 0.0;

(Or just drop in the full opv_demod.hpp from this folder if your laptop tree
is in sync with the session work.)

## Commit + push
    git add src/opv_demod.hpp
    git commit -m "Recast DETECT MAGIC as a true PEAK (level->peak frame sync, match VHDL)

Frame sync HUNTING transitioned on level (corr>=thresh); the authoritative
frame_sync_detector_soft.vhd uses peak (corr_prev>=HUNTING_THRESHOLD and
corr falling) and captures P(0) on the transition clock. Level==peak at the
sharp 40 sps peak so it passed, but the fractional channel rate exposed false
locks and a one-symbol P(0) shift. HUNTING now peak-detects, captures P(0),
and seeds symbols_since_sync_=1 for byte-identical framing to the LOCKED
resync path. Peak history cleared on both lost-lock paths.

Measured: native 20/20, frame 1 perfect, 0 spurious misses."
    git push

## Test plan (40 sps — the default path)
1. Loopback: opv-mod -> opv-demod -c, confirm N-in -> N-out, frame 1 perfect.
2. OTA with Paul: same, watch for the expected 1-2 flywheel frames after PTT release
   (Sync~0, high metric) — those are correct and CRC-rejected, not a regression.

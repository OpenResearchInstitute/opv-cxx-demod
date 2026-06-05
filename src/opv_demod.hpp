#pragma once
//------------------------------------------------------------------------------
// opv_demod.hpp - OPV MSK demodulator DSP core (header-only library)
//------------------------------------------------------------------------------
// The reusable OPV *receive* DSP: constants, the MSK demodulators (non-coherent
// AFC + coherent Costas), symbol-lock detector, sync tracker, Viterbi decoder,
// and frame decoder.  Consumed two ways:
//
//   1. by opv-demod.cpp - the standalone PlutoSDR/LibreSDR/Interlocutor program
//      (unchanged in behaviour; it just #includes this header now), and
//   2. as a git submodule by other designs (e.g. the Haifuraiya 'dogu' payload)
//      that embed the demodulator as a library and run many channel instances
//      instead of forking the program per channel.
//
// Header-only: no separate build or link step, so the repo keeps its
// self-contained, dependency-free character.  License: CERN-OHL-S v2.
//------------------------------------------------------------------------------

#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <array>
#include <iomanip>
#include <cstring>
#include <string>
#include <cstdio>
//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------
constexpr size_t SAMPLES_PER_SYMBOL = 40;
constexpr double SAMPLE_RATE = 2168000.0;
constexpr double SYMBOL_RATE = SAMPLE_RATE / SAMPLES_PER_SYMBOL;
constexpr double FREQ_DEV = 13550.0;
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

constexpr uint32_t SYNC_WORD = 0x02B8DB;
constexpr size_t SYNC_BITS = 24;

constexpr size_t FRAME_BYTES = 134;
constexpr size_t FRAME_BITS = FRAME_BYTES * 8;
constexpr size_t ENCODED_BITS = FRAME_BITS * 2;
constexpr size_t FRAME_SYMBOLS = SYNC_BITS + ENCODED_BITS;

constexpr int SOFT_MAX = 7;
constexpr uint8_t G1_MASK = 0x4F;
constexpr uint8_t G2_MASK = 0x6D;
constexpr int NUM_STATES = 64;

// Sync thresholds
constexpr int SYNC_MISS_LIMIT = 3;          // Allow more misses before losing lock

// OPV Header
constexpr size_t OPV_STATION_ID_SIZE = 6;
constexpr size_t OPV_TOKEN_OFFSET = 6;
constexpr size_t OPV_RESERVED_OFFSET = 9;

using sample_t = std::complex<double>;
struct IQSample { int16_t I, Q; };

//------------------------------------------------------------------------------
// Sync State Machine
//------------------------------------------------------------------------------
enum class SyncState { HUNTING, VERIFYING, LOCKED };

inline const char* state_name(SyncState s) {
    switch (s) {
        case SyncState::HUNTING: return "HUNTING";
        case SyncState::VERIFYING: return "VERIFYING";
        case SyncState::LOCKED: return "LOCKED";
    }
    return "?";
}

//------------------------------------------------------------------------------
// Symbol Lock Detector
//------------------------------------------------------------------------------
// Tracks the magnitude of the Timing Error Detector (TED) output over a
// sliding window.  When the timing loop has converged on a real signal the
// TED residual is small and stable; when running on noise it is large and
// erratic.  This gate prevents the frame-sync correlator from searching on
// noise, eliminating false locks that waste preamble frames.
//
// Hysteresis (separate lock/unlock thresholds) prevents chattering at the
// boundary.  Window size of ~100 symbols ≈ 1/20 of a frame period — fast
// enough to declare lock well within one preamble frame.
//------------------------------------------------------------------------------
class SymbolLockDetector {
public:
    static constexpr size_t  WINDOW        = 100;   // symbols to average
    static constexpr double  LOCK_THRESH   = 0.25;  // avg |TED| below → locked
    static constexpr double  UNLOCK_THRESH = 0.50;  // avg |TED| above → unlocked

    SymbolLockDetector()
        : buf_{}, idx_(0), sum_(0.0), count_(0), locked_(false) {}

    // Call once per symbol with the raw TED output from the demodulator.
    // Returns true when symbol timing is considered locked.
    bool update(double ted) {
        double mag = std::abs(ted);

        // Subtract oldest value, add newest
        sum_ -= buf_[idx_];
        buf_[idx_] = mag;
        sum_ += mag;
        idx_ = (idx_ + 1) % WINDOW;
        if (count_ < WINDOW) ++count_;

        double avg = sum_ / count_;

        if (!locked_ && count_ >= WINDOW && avg < LOCK_THRESH) {
            locked_ = true;
            fprintf(stderr, "[sym_lock] LOCKED  (avg |TED| = %.3f)\n", avg);
        } else if (locked_ && avg > UNLOCK_THRESH) {
            locked_ = false;
            fprintf(stderr, "[sym_lock] UNLOCKED (avg |TED| = %.3f)\n", avg);
        }

        return locked_;
    }

    bool is_locked() const { return locked_; }
    void reset() { sum_ = 0; count_ = 0; idx_ = 0; locked_ = false; buf_ = {}; }

private:
    std::array<double, WINDOW> buf_;
    size_t idx_;
    double sum_;
    size_t count_;
    bool locked_;
};

//------------------------------------------------------------------------------
// Base-40 Decoder
//------------------------------------------------------------------------------
inline std::string decode_base40(const uint8_t* bytes, size_t len = 6) {
    uint64_t value = 0;
    for (size_t i = 0; i < len; ++i) value = (value << 8) | bytes[i];
    if (value == 0) return "(empty)";
    
    std::string result;
    while (value > 0) {
        int d = value % 40;
        value /= 40;
        char c = (d == 0) ? '\0' : 
                 (d <= 26) ? ('A' + d - 1) :
                 (d <= 36) ? ('0' + d - 27) :
                 (d == 37) ? '-' : (d == 38) ? '/' : '.';
        if (c) result += c;  // Append to match HDL byte order
    }
    return result.empty() ? "(empty)" : result;
}

//------------------------------------------------------------------------------
// MSK Demodulator with AFC (Non-coherent energy detection)
//------------------------------------------------------------------------------
class MSKDemodulatorAFC {
public:
    MSKDemodulatorAFC() 
        : freq_offset_(0), phase_f1_(0), phase_f2_(0),
          prev_corr_f1_(0), prev_corr_f2_(0),
          afc_alpha_(0.001),
          // Timing recovery state
          mu_(0.0),              // Fractional sample offset (0 to 1)
          timing_freq_(0.0),     // Clock frequency offset estimate
          alpha_timing_(0.005),  // Timing loop proportional gain
          beta_timing_(0.00001), // Timing loop integral gain
          last_ted_(0.0),        // Last TED output (for symbol lock detector)
          tracking_enabled_(true) // AFC/timing integrators active (frozen when no signal)
    {}
    
    // Linear interpolation
    sample_t interp(const sample_t* s, double idx, size_t len) {
        if (idx < 0) idx = 0;
        if (idx >= len - 1) idx = len - 2;
        size_t i = static_cast<size_t>(idx);
        double f = idx - i;
        return s[i] * (1.0 - f) + s[i + 1] * f;
    }
    
    // Estimate carrier offset from spectrum (coarse AFC)
    double estimate_offset(const sample_t* samples, size_t num_samples) {
        double best_offset = 0;
        double best_energy = 0;
        
        for (double offset = -1500; offset <= 1500; offset += 25) {
            double phase_f1 = 0, phase_f2 = 0;
            double phase_inc_f1 = TWO_PI * (-FREQ_DEV + offset) / SAMPLE_RATE;
            double phase_inc_f2 = TWO_PI * (+FREQ_DEV + offset) / SAMPLE_RATE;
            
            double total_energy = 0;
            size_t test_samples = std::min(num_samples, size_t(SAMPLES_PER_SYMBOL * 1000));
            
            for (size_t sym = 0; sym < test_samples / SAMPLES_PER_SYMBOL; ++sym) {
                std::complex<double> corr_f1(0), corr_f2(0);
                
                for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
                    size_t idx = sym * SAMPLES_PER_SYMBOL + i;
                    std::complex<double> lo_f1(std::cos(phase_f1), std::sin(phase_f1));
                    std::complex<double> lo_f2(std::cos(phase_f2), std::sin(phase_f2));
                    
                    corr_f1 += samples[idx] * std::conj(lo_f1);
                    corr_f2 += samples[idx] * std::conj(lo_f2);
                    
                    phase_f1 += phase_inc_f1;
                    phase_f2 += phase_inc_f2;
                }
                
                total_energy += std::norm(corr_f1) + std::norm(corr_f2);
            }
            
            if (total_energy > best_energy) {
                best_energy = total_energy;
                best_offset = offset;
            }
        }
        
        // Fine tune
        double fine_best = best_offset;
        for (double offset = best_offset - 30; offset <= best_offset + 30; offset += 5) {
            double phase_f1 = 0, phase_f2 = 0;
            double phase_inc_f1 = TWO_PI * (-FREQ_DEV + offset) / SAMPLE_RATE;
            double phase_inc_f2 = TWO_PI * (+FREQ_DEV + offset) / SAMPLE_RATE;
            
            double total_energy = 0;
            size_t test_samples = std::min(num_samples, size_t(SAMPLES_PER_SYMBOL * 1000));
            
            for (size_t sym = 0; sym < test_samples / SAMPLES_PER_SYMBOL; ++sym) {
                std::complex<double> corr_f1(0), corr_f2(0);
                
                for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
                    size_t idx = sym * SAMPLES_PER_SYMBOL + i;
                    std::complex<double> lo_f1(std::cos(phase_f1), std::sin(phase_f1));
                    std::complex<double> lo_f2(std::cos(phase_f2), std::sin(phase_f2));
                    
                    corr_f1 += samples[idx] * std::conj(lo_f1);
                    corr_f2 += samples[idx] * std::conj(lo_f2);
                    
                    phase_f1 += phase_inc_f1;
                    phase_f2 += phase_inc_f2;
                }
                
                total_energy += std::norm(corr_f1) + std::norm(corr_f2);
            }
            
            if (total_energy > best_energy) {
                best_energy = total_energy;
                fine_best = offset;
            }
        }
        
        return fine_best;
    }
    
    void set_freq_offset(double offset) { freq_offset_ = offset; }
    
    void demodulate(const sample_t* samples, size_t num_samples,
                    std::vector<double>& soft_out,
                    std::vector<double>* ted_out = nullptr) {
        soft_out.clear();
        if (ted_out) ted_out->clear();
        
        double phase_inc_f1 = TWO_PI * (-FREQ_DEV + freq_offset_) / SAMPLE_RATE;
        double phase_inc_f2 = TWO_PI * (+FREQ_DEV + freq_offset_) / SAMPLE_RATE;
        
        // Early-late spacing: T/4 = 10 samples
        const double EL_OFFSET = SAMPLES_PER_SYMBOL / 4.0;
        
        // Current sample position (fractional)
        double pos = mu_;
        
        // Process symbols while we have enough samples
        // Need: pos + SPS + EL_OFFSET < num_samples
        while (pos + SAMPLES_PER_SYMBOL + EL_OFFSET < num_samples) {
            
            std::complex<double> corr_f1(0, 0), corr_f2(0, 0);
            std::complex<double> corr_f1_e(0, 0), corr_f2_e(0, 0);  // Early
            std::complex<double> corr_f1_l(0, 0), corr_f2_l(0, 0);  // Late
            
            // Recursive NCO: seed the two LO phasors once per symbol (one sincos
            // pair each) and the per-sample step rotators from the current phase
            // increments, then advance with a complex multiply per sample instead
            // of cos/sin every sample. Re-seeding each symbol bounds the phasor
            // magnitude drift to <= SPS steps, so no renormalization is needed.
            // Bit-equivalent to the original: lo = exp(j*(phase + i*phase_inc)).
            std::complex<double> lo1(std::cos(phase_f1_), std::sin(phase_f1_));
            std::complex<double> lo2(std::cos(phase_f2_), std::sin(phase_f2_));
            const std::complex<double> w1(std::cos(phase_inc_f1), std::sin(phase_inc_f1));
            const std::complex<double> w2(std::cos(phase_inc_f2), std::sin(phase_inc_f2));
            
            // Integrate over symbol period
            for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
                double p_on = pos + i;
                double p_early = p_on - EL_OFFSET;
                double p_late = p_on + EL_OFFSET;
                
                sample_t s_on = interp(samples, p_on, num_samples);
                sample_t s_e = (p_early >= 0) ? interp(samples, p_early, num_samples) : samples[0];
                sample_t s_l = interp(samples, p_late, num_samples);
                
                corr_f1 += s_on * std::conj(lo1);
                corr_f2 += s_on * std::conj(lo2);
                corr_f1_e += s_e * std::conj(lo1);
                corr_f2_e += s_e * std::conj(lo2);
                corr_f1_l += s_l * std::conj(lo1);
                corr_f2_l += s_l * std::conj(lo2);
                
                lo1 *= w1;
                lo2 *= w2;
            }
            
            // Update persistent phases (advance one symbol's worth of phase)
            phase_f1_ += SAMPLES_PER_SYMBOL * phase_inc_f1;
            phase_f2_ += SAMPLES_PER_SYMBOL * phase_inc_f2;
            
            // Wrap phases
            while (phase_f1_ > PI) phase_f1_ -= TWO_PI;
            while (phase_f1_ < -PI) phase_f1_ += TWO_PI;
            while (phase_f2_ > PI) phase_f2_ -= TWO_PI;
            while (phase_f2_ < -PI) phase_f2_ += TWO_PI;
            
            double e1 = std::norm(corr_f1);
            double e2 = std::norm(corr_f2);
            
            // Soft decision
            soft_out.push_back(e2 - e1);
            
            // === Timing Error Detector (Early-Late Gate) ===
            double ted;
            if (e1 > e2) {
                double ee = std::norm(corr_f1_e);
                double el = std::norm(corr_f1_l);
                ted = (el - ee) / (el + ee + 1e-10);
            } else {
                double ee = std::norm(corr_f2_e);
                double el = std::norm(corr_f2_l);
                ted = (el - ee) / (el + ee + 1e-10);
            }
            
            // 2nd order loop filter
            // Only update integrators when tracking a real signal.
            // On noise the TED and AFC chase random correlations,
            // causing drift that hurts acquisition when a real signal arrives.
            if (tracking_enabled_) {
                timing_freq_ += beta_timing_ * ted;
                timing_freq_ = std::clamp(timing_freq_, -0.1, 0.1);
            }
            double timing_adj = alpha_timing_ * ted + timing_freq_;
            timing_adj = std::clamp(timing_adj, -2.0, 2.0);  // Max 2 samples per symbol
            
            // Record TED for symbol lock detector
            last_ted_ = ted;
            if (ted_out) ted_out->push_back(ted);
            
            // === AFC ===
            if (tracking_enabled_ && soft_out.size() > 1) {
                std::complex<double> dom, prev_dom;
                if (e1 > e2) {
                    dom = corr_f1;
                    prev_dom = prev_corr_f1_;
                } else {
                    dom = corr_f2;
                    prev_dom = prev_corr_f2_;
                }
                
                double pd = std::arg(dom * std::conj(prev_dom));
                double ferr = pd * SYMBOL_RATE / TWO_PI;
                
                freq_offset_ += afc_alpha_ * ferr;
                freq_offset_ = std::clamp(freq_offset_, -2000.0, 2000.0);
                
                phase_inc_f1 = TWO_PI * (-FREQ_DEV + freq_offset_) / SAMPLE_RATE;
                phase_inc_f2 = TWO_PI * (+FREQ_DEV + freq_offset_) / SAMPLE_RATE;
            }
            
            prev_corr_f1_ = corr_f1;
            prev_corr_f2_ = corr_f2;
            
            // Advance to next symbol with timing adjustment
            pos += SAMPLES_PER_SYMBOL + timing_adj;
        }
        
        // Save fractional remainder for next chunk
        // mu_ = how far into the next symbol period we are
        size_t samples_used = static_cast<size_t>(pos);
        mu_ = pos - samples_used;
        
        // We need to remember how many samples to skip at the start of next chunk
        // This is stored implicitly: next call should start at sample 0, but
        // we need to account for the fact that we stopped mid-stream.
        // Actually, for streaming, we need to handle this differently...
        
        // For streaming: we consumed 'pos' samples worth, next chunk starts fresh
        // but mu_ tells us our timing phase within a symbol
        leftover_samples_ = num_samples - samples_used;
    }
    
    double get_freq_offset() const { return freq_offset_; }
    double get_timing_freq() const { return timing_freq_; }
    double get_last_ted() const { return last_ted_; }
    void set_afc_bandwidth(double alpha) { afc_alpha_ = alpha; }
    void set_tracking_enabled(bool en) { tracking_enabled_ = en; }
    size_t get_leftover() const { return leftover_samples_; }

private:
    double freq_offset_;
    double phase_f1_, phase_f2_;
    std::complex<double> prev_corr_f1_, prev_corr_f2_;
    double afc_alpha_;
    
    // Timing recovery
    double mu_;              // Fractional timing offset (0 to 1 symbol)
    double timing_freq_;     // Timing frequency offset estimate
    double alpha_timing_;    // Loop proportional gain
    double beta_timing_;     // Loop integral gain
    double last_ted_;        // Last TED output for symbol lock detection
    bool tracking_enabled_;   // When false, freeze AFC and timing integrators
    size_t leftover_samples_ = 0;
};

//------------------------------------------------------------------------------
// Coherent MSK Demodulator with Costas Loop
//------------------------------------------------------------------------------
// This demodulator recovers carrier phase using a decision-directed Costas loop.
// Instead of energy detection (|corr|²), it uses the real part of phase-aligned
// correlations, providing ~3dB improvement at low SNR.
//
// Architecture:
//   1. Correlate with F1 and F2 tones (same as non-coherent)
//   2. Track carrier phase using Costas loop feedback
//   3. Rotate correlations to align with reference phase
//   4. Soft decision = Re(corr_f2) - Re(corr_f1) (signed, not magnitude!)
//
// The differential encoding in OPV resolves the 180° phase ambiguity.
//------------------------------------------------------------------------------
class CoherentMSKDemodulator {
public:
    CoherentMSKDemodulator() 
        : freq_offset_(0), carrier_phase_(0),
          phase_f1_(0), phase_f2_(0),
          loop_freq_(0),
          // Costas loop gains (2nd order loop)
          // BW ~= 0.01 * symbol_rate for acquisition, narrower for tracking
          pll_alpha_(0.01),    // Phase gain (proportional)
          pll_beta_(0.001)     // Frequency gain (integral)
    {}
    
    // Estimate carrier offset from spectrum (coarse AFC) - same as non-coherent
    double estimate_offset(const sample_t* samples, size_t num_samples) {
        double best_offset = 0;
        double best_energy = 0;
        
        for (double offset = -1500; offset <= 1500; offset += 25) {
            double phase_f1 = 0, phase_f2 = 0;
            double phase_inc_f1 = TWO_PI * (-FREQ_DEV + offset) / SAMPLE_RATE;
            double phase_inc_f2 = TWO_PI * (+FREQ_DEV + offset) / SAMPLE_RATE;
            
            double total_energy = 0;
            size_t test_samples = std::min(num_samples, size_t(SAMPLES_PER_SYMBOL * 1000));
            
            for (size_t sym = 0; sym < test_samples / SAMPLES_PER_SYMBOL; ++sym) {
                std::complex<double> corr_f1(0), corr_f2(0);
                
                for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
                    size_t idx = sym * SAMPLES_PER_SYMBOL + i;
                    std::complex<double> lo_f1(std::cos(phase_f1), std::sin(phase_f1));
                    std::complex<double> lo_f2(std::cos(phase_f2), std::sin(phase_f2));
                    
                    corr_f1 += samples[idx] * std::conj(lo_f1);
                    corr_f2 += samples[idx] * std::conj(lo_f2);
                    
                    phase_f1 += phase_inc_f1;
                    phase_f2 += phase_inc_f2;
                }
                
                total_energy += std::norm(corr_f1) + std::norm(corr_f2);
            }
            
            if (total_energy > best_energy) {
                best_energy = total_energy;
                best_offset = offset;
            }
        }
        
        // Fine tune
        double fine_best = best_offset;
        for (double offset = best_offset - 30; offset <= best_offset + 30; offset += 5) {
            double phase_f1 = 0, phase_f2 = 0;
            double phase_inc_f1 = TWO_PI * (-FREQ_DEV + offset) / SAMPLE_RATE;
            double phase_inc_f2 = TWO_PI * (+FREQ_DEV + offset) / SAMPLE_RATE;
            
            double total_energy = 0;
            size_t test_samples = std::min(num_samples, size_t(SAMPLES_PER_SYMBOL * 1000));
            
            for (size_t sym = 0; sym < test_samples / SAMPLES_PER_SYMBOL; ++sym) {
                std::complex<double> corr_f1(0), corr_f2(0);
                
                for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
                    size_t idx = sym * SAMPLES_PER_SYMBOL + i;
                    std::complex<double> lo_f1(std::cos(phase_f1), std::sin(phase_f1));
                    std::complex<double> lo_f2(std::cos(phase_f2), std::sin(phase_f2));
                    
                    corr_f1 += samples[idx] * std::conj(lo_f1);
                    corr_f2 += samples[idx] * std::conj(lo_f2);
                    
                    phase_f1 += phase_inc_f1;
                    phase_f2 += phase_inc_f2;
                }
                
                total_energy += std::norm(corr_f1) + std::norm(corr_f2);
            }
            
            if (total_energy > best_energy) {
                best_energy = total_energy;
                fine_best = offset;
            }
        }
        
        return fine_best;
    }
    
    void set_freq_offset(double offset) { freq_offset_ = offset; }
    
    void demodulate(const sample_t* samples, size_t num_samples,
                    std::vector<double>& soft_out) {
        soft_out.clear();
        
        double phase_inc_f1 = TWO_PI * (-FREQ_DEV + freq_offset_) / SAMPLE_RATE;
        double phase_inc_f2 = TWO_PI * (+FREQ_DEV + freq_offset_) / SAMPLE_RATE;
        
        for (size_t sym = 0; sym < num_samples / SAMPLES_PER_SYMBOL; ++sym) {
            std::complex<double> corr_f1(0, 0), corr_f2(0, 0);
            
            // Integrate over symbol with carrier phase correction
            for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
                size_t idx = sym * SAMPLES_PER_SYMBOL + i;
                
                // Apply carrier phase correction to the sample
                std::complex<double> phase_rot(std::cos(carrier_phase_), -std::sin(carrier_phase_));
                std::complex<double> corrected = samples[idx] * phase_rot;
                
                // Correlate with F1 and F2 tones
                std::complex<double> lo_f1(std::cos(phase_f1_), std::sin(phase_f1_));
                std::complex<double> lo_f2(std::cos(phase_f2_), std::sin(phase_f2_));
                
                corr_f1 += corrected * std::conj(lo_f1);
                corr_f2 += corrected * std::conj(lo_f2);
                
                phase_f1_ += phase_inc_f1;
                phase_f2_ += phase_inc_f2;
                
                // Advance carrier phase by loop frequency correction
                carrier_phase_ += loop_freq_;
            }
            
            // Wrap phases
            while (phase_f1_ > PI) phase_f1_ -= TWO_PI;
            while (phase_f1_ < -PI) phase_f1_ += TWO_PI;
            while (phase_f2_ > PI) phase_f2_ -= TWO_PI;
            while (phase_f2_ < -PI) phase_f2_ += TWO_PI;
            while (carrier_phase_ > PI) carrier_phase_ -= TWO_PI;
            while (carrier_phase_ < -PI) carrier_phase_ += TWO_PI;
            
            // Determine which tone is dominant (for phase error computation)
            double f1_energy = std::norm(corr_f1);
            double f2_energy = std::norm(corr_f2);
            
            // ===== COHERENT SOFT DECISION =====
            // Use REAL parts (phase-sensitive), not magnitudes
            // With proper phase tracking, the correlations should be mostly real
            double soft_f1 = corr_f1.real();
            double soft_f2 = corr_f2.real();
            
            // Soft decision: positive = F2 (bit 0), negative = F1 (bit 1)
            // The sign now carries phase information!
            soft_out.push_back(soft_f2 - soft_f1);
            
            // ===== COSTAS LOOP PHASE ERROR =====
            // Decision-directed: use the dominant tone's phase to track carrier
            // Phase error = Im(corr) / Re(corr) for small angles ≈ atan2(Im, Re)
            std::complex<double> dominant = (f1_energy > f2_energy) ? corr_f1 : corr_f2;
            
            // Normalized phase error (avoid division by zero)
            double mag = std::abs(dominant);
            double phase_error = 0;
            if (mag > 1e-10) {
                // For decision-directed, we want the correlation to be real and positive
                // But MSK alternates phase by ±90° per symbol, so we track that
                // Simple approach: drive imaginary part to zero
                phase_error = dominant.imag() / mag;  // sin(error) ≈ error for small angles
            }
            
            // ===== LOOP FILTER (2nd order) =====
            // Updates both phase and frequency estimates
            loop_freq_ += pll_beta_ * phase_error;  // Integral path (frequency)
            carrier_phase_ += pll_alpha_ * phase_error;  // Proportional path (phase)
            
            // Limit loop frequency to avoid runaway
            loop_freq_ = std::clamp(loop_freq_, -0.1, 0.1);
            
            // NOTE: Coarse inter-symbol AFC is intentionally absent here.
            // The 2nd-order Costas loop already provides frequency tracking
            // via its integral path (loop_freq_). Running a separate AFC that
            // also updates freq_offset_ and recomputes phase_inc_f1/f2 would
            // create two coupled feedback paths chasing the same residual offset,
            // which can cause oscillation and degrades the 3dB coherent gain.
            // The initial estimate_offset() call gets close enough for PLL
            // pull-in before demodulate() is invoked. The Costas loop owns it
            // from there.
        }
    }
    
    double get_freq_offset() const { return freq_offset_; }
    
    // Set Costas loop bandwidth
    // Wider bandwidth: faster acquisition, more noise
    // Narrower bandwidth: better noise rejection, slower tracking
    void set_pll_bandwidth(double bw) {
        // Natural frequency ωn and damping ζ for 2nd order loop
        // bw ≈ ωn / (2π) for critically damped loop
        double wn = bw * TWO_PI;
        double zeta = 0.707;  // Critically damped
        pll_alpha_ = 2.0 * zeta * wn / SYMBOL_RATE;
        pll_beta_ = wn * wn / (SYMBOL_RATE * SYMBOL_RATE);
    }

private:
    double freq_offset_;
    double carrier_phase_;      // Carrier phase estimate
    double phase_f1_, phase_f2_;
    double loop_freq_;          // Loop frequency correction (radians per sample)
    double pll_alpha_;          // Phase error gain
    double pll_beta_;           // Frequency error gain
};

//------------------------------------------------------------------------------
// Sync State Machine with Circular Buffer (like HDL)
//------------------------------------------------------------------------------
// Key insight from HDL: Use a FIXED-SIZE circular buffer with wrap-around.
// This eliminates index shifting and buffer trimming issues.
//
// Architecture:
// 1. Large circular buffer holds ~2 frames of soft symbols
// 2. Write pointer advances and wraps around
// 3. When sync found, we record payload position RELATIVE to write pointer
// 4. Frame extraction uses circular indexing into the buffer
// 5. Multiple frames can be "in flight" without index corruption
//------------------------------------------------------------------------------
class SyncTracker {
public:
    static constexpr size_t CIRC_BUF_SIZE = FRAME_SYMBOLS * 3;  // 3 frames worth
    
    SyncTracker() : state_(SyncState::HUNTING),
                    symbols_since_sync_(0), consecutive_misses_(0),
                    total_frames_(0), corr_buf_idx_(0),
                    circ_write_idx_(0), total_symbols_(0),
                    symbol_locked_(false) {
        // Precompute sync word correlation pattern
        // For each bit: '1' expects negative soft (F1), '0' expects positive (F2)
        for (int i = 0; i < (int)SYNC_BITS; ++i) {
            int bit = (SYNC_WORD >> (SYNC_BITS - 1 - i)) & 1;
            sync_pattern_[i] = (bit == 1) ? -1.0 : +1.0;
        }
        // Initialize buffers
        soft_corr_buf_.fill(0.0);
        circ_buf_.fill(0.0);
        
        // Clear pending frame
        pending_frame_.reserve(ENCODED_BITS);
        verified_frame_.reserve(ENCODED_BITS);
    }
    
    struct Result {
        bool frame_ready;
        double sync_quality;  // Normalized correlation value (-1 to +1)
        std::vector<double> payload;  // The actual payload data (copied out)
    };
    
    // Symbol lock gate: call from main loop with SymbolLockDetector output
    void set_symbol_lock(bool locked) {
        if (!symbol_locked_ && locked) {
            fprintf(stderr, "[sync] Symbol lock acquired — enabling frame sync search\n");
        } else if (symbol_locked_ && !locked) {
            fprintf(stderr, "[sync] Symbol lock lost — disabling frame sync search\n");
            // If we lose symbol lock, drop back to HUNTING
            if (state_ != SyncState::HUNTING) {
                fprintf(stderr, "[sync] Forcing HUNTING (symbol lock lost)\n");
                state_ = SyncState::HUNTING;
                collecting_payload_ = false;
                verified_frame_.clear();
            }
        }
        symbol_locked_ = locked;
    }
    
    Result process(double soft_val, size_t sym_idx) {
        Result res = {false, 0.0, {}};
        
        // Update circular buffer for soft correlation (sync detection)
        soft_corr_buf_[corr_buf_idx_] = soft_val;
        corr_buf_idx_ = (corr_buf_idx_ + 1) % SYNC_BITS;
        
        // Update main circular buffer (for payload extraction)
        circ_buf_[circ_write_idx_] = soft_val;
        circ_write_idx_ = (circ_write_idx_ + 1) % CIRC_BUF_SIZE;
        total_symbols_++;
        
        // If collecting payload, add to pending frame
        if (collecting_payload_) {
            pending_frame_.push_back(soft_val);
        }
        
        symbols_since_sync_++;
        
        switch (state_) {
        case SyncState::HUNTING: {
            // ── Symbol lock gate ──────────────────────────────────────
            // Don't search for frame sync until the timing loop has
            // converged.  This prevents false locks on noise that would
            // waste preamble frames while the flywheel runs down.
            if (!symbol_locked_) break;
            
            // Need at least 24 symbols before checking for sync
            if (total_symbols_ < SYNC_BITS) break;
            
            double raw_corr;
            double norm_corr = soft_correlate(&raw_corr);
            
            if (raw_corr >= RAW_SYNC_HUNTING_THRESHOLD && norm_corr >= SOFT_SYNC_HUNTING_THRESHOLD) {
                state_ = SyncState::VERIFYING;
                sync_quality_ = norm_corr;
                symbols_since_sync_ = 0;
                
                // Start collecting payload for this frame
                collecting_payload_ = true;
                pending_frame_.clear();
                
                fprintf(stderr, "[%zu] HUNTING→VERIFYING (corr=%.3f, raw=%.0f)\n", 
                        sym_idx, norm_corr, raw_corr);
            }
            break;
        }
        
        case SyncState::VERIFYING: {
            // ── Two-sync acquisition (matches HDL) ────────────────────
            // Phase 1: collect payload after first sync hit
            if (collecting_payload_ && pending_frame_.size() >= ENCODED_BITS) {
                // Payload complete — stop collecting but stay in VERIFYING
                collecting_payload_ = false;
                verified_frame_ = std::move(pending_frame_);
                verified_quality_ = sync_quality_;
                pending_frame_.clear();
                pending_frame_.reserve(ENCODED_BITS);
            }
            
            // Phase 2: verify second sync at frame boundary
            if (symbols_since_sync_ >= FRAME_SYMBOLS) {
                double raw_corr;
                double corr = soft_correlate(&raw_corr);
                
                if (corr >= SOFT_SYNC_LOCKED_THRESHOLD) {
                    // ✓ Second sync confirmed — real signal
                    state_ = SyncState::LOCKED;
                    consecutive_misses_ = 0;
                    total_frames_++;
                    
                    // Output the verified first frame
                    res.frame_ready = true;
                    res.sync_quality = verified_quality_;
                    res.payload = std::move(verified_frame_);
                    
                    // Start collecting next frame's payload
                    sync_quality_ = corr;
                    collecting_payload_ = true;
                    pending_frame_.clear();
                    symbols_since_sync_ = 0;
                    
                    fprintf(stderr, "[%zu] VERIFYING→LOCKED (frame %d, verify corr=%.3f)\n", 
                            sym_idx, total_frames_, corr);
                } else {
                    // ✗ Failed verification — false alarm, back to hunting
                    state_ = SyncState::HUNTING;
                    collecting_payload_ = false;
                    verified_frame_.clear();
                    
                    fprintf(stderr, "[%zu] VERIFYING→HUNTING (verify FAILED, corr=%.3f)\n", 
                            sym_idx, corr);
                }
            }
            break;
        }
        
        case SyncState::LOCKED: {
            // Check for sync at expected position
            if (symbols_since_sync_ == FRAME_SYMBOLS) {
                double raw_corr;
                double corr = soft_correlate(&raw_corr);
                
                if (corr >= SOFT_SYNC_LOCKED_THRESHOLD) {
                    // Good sync - start collecting next payload
                    consecutive_misses_ = 0;
                    sync_quality_ = corr;
                    collecting_payload_ = true;
                    pending_frame_.clear();
                    
                    fprintf(stderr, "[%zu] LOCKED: sync OK (corr=%.3f)\n", sym_idx, corr);
                } else {
                    // Missed sync
                    consecutive_misses_++;
                    fprintf(stderr, "[%zu] LOCKED: sync MISS #%d (corr=%.3f)\n", 
                            sym_idx, consecutive_misses_, corr);
                    
                    if (consecutive_misses_ >= SYNC_MISS_LIMIT) {
                        state_ = SyncState::HUNTING;
                        collecting_payload_ = false;
                        fprintf(stderr, "[%zu] LOCKED→HUNTING (lost lock)\n", sym_idx);
                        break;
                    }
                    
                    // Flywheel: collect payload anyway assuming sync was there
                    sync_quality_ = corr;
                    collecting_payload_ = true;
                    pending_frame_.clear();
                }
                
                // Reset counter for next frame
                symbols_since_sync_ = 0;
            }
            
            // Output frame when payload is complete
            if (collecting_payload_ && pending_frame_.size() >= ENCODED_BITS) {
                res.frame_ready = true;
                res.sync_quality = sync_quality_;
                res.payload = std::move(pending_frame_);
                total_frames_++;
                
                // Prepare for next frame (will start collecting after sync)
                pending_frame_.clear();
                pending_frame_.reserve(ENCODED_BITS);
                collecting_payload_ = false;
            }
            break;
        }
        }
        
        return res;
    }
    
    SyncState get_state() const { return state_; }
    int get_total_frames() const { return total_frames_; }

private:
    // Soft correlation: returns normalized correlation (-1 to +1)
    double soft_correlate(double* raw_corr = nullptr) {
        double sum = 0.0;
        double energy = 0.0;
        
        for (size_t i = 0; i < SYNC_BITS; ++i) {
            size_t buf_idx = (corr_buf_idx_ + i) % SYNC_BITS;
            double soft = soft_corr_buf_[buf_idx];
            sum += soft * sync_pattern_[i];
            energy += std::abs(soft);
        }
        
        if (raw_corr) *raw_corr = sum;
        if (energy < MIN_SYNC_ENERGY) return 0.0;
        return sum / energy;
    }
    
    SyncState state_;
    bool symbol_locked_;     // Gate: don't search for sync unless timing is locked
    
    // Circular buffer for sync correlation (last 24 symbols)
    std::array<double, SYNC_BITS> soft_corr_buf_;
    size_t corr_buf_idx_;
    
    // Main circular buffer for all symbols
    std::array<double, CIRC_BUF_SIZE> circ_buf_;
    size_t circ_write_idx_;
    size_t total_symbols_;
    
    // Precomputed sync pattern
    std::array<double, SYNC_BITS> sync_pattern_;
    
    // Frame collection state
    bool collecting_payload_ = false;
    std::vector<double> pending_frame_;  // Collects payload symbols directly
    
    // Two-sync verification: holds first frame until second sync confirms
    std::vector<double> verified_frame_;
    double verified_quality_ = 0.0;
    
    size_t symbols_since_sync_;
    double sync_quality_;
    int consecutive_misses_;
    int total_frames_;
    
    // Thresholds
    static constexpr double SOFT_SYNC_HUNTING_THRESHOLD = 0.85;
    static constexpr double SOFT_SYNC_LOCKED_THRESHOLD = 0.70;  // Was 0.40, raised to reject noise
    static constexpr double RAW_SYNC_HUNTING_THRESHOLD = 5000.0;
    static constexpr double MIN_SYNC_ENERGY = 100.0;
};

//------------------------------------------------------------------------------
// Deinterleave
//------------------------------------------------------------------------------
inline size_t deinterleave_addr(size_t idx) {
    size_t pos = (idx % 32) * 67 + (idx / 32);
    return (pos / 8) * 8 + (7 - pos % 8);
}

//------------------------------------------------------------------------------
// Viterbi Decoder
//------------------------------------------------------------------------------
class ViterbiDecoder {
public:
    int decode(const std::array<int, ENCODED_BITS>& soft_in,
               std::array<uint8_t, FRAME_BITS>& bits_out) {
        std::array<int, NUM_STATES> metrics;
        metrics.fill(0x7FFFFFFF);
        metrics[0] = 0;
        
        std::vector<std::array<uint8_t, NUM_STATES>> decisions(FRAME_BITS);
        
        for (size_t t = 0; t < FRAME_BITS; ++t) {
            int sg1 = soft_in[t * 2], sg2 = soft_in[t * 2 + 1];
            std::array<int, NUM_STATES> next;
            next.fill(0x7FFFFFFF);
            
            for (int s = 0; s < NUM_STATES; ++s) {
                int p0 = s / 2, p1 = p0 + 32;
                int in = s % 2;
                
                int f0 = (in << 6) | p0, f1 = (in << 6) | p1;
                int e1_0 = __builtin_parity(f0 & G1_MASK), e2_0 = __builtin_parity(f0 & G2_MASK);
                int e1_1 = __builtin_parity(f1 & G1_MASK), e2_1 = __builtin_parity(f1 & G2_MASK);
                
                int bm0 = (e1_0 ? SOFT_MAX - sg1 : sg1) + (e2_0 ? SOFT_MAX - sg2 : sg2);
                int bm1 = (e1_1 ? SOFT_MAX - sg1 : sg1) + (e2_1 ? SOFT_MAX - sg2 : sg2);
                
                int m0 = (metrics[p0] < 0x7FFFFFF0) ? metrics[p0] + bm0 : 0x7FFFFFFF;
                int m1 = (metrics[p1] < 0x7FFFFFF0) ? metrics[p1] + bm1 : 0x7FFFFFFF;
                
                if (m0 <= m1) { next[s] = m0; decisions[t][s] = 0; }
                else { next[s] = m1; decisions[t][s] = 1; }
            }
            metrics = next;
        }
        
        int best = 0;
        for (int s = 1; s < NUM_STATES; ++s)
            if (metrics[s] < metrics[best]) best = s;
        
        int s = best;
        for (int t = FRAME_BITS - 1; t >= 0; --t) {
            bits_out[t] = s % 2;
            s = (decisions[t][s] == 0) ? s / 2 : s / 2 + 32;
        }
        
        return metrics[best];
    }
};

//------------------------------------------------------------------------------
// Frame Decoder
//------------------------------------------------------------------------------
class FrameDecoder {
public:
    int decode(const double* soft, std::array<uint8_t, FRAME_BYTES>& out) {
        // Scale
        double scale = 0;
        for (size_t i = 0; i < ENCODED_BITS; ++i) scale += std::abs(soft[i]);
        scale /= ENCODED_BITS;
        if (scale < 1e-10) return -1;
        
        // Quantize
        std::array<int, ENCODED_BITS> qs;
        for (size_t i = 0; i < ENCODED_BITS; ++i) {
            double n = (-soft[i] / scale) * 3.5 + 3.5;
            qs[i] = std::clamp(int(n + 0.5), 0, SOFT_MAX);
        }
        
        // Deinterleave
        std::array<int, ENCODED_BITS> deint;
        for (size_t i = 0; i < ENCODED_BITS; ++i)
            deint[i] = qs[deinterleave_addr(i)];
        
        // Viterbi
        std::array<uint8_t, FRAME_BITS> bits;
        int metric = vit_.decode(deint, bits);
        
        // Pack
        std::array<uint8_t, FRAME_BYTES> packed;
        for (size_t i = 0; i < FRAME_BYTES; ++i) {
            uint8_t b = 0;
            for (int j = 0; j < 8; ++j)
                b |= bits[FRAME_BITS - 1 - i * 8 - j] << j;
            packed[i] = b;
        }
        
        // Derandomize
        uint8_t lfsr = 0xFF;
        for (size_t i = 0; i < FRAME_BYTES; ++i) {
            uint8_t r = 0;
            for (int b = 7; b >= 0; --b) {
                r |= ((lfsr >> 7) & 1) << b;
                lfsr = (lfsr << 1) | (((lfsr >> 7) ^ (lfsr >> 6) ^ (lfsr >> 4) ^ (lfsr >> 2)) & 1);
            }
            out[i] = packed[i] ^ r;
        }
        
        return metric;
    }

private:
    ViterbiDecoder vit_;
};

//------------------------------------------------------------------------------
// ChannelReceiver - high-level per-channel orchestration (streaming)
//------------------------------------------------------------------------------
// Owns one channel's complete receive chain (demodulator + symbol-lock detector
// + sync tracker + frame decoder) and turns blocks of complex baseband into
// decoded frames.  This is the single-channel unit a multi-channel design holds
// many of: the Haifuraiya 'dogu' payload keeps one ChannelReceiver per
// channelizer output and schedules them, while the standalone opv-demod program
// drives exactly one in streaming mode.  Keeping the orchestration here (rather
// than only in opv-demod.cpp's main) means the standalone program's loopback /
// Doppler / coherent test suite exercises the very same code a host design reuses.
//
// This mirrors the non-coherent streaming path exactly: per-symbol symbol-lock
// gating drives both the sync tracker's lock input and the demodulator's tracking
// enable.  The 'gate' argument to process() reproduces the un-gated tail flush of
// the streaming loop (the final partial block of leftover samples).
//------------------------------------------------------------------------------
class ChannelReceiver {
public:
    struct Frame {
        std::array<uint8_t, FRAME_BYTES> bytes;  // decoded 134-byte frame
        int    metric       = -1;                // FEC metric (0 = perfect decode)
        double sync_quality = 0.0;               // sync correlation at capture
    };

    ChannelReceiver() = default;

    // --- configuration passthrough (callers need not reach into the demod) ---
    void   set_freq_offset(double hz)      { demod_.set_freq_offset(hz); }
    double get_freq_offset() const         { return demod_.get_freq_offset(); }
    void   set_afc_bandwidth(double alpha) { demod_.set_afc_bandwidth(alpha); }
    double estimate_offset(const sample_t* s, size_t n) { return demod_.estimate_offset(s, n); }

    // --- status accessors ---
    double    get_timing_freq() const { return demod_.get_timing_freq(); }
    bool      symbol_locked()   const { return sym_lock_.is_locked(); }
    SyncState sync_state()      const { return tracker_.get_state(); }
    size_t    get_leftover()    const { return demod_.get_leftover(); }
    size_t    total_symbols()   const { return total_symbols_; }

    // Primary streaming entry point.  Invokes on_frame(const Frame&) the instant
    // each frame completes, inline within this block's processing — so a host
    // receives frames with minimal latency and no per-block buffering.  (It also
    // keeps frame emission interleaved with the receiver's own diagnostics in the
    // exact order the original streaming loop produced.)
    //
    // gate=true applies per-symbol symbol-lock gating (the normal streaming chunk
    // path); gate=false runs un-gated, reproducing the streaming loop's final tail
    // flush of leftover samples.
    template <class OnFrame>
    void process(const sample_t* samples, size_t n, OnFrame&& on_frame, bool gate = true) {
        std::vector<double> soft, ted_vals;
        demod_.demodulate(samples, n, soft, &ted_vals);

        for (size_t i = 0; i < soft.size(); ++i) {
            if (gate) {
                double ted = (i < ted_vals.size()) ? ted_vals[i] : 0.0;
                bool sym_locked = sym_lock_.update(ted);
                tracker_.set_symbol_lock(sym_locked);
                demod_.set_tracking_enabled(sym_locked);
            }

            auto res = tracker_.process(soft[i], total_symbols_ + i);
            if (res.frame_ready && !res.payload.empty()) {
                Frame f;
                int metric = fdec_.decode(res.payload.data(), f.bytes);
                if (metric >= 0) {
                    f.metric       = metric;
                    f.sync_quality = res.sync_quality;
                    on_frame(static_cast<const Frame&>(f));
                }
            }
        }
        total_symbols_ += soft.size();
    }

    // Convenience: collect this block's completed frames into a vector instead of
    // supplying a callback.  Frames are returned after the whole block is
    // processed (fine for offline/file use; for live hosts prefer the callback
    // form above so frames are delivered the moment they complete).
    std::vector<Frame> process(const sample_t* samples, size_t n, bool gate = true) {
        std::vector<Frame> frames;
        process(samples, n, [&frames](const Frame& f){ frames.push_back(f); }, gate);
        return frames;
    }

private:
    MSKDemodulatorAFC  demod_;
    SyncTracker        tracker_;
    FrameDecoder       fdec_;
    SymbolLockDetector sym_lock_;
    size_t             total_symbols_ = 0;
};

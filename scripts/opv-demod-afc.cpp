//------------------------------------------------------------------------------
// opv-demod-afc.cpp - OPV MSK Demodulator with AFC and Sync State Machine
//------------------------------------------------------------------------------
// MSK demodulator with automatic frequency control and proper sync tracking.
// Uses correlation-based detection (robust) with frequency offset estimation
// and correction (tracks drift).
//
// Signal parameters:
//   MSK modulation: F1=-13550 Hz (bit '1'), F2=+13550 Hz (bit '0')
//   Symbol rate: 54.2 kbaud (40 samples/symbol at 2.168 MSPS)
//   Sync word: 0x02B8DB (24 bits)
//   Frame: 24-bit sync + 2144 encoded bits = 2168 symbols
//
// Architecture:
//   - Dual-tone correlation with integrate-and-dump
//   - AFC: estimates frequency offset from tone phase rotation
//   - State machine: HUNTING → VERIFYING → LOCKED (flywheel)
//
// Author: ORI/Abraxas3d collaboration
// License: CERN-OHL-S v2
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
constexpr int SYNC_MISS_LIMIT = 5;          // Allow more misses before losing lock

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

const char* state_name(SyncState s) {
    switch (s) {
        case SyncState::HUNTING: return "HUNTING";
        case SyncState::VERIFYING: return "VERIFYING";
        case SyncState::LOCKED: return "LOCKED";
    }
    return "?";
}

//------------------------------------------------------------------------------
// Base-40 Decoder
//------------------------------------------------------------------------------
std::string decode_base40(const uint8_t* bytes, size_t len = 6) {
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
          afc_alpha_(0.001) {}
    
    // Estimate carrier offset from spectrum (coarse AFC)
    double estimate_offset(const sample_t* samples, size_t num_samples) {
        // Try a range of offsets and find the one with best total tone energy
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
                
                // Sum of both tone energies (MSK uses one or the other per symbol)
                total_energy += std::norm(corr_f1) + std::norm(corr_f2);
            }
            
            if (total_energy > best_energy) {
                best_energy = total_energy;
                best_offset = offset;
            }
        }
        
        // Fine tune with smaller steps
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
            
            // Integrate over symbol
            for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
                size_t idx = sym * SAMPLES_PER_SYMBOL + i;
                
                std::complex<double> lo_f1(std::cos(phase_f1_), std::sin(phase_f1_));
                std::complex<double> lo_f2(std::cos(phase_f2_), std::sin(phase_f2_));
                
                corr_f1 += samples[idx] * std::conj(lo_f1);
                corr_f2 += samples[idx] * std::conj(lo_f2);
                
                phase_f1_ += phase_inc_f1;
                phase_f2_ += phase_inc_f2;
            }
            
            // Wrap phases
            while (phase_f1_ > PI) phase_f1_ -= TWO_PI;
            while (phase_f1_ < -PI) phase_f1_ += TWO_PI;
            while (phase_f2_ > PI) phase_f2_ -= TWO_PI;
            while (phase_f2_ < -PI) phase_f2_ += TWO_PI;
            
            double f1_energy = std::norm(corr_f1);
            double f2_energy = std::norm(corr_f2);
            
            // Soft decision: positive = F2 (bit 0), negative = F1 (bit 1)
            soft_out.push_back(f2_energy - f1_energy);
            
            // AFC: estimate frequency error from phase rotation of dominant tone
            if (sym > 0) {
                std::complex<double> dominant_corr, prev_dominant;
                if (f1_energy > f2_energy) {
                    dominant_corr = corr_f1;
                    prev_dominant = prev_corr_f1_;
                } else {
                    dominant_corr = corr_f2;
                    prev_dominant = prev_corr_f2_;
                }
                
                // Phase difference between symbols indicates frequency error
                double phase_diff = std::arg(dominant_corr * std::conj(prev_dominant));
                
                // Convert to frequency: phase_diff per symbol period
                double freq_err = phase_diff * SYMBOL_RATE / TWO_PI;
                
                // Update frequency estimate (low-pass filter)
                freq_offset_ += afc_alpha_ * freq_err;
                
                // Limit AFC range
                freq_offset_ = std::clamp(freq_offset_, -2000.0, 2000.0);
                
                // Update phase increments with new estimate
                phase_inc_f1 = TWO_PI * (-FREQ_DEV + freq_offset_) / SAMPLE_RATE;
                phase_inc_f2 = TWO_PI * (+FREQ_DEV + freq_offset_) / SAMPLE_RATE;
            }
            
            prev_corr_f1_ = corr_f1;
            prev_corr_f2_ = corr_f2;
        }
    }
    
    double get_freq_offset() const { return freq_offset_; }
    void set_afc_bandwidth(double alpha) { afc_alpha_ = alpha; }

private:
    double freq_offset_;
    double phase_f1_, phase_f2_;
    std::complex<double> prev_corr_f1_, prev_corr_f2_;
    double afc_alpha_;
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
          afc_alpha_(0.001),
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
            
            // ===== AFC: COARSE FREQUENCY TRACKING =====
            // This helps when residual offset exceeds PLL pull-in range
            // Use phase rotation of dominant tone between symbols
            if (sym > 0) {
                double phase_diff = std::arg(dominant * std::conj(prev_dominant_));
                double freq_err = phase_diff * SYMBOL_RATE / TWO_PI;
                freq_offset_ += afc_alpha_ * freq_err;
                freq_offset_ = std::clamp(freq_offset_, -2000.0, 2000.0);
                
                phase_inc_f1 = TWO_PI * (-FREQ_DEV + freq_offset_) / SAMPLE_RATE;
                phase_inc_f2 = TWO_PI * (+FREQ_DEV + freq_offset_) / SAMPLE_RATE;
            }
            
            prev_dominant_ = dominant;
        }
    }
    
    double get_freq_offset() const { return freq_offset_; }
    void set_afc_bandwidth(double alpha) { afc_alpha_ = alpha; }
    
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
    std::complex<double> prev_dominant_;
    double afc_alpha_;
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
                    circ_write_idx_(0), total_symbols_(0) {
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
    }
    
    struct Result {
        bool frame_ready;
        double sync_quality;  // Normalized correlation value (-1 to +1)
        std::vector<double> payload;  // The actual payload data (copied out)
    };
    
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
            if (symbols_since_sync_ >= ENCODED_BITS) {
                // Frame complete - output it
                res.frame_ready = true;
                res.sync_quality = sync_quality_;
                res.payload = std::move(pending_frame_);
                total_frames_++;
                
                // Prepare for next frame
                pending_frame_.clear();
                pending_frame_.reserve(ENCODED_BITS);
                collecting_payload_ = false;  // Will start after next sync
                
                // Transition to LOCKED
                state_ = SyncState::LOCKED;
                consecutive_misses_ = 0;
                
                // symbols_since_sync_ stays at ENCODED_BITS
                // Next sync expected in SYNC_BITS more symbols
                
                fprintf(stderr, "[%zu] VERIFYING→LOCKED (frame %d)\n", sym_idx, total_frames_);
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
    
    size_t symbols_since_sync_;
    double sync_quality_;
    int consecutive_misses_;
    int total_frames_;
    
    // Thresholds
    static constexpr double SOFT_SYNC_HUNTING_THRESHOLD = 0.85;
    static constexpr double SOFT_SYNC_LOCKED_THRESHOLD = 0.40;
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
// Frame Display
//------------------------------------------------------------------------------
void print_frame(int num, const std::array<uint8_t, FRAME_BYTES>& f, int metric, double sync_corr) {
    fprintf(stderr, "┌─────────────────────────────────────────────────────────────────┐\n");
    fprintf(stderr, "│ FRAME %4d  │  Sync: %.3f  │  Metric: %5d", num, sync_corr, metric);
    if (metric == 0) fprintf(stderr, " (perfect)");
    fprintf(stderr, "\n├─────────────────────────────────────────────────────────────────┤\n");
    
    fprintf(stderr, "│ Station ID:  %-12s (Base-40)\n", decode_base40(&f[0]).c_str());
    
    uint32_t tok = (f[6] << 16) | (f[7] << 8) | f[8];
    fprintf(stderr, "│ Token:       0x%06X%s\n", tok, (tok == 0xBBAADD) ? " (default)" : "");
    
    uint32_t res = (f[9] << 16) | (f[10] << 8) | f[11];
    fprintf(stderr, "│ Reserved:    0x%06X\n", res);
    
    fprintf(stderr, "├─────────────────────────────────────────────────────────────────┤\n");
    fprintf(stderr, "│ Hex Dump:                                                       │\n");
    
    for (size_t i = 0; i < FRAME_BYTES; i += 16) {
        fprintf(stderr, "│ %02zx: ", i);
        for (size_t j = i; j < i + 16 && j < FRAME_BYTES; ++j)
            fprintf(stderr, "%02X ", f[j]);
        for (size_t j = FRAME_BYTES; j < i + 16; ++j)
            fprintf(stderr, "   ");
        fprintf(stderr, " │");
        for (size_t j = i; j < i + 16 && j < FRAME_BYTES; ++j) {
            char c = (f[j] >= 0x20 && f[j] < 0x7F) ? f[j] : '.';
            fprintf(stderr, "%c", c);
        }
        fprintf(stderr, "│\n");
    }
    fprintf(stderr, "└─────────────────────────────────────────────────────────────────┘\n\n");
}

//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    bool quiet = false, raw = false, coherent = false, streaming = false;
    double afc_bw = 0.001;
    double pll_bw = 50.0;  // PLL bandwidth in Hz
    double init_offset = 0.0;  // Initial frequency offset for streaming mode
    bool have_init_offset = false;
    
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-q")) quiet = true;
        else if (!strcmp(argv[i], "-r")) raw = true;
        else if (!strcmp(argv[i], "-c")) coherent = true;
        else if (!strcmp(argv[i], "-s")) streaming = true;
        else if (!strcmp(argv[i], "-a") && i + 1 < argc) afc_bw = atof(argv[++i]);
        else if (!strcmp(argv[i], "-p") && i + 1 < argc) pll_bw = atof(argv[++i]);
        else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
            init_offset = atof(argv[++i]);
            have_init_offset = true;
        }
        else if (!strcmp(argv[i], "-h")) {
            fprintf(stderr, "Usage: %s [options] < input.iq\n\n", argv[0]);
            fprintf(stderr, "Options:\n");
            fprintf(stderr, "  -q          Quiet mode\n");
            fprintf(stderr, "  -r          Raw output to stdout\n");
            fprintf(stderr, "  -s          Streaming mode (for live PlutoSDR input)\n");
            fprintf(stderr, "  -c          Coherent mode (Costas loop, ~3dB better)\n");
            fprintf(stderr, "  -a <bw>     AFC bandwidth (default: 0.001)\n");
            fprintf(stderr, "  -o <hz>     Initial frequency offset (streaming mode)\n");
            fprintf(stderr, "  -p <hz>     PLL bandwidth in Hz (default: 50, coherent only)\n");
            fprintf(stderr, "  -h          Help\n");
            return 0;
        }
    }
    
    if (!quiet) {
        fprintf(stderr, "╔═══════════════════════════════════════════════════════════════════╗\n");
        if (coherent)
            fprintf(stderr, "║       OPV MSK Demodulator with Costas Loop v1.0 (coherent)       ║\n");
        else if (streaming)
            fprintf(stderr, "║       OPV MSK Demodulator with AFC v1.0 (streaming)              ║\n");
        else
            fprintf(stderr, "║           OPV MSK Demodulator with AFC v1.0                       ║\n");
        fprintf(stderr, "╚═══════════════════════════════════════════════════════════════════╝\n\n");
    }
    
    // =========================================================================
    // STREAMING MODE - process data as it arrives
    // =========================================================================
    if (streaming) {
        if (!quiet)
            fprintf(stderr, "Streaming mode: processing data as it arrives...\n\n");
        
        MSKDemodulatorAFC demod;
        SyncTracker tracker;
        FrameDecoder fdec;
        
        // Set initial offset if provided, otherwise start at 0
        if (have_init_offset) {
            demod.set_freq_offset(init_offset);
            if (!quiet)
                fprintf(stderr, "Initial frequency offset: %.1f Hz\n", init_offset);
        }
        demod.set_afc_bandwidth(afc_bw);
        
        // Process in chunks of one frame worth of samples
        const size_t CHUNK_SAMPLES = FRAME_SYMBOLS * SAMPLES_PER_SYMBOL;  // ~86720 samples
        std::vector<sample_t> chunk_buf;
        chunk_buf.reserve(CHUNK_SAMPLES);
        
        int decoded = 0, perfect = 0;
        size_t total_samples = 0;
        size_t total_symbols = 0;
        bool first_chunk = true;
        
        IQSample iq;
        while (std::cin.read(reinterpret_cast<char*>(&iq), sizeof(iq))) {
            chunk_buf.push_back(sample_t(iq.I, iq.Q));
            
            // Process when we have a full chunk
            if (chunk_buf.size() >= CHUNK_SAMPLES) {
                total_samples += chunk_buf.size();
                
                // On first chunk, estimate frequency offset
                if (first_chunk) {
                    if (!have_init_offset) {
                        double est_offset = demod.estimate_offset(chunk_buf.data(), chunk_buf.size());
                        demod.set_freq_offset(est_offset);
                        if (!quiet)
                            fprintf(stderr, "Estimated carrier offset: %.1f Hz\n\n", est_offset);
                    }
                    first_chunk = false;
                }
                
                // Demodulate chunk
                std::vector<double> soft;
                demod.demodulate(chunk_buf.data(), chunk_buf.size(), soft);
                
                // Process through sync tracker
                for (size_t i = 0; i < soft.size(); ++i) {
                    auto res = tracker.process(soft[i], total_symbols + i);
                    
                    if (res.frame_ready && !res.payload.empty()) {
                        std::array<uint8_t, FRAME_BYTES> frame;
                        int metric = fdec.decode(res.payload.data(), frame);
                        
                        if (metric >= 0) {
                            decoded++;
                            if (metric == 0) perfect++;
                            
                            if (!quiet)
                                print_frame(decoded, frame, metric, res.sync_quality);
                            
                            if (raw)
                                std::cout.write(reinterpret_cast<char*>(frame.data()), FRAME_BYTES);
                            
                            std::cout.flush();  // Flush immediately in streaming mode
                        }
                    }
                }
                
                total_symbols += soft.size();
                chunk_buf.clear();
                
                // Periodic status update
                if (!quiet && (total_samples % (size_t)(SAMPLE_RATE * 5) < CHUNK_SAMPLES)) {
                    fprintf(stderr, "[%.1fs] %zu symbols, %d frames (%d perfect), AFC: %.1f Hz\n",
                            total_samples / SAMPLE_RATE, total_symbols, decoded, perfect,
                            demod.get_freq_offset());
                }
            }
        }
        
        // Process any remaining samples
        if (!chunk_buf.empty()) {
            std::vector<double> soft;
            demod.demodulate(chunk_buf.data(), chunk_buf.size(), soft);
            
            for (size_t i = 0; i < soft.size(); ++i) {
                auto res = tracker.process(soft[i], total_symbols + i);
                
                if (res.frame_ready && !res.payload.empty()) {
                    std::array<uint8_t, FRAME_BYTES> frame;
                    int metric = fdec.decode(res.payload.data(), frame);
                    
                    if (metric >= 0) {
                        decoded++;
                        if (metric == 0) perfect++;
                        
                        if (!quiet)
                            print_frame(decoded, frame, metric, res.sync_quality);
                        
                        if (raw)
                            std::cout.write(reinterpret_cast<char*>(frame.data()), FRAME_BYTES);
                    }
                }
            }
        }
        
        if (!quiet) {
            fprintf(stderr, "\n════════════════════════════════════════════════════════════════════\n");
            fprintf(stderr, "Summary: %d frames (%d perfect, %d errors)\n", decoded, perfect, decoded - perfect);
            fprintf(stderr, "Total: %.3f sec, %zu symbols\n", total_samples / SAMPLE_RATE, total_symbols);
            fprintf(stderr, "Final state: %s, AFC: %.1f Hz\n", 
                    state_name(tracker.get_state()), demod.get_freq_offset());
            fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
        }
        
        return decoded > 0 ? 0 : 1;
    }
    
    // =========================================================================
    // BATCH MODE - load all samples then process (original behavior)
    // =========================================================================
    
    // Load samples
    std::vector<sample_t> samples;
    IQSample iq;
    while (std::cin.read(reinterpret_cast<char*>(&iq), sizeof(iq)))
        samples.push_back(sample_t(iq.I, iq.Q));
    
    if (!quiet)
        fprintf(stderr, "Loaded %zu samples (%.3f sec)\n", samples.size(), samples.size() / SAMPLE_RATE);
    
    // Demodulate using selected mode
    std::vector<double> soft;
    double final_offset;
    
    if (coherent) {
        // Coherent demodulation with Costas loop
        CoherentMSKDemodulator demod;
        
        double est_offset = demod.estimate_offset(samples.data(), samples.size());
        demod.set_freq_offset(est_offset);
        
        if (!quiet)
            fprintf(stderr, "Estimated carrier offset: %.1f Hz\n", est_offset);
        
        demod.set_afc_bandwidth(afc_bw);
        demod.set_pll_bandwidth(pll_bw);
        
        if (!quiet)
            fprintf(stderr, "PLL bandwidth: %.1f Hz\n", pll_bw);
        
        demod.demodulate(samples.data(), samples.size(), soft);
        final_offset = demod.get_freq_offset();
    } else {
        // Non-coherent energy detection (original)
        MSKDemodulatorAFC demod;
        
        double est_offset = demod.estimate_offset(samples.data(), samples.size());
        demod.set_freq_offset(est_offset);
        
        if (!quiet)
            fprintf(stderr, "Estimated carrier offset: %.1f Hz\n", est_offset);
        
        demod.set_afc_bandwidth(afc_bw);
        demod.demodulate(samples.data(), samples.size(), soft);
        final_offset = demod.get_freq_offset();
    }
    
    if (!quiet)
        fprintf(stderr, "Demodulated %zu symbols, final AFC offset: %.1f Hz\n\n", 
                soft.size(), final_offset);
    
    // Process through state machine
    SyncTracker tracker;
    FrameDecoder fdec;
    int decoded = 0, perfect = 0;
    
    for (size_t i = 0; i < soft.size(); ++i) {
        auto res = tracker.process(soft[i], i);
        
        if (res.frame_ready && !res.payload.empty()) {
            std::array<uint8_t, FRAME_BYTES> frame;
            int metric = fdec.decode(res.payload.data(), frame);
            
            if (metric >= 0) {
                decoded++;
                if (metric == 0) perfect++;
                
                if (!quiet)
                    print_frame(decoded, frame, metric, res.sync_quality);
                
                if (raw)
                    std::cout.write(reinterpret_cast<char*>(frame.data()), FRAME_BYTES);
            }
        }
    }
    
    if (!quiet) {
        fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
        fprintf(stderr, "Summary: %d frames (%d perfect, %d errors)\n", decoded, perfect, decoded - perfect);
        fprintf(stderr, "Final state: %s, AFC: %.1f Hz\n", 
                state_name(tracker.get_state()), final_offset);
        fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
    }
    
    return decoded > 0 ? 0 : 1;
}

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
        : freq_offset_(0),
          // pll_alpha_/pll_beta_ are vestigial now: demodulate() uses fixed Costas
          // gains; set_pll_bandwidth() (called from the driver) still writes them.
          pll_alpha_(0.01),
          pll_beta_(0.001)
    {}
    
    // Estimate carrier offset from spectrum (coarse AFC) - same as non-coherent
    // ---- iterative radix-2 FFT (in-place, N a power of two) ----
    static void fft_inplace(std::vector<std::complex<double>>& a) {
        size_t N = a.size();
        for (size_t i = 1, j = 0; i < N; ++i) {          // bit-reversal permutation
            size_t bit = N >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) std::swap(a[i], a[j]);
        }
        for (size_t len = 2; len <= N; len <<= 1) {
            double ang = -TWO_PI / (double)len;
            std::complex<double> wlen(std::cos(ang), std::sin(ang));
            for (size_t i = 0; i < N; i += len) {
                std::complex<double> w(1.0, 0.0);
                for (size_t k = 0; k < len/2; ++k) {
                    std::complex<double> u = a[i+k], v = a[i+k+len/2]*w;
                    a[i+k] = u + v; a[i+k+len/2] = u - v;
                    w *= wlen;
                }
            }
        }
    }

    // de Buda coarse carrier estimate. Squaring removes the MSK data, leaving
    // spectral lines at 2*foff +/- 2*FREQ_DEV; their midpoint/2 is foff. Returns
    // the REFERENCE CORRECTION (= -foff): set_freq_offset() of this value centres
    // the tone correlators so the Costas only has to track the residual. (The
    // seed sign was validated in the offline model -- +foff diverges.)
    // de Buda coarse carrier estimate (Welch-averaged for low-SNR robustness).
    // Square removes the MSK data -> deterministic lines at 2*foff +/- 2*FREQ_DEV;
    // averaging |FFT(s^2)|^2 over many windows stacks the lines while noise averages
    // down. Midpoint of the two lines is 2*foff. Returns the REFERENCE CORRECTION
    // (= -foff) for set_freq_offset(); a confidence gate returns 0 (do no harm) when
    // no clear line is present. Seed sign validated in the offline model.
    // de Buda coarse carrier estimate (Welch-averaged, joint two-line search).
    // Squaring removes the MSK data, leaving deterministic lines at 2*foff +/- 2*FREQ_DEV
    // (always exactly 4*FREQ_DEV apart). Averaging |FFT(s^2)|^2 over many windows stacks
    // the lines while noise averages down; a joint scan over candidate offsets finds the
    // pair. Returns the REFERENCE CORRECTION (= -foff) for set_freq_offset(); a confidence
    // gate returns 0 (do no harm) when no clear pair is present. Seed sign and the +/-15 kHz
    // span (LEO Doppler at 435 MHz + LO offset) chosen to cover real on-orbit conditions.
    double estimate_offset(const sample_t* samples, size_t num_samples) {
        const size_t W = 32768;
        if (num_samples < W) return 0.0;
        const size_t K = std::min<size_t>(512, num_samples / W);
        std::vector<double> P(W, 0.0);
        std::vector<std::complex<double>> buf(W);
        for (size_t w = 0; w < K; ++w) {
            const sample_t* base = samples + w * W;
            for (size_t i = 0; i < W; ++i) { std::complex<double> s = base[i]; buf[i] = s * s; }
            fft_inplace(buf);
            for (size_t k = 0; k < W; ++k) P[k] += std::norm(buf[k]);
        }
        const double binHz = SAMPLE_RATE / (double)W;
        auto Pat = [&](double f)->double {              // power at frequency f (wraps)
            long k = std::lround(f / binHz) % (long)W; if (k < 0) k += (long)W;
            return P[(size_t)k];
        };
        // joint scan: the two lines sit at 2*foff +/- 2*FREQ_DEV
        const double foff_max = 5000.0;   // pre-corrected residual + LO offset; widen if uncorrected
        double best = -1.0, bestf = 0.0;
        for (double fc = -foff_max; fc <= foff_max; fc += binHz * 0.5) {
            double sc = Pat(2.0*fc - 2.0*FREQ_DEV) + Pat(2.0*fc + 2.0*FREQ_DEV);
            if (sc > best) { best = sc; bestf = fc; }
        }
        std::vector<double> tmp(P);                      // confidence gate vs median floor
        std::nth_element(tmp.begin(), tmp.begin() + W/2, tmp.end());
        double floor = tmp[W/2] + 1e-12;
        if (best < 16.0 * floor) return 0.0;             // no clear pair -> don't detune
        return -bestf;                                   // reference correction
    }
    
    void set_freq_offset(double offset) { freq_offset_ = offset; }
    
    // Front-end (batch / native 40 sps): per-symbol tone correlations over fixed
    // integer windows, continuous-phase reference. Active tone lands its energy
    // on the imaginary axis (sign = differential precoding); wrong-tone crosstalk
    // lands on the real axis. Feeds combine(); the fractional track_correlations()
    // is the channelized-rate alternative. No sync word -- protocol-agnostic.
    void batch_correlations(const sample_t* samples, size_t num_samples,
                            std::vector<std::complex<double>>& Y1,
                            std::vector<std::complex<double>>& Y2) const {
        size_t nsym = num_samples / SAMPLES_PER_SYMBOL;
        Y1.assign(nsym, std::complex<double>(0,0));
        Y2.assign(nsym, std::complex<double>(0,0));
        if (nsym < 3) return;
        const double inc1 = TWO_PI * (-FREQ_DEV + freq_offset_) / SAMPLE_RATE;
        const double inc2 = TWO_PI * (+FREQ_DEV + freq_offset_) / SAMPLE_RATE;
        const std::complex<double> w1(std::cos(inc1), std::sin(inc1));
        const std::complex<double> w2(std::cos(inc2), std::sin(inc2));
        for (size_t k = 0; k < nsym; ++k) {
            double b1 = inc1 * (double)(k * SAMPLES_PER_SYMBOL);
            double b2 = inc2 * (double)(k * SAMPLES_PER_SYMBOL);
            std::complex<double> lo1(std::cos(b1), std::sin(b1));
            std::complex<double> lo2(std::cos(b2), std::sin(b2));
            std::complex<double> a1(0,0), a2(0,0);
            for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
                size_t n = k * SAMPLES_PER_SYMBOL + i;
                a1 += samples[n] * lo1;  a2 += samples[n] * lo2;
                lo1 *= w1;               lo2 *= w2;
            }
            Y1[k] = a1;  Y2[k] = a2;
        }
    }
    double get_freq_offset() const { return freq_offset_; }

    // ====================================================================
    // Fractional-timing front-end (for the A53 / native channelized rate)
    // --------------------------------------------------------------------
    // The batch demodulate() above assumes an integer SAMPLES_PER_SYMBOL and
    // fixed windows k*SPS. At a channelized rate (~11.53 sps for a 625 ksps
    // FunCube+ channel) the samples/symbol is non-integer, so we recover symbol
    // timing with a PI loop instead of slicing fixed windows. SPS becomes a
    // nominal real number (sps_nom_); the loop tracks the true symbol instants.
    //
    // track_correlations() emits the per-symbol tone correlations Y1,Y2 at the
    // recovered instants -- the SAME quantities the batch path builds -- so the
    // existing Costas + Massey + parity back-end consumes them unchanged.
    // ====================================================================
    void set_nominal_sps(double sps) { sps_nom_ = sps; }
    double get_nominal_sps() const { return sps_nom_; }

    // Design the PI timing loop from a normalized loop bandwidth (Bn*T) and
    // damping. ted_slope_ is the measured normalized TED gain (err per sample
    // of timing error); defaults are validated in the offline model.
    void set_timing_bandwidth(double BnT, double zeta = 1.0) {
        double wnT = TWO_PI * BnT;
        alpha_t_ = (2.0 * zeta * wnT) / ted_slope_;
        beta_t_  = (wnT * wnT)        / ted_slope_;
    }
    void set_timing_gains(double a, double b) { alpha_t_ = a; beta_t_ = b; }

    // Interpolating matched filter: tone correlations over one symbol of length
    // sps_nom_, sampled at MF_M sub-points, continuous-phase reference evaluated
    // at the absolute fractional position `base`. Mirrors the batch reference
    // (lo = e^{+j inc n}; Y = sum s*lo) but at fractional positions.
    void corr_at(const sample_t* s, size_t n, double base,
                 std::complex<double>& Y1, std::complex<double>& Y2) const {
        const double Fs   = sps_nom_ * SYMBOL_RATE;
        const double inc1 = TWO_PI * (-FREQ_DEV + freq_offset_) / Fs;
        const double inc2 = TWO_PI * (+FREQ_DEV + freq_offset_) / Fs;
        const int    M    = std::max(MF_M, (int)std::lround(sps_nom_)); // >=1 sub-sample/sample
        const double step = sps_nom_ / (double)M;
        // LO phase is referenced to the ABSOLUTE sample position (base + the
        // cumulative streaming offset), not the buffer-local position. This keeps
        // the tone reference phase-continuous across streaming chunk boundaries;
        // strm_abs_base_ is 0 for the batch/one-shot path, so they are unaffected.
        const double ph   = base + strm_abs_base_;
        std::complex<double> lo1(std::cos(inc1*ph), std::sin(inc1*ph));
        std::complex<double> lo2(std::cos(inc2*ph), std::sin(inc2*ph));
        const std::complex<double> w1(std::cos(inc1*step), std::sin(inc1*step));
        const std::complex<double> w2(std::cos(inc2*step), std::sin(inc2*step));
        std::complex<double> a1(0,0), a2(0,0);
        for (int j = 0; j < M; ++j) {
            double p = base + j*step;
            sample_t v = interp_lin(s, p, n);
            a1 += v*lo1;  a2 += v*lo2;
            lo1 *= w1;    lo2 *= w2;
        }
        Y1 = a1;  Y2 = a2;
    }

    // PI timing loop with normalized ML-gradient TED (err = Re{conj(y)*dy/dtau},
    // y the dominant tone). Produces timing-recovered Y1,Y2.
    void track_correlations(const sample_t* s, size_t n,
                            std::vector<std::complex<double>>& Y1o,
                            std::vector<std::complex<double>>& Y2o) {
        Y1o.clear();  Y2o.clear();
        const double EL = el_offset_;
        double pos = EL + 1.0, freq = 0.0;
        while (pos + sps_nom_ + EL + 2.0 < (double)n) {
            std::complex<double> Y1, Y2, Y1e, Y2e, Y1l, Y2l;
            corr_at(s, n, pos,      Y1,  Y2);
            corr_at(s, n, pos - EL, Y1e, Y2e);
            corr_at(s, n, pos + EL, Y1l, Y2l);
            bool t1 = std::norm(Y1) > std::norm(Y2);
            std::complex<double> ya = t1 ? Y1 : Y2;
            std::complex<double> dy = t1 ? (Y1l - Y1e) : (Y2l - Y2e);
            double err = (ya.real()*dy.real() + ya.imag()*dy.imag())
                       / (std::norm(ya) + 1e-9);
            freq += beta_t_ * err;
            freq  = std::clamp(freq, -0.05, 0.05);
            double adj = alpha_t_ * err + freq;
            adj = std::clamp(adj, -2.0, 2.0);
            Y1o.push_back(Y1);  Y2o.push_back(Y2);
            pos += sps_nom_ + adj;
        }
    }

    // Back-end: decision-switched Costas + Massey 2T combine + soft differential
    // decode, emitting BOTH parity interpretations (dec0, dec1). No sync word and
    // no polarity decision are made here -- the sync correlator (which owns the
    // protocol) resolves the 4-fold parity/polarity ambiguity downstream. The
    // front-end (batch or fractional-tracked) supplies the tone correlations.
    // This is what keeps the demodulator protocol-agnostic and reusable.
    void combine(const std::vector<std::complex<double>>& Y1,
                 const std::vector<std::complex<double>>& Y2,
                 std::vector<double>& dec0, std::vector<double>& dec1) const {
        size_t nsym = Y1.size();
        dec0.assign(nsym, 0.0);  dec1.assign(nsym, 0.0);
        if (nsym < 3) return;
        // Decision-switched Costas (Hodgart form) -> de-rotated arms X, Yv.
        // Runs once; it is parity-independent (operates on the dominant tone).
        std::vector<double> X(nsym, 0.0), Yv(nsym, 0.0);
        {
            const double pll_a = 0.01, pll_b = 2e-4;
            double theta = 0.0, freq = 0.0;
            for (size_t k = 0; k < nsym; ++k) {
                std::complex<double> rot(std::cos(theta), -std::sin(theta));
                std::complex<double> y1 = Y1[k]*rot, y2 = Y2[k]*rot;
                X[k]  = y1.imag();  Yv[k] = y2.imag();
                const std::complex<double>& act = (std::norm(y2) > std::norm(y1)) ? y2 : y1;
                double m = std::abs(act) + 1e-9;
                double err = -(act.real() * ((act.imag() < 0) ? -1.0 : 1.0)) / m;
                freq  += pll_b * err;
                theta += pll_a * err + freq;
            }
        }
        auto boxplus = [](double a, double b) {
            double s = ((a < 0) != (b < 0)) ? -1.0 : 1.0;
            return s * std::min(std::fabs(a), std::fabs(b));
        };
        for (int parity = 0; parity < 2; ++parity) {
            std::vector<double> enc(nsym, 0.0);
            for (size_t i = 0; i + 1 < nsym; ++i) {
                double A = X[i] + X[i+1], B = Yv[i] + Yv[i+1];
                double sgn = (((i + parity) & 1) == 0) ? 1.0 : -1.0;
                enc[i] = A - sgn * B;
            }
            std::vector<double>& dec = (parity == 0) ? dec0 : dec1;
            dec[0] = 0.0;
            for (size_t i = 1; i < nsym; ++i) dec[i] = boxplus(enc[i], enc[i-1]);
        }
    }

    // Streaming front-end: stateful across chunks. Carries the timing loop, the
    // Costas phase, and the 2T/differential boundary state so the symbol stream
    // is CONTINUOUS across block boundaries (a dropped symbol would flip parity).
    // Emits both parity streams (dec0,dec1) for the symbols completed in this
    // block, and reports how many input samples were consumed; the caller keeps
    // the unconsumed tail and prepends it to the next block.
    void demodulate_stream(const sample_t* s, size_t n,
                           std::vector<double>& dec0, std::vector<double>& dec1,
                           size_t& consumed,
                           std::vector<std::complex<double>>* Y1dbg = nullptr,
                           std::vector<std::complex<double>>* Y2dbg = nullptr) {
        if (!strm_init_) { strm_pos_ = el_offset_ + 1.0; strm_init_ = true; }
        const double EL = el_offset_;
        auto boxplus = [](double a, double b) {
            double sg = ((a < 0) != (b < 0)) ? -1.0 : 1.0;
            return sg * std::min(std::fabs(a), std::fabs(b));
        };
        double pos = strm_pos_;
        // Forward margin must cover the LATE gate's matched-filter footprint
        // (base=pos+EL, M sub-samples spanning ~sps_nom_) plus the cubic interp
        // margin, with slack -- otherwise interp_lin clamps the tail samples and
        // corrupts X[last] (which also poisons the prior symbol's 2T pairing).
        // Any symbol short of this is deferred; the leftover tail carries it to
        // the next block where the prepended samples give it clean context.
        const double fwd_margin = sps_nom_ + EL + 4.0;
        while (pos + fwd_margin < (double)n && pos - EL - 1.0 >= 0.0) {
            std::complex<double> Y1, Y2, Y1e, Y2e, Y1l, Y2l;
            corr_at(s, n, pos,      Y1,  Y2);
            corr_at(s, n, pos - EL, Y1e, Y2e);
            corr_at(s, n, pos + EL, Y1l, Y2l);
            if (Y1dbg) Y1dbg->push_back(Y1);
            if (Y2dbg) Y2dbg->push_back(Y2);

            // timing TED (normalized ML-gradient on the dominant tone)
            bool t1 = std::norm(Y1) > std::norm(Y2);
            std::complex<double> ya = t1 ? Y1 : Y2;
            std::complex<double> dy = t1 ? (Y1l - Y1e) : (Y2l - Y2e);
            double terr = (ya.real()*dy.real() + ya.imag()*dy.imag()) / (std::norm(ya) + 1e-9);
            strm_tfreq_ += beta_t_ * terr;
            strm_tfreq_  = std::clamp(strm_tfreq_, -0.05, 0.05);
            double adj = std::clamp(alpha_t_ * terr + strm_tfreq_, -2.0, 2.0);

            // decision-switched Costas (streaming) -> de-rotated arms
            std::complex<double> rot(std::cos(cb_theta_), -std::sin(cb_theta_));
            std::complex<double> y1 = Y1*rot, y2 = Y2*rot;
            double Xk = y1.imag(), Yvk = y2.imag();
            const std::complex<double>& act = (std::norm(y2) > std::norm(y1)) ? y2 : y1;
            double m = std::abs(act) + 1e-9;
            double cerr = -(act.real() * ((act.imag() < 0) ? -1.0 : 1.0)) / m;
            cb_cfreq_ += 2e-4 * cerr;
            cb_theta_ += 0.01 * cerr + cb_cfreq_;

            // 2T combine + soft-differential, one-symbol delayed (needs X[k+1]).
            if (cb_have_) {
                double A = cb_Xp_ + Xk, B = cb_Yvp_ + Yvk;
                for (int parity = 0; parity < 2; ++parity) {
                    double sgn = (((cb_idx_ + parity) & 1) == 0) ? 1.0 : -1.0;
                    double enc = A - sgn * B;
                    double encprev = (parity == 0) ? cb_e0p_ : cb_e1p_;
                    double decv = cb_have2_ ? boxplus(enc, encprev) : 0.0;
                    if (parity == 0) { dec0.push_back(decv); cb_e0p_ = enc; }
                    else             { dec1.push_back(decv); cb_e1p_ = enc; }
                }
                cb_have2_ = true;
                cb_idx_++;
            }
            cb_Xp_ = Xk; cb_Yvp_ = Yvk; cb_have_ = true;
            pos += sps_nom_ + adj;
        }
        double keep_from = pos - EL - 1.0;
        if (keep_from < 0.0) keep_from = 0.0;
        if (keep_from > (double)n) keep_from = (double)n;
        consumed = (size_t)keep_from;
        strm_pos_ = pos - (double)consumed;
        strm_abs_base_ += (double)consumed;   // keep tone LO phase continuous across chunks
    }

    void reset_stream() {
        strm_init_ = false; strm_tfreq_ = 0.0; strm_abs_base_ = 0.0;
        cb_theta_ = 0.0; cb_cfreq_ = 0.0;
        cb_have_ = false; cb_have2_ = false; cb_idx_ = 0;
        cb_Xp_ = cb_Yvp_ = cb_e0p_ = cb_e1p_ = 0.0;
    }

    static sample_t interp_lin(const sample_t* s, double idx, size_t len) {
        // Cubic (Catmull-Rom) interpolation; the coherent 2T detector is more
        // timing-sensitive than the non-coherent path, and cubic measurably
        // out-performs linear here (validated in the offline model).
        if (idx < 1) idx = 1;
        if (idx >= (double)(len - 2)) idx = (double)(len - 3);
        size_t i = (size_t)idx;
        double f = idx - (double)i;
        sample_t a = s[i-1], b = s[i], c = s[i+1], d = s[i+2];
        return b + 0.5*f*(c - a + f*(2.0*a - 5.0*b + 4.0*c - d + f*(3.0*(b - c) + d - a)));
    }

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
    double pll_alpha_;          // set by set_pll_bandwidth(); demodulate() uses fixed gains
    double pll_beta_;

    // Fractional-timing front-end state / parameters
    static constexpr int MF_M = 12;     // matched-filter sub-sample FLOOR (adaptive: max(12, round(sps)))
    double sps_nom_   = (double)SAMPLES_PER_SYMBOL;  // nominal samples/symbol (40 default)
    double el_offset_ = 0.5;               // early-late spacing, samples
    double ted_slope_ = 0.018;             // measured normalized TED gain (err / sample)
    double alpha_t_   = 0.06;              // PI proportional gain (model-validated)
    double beta_t_    = 0.0025;            // PI integral gain

    // Streaming (demodulate_stream) state, carried across chunk boundaries.
    bool   strm_init_  = false;            // lazy init of strm_pos_ to EL+1
    double strm_abs_base_ = 0.0;           // cumulative consumed samples (absolute LO phase ref)
    double strm_pos_   = 0.0;              // fractional timing position within the block
    double strm_tfreq_ = 0.0;              // timing-loop integral (NCO frequency)
    double cb_theta_   = 0.0;              // Costas phase accumulator
    double cb_cfreq_   = 0.0;              // Costas frequency integral
    bool   cb_have_    = false;            // a prior symbol's X/Yv is held (for 2T pairing)
    bool   cb_have2_   = false;            // a prior enc is held (for soft-differential)
    long   cb_idx_     = 0;                // global symbol index, for (-1)^i parity continuity
    double cb_Xp_      = 0.0;              // held X[k-1]  (Im of de-rotated tone-1 corr)
    double cb_Yvp_     = 0.0;              // held Yv[k-1] (Im of de-rotated tone-2 corr)
    double cb_e0p_     = 0.0;              // held enc[k-1] for parity 0
    double cb_e1p_     = 0.0;              // held enc[k-1] for parity 1
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
                    symbol_locked_(false),
                    corr_buf_idx_(0), circ_write_idx_(0), total_symbols_(0),
                    symbols_since_sync_(0), consecutive_misses_(0),
                    total_frames_(0) {
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

    // Resolve the 4-fold parity/polarity ambiguity using the sync word, returning
    // the single soft stream (polarity applied) that process()/FrameDecoder use.
    // Valid because parity and polarity are global constants for the stream. This
    // is where the protocol's sync knowledge lives -- NOT in the demodulator, so
    // the demod stays reusable under a different framing.
    static std::vector<double> resolve(const std::vector<double>& dec0,
                                       const std::vector<double>& dec1) {
        double pat[SYNC_BITS];
        for (size_t i = 0; i < SYNC_BITS; ++i) {
            int bit = (SYNC_WORD >> (SYNC_BITS - 1 - i)) & 1;
            pat[i] = (bit == 1) ? -1.0 : +1.0;
        }
        const std::vector<double>* str[2] = { &dec0, &dec1 };
        double best = -1.0; int bp = 0; double bs = 1.0;
        for (int p = 0; p < 2; ++p) {
            const std::vector<double>& d = *str[p];
            if (d.size() < SYNC_BITS) continue;
            for (size_t pos = 0; pos + SYNC_BITS <= d.size(); ++pos) {
                double sum = 0.0, en = 1e-9;
                for (size_t j = 0; j < SYNC_BITS; ++j) { double v = d[pos+j]; sum += pat[j]*v; en += std::fabs(v); }
                double c = sum / en;                       // normalized; sign = polarity
                if (std::fabs(c) > best) { best = std::fabs(c); bp = p; bs = (c < 0) ? -1.0 : 1.0; }
            }
        }
        std::vector<double> out = *str[bp];
        if (bs < 0) for (double& x : out) x = -x;
        return out;
    }

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

//------------------------------------------------------------------------------
// CoherentChannelReceiver  --  live (streaming) coherent sibling of
// ChannelReceiver. The proven non-coherent ChannelReceiver above is UNTOUCHED;
// coherent does not become the default until it is proven over the air.
//
// Three layers, mirroring the HDL's modular split:
//   (1) waveform  : CoherentMSKDemodulator streaming front-end (fractional
//                   timing + Costas + 2T/differential), carried statefully
//                   across chunks. Emits BOTH parity interpretations (dec0,dec1)
//                   with no sync-word or polarity decision -- protocol-agnostic.
//   (2) framing   : FOUR unmodified SyncTrackers fed dec0, -dec0, dec1, -dec1.
//                   Only the correct parity+polarity hypothesis correlates the
//                   sync word and locks; the scrambled-parity and inverted-
//                   polarity streams stay HUNTING on noise. Per-ACQUISITION
//                   4-fold resolution falls out for free: every burst, the
//                   matching tracker locks; on silence all drop to HUNTING and
//                   the next burst re-resolves (just as the radio did via sync).
//   (3) fec       : FrameDecoder on whichever hypothesis produced a frame.
//
// Symbol continuity across chunk boundaries is mandatory for coherent (a dropped
// symbol flips the (-1)^n parity), so this receiver buffers the leftover sample
// tail and prepends it to the next block -- unlike the AFC path which tolerates
// boundary slips.
//------------------------------------------------------------------------------
class CoherentChannelReceiver {
public:
    struct Frame {
        std::array<uint8_t, FRAME_BYTES> bytes;
        int    metric       = -1;
        double sync_quality = 0.0;
        int    hypothesis   = -1;   // 0..3 = {dec0+, dec0-, dec1+, dec1-}
    };

    CoherentChannelReceiver() {
        demod_.reset_stream();
        // First cut: let all four hypotheses search. The sync-word threshold
        // (8:1 PSLR pattern) guards against false locks on noise; SymbolLock
        // gating is a later refinement.
        for (auto& t : trk_) t.set_symbol_lock(true);
    }

    // --- configuration passthrough ---
    void   set_nominal_sps(double sps)      { demod_.set_nominal_sps(sps); }
    double get_nominal_sps() const          { return demod_.get_nominal_sps(); }
    void   set_freq_offset(double hz)       { demod_.set_freq_offset(hz); }
    double get_freq_offset() const          { return demod_.get_freq_offset(); }
    void   set_timing_bandwidth(double BnT, double zeta = 0.707) { demod_.set_timing_bandwidth(BnT, zeta); }
    void   set_timing_gains(double a, double b) { demod_.set_timing_gains(a, b); }

    // --- status accessors ---
    SyncState sync_state()    const { return trk_[committed_ >= 0 ? committed_ : 0].get_state(); }
    int       locked_hyp()    const { return committed_; }
    size_t    total_symbols() const { return total_symbols_; }

    // Streaming entry point. Buffers the leftover sample tail internally so the
    // symbol stream is continuous across calls; emits each frame the instant it
    // completes. gate is accepted for interface symmetry with ChannelReceiver
    // (unused in this first cut -- all hypotheses search unconditionally).
    template <class OnFrame>
    void process(const sample_t* samples, size_t n, OnFrame&& on_frame, bool /*gate*/ = true) {
        // buf = leftover tail ++ new samples
        std::vector<sample_t> buf;
        buf.reserve(leftover_.size() + n);
        buf.insert(buf.end(), leftover_.begin(), leftover_.end());
        buf.insert(buf.end(), samples, samples + n);

        std::vector<double> dec0, dec1;
        size_t consumed = 0;
        demod_.demodulate_stream(buf.data(), buf.size(), dec0, dec1, consumed);
        if (consumed > buf.size()) consumed = buf.size();
        leftover_.assign(buf.begin() + consumed, buf.end());

        const size_t nsym = std::min(dec0.size(), dec1.size());
        for (size_t i = 0; i < nsym; ++i) {
            const double soft[4] = { dec0[i], -dec0[i], dec1[i], -dec1[i] };
            for (int h = 0; h < 4; ++h) {
                auto res = trk_[h].process(soft[h], total_symbols_ + i);
                if (!res.frame_ready || res.payload.empty()) continue;
                Frame f;
                int metric = fdec_.decode(res.payload.data(), f.bytes);
                if (metric < 0) continue;
                f.metric       = metric;
                f.sync_quality = res.sync_quality;
                f.hypothesis   = h;

                if (committed_ < 0) {
                    // Resolve this acquisition. The sync word alone cannot separate
                    // the parities (when the orthogonal arm B~=0, dec0~=dec1 at sync,
                    // so multiple trackers correlate 1.0). FEC is the true arbiter:
                    // the correct parity/polarity decodes to valid codewords (low
                    // metric); the others are forced garbage (metric ~ ENCODED_BITS).
                    // Commit to the first hypothesis that clears the threshold.
                    if (metric <= COMMIT_METRIC_MAX) {
                        committed_ = h;
                        on_frame(static_cast<const Frame&>(f));
                    }
                    // else: wrong hypothesis -- suppress.
                } else if (h == committed_) {
                    // Committed: emit this hypothesis' frames (any metric -- real
                    // channel errors are valid corrected data); suppress the rest.
                    on_frame(static_cast<const Frame&>(f));
                }
            }
            // Release the commitment when the committed hypothesis loses lock
            // (silence / end of burst). The next burst re-resolves from scratch --
            // per-ACQUISITION resolution, exactly as the radio did via sync.
            if (committed_ >= 0 && trk_[committed_].get_state() == SyncState::HUNTING)
                committed_ = -1;
        }
        total_symbols_ += nsym;
    }

    std::vector<Frame> process(const sample_t* samples, size_t n, bool gate = true) {
        std::vector<Frame> frames;
        process(samples, n, [&frames](const Frame& f){ frames.push_back(f); }, gate);
        return frames;
    }

private:
    // FEC-metric ceiling for committing to a hypothesis. Far above a valid decode
    // (0..few) and far below forced garbage (~ENCODED_BITS); scales with frame size.
    static constexpr int COMMIT_METRIC_MAX = (int)(ENCODED_BITS / 2);

    CoherentMSKDemodulator demod_;
    SyncTracker            trk_[4];          // dec0+, dec0-, dec1+, dec1-
    FrameDecoder           fdec_;
    std::vector<sample_t>  leftover_;        // un-consumed sample tail (symbol continuity)
    size_t                 total_symbols_ = 0;
    int                    committed_     = -1;   // resolved hypothesis (-1 = re-resolving)
};

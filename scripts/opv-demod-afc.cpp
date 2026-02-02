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
// MSK Demodulator with AFC
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
// Sync State Machine with Soft Correlation
//------------------------------------------------------------------------------
class SyncTracker {
public:
    SyncTracker() : state_(SyncState::HUNTING),
                    symbols_since_sync_(0), consecutive_misses_(0),
                    total_frames_(0), corr_buf_idx_(0) {
        // Precompute sync word correlation pattern
        // For each bit: '1' expects negative soft (F1), '0' expects positive (F2)
        for (int i = 0; i < (int)SYNC_BITS; ++i) {
            int bit = (SYNC_WORD >> (SYNC_BITS - 1 - i)) & 1;
            sync_pattern_[i] = (bit == 1) ? -1.0 : +1.0;
        }
        // Initialize soft correlation buffer
        soft_corr_buf_.fill(0.0);
    }
    
    struct Result {
        bool frame_ready;
        size_t payload_start;
        double sync_quality;  // Normalized correlation value (-1 to +1)
    };
    
    Result process(double soft_val, size_t sym_idx) {
        Result res = {false, 0, 0.0};
        
        // Update circular buffer for soft correlation
        soft_corr_buf_[corr_buf_idx_] = soft_val;
        corr_buf_idx_ = (corr_buf_idx_ + 1) % SYNC_BITS;
        
        // Buffer soft values for payload extraction
        soft_buf_.push_back(soft_val);
        
        symbols_since_sync_++;
        
        switch (state_) {
        case SyncState::HUNTING: {
            // Need at least 24 symbols before checking for sync
            if (soft_buf_.size() < SYNC_BITS) break;
            
            double raw_corr;
            double norm_corr = soft_correlate(&raw_corr);
            // In HUNTING mode, require BOTH high raw correlation (strong signal)
            // AND high normalized correlation (good pattern match)
            if (raw_corr >= RAW_SYNC_HUNTING_THRESHOLD && norm_corr >= SOFT_SYNC_HUNTING_THRESHOLD) {
                state_ = SyncState::VERIFYING;
                payload_start_ = soft_buf_.size();
                sync_quality_ = norm_corr;
                symbols_since_sync_ = 0;
                fprintf(stderr, "[%zu] HUNTING→VERIFYING (corr=%.3f, raw=%.0f)\n", sym_idx, norm_corr, raw_corr);
            }
            // Limit buffer in hunting mode
            if (soft_buf_.size() > 50000) {
                soft_buf_.erase(soft_buf_.begin(), soft_buf_.begin() + 40000);
                payload_start_ = 0;
            }
            break;
        }
        
        case SyncState::VERIFYING: {
            if (symbols_since_sync_ >= ENCODED_BITS) {
                // Frame complete - output it
                res.frame_ready = true;
                res.payload_start = payload_start_;
                res.sync_quality = sync_quality_;
                total_frames_++;
                
                // Transition to LOCKED
                state_ = SyncState::LOCKED;
                symbols_since_sync_ = 0;
                consecutive_misses_ = 0;
                
                fprintf(stderr, "[%zu] VERIFYING→LOCKED (frame %d)\n", sym_idx, total_frames_);
            }
            break;
        }
        
        case SyncState::LOCKED: {
            // Check for sync at expected position
            if (symbols_since_sync_ == FRAME_SYMBOLS) {
                double raw_corr;
                double corr = soft_correlate(&raw_corr);
                
                // The payload we just received starts ENCODED_BITS symbols ago
                size_t received_payload_start = soft_buf_.size() - ENCODED_BITS;
                
                if (corr >= SOFT_SYNC_LOCKED_THRESHOLD) {
                    // Good sync
                    consecutive_misses_ = 0;
                    sync_quality_ = corr;
                    fprintf(stderr, "[%zu] LOCKED: sync OK (corr=%.3f)\n", sym_idx, corr);
                } else {
                    // Missed sync
                    consecutive_misses_++;
                    fprintf(stderr, "[%zu] LOCKED: sync MISS #%d (corr=%.3f)\n", 
                            sym_idx, consecutive_misses_, corr);
                    
                    if (consecutive_misses_ >= SYNC_MISS_LIMIT) {
                        state_ = SyncState::HUNTING;
                        fprintf(stderr, "[%zu] LOCKED→HUNTING (lost lock)\n", sym_idx);
                        break;
                    }
                    
                    // Flywheel: continue anyway
                    sync_quality_ = corr;
                }
                
                // Output the frame we just received
                res.frame_ready = true;
                res.payload_start = received_payload_start;
                res.sync_quality = sync_quality_;
                total_frames_++;
                
                symbols_since_sync_ = 0;
                
                // Trim buffer (keep enough for next frame)
                if (soft_buf_.size() > FRAME_SYMBOLS * 4) {
                    size_t keep = FRAME_SYMBOLS * 2;
                    size_t trim = soft_buf_.size() - keep;
                    soft_buf_.erase(soft_buf_.begin(), soft_buf_.begin() + trim);
                }
            }
            break;
        }
        }
        
        return res;
    }
    
    SyncState get_state() const { return state_; }
    int get_total_frames() const { return total_frames_; }
    
    const double* get_payload(size_t start) const {
        if (start + ENCODED_BITS <= soft_buf_.size()) {
            return &soft_buf_[start];
        }
        return nullptr;
    }

private:
    // Soft correlation: returns normalized correlation (-1 to +1)
    // +1 = perfect match, 0 = random, -1 = inverted
    // Also outputs raw (unnormalized) correlation for energy check
    double soft_correlate(double* raw_corr = nullptr) {
        double sum = 0.0;
        double energy = 0.0;
        
        for (size_t i = 0; i < SYNC_BITS; ++i) {
            // Get soft value from circular buffer (oldest to newest)
            size_t buf_idx = (corr_buf_idx_ + i) % SYNC_BITS;
            double soft = soft_corr_buf_[buf_idx];
            
            // Multiply by expected sign and accumulate
            sum += soft * sync_pattern_[i];
            energy += std::abs(soft);
        }
        
        if (raw_corr) *raw_corr = sum;
        
        // Normalize by total energy to get -1 to +1 range
        if (energy < MIN_SYNC_ENERGY) return 0.0;  // Not enough signal
        return sum / energy;
    }
    
    SyncState state_;
    std::vector<double> soft_buf_;
    
    // Circular buffer for soft correlation (last 24 soft values)
    std::array<double, SYNC_BITS> soft_corr_buf_;
    size_t corr_buf_idx_;
    
    // Precomputed sync pattern: +1 for expected '0', -1 for expected '1'
    std::array<double, SYNC_BITS> sync_pattern_;
    
    size_t symbols_since_sync_;
    size_t payload_start_;
    double sync_quality_;
    int consecutive_misses_;
    int total_frames_;
    
    // Soft correlation thresholds
    static constexpr double SOFT_SYNC_HUNTING_THRESHOLD = 0.85;  // Normalized: need strong pattern match
    static constexpr double SOFT_SYNC_LOCKED_THRESHOLD = 0.40;   // Normalized: relaxed for flywheel
    static constexpr double RAW_SYNC_HUNTING_THRESHOLD = 5000.0; // Raw: need strong signal (~200/symbol avg)
    static constexpr double MIN_SYNC_ENERGY = 100.0;             // Minimum energy (avoid div by zero)
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
    bool quiet = false, raw = false;
    double afc_bw = 0.001;
    
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-q")) quiet = true;
        else if (!strcmp(argv[i], "-r")) raw = true;
        else if (!strcmp(argv[i], "-a") && i + 1 < argc) afc_bw = atof(argv[++i]);
        else if (!strcmp(argv[i], "-h")) {
            fprintf(stderr, "Usage: %s [options] < input.iq\n\n", argv[0]);
            fprintf(stderr, "Options:\n");
            fprintf(stderr, "  -q          Quiet mode\n");
            fprintf(stderr, "  -r          Raw output to stdout\n");
            fprintf(stderr, "  -a <bw>     AFC bandwidth (default: 0.001)\n");
            fprintf(stderr, "  -h          Help\n");
            return 0;
        }
    }
    
    if (!quiet) {
        fprintf(stderr, "╔═══════════════════════════════════════════════════════════════════╗\n");
        fprintf(stderr, "║           OPV MSK Demodulator with AFC v1.0                       ║\n");
        fprintf(stderr, "╚═══════════════════════════════════════════════════════════════════╝\n\n");
    }
    
    // Load samples
    std::vector<sample_t> samples;
    IQSample iq;
    while (std::cin.read(reinterpret_cast<char*>(&iq), sizeof(iq)))
        samples.push_back(sample_t(iq.I, iq.Q));
    
    if (!quiet)
        fprintf(stderr, "Loaded %zu samples (%.3f sec)\n", samples.size(), samples.size() / SAMPLE_RATE);
    
    // Demodulate
    MSKDemodulatorAFC demod;
    
    // Estimate carrier offset from spectrum (coarse AFC)
    double est_offset = demod.estimate_offset(samples.data(), samples.size());
    demod.set_freq_offset(est_offset);
    
    if (!quiet)
        fprintf(stderr, "Estimated carrier offset: %.1f Hz\n", est_offset);
    
    demod.set_afc_bandwidth(afc_bw);
    std::vector<double> soft;
    demod.demodulate(samples.data(), samples.size(), soft);
    
    if (!quiet)
        fprintf(stderr, "Demodulated %zu symbols, final AFC offset: %.1f Hz\n\n", 
                soft.size(), demod.get_freq_offset());
    
    // Process through state machine
    SyncTracker tracker;
    FrameDecoder fdec;
    int decoded = 0, perfect = 0;
    
    for (size_t i = 0; i < soft.size(); ++i) {
        auto res = tracker.process(soft[i], i);
        
        if (res.frame_ready) {
            const double* payload = tracker.get_payload(res.payload_start);
            if (payload) {
                std::array<uint8_t, FRAME_BYTES> frame;
                int metric = fdec.decode(payload, frame);
                
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
        fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
        fprintf(stderr, "Summary: %d frames (%d perfect, %d errors)\n", decoded, perfect, decoded - perfect);
        fprintf(stderr, "Final state: %s, AFC: %.1f Hz\n", 
                state_name(tracker.get_state()), demod.get_freq_offset());
        fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
    }
    
    return decoded > 0 ? 0 : 1;
}

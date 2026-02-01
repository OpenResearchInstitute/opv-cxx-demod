/**
 * opv-mod-hdl.cpp - OPV transmitter matching HDL modulator exactly
 * 
 * The HDL msk_modulator uses two NCOs (F1 and F2) running continuously,
 * with amplitude/polarity control (d_s1, d_s2) that can be +1, -1, or 0.
 * Only one tone is active at a time, and the polarity maintains phase coherence.
 * 
 * Output = sin(F1)*d_s1 + sin(F2)*d_s2
 * Where I=sin, Q=cos (HDL convention)
 */

#include <iostream>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <array>
#include <getopt.h>

// =============================================================================
// PARAMETERS
// =============================================================================

constexpr size_t FRAME_BYTES = 134;
constexpr size_t FRAME_BITS = FRAME_BYTES * 8;
constexpr size_t ENCODED_BITS = FRAME_BITS * 2;

constexpr uint32_t SYNC_WORD = 0x02B8DB;

constexpr size_t SAMPLES_PER_SYMBOL = 40;
constexpr double SAMPLE_RATE = 2168000.0;
constexpr double SYMBOL_RATE = 54200.0;
constexpr double FREQ_DEV = SYMBOL_RATE / 4.0;  // 13550 Hz
constexpr double F1_FREQ = -FREQ_DEV;  // Lower tone
constexpr double F2_FREQ = +FREQ_DEV;  // Upper tone

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

// =============================================================================
// TYPES
// =============================================================================

using frame_t = std::array<uint8_t, FRAME_BYTES>;
using encoded_bits_t = std::array<uint8_t, ENCODED_BITS>;

struct IQSample { int16_t I, Q; };

// =============================================================================
// CCSDS LFSR RANDOMIZER
// =============================================================================

class LFSR {
public:
    void reset() { state = 0xFF; }
    
    uint8_t next_byte() {
        uint8_t out = 0;
        for (int i = 7; i >= 0; --i) {
            out |= ((state >> 7) & 1) << i;
            uint8_t fb = ((state >> 7) ^ (state >> 6) ^ (state >> 4) ^ (state >> 2)) & 1;
            state = (state << 1) | fb;
        }
        return out;
    }
    
private:
    uint8_t state = 0xFF;
};

// =============================================================================
// CONVOLUTIONAL ENCODER (K=7, Rate 1/2)
// =============================================================================

class ConvEncoder {
public:
    void reset() { sr = 0; }
    
    void encode_bit(uint8_t in, uint8_t& g1, uint8_t& g2) {
        uint8_t state = (in << 6) | sr;
        g1 = __builtin_parity(state & 0x4F);
        g2 = __builtin_parity(state & 0x6D);
        sr = ((sr << 1) | in) & 0x3F;
    }
    
private:
    uint8_t sr = 0;
};

// =============================================================================
// 67x32 BIT INTERLEAVER
// =============================================================================

void interleave(encoded_bits_t& bits) {
    encoded_bits_t temp;
    for (size_t i = 0; i < ENCODED_BITS; ++i) {
        temp[(i % 32) * 67 + (i / 32)] = bits[i];
    }
    bits = temp;
}

// =============================================================================
// FRAME ENCODER
// =============================================================================

encoded_bits_t encode_frame(const frame_t& payload) {
    LFSR lfsr; lfsr.reset();
    ConvEncoder conv; conv.reset();
    encoded_bits_t encoded;
    size_t out_idx = 0;
    
    std::array<uint8_t, FRAME_BYTES> randomized;
    for (size_t i = 0; i < FRAME_BYTES; ++i) {
        randomized[i] = payload[i] ^ lfsr.next_byte();
    }
    
    for (int byte_idx = FRAME_BYTES - 1; byte_idx >= 0; --byte_idx) {
        uint8_t byte = randomized[byte_idx];
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
            uint8_t g1, g2;
            conv.encode_bit((byte >> bit_pos) & 1, g1, g2);
            encoded[out_idx++] = g1;
            encoded[out_idx++] = g2;
        }
    }
    
    interleave(encoded);
    return encoded;
}

// =============================================================================
// HDL-ACCURATE PARALLEL-TONE MSK MODULATOR
// 
// Matches msk_modulator.vhd logic exactly:
// - Two NCOs running at F1 and F2
// - Differential encoding determines which tone is active
// - Tone polarity (±1) determined by b_n and d_val_xor_T
// =============================================================================

class HDLModulator {
public:
    void reset() {
        phase_f1 = 0.0;
        phase_f2 = 0.0;
        d_val_xor_T = 0;  // "000" initially
        b_n = 1;          // '1' initially
    }
    
    void modulate_bit(uint8_t tx_bit, std::array<IQSample, SAMPLES_PER_SYMBOL>& output) {
        // Compute d_val from tx_bit (lines 223)
        int8_t d_val = (tx_bit == 0) ? 1 : -1;
        
        // Compute d_val_xor (lines 225-228)
        int8_t d_val_xor;
        if (d_val == 1 && d_val_xor_T == 1) d_val_xor = 1;
        else if (d_val == 1 && d_val_xor_T == -1) d_val_xor = -1;
        else if (d_val == -1 && d_val_xor_T == 1) d_val_xor = -1;
        else if (d_val == -1 && d_val_xor_T == -1) d_val_xor = 1;
        else d_val_xor = 1;  // default when d_val_xor_T = 0
        
        // HDL TIMING: d_val_xor_T is updated on tclk_dly(0), BEFORE d_s1/d_s2
        // are computed on tclk_dly(1). So we must update d_val_xor_T first!
        d_val_xor_T = d_val_xor;
        
        // Compute d_pos and d_neg (lines 230-231)
        int8_t d_pos = (d_val + 1) >> 1;  // +1→1, -1→0
        int8_t d_neg = (d_val - 1) >> 1;  // +1→0, -1→-1
        
        // Compute d_pos_enc and d_neg_enc (lines 233-234)
        int8_t d_pos_enc = d_pos;
        int8_t d_neg_enc = (b_n == 0) ? d_neg : -d_neg;
        
        // NOW compute d_s1/d_s2 using the UPDATED d_val_xor_T
        int8_t d_s1, d_s2;
        
        // d_pos_xor (F1 amplitude)
        if (d_pos_enc == 1 && d_val_xor_T == 1) d_s1 = 1;
        else if (d_pos_enc == 1 && d_val_xor_T == -1) d_s1 = -1;
        else d_s1 = 0;
        
        // d_neg_xor (F2 amplitude)
        if (d_neg_enc == -1 && d_val_xor_T == 1) d_s2 = -1;
        else if (d_neg_enc == -1 && d_val_xor_T == -1) d_s2 = 1;
        else if (d_neg_enc == 1 && d_val_xor_T == 1) d_s2 = 1;
        else if (d_neg_enc == 1 && d_val_xor_T == -1) d_s2 = -1;
        else d_s2 = 0;
        
        // Generate samples
        double phase_inc_f1 = TWO_PI * F1_FREQ / SAMPLE_RATE;
        double phase_inc_f2 = TWO_PI * F2_FREQ / SAMPLE_RATE;
        
        for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
            // HDL: tx_samples_I = s1s + s2s (sin components)
            //      tx_samples_Q = s1c + s2c (cos components)
            double sin_f1 = std::sin(phase_f1);
            double cos_f1 = std::cos(phase_f1);
            double sin_f2 = std::sin(phase_f2);
            double cos_f2 = std::cos(phase_f2);
            
            double I = d_s1 * sin_f1 + d_s2 * sin_f2;
            double Q = d_s1 * cos_f1 + d_s2 * cos_f2;
            
            output[i].I = static_cast<int16_t>(16383.0 * I);
            output[i].Q = static_cast<int16_t>(16383.0 * Q);
            
            // Update NCO phases
            phase_f1 += phase_inc_f1;
            phase_f2 += phase_inc_f2;
            while (phase_f1 > PI) phase_f1 -= TWO_PI;
            while (phase_f1 < -PI) phase_f1 += TWO_PI;
            while (phase_f2 > PI) phase_f2 -= TWO_PI;
            while (phase_f2 < -PI) phase_f2 += TWO_PI;
        }
        
        // Toggle b_n for next symbol
        b_n = 1 - b_n;
    }
    
private:
    double phase_f1 = 0.0;
    double phase_f2 = 0.0;
    int8_t d_val_xor_T = 0;
    int8_t b_n = 1;
};

// =============================================================================
// GLOBALS
// =============================================================================

HDLModulator g_mod;
bool g_verbose = false;
bool g_reset_per_frame = false;

// =============================================================================
// OUTPUT
// =============================================================================

void output(const std::array<IQSample, SAMPLES_PER_SYMBOL>& s) {
    for (const auto& x : s) {
        std::cout.write(reinterpret_cast<const char*>(&x.I), 2);
        std::cout.write(reinterpret_cast<const char*>(&x.Q), 2);
    }
}

// =============================================================================
// TRANSMIT FUNCTIONS
// =============================================================================

void send_sync_word() {
    std::array<IQSample, SAMPLES_PER_SYMBOL> samples;
    for (int i = 23; i >= 0; --i) {
        g_mod.modulate_bit((SYNC_WORD >> i) & 1, samples);
        output(samples);
    }
}

void send_encoded_frame(const encoded_bits_t& encoded) {
    std::array<IQSample, SAMPLES_PER_SYMBOL> samples;
    for (size_t byte_idx = 0; byte_idx < ENCODED_BITS / 8; ++byte_idx) {
        for (int bit_in_byte = 7; bit_in_byte >= 0; --bit_in_byte) {
            g_mod.modulate_bit(encoded[byte_idx * 8 + bit_in_byte], samples);
            output(samples);
        }
    }
}

frame_t build_bert_frame(const char* callsign, uint32_t frame_num) {
    frame_t frame = {};
    
    size_t len = strlen(callsign);
    for (size_t i = 0; i < 6 && i < len; ++i) {
        frame[i] = callsign[i];
    }
    
    frame[6] = (frame_num >> 16) & 0xFF;
    frame[7] = (frame_num >> 8) & 0xFF;
    frame[8] = frame_num & 0xFF;
    
    frame[9] = 0;
    frame[10] = 0;
    frame[11] = 0;
    
    for (size_t i = 0; i < FRAME_BYTES - 12; ++i) {
        frame[12 + i] = (frame_num + i) & 0xFF;
    }
    
    return frame;
}

// =============================================================================
// MAIN
// =============================================================================

void usage(const char* prog) {
    std::cerr << "Usage: " << prog << " -S CALLSIGN -B FRAMES [-r] [-c] [-v]\n";
    exit(1);
}

int main(int argc, char* argv[]) {
    const char* callsign = nullptr;
    int bert_frames = 0;
    bool continuous = false;
    
    int opt;
    while ((opt = getopt(argc, argv, "S:B:rcvh")) != -1) {
        switch (opt) {
            case 'S': callsign = optarg; break;
            case 'B': bert_frames = std::atoi(optarg); break;
            case 'r': g_reset_per_frame = true; break;
            case 'c': continuous = true; break;
            case 'v': g_verbose = true; break;
            default: usage(argv[0]);
        }
    }
    
    if (!callsign || bert_frames <= 0) {
        usage(argv[0]);
    }
    
    if (g_verbose) {
        std::cerr << "OPV Transmitter (HDL-accurate parallel-tone MSK)\n";
        std::cerr << "  Callsign: " << callsign << "\n";
        std::cerr << "  BERT frames: " << bert_frames << "\n";
    }
    
    uint32_t frame_num = 0;
    
    do {
        g_mod.reset();
        
        for (int f = 0; f < bert_frames; ++f) {
            if (g_reset_per_frame) {
                g_mod.reset();
            }
            
            frame_t frame = build_bert_frame(callsign, frame_num++);
            encoded_bits_t encoded = encode_frame(frame);
            
            send_sync_word();
            send_encoded_frame(encoded);
            
            if (g_verbose && ((f + 1) % 10 == 0 || f == bert_frames - 1)) {
                std::cerr << "Sent frame " << (f + 1) << "/" << bert_frames << "\n";
            }
        }
        
    } while (continuous);
    
    std::array<IQSample, SAMPLES_PER_SYMBOL> zeros = {};
    for (int i = 0; i < 100; ++i) output(zeros);
    
    if (g_verbose) std::cerr << "Done.\n";
    return 0;
}

/**
 * opv-mod-hdl-fixed.cpp - OPV transmitter matching HDL modulator exactly
 * 
 * FIXES from original:
 * 1. Convolutional encoder masks: 0x4F/0x6D (not 0x79/0x5B)
 * 2. Base-40 callsign encoding (not ASCII)
 * 3. Proper OPV header format
 * 
 * Copyright 2026 Open Research Institute, Inc.
 * SPDX-License-Identifier: CERN-OHL-S-2.0
 */

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <array>
#include <string>
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

bool g_verbose = false;
constexpr double F1_FREQ = -FREQ_DEV;  // Lower tone (bit '1')
constexpr double F2_FREQ = +FREQ_DEV;  // Upper tone (bit '0')

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

// =============================================================================
// TYPES
// =============================================================================

using frame_t = std::array<uint8_t, FRAME_BYTES>;
using encoded_bits_t = std::array<uint8_t, ENCODED_BITS>;

struct IQSample { int16_t I, Q; };

// =============================================================================
// BASE-40 CALLSIGN ENCODER
// =============================================================================

class Base40Encoder {
public:
    // Encode callsign string to 6-byte Base-40 value
    // HDL convention: first character in least significant position
    static void encode(const std::string& callsign, uint8_t* out) {
        uint64_t value = 0;
        
        // Iterate in reverse to put first char in LSB position (HDL convention)
        for (int i = callsign.length() - 1; i >= 0; --i) {
            value *= 40;
            value += char_to_digit(callsign[i]);
        }
        
        // Pack into 6 bytes (big-endian)
        out[0] = (value >> 40) & 0xFF;
        out[1] = (value >> 32) & 0xFF;
        out[2] = (value >> 24) & 0xFF;
        out[3] = (value >> 16) & 0xFF;
        out[4] = (value >> 8) & 0xFF;
        out[5] = value & 0xFF;
    }
    
private:
    static int char_to_digit(char c) {
        if (c >= 'A' && c <= 'Z') return c - 'A' + 1;
        if (c >= 'a' && c <= 'z') return c - 'a' + 1;  // Accept lowercase
        if (c >= '0' && c <= '9') return c - '0' + 27;
        if (c == '-') return 37;
        if (c == '/') return 38;
        if (c == '.') return 39;
        return 0;  // Unknown -> unused
    }
};

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
// FIXED: Uses correct polynomial masks matching HDL's bit indexing
// =============================================================================

class ConvEncoder {
public:
    void reset() { sr = 0; }
    
    void encode_bit(uint8_t in, uint8_t& g1, uint8_t& g2) {
        uint8_t state = (in << 6) | sr;
        // FIXED: HDL uses 6-i indexing, so:
        // G1 = 171 octal = 1111001 -> mask = 0x4F (not 0x79!)
        // G2 = 133 octal = 1011011 -> mask = 0x6D (not 0x5B!)
        g1 = __builtin_parity(state & 0x4F);
        g2 = __builtin_parity(state & 0x6D);
        sr = ((sr << 1) | in) & 0x3F;
    }
    
private:
    uint8_t sr = 0;
};

// =============================================================================
// 67x32 BIT INTERLEAVER (with MSB-first byte correction to match HDL)
// =============================================================================

void interleave(encoded_bits_t& bits) {
    encoded_bits_t temp = {};
    for (size_t i = 0; i < ENCODED_BITS; ++i) {
        size_t interleaved_pos = (i % 32) * 67 + (i / 32);
        // Apply MSB-first byte correction (same as demodulator expects)
        size_t byte_num = interleaved_pos / 8;
        size_t bit_in_byte = interleaved_pos % 8;
        size_t corrected_pos = byte_num * 8 + (7 - bit_in_byte);
        temp[corrected_pos] = bits[i];
    }
    bits = temp;
}

// =============================================================================
// FRAME ENCODER
// =============================================================================

encoded_bits_t encode_frame(const frame_t& payload) {
    LFSR lfsr; lfsr.reset();
    ConvEncoder conv; conv.reset();
    encoded_bits_t encoded = {};
    size_t out_idx = 0;
    
    // Randomize payload
    std::array<uint8_t, FRAME_BYTES> randomized;
    for (size_t i = 0; i < FRAME_BYTES; ++i) {
        randomized[i] = payload[i] ^ lfsr.next_byte();
    }
    
    if (g_verbose) {
        std::cerr << "Payload[0:11]: ";
        for (int i = 0; i < 12; ++i) {
            std::cerr << std::hex << std::setw(2) << std::setfill('0') << (int)payload[i] << " ";
        }
        std::cerr << std::dec << "\n";
        
        std::cerr << "Randomized[0:5]: ";
        for (int i = 0; i < 6; ++i) {
            std::cerr << std::hex << std::setw(2) << std::setfill('0') << (int)randomized[i] << " ";
        }
        std::cerr << std::dec << "\n";
    }
    
    // Encode from last byte to first (HDL byte order)
    for (int byte_idx = FRAME_BYTES - 1; byte_idx >= 0; --byte_idx) {
        uint8_t byte = randomized[byte_idx];
        
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
            uint8_t g1, g2;
            uint8_t in_bit = (byte >> bit_pos) & 1;
            conv.encode_bit(in_bit, g1, g2);
            encoded[out_idx++] = g1;
            encoded[out_idx++] = g2;
        }
    }
    
    if (g_verbose) {
        std::cerr << "Before interleave [0:31]: ";
        for (int i = 0; i < 32; ++i) std::cerr << (int)encoded[i];
        std::cerr << "\n";
    }
    
    interleave(encoded);
    
    if (g_verbose) {
        std::cerr << "After interleave [0:31]:  ";
        for (int i = 0; i < 32; ++i) std::cerr << (int)encoded[i];
        std::cerr << "\n";
    }
    
    return encoded;
}

// =============================================================================
// HDL-ACCURATE PARALLEL-TONE MSK MODULATOR
// =============================================================================

class HDLModulator {
public:
    void reset() {
        phase_f1 = 0.0;
        phase_f2 = 0.0;
        d_val_xor_T = 0;
        b_n = 1;
    }
    
    void modulate_bit(uint8_t tx_bit, std::array<IQSample, SAMPLES_PER_SYMBOL>& output) {
        // d_val mapping: determines which tone is active
        // bit '0' -> d_val=+1 -> activates d_pos path (F1)
        // bit '1' -> d_val=-1 -> activates d_neg path (F2)
        int8_t d_val = (tx_bit == 0) ? 1 : -1;
        
        int8_t d_val_xor;
        if (d_val == 1 && d_val_xor_T == 1) d_val_xor = 1;
        else if (d_val == 1 && d_val_xor_T == -1) d_val_xor = -1;
        else if (d_val == -1 && d_val_xor_T == 1) d_val_xor = -1;
        else if (d_val == -1 && d_val_xor_T == -1) d_val_xor = 1;
        else d_val_xor = 1;
        
        int8_t d_pos = (d_val + 1) >> 1;
        int8_t d_neg = (d_val - 1) >> 1;
        
        int8_t d_pos_enc = d_pos;
        int8_t d_neg_enc = (b_n == 0) ? d_neg : -d_neg;
        
        int8_t d_s1, d_s2;
        
        if (d_pos_enc == 1 && d_val_xor_T == 1) d_s1 = 1;
        else if (d_pos_enc == 1 && d_val_xor_T == -1) d_s1 = -1;
        else d_s1 = 0;
        
        if (d_neg_enc == -1 && d_val_xor_T == 1) d_s2 = -1;
        else if (d_neg_enc == -1 && d_val_xor_T == -1) d_s2 = 1;
        else if (d_neg_enc == 1 && d_val_xor_T == 1) d_s2 = 1;
        else if (d_neg_enc == 1 && d_val_xor_T == -1) d_s2 = -1;
        else d_s2 = 0;
        
        double phase_inc_f1 = TWO_PI * F1_FREQ / SAMPLE_RATE;
        double phase_inc_f2 = TWO_PI * F2_FREQ / SAMPLE_RATE;
        
        for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
            double sin_f1 = std::sin(phase_f1);
            double cos_f1 = std::cos(phase_f1);
            double sin_f2 = std::sin(phase_f2);
            double cos_f2 = std::cos(phase_f2);
            
            double I = d_s1 * sin_f1 + d_s2 * sin_f2;
            double Q = d_s1 * cos_f1 + d_s2 * cos_f2;
            
            output[i].I = static_cast<int16_t>(16383.0 * I);
            output[i].Q = static_cast<int16_t>(16383.0 * Q);
            
            phase_f1 += phase_inc_f1;
            phase_f2 += phase_inc_f2;
            while (phase_f1 > PI) phase_f1 -= TWO_PI;
            while (phase_f1 < -PI) phase_f1 += TWO_PI;
            while (phase_f2 > PI) phase_f2 -= TWO_PI;
            while (phase_f2 < -PI) phase_f2 += TWO_PI;
        }
        
        d_val_xor_T = d_val_xor;
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
    
    if (g_verbose) {
        std::cerr << "Encoded bits [0:31]: ";
        for (int i = 0; i < 32; ++i) std::cerr << (int)encoded[i];
        std::cerr << "\n";
    }
    
    for (size_t i = 0; i < ENCODED_BITS; ++i) {
        g_mod.modulate_bit(encoded[i], samples);
        output(samples);
    }
}

// Build OPV frame with proper Base-40 callsign encoding
frame_t build_opv_frame(const std::string& callsign, uint32_t token, uint32_t frame_num) {
    frame_t frame = {};
    
    // Station ID: 6 bytes Base-40 encoded
    Base40Encoder::encode(callsign, &frame[0]);
    
    // Token: 3 bytes (default 0xBBAADD)
    frame[6] = (token >> 16) & 0xFF;
    frame[7] = (token >> 8) & 0xFF;
    frame[8] = token & 0xFF;
    
    // Reserved: 3 bytes
    frame[9] = 0;
    frame[10] = 0;
    frame[11] = 0;
    
    // Payload: fill with test pattern (frame counter + index)
    for (size_t i = 0; i < FRAME_BYTES - 12; ++i) {
        frame[12 + i] = (frame_num + i) & 0xFF;
    }
    
    return frame;
}

// =============================================================================
// MAIN
// =============================================================================

void usage(const char* prog) {
    std::cerr << "Usage: " << prog << " -S CALLSIGN -B FRAMES [-t TOKEN] [-r] [-c] [-v]\n\n";
    std::cerr << "Options:\n";
    std::cerr << "  -S CALLSIGN   Station callsign (e.g., W5NYV, UM5BK)\n";
    std::cerr << "  -B FRAMES     Number of BERT frames to transmit\n";
    std::cerr << "  -t TOKEN      24-bit token (default: 0xBBAADD)\n";
    std::cerr << "  -r            Reset modulator per frame\n";
    std::cerr << "  -c            Continuous mode (loop forever)\n";
    std::cerr << "  -v            Verbose output\n";
    exit(1);
}

int main(int argc, char* argv[]) {
    std::string callsign;
    int bert_frames = 0;
    bool continuous = false;
    uint32_t token = 0xBBAADD;  // Default token
    
    int opt;
    while ((opt = getopt(argc, argv, "S:B:t:rcvh")) != -1) {
        switch (opt) {
            case 'S': callsign = optarg; break;
            case 'B': bert_frames = std::atoi(optarg); break;
            case 't': token = std::strtoul(optarg, nullptr, 0); break;
            case 'r': g_reset_per_frame = true; break;
            case 'c': continuous = true; break;
            case 'v': g_verbose = true; break;
            default: usage(argv[0]);
        }
    }
    
    if (callsign.empty() || bert_frames <= 0) {
        usage(argv[0]);
    }
    
    // Validate callsign length (max ~9 chars for 48-bit Base-40)
    if (callsign.length() > 9) {
        std::cerr << "Warning: Callsign truncated to 9 characters for Base-40 encoding\n";
        callsign = callsign.substr(0, 9);
    }
    
    if (g_verbose) {
        std::cerr << "OPV Transmitter (HDL-accurate, FIXED)\n";
        std::cerr << "  Callsign: " << callsign << "\n";
        std::cerr << "  Token:    0x" << std::hex << token << std::dec << "\n";
        std::cerr << "  BERT frames: " << bert_frames << "\n";
        std::cerr << "  Conv encoder masks: G1=0x4F, G2=0x6D\n";
        std::cerr << "\n";
    }
    
    uint32_t frame_num = 0;
    
    do {
        g_mod.reset();
        
        for (int f = 0; f < bert_frames; ++f) {
            if (g_reset_per_frame) {
                g_mod.reset();
            }
            
            frame_t frame = build_opv_frame(callsign, token, frame_num++);
            encoded_bits_t encoded = encode_frame(frame);
            
            send_sync_word();
            send_encoded_frame(encoded);
            
            if (g_verbose && ((f + 1) % 10 == 0 || f == bert_frames - 1)) {
                std::cerr << "Sent frame " << (f + 1) << "/" << bert_frames << "\n";
            }
        }
        
    } while (continuous);
    
    // Trailing zeros
    std::array<IQSample, SAMPLES_PER_SYMBOL> zeros = {};
    for (int i = 0; i < 100; ++i) output(zeros);
    
    if (g_verbose) std::cerr << "Done.\n";
    return 0;
}

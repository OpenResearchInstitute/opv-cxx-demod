/**
 * opv-mod.cpp - OPV MSK Modulator
 *
 * Modes:
 *   -B N        BERT mode: generate N test frames internally
 *   -R          Raw mode: read 134-byte frames from stdin
 *
 * Options:
 *   -P          Prepend one preamble frame before payload frames.
 *               The preamble frame contains 2168 bits of 0xCC 
 *               (1100 1100 repeating) sent raw (no FEC).
 *               This gives the receiver's time to acquire symbol
 *               timing before the first real frame arrives.
 *
 * Matches HDL modulator implementation exactly.
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
#include <unistd.h>
#include <cassert>
#include "opv_codec.hpp"

// =============================================================================
// PARAMETERS
// =============================================================================


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


// =============================================================================
// CONVOLUTIONAL ENCODER (K=7, Rate 1/2)
// Uses correct polynomial masks matching HDL's bit indexing
// =============================================================================


// =============================================================================
// 67x32 BIT INTERLEAVER (with MSB-first byte correction to match HDL)
// =============================================================================


// =============================================================================
// FRAME ENCODER
// =============================================================================


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
            
            //output[i].I = static_cast<int16_t>(16383.0 * I);
            //output[i].Q = static_cast<int16_t>(16383.0 * Q);
            output[i].I = static_cast<int16_t>(32760.0 * I);
            output[i].Q = static_cast<int16_t>(32760.0 * Q);
            
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

// Build OPV frame with proper Base-40 callsign encoding (BERT mode)
frame_t build_bert_frame(const std::string& callsign, uint32_t token, uint32_t frame_num) {
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

// Send one preamble frame: 2168 bits of 0xCC (raw, no FEC).
void send_preamble_frame() {
    std::array<IQSample, SAMPLES_PER_SYMBOL> samples;
    constexpr uint8_t PREAMBLE_BYTE = 0xCC;  // 1100 1100 repeating
    for (size_t i = 0; i < (24 + ENCODED_BITS); ++i) {
        uint8_t bit = (PREAMBLE_BYTE >> (7 - (i % 8))) & 1;
        g_mod.modulate_bit(bit, samples);
        output(samples);
    }
}

// Read a 134-byte frame from stdin (Raw mode)
// Returns true if frame read successfully, false on EOF or error
bool read_frame_from_stdin(frame_t& frame) {
    size_t bytes_read = 0;
    
    while (bytes_read < FRAME_BYTES) {
        ssize_t n = read(STDIN_FILENO, frame.data() + bytes_read, FRAME_BYTES - bytes_read);
        if (n <= 0) {
            if (n == 0 && bytes_read == 0) {
                // Clean EOF at frame boundary
                return false;
            }
            if (n == 0) {
                std::cerr << "Warning: EOF after partial frame (" << bytes_read << " bytes)\n";
                return false;
            }
            // Read error
            std::cerr << "Error reading from stdin\n";
            return false;
        }
        bytes_read += n;
    }
    
    return true;
}

// =============================================================================
// MAIN
// =============================================================================

void usage(const char* prog) {
    std::cerr << "Usage: " << prog << " [OPTIONS]\n\n";
    std::cerr << "Modes (mutually exclusive):\n";
    std::cerr << "  -B FRAMES     BERT mode: generate N test frames\n";
    std::cerr << "  -R            Raw mode: read 134-byte frames from stdin\n";
    std::cerr << "\n";
    std::cerr << "Options:\n";
    std::cerr << "  -S CALLSIGN   Station callsign (required for BERT mode)\n";
    std::cerr << "  -P            Prepend one preamble frame (0xCC pattern)\n";
    std::cerr << "  -t TOKEN      24-bit token (default: 0xBBAADD)\n";
    std::cerr << "  -c            Continuous mode (loop BERT forever)\n";
    std::cerr << "  -v            Verbose output to stderr\n";
    std::cerr << "\n";
    std::cerr << "Output: 16-bit I/Q samples (little-endian, interleaved) to stdout\n";
    std::cerr << "\n";
    std::cerr << "Examples:\n";
    std::cerr << "  " << prog << " -S W5NYV -B 10              # 10 BERT frames\n";
    std::cerr << "  " << prog << " -S W5NYV -P -B 10           # preamble + 10 BERT frames\n";
    std::cerr << "  " << prog << " -R < frames.bin             # Modulate pre-built frames\n";
    std::cerr << "  cat frames.bin | " << prog << " -R         # Same, via pipe\n";
    exit(1);
}

int main(int argc, char* argv[]) {
    std::string callsign;
    int bert_frames = 0;
    bool continuous = false;
    bool raw_mode = false;
    bool preamble = false;
    uint32_t token = 0xBBAADD;  // Default token

    int opt;
    while ((opt = getopt(argc, argv, "S:B:t:PRcvh")) != -1) {
        switch (opt) {
            case 'S': callsign = optarg; break;
            case 'B': bert_frames = std::atoi(optarg); break;
            case 't': token = std::strtoul(optarg, nullptr, 0); break;
            case 'P': preamble = true; break;
            case 'R': raw_mode = true; break;
            case 'c': continuous = true; break;
            case 'v': g_verbose = true; break;
            default: usage(argv[0]);
        }
    }
    
    // Validate mode selection
    if (raw_mode && bert_frames > 0) {
        std::cerr << "Error: -R and -B are mutually exclusive\n";
        usage(argv[0]);
    }
    
    if (!raw_mode && bert_frames <= 0) {
        std::cerr << "Error: Must specify either -R (raw mode) or -B N (BERT mode)\n";
        usage(argv[0]);
    }
    
    if (!raw_mode && callsign.empty()) {
        std::cerr << "Error: BERT mode requires -S CALLSIGN\n";
        usage(argv[0]);
    }
    
    // Validate callsign length (max ~9 chars for 48-bit Base-40)
    if (!callsign.empty() && callsign.length() > 9) {
        std::cerr << "Warning: Callsign truncated to 9 characters for Base-40 encoding\n";
        callsign = callsign.substr(0, 9);
    }
    
    if (g_verbose) {
        std::cerr << "OPV Modulator\n";
        if (raw_mode) {
            std::cerr << "  Mode: Raw (reading 134-byte frames from stdin)\n";
        } else {
            std::cerr << "  Mode: BERT\n";
            std::cerr << "  Callsign: " << callsign << "\n";
            std::cerr << "  Token:    0x" << std::hex << token << std::dec << "\n";
            std::cerr << "  Frames:   " << bert_frames << "\n";
        }
        std::cerr << "  Preamble: " << (preamble ? "yes (0xCC x2168)" : "no") << "\n";
        std::cerr << "  Conv encoder: G1=0x67 (171), G2=0x76 (133)\n";
        std::cerr << "\n";
    }
    
    // =========================================================================
    // PREAMBLE (optional, precedes all payload frames)
    // =========================================================================
    g_mod.reset();
    if (preamble) {
        send_preamble_frame();
        if (g_verbose) std::cerr << "Preamble frame sent.\n";
    }

    // =========================================================================
    // RAW MODE: Read frames from stdin
    // =========================================================================
    if (raw_mode) {
        frame_t frame;
        uint64_t frame_count = 0;
        
        while (read_frame_from_stdin(frame)) {
            if (g_reset_per_frame) {
                g_mod.reset();
            }
            
            encoded_bits_t encoded = encode_frame(frame);
            send_sync_word();
            send_encoded_frame(encoded);
            
            frame_count++;
            
            if (g_verbose && (frame_count % 100 == 0)) {
                std::cerr << "Sent " << frame_count << " frames\n";
            }
        }
        
        if (g_verbose) {
            std::cerr << "End of input. Total frames: " << frame_count << "\n";
        }
    }
    // =========================================================================
    // BERT MODE: Generate test frames
    // =========================================================================
    else {
        uint32_t frame_num = 0;

        do {
            
            for (int f = 0; f < bert_frames; ++f) {
                if (g_reset_per_frame) {
                    g_mod.reset();
                }
                
                frame_t frame = build_bert_frame(callsign, token, frame_num++);
                encoded_bits_t encoded = encode_frame(frame);
                
                send_sync_word();
                send_encoded_frame(encoded);
                
                if (g_verbose && ((f + 1) % 10 == 0 || f == bert_frames - 1)) {
                    std::cerr << "Sent frame " << (f + 1) << "/" << bert_frames << "\n";
                }
            }
            
        } while (continuous);
    }
    
    // Trailing zeros to flush
    std::array<IQSample, SAMPLES_PER_SYMBOL> zeros = {};
    for (int i = 0; i < 100; ++i) output(zeros);
    
    if (g_verbose) std::cerr << "Done.\n";
    return 0;
}

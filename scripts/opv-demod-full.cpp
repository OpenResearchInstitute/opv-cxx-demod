/**
 * opv-demod-full.cpp - Complete OPV software demodulator
 * 
 * Tests the full chain: demod -> sync detect -> deinterleave -> Viterbi -> derandomize
 */

#include <iostream>
#include <fstream>
#include <cstdint>
#include <cmath>
#include <array>
#include <vector>
#include <iomanip>

constexpr size_t SAMPLES_PER_SYMBOL = 40;
constexpr double SAMPLE_RATE = 2168000.0;
constexpr double FREQ_DEV = 54200.0 / 4.0;
constexpr double F1_FREQ = -FREQ_DEV;
constexpr double F2_FREQ = +FREQ_DEV;
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

constexpr uint32_t SYNC_WORD = 0x02B8DB;
constexpr size_t SYNC_BITS = 24;
constexpr size_t PAYLOAD_BITS = 2144;
constexpr size_t FRAME_BYTES = 134;

struct IQSample { int16_t I, Q; };

// =============================================================================
// DEINTERLEAVER
// =============================================================================
void deinterleave(const std::vector<uint8_t>& in, std::vector<uint8_t>& out) {
    out.resize(PAYLOAD_BITS);
    for (size_t i = 0; i < PAYLOAD_BITS; ++i) {
        size_t row = i / 32;
        size_t col = i % 32;
        size_t src = col * 67 + row;
        out[i] = (src < in.size()) ? in[src] : 0;
    }
}

// =============================================================================
// VITERBI DECODER (Simplified - just extracts G1 bits)
// =============================================================================
void viterbi_decode(const std::vector<uint8_t>& encoded, std::vector<uint8_t>& decoded) {
    // 2144 bits -> 1072 pairs -> 1072 decoded bits -> 134 bytes
    decoded.resize(FRAME_BYTES);
    
    // Extract bits (G1 at even positions, G2 at odd)
    std::vector<uint8_t> decoded_bits(1072);
    for (size_t i = 0; i < 1072; ++i) {
        // Simple: just use G1 bit (real Viterbi would use both)
        decoded_bits[i] = encoded[2 * i];
    }
    
    // Pack into bytes, reversing encoder's backward byte order
    for (size_t byte_idx = 0; byte_idx < FRAME_BYTES; ++byte_idx) {
        uint8_t byte = 0;
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
            size_t bit_idx = (FRAME_BYTES - 1 - byte_idx) * 8 + (7 - bit_pos);
            if (bit_idx < 1072) {
                byte |= (decoded_bits[bit_idx] & 1) << bit_pos;
            }
        }
        decoded[byte_idx] = byte;
    }
}

// =============================================================================
// LFSR DERANDOMIZER
// =============================================================================
void derandomize(std::vector<uint8_t>& data) {
    uint8_t state = 0xFF;
    for (size_t i = 0; i < data.size(); ++i) {
        uint8_t lfsr_byte = 0;
        for (int j = 7; j >= 0; --j) {
            lfsr_byte |= ((state >> 7) & 1) << j;
            uint8_t fb = ((state >> 7) ^ (state >> 6) ^ (state >> 4) ^ (state >> 2)) & 1;
            state = (state << 1) | fb;
        }
        data[i] ^= lfsr_byte;
    }
}

// =============================================================================
// MAIN
// =============================================================================
int main(int argc, char* argv[]) {
    bool verbose = (argc > 1 && std::string(argv[1]) == "-v");
    
    double phase_f1 = 0.0, phase_f2 = 0.0;
    double f1_accum = 0.0, f2_accum = 0.0;
    int sample_count = 0;
    int prev_encoded = 0;
    int cclk = 0;
    
    uint32_t shift_reg = 0;
    std::vector<uint8_t> payload_bits;
    bool collecting = false;
    
    int frames_found = 0;
    int frames_decoded = 0;
    long total_samples = 0;
    long total_bits = 0;
    
    std::cerr << "OPV Full Software Demodulator\n";
    std::cerr << "=============================\n\n";
    
    IQSample s;
    while (std::cin.read(reinterpret_cast<char*>(&s), sizeof(s))) {
        total_samples++;
        
        double I = s.I / 32768.0;
        double Q = s.Q / 32768.0;
        
        double sin_f1 = std::sin(phase_f1), cos_f1 = std::cos(phase_f1);
        double sin_f2 = std::sin(phase_f2), cos_f2 = std::cos(phase_f2);
        
        f1_accum += I * sin_f1 + Q * cos_f1;
        f2_accum += I * sin_f2 + Q * cos_f2;
        
        phase_f1 += TWO_PI * F1_FREQ / SAMPLE_RATE;
        phase_f2 += TWO_PI * F2_FREQ / SAMPLE_RATE;
        while (phase_f1 > PI) phase_f1 -= TWO_PI;
        while (phase_f1 < -PI) phase_f1 += TWO_PI;
        while (phase_f2 > PI) phase_f2 -= TWO_PI;
        while (phase_f2 < -PI) phase_f2 += TWO_PI;
        
        sample_count++;
        
        if (sample_count >= SAMPLES_PER_SYMBOL) {
            total_bits++;
            
            double f2_comp = (cclk == 0) ? -f2_accum : f2_accum;
            double data_sum = f1_accum - f2_comp;
            
            int encoded = (data_sum < 0) ? 1 : 0;
            int decoded_bit = encoded ^ prev_encoded;
            prev_encoded = encoded;
            
            if (collecting) {
                payload_bits.push_back(decoded_bit);
                
                if (payload_bits.size() == PAYLOAD_BITS) {
                    collecting = false;
                    
                    // Deinterleave
                    std::vector<uint8_t> deinterleaved;
                    deinterleave(payload_bits, deinterleaved);
                    
                    // Viterbi decode
                    std::vector<uint8_t> decoded;
                    viterbi_decode(deinterleaved, decoded);
                    
                    // Derandomize
                    derandomize(decoded);
                    
                    // Print result
                    std::cerr << "Frame " << frames_found << ": ";
                    for (size_t i = 0; i < std::min(size_t(16), decoded.size()); ++i) {
                        std::cerr << std::hex << std::setw(2) << std::setfill('0') 
                                  << (int)decoded[i] << " ";
                    }
                    std::cerr << "...\n" << std::dec;
                    
                    // Check callsign
                    bool valid = true;
                    for (int i = 0; i < 5 && decoded[i] != 0; ++i) {
                        if (decoded[i] < 0x20 || decoded[i] > 0x7F) valid = false;
                    }
                    
                    if (valid && decoded[0] >= 'A' && decoded[0] <= 'Z') {
                        std::cerr << "  Callsign: ";
                        for (int i = 0; i < 6 && decoded[i] != 0; ++i) {
                            std::cerr << (char)decoded[i];
                        }
                        std::cerr << "\n";
                        frames_decoded++;
                    }
                    
                    payload_bits.clear();
                }
            } else {
                shift_reg = ((shift_reg << 1) | decoded_bit) & 0xFFFFFF;
                
                if (shift_reg == SYNC_WORD) {
                    frames_found++;
                    std::cerr << "\nSYNC DETECTED at bit " << total_bits << "\n";
                    collecting = true;
                    payload_bits.clear();
                }
            }
            
            sample_count = 0;
            f1_accum = 0.0;
            f2_accum = 0.0;
            cclk = 1 - cclk;
        }
    }
    
    std::cerr << "\n=============================\n";
    std::cerr << "Total samples: " << total_samples << "\n";
    std::cerr << "Total bits: " << total_bits << "\n";
    std::cerr << "Syncs found: " << frames_found << "\n";
    std::cerr << "Frames decoded: " << frames_decoded << "\n";
    
    return 0;
}

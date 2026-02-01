/**
 * opv-mod-fresh.cpp - OPV transmitter with DIFFERENTIAL encoding
 * 
 * The HDL demodulator uses differential decoding (line 237):
 *   data_bit_dec <= data_bit_enc WHEN data_bit_enc_t = '0' ELSE NOT data_bit_enc;
 * 
 * So we MUST use differential encoding on transmit.
 */

#include <iostream>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <array>
#include <getopt.h>

// =============================================================================
// PARAMETERS (from HDL)
// =============================================================================

constexpr size_t FRAME_BYTES = 134;
constexpr size_t FRAME_BITS = FRAME_BYTES * 8;
constexpr size_t ENCODED_BITS = FRAME_BITS * 2;

constexpr uint32_t SYNC_WORD = 0x02B8DB;

constexpr size_t SAMPLES_PER_SYMBOL = 40;
constexpr double SAMPLE_RATE = 2168000.0;
constexpr double SYMBOL_RATE = 54200.0;
constexpr double FREQ_DEV = SYMBOL_RATE / 4.0;  // 13550 Hz

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
    
    // Step 1: Apply LFSR in FORWARD byte order
    std::array<uint8_t, FRAME_BYTES> randomized;
    for (size_t i = 0; i < FRAME_BYTES; ++i) {
        randomized[i] = payload[i] ^ lfsr.next_byte();
    }
    
    // Step 2: Encode bytes BACKWARD, MSB first within byte
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
// DIFFERENTIAL MSK MODULATOR
// 
// The HDL demodulator expects differential encoding.
// From msk_demodulator.vhd line 237:
//   data_bit_dec <= data_bit_enc WHEN data_bit_enc_t = '0' ELSE NOT data_bit_enc;
// 
// This is differential decoding: output = received XOR previous_received
// So we need differential encoding: transmitted = input XOR previous_transmitted
// =============================================================================

class DifferentialMSK {
public:
    void reset() {
        phase = 0.0;
        prev_encoded = 0;  // Initial state
    }
    
    void modulate_bit(uint8_t tx_bit, std::array<IQSample, SAMPLES_PER_SYMBOL>& output) {
        // Differential encoding: encoded = tx_bit XOR prev_encoded
        uint8_t encoded = tx_bit ^ prev_encoded;
        prev_encoded = encoded;
        
        // Frequency selection based on encoded bit
        // bit 0 -> F1 (lower freq, -FREQ_DEV)
        // bit 1 -> F2 (higher freq, +FREQ_DEV)
        double freq = (encoded & 1) ? FREQ_DEV : -FREQ_DEV;
        double phase_inc = TWO_PI * freq / SAMPLE_RATE;
        
        for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
            output[i].I = static_cast<int16_t>(16383.0 * std::cos(phase));
            output[i].Q = static_cast<int16_t>(16383.0 * std::sin(phase));
            phase += phase_inc;
            while (phase > PI) phase -= TWO_PI;
            while (phase < -PI) phase += TWO_PI;
        }
    }
    
private:
    double phase = 0.0;
    uint8_t prev_encoded = 0;
};

// =============================================================================
// GLOBALS
// =============================================================================

DifferentialMSK g_mod;
bool g_verbose = false;

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
    // Transmit MSB-first within each byte (matching HDL byte packing)
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
    std::cerr << "Usage: " << prog << " -S CALLSIGN -B FRAMES [-c] [-v]\n";
    exit(1);
}

int main(int argc, char* argv[]) {
    const char* callsign = nullptr;
    int bert_frames = 0;
    bool continuous = false;
    
    int opt;
    while ((opt = getopt(argc, argv, "S:B:cvh")) != -1) {
        switch (opt) {
            case 'S': callsign = optarg; break;
            case 'B': bert_frames = std::atoi(optarg); break;
            case 'c': continuous = true; break;
            case 'v': g_verbose = true; break;
            default: usage(argv[0]);
        }
    }
    
    if (!callsign || bert_frames <= 0) {
        usage(argv[0]);
    }
    
    if (g_verbose) {
        std::cerr << "OPV Transmitter (Differential MSK)\n";
        std::cerr << "  Callsign: " << callsign << "\n";
        std::cerr << "  BERT frames: " << bert_frames << "\n";
    }
    
    uint32_t frame_num = 0;
    
    do {
        g_mod.reset();
        
        for (int f = 0; f < bert_frames; ++f) {
            frame_t frame = build_bert_frame(callsign, frame_num++);
            encoded_bits_t encoded = encode_frame(frame);
            
            send_sync_word();
            send_encoded_frame(encoded);
            
            if (g_verbose && ((f + 1) % 10 == 0 || f == bert_frames - 1)) {
                std::cerr << "Sent frame " << (f + 1) << "/" << bert_frames << "\n";
            }
        }
        
    } while (continuous);
    
    // Trailing silence
    std::array<IQSample, SAMPLES_PER_SYMBOL> zeros = {};
    for (int i = 0; i < 100; ++i) output(zeros);
    
    if (g_verbose) std::cerr << "Done.\n";
    return 0;
}

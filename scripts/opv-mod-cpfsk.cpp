/**
 * opv-mod-cpfsk - Simple CPFSK MSK modulator
 * 
 * This is the simplest possible MSK implementation:
 * - Single NCO that switches between F1 and F2
 * - Continuous phase (no discontinuities)
 * - No differential encoding (receiver handles that)
 * 
 * Use this to compare against the parallel-tone version.
 */

#include <iostream>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <array>
#include <getopt.h>

constexpr size_t FRAME_BYTES = 134;
constexpr size_t FRAME_BITS = FRAME_BYTES * 8;
constexpr size_t ENCODED_BITS = FRAME_BITS * 2;

constexpr uint32_t SYNC_WORD = 0x02B8DB;

constexpr size_t SAMPLES_PER_SYMBOL = 40;
constexpr double SAMPLE_RATE = 2168000.0;
constexpr double SYMBOL_RATE = 54200.0;
constexpr double FREQ_DEV = SYMBOL_RATE / 4.0;

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

using frame_t = std::array<uint8_t, FRAME_BYTES>;
using encoded_bits_t = std::array<uint8_t, ENCODED_BITS>;

struct IQSample { int16_t I, Q; };

// LFSR
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

// Convolutional Encoder
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

void interleave(encoded_bits_t& bits) {
    encoded_bits_t temp;
    for (size_t i = 0; i < ENCODED_BITS; ++i) {
        temp[(i % 32) * 67 + (i / 32)] = bits[i];
    }
    bits = temp;
}

bool g_forward_bytes = false;  // BACKWARD byte order (matches HDL)

encoded_bits_t encode_frame(const frame_t& payload) {
    LFSR lfsr; lfsr.reset();
    ConvEncoder conv; conv.reset();
    encoded_bits_t encoded;
    size_t out_idx = 0;
    
    // Step 1: Apply LFSR in FORWARD byte order (matching HDL RANDOMIZE state)
    std::array<uint8_t, FRAME_BYTES> randomized;
    for (size_t i = 0; i < FRAME_BYTES; ++i) {
        randomized[i] = payload[i] ^ lfsr.next_byte();
    }
    
    // Step 2: Encode bytes
    if (g_forward_bytes) {
        // Forward byte order (byte 0 first)
        for (size_t byte_idx = 0; byte_idx < FRAME_BYTES; ++byte_idx) {
            uint8_t byte = randomized[byte_idx];
            for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
                uint8_t g1, g2;
                conv.encode_bit((byte >> bit_pos) & 1, g1, g2);
                encoded[out_idx++] = g1;
                encoded[out_idx++] = g2;
            }
        }
    } else {
        // Backward byte order (byte 133 first, matching HDL)
        for (int byte_idx = FRAME_BYTES - 1; byte_idx >= 0; --byte_idx) {
            uint8_t byte = randomized[byte_idx];
            for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
                uint8_t g1, g2;
                conv.encode_bit((byte >> bit_pos) & 1, g1, g2);
                encoded[out_idx++] = g1;
                encoded[out_idx++] = g2;
            }
        }
    }
    
    interleave(encoded);
    return encoded;
}

// Simple CPFSK modulator
class CPFSKModulator {
public:
    void reset() { phase = 0.0; }
    
    void modulate_bit(uint8_t bit, std::array<IQSample, SAMPLES_PER_SYMBOL>& output, bool invert = false) {
        // Normal: bit 0 = -FREQ_DEV, bit 1 = +FREQ_DEV
        // Inverted: bit 0 = +FREQ_DEV, bit 1 = -FREQ_DEV
        uint8_t b = invert ? (bit ^ 1) : bit;
        double freq = (b & 1) ? FREQ_DEV : -FREQ_DEV;
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
};

CPFSKModulator g_mod;
bool g_verbose = false;
bool g_invert_polarity = false;

void output(const std::array<IQSample, SAMPLES_PER_SYMBOL>& s) {
    for (const auto& x : s) {
        std::cout.write(reinterpret_cast<const char*>(&x.I), 2);
        std::cout.write(reinterpret_cast<const char*>(&x.Q), 2);
    }
}

void send_preamble(size_t num_bits) {
    std::array<IQSample, SAMPLES_PER_SYMBOL> samples;
    for (size_t i = 0; i < num_bits; ++i) {
        g_mod.modulate_bit(i & 1, samples, g_invert_polarity);
        output(samples);
    }
}

void send_sync_word() {
    std::array<IQSample, SAMPLES_PER_SYMBOL> samples;
    for (int i = 23; i >= 0; --i) {
        g_mod.modulate_bit((SYNC_WORD >> i) & 1, samples, g_invert_polarity);
        output(samples);
    }
}

void send_encoded_frame(const encoded_bits_t& encoded) {
    std::array<IQSample, SAMPLES_PER_SYMBOL> samples;
    // Transmit MSB-first within each byte (matching HDL)
    // encoded[0] is byte0 bit0 (LSB), encoded[7] is byte0 bit7 (MSB)
    // Transmit: bit7, bit6, ..., bit0 for each byte
    for (size_t byte_idx = 0; byte_idx < ENCODED_BITS / 8; ++byte_idx) {
        for (int bit_in_byte = 7; bit_in_byte >= 0; --bit_in_byte) {
            g_mod.modulate_bit(encoded[byte_idx * 8 + bit_in_byte], samples, g_invert_polarity);
            output(samples);
        }
    }
}

frame_t build_bert_frame(const char* callsign, uint32_t frame_num) {
    frame_t frame = {};
    size_t len = strlen(callsign);
    for (size_t i = 0; i < 6 && i < len; ++i) frame[i] = callsign[i];
    frame[6] = (frame_num >> 16) & 0xFF;
    frame[7] = (frame_num >> 8) & 0xFF;
    frame[8] = frame_num & 0xFF;
    for (size_t i = 0; i < FRAME_BYTES - 12; ++i) frame[12 + i] = (frame_num + i) & 0xFF;
    return frame;
}

void usage(const char* prog) {
    std::cerr << "Usage: " << prog << " -S CALLSIGN -B FRAMES [-c] [-v] [-F] [-I]\n";
    std::cerr << "  -F         Forward byte order (default: backward/HDL-style)\n";
    std::cerr << "  -I         Invert bit polarity (swap 0/1 frequency mapping)\n";
    exit(1);
}

int main(int argc, char* argv[]) {
    const char* callsign = nullptr;
    int bert_frames = 0;
    bool continuous = false;
    
    int opt;
    while ((opt = getopt(argc, argv, "S:B:cvFIh")) != -1) {
        switch (opt) {
            case 'S': callsign = optarg; break;
            case 'B': bert_frames = std::atoi(optarg); break;
            case 'c': continuous = true; break;
            case 'v': g_verbose = true; break;
            case 'F': g_forward_bytes = true; break;
            case 'I': g_invert_polarity = true; break;
            default: usage(argv[0]);
        }
    }
    
    if (!callsign || bert_frames <= 0) usage(argv[0]);
    
    if (g_verbose) {
        std::cerr << "OPV CPFSK Transmitter (Simple)\n";
        std::cerr << "  Callsign: " << callsign << "\n";
        std::cerr << "  BERT frames: " << bert_frames << "\n";
        std::cerr << "  Byte order: " << (g_forward_bytes ? "FORWARD" : "BACKWARD") << "\n";
        std::cerr << "  Polarity: " << (g_invert_polarity ? "INVERTED" : "NORMAL") << "\n";
    }
    
    uint32_t frame_num = 0;
    
    do {
        g_mod.reset();
        
        // Send preamble for receiver lock
        if (g_verbose) std::cerr << "Sending preamble (2168 bits)...\n";
        send_preamble(2168);
        
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
    
    std::array<IQSample, SAMPLES_PER_SYMBOL> zeros = {};
    for (int i = 0; i < 100; ++i) output(zeros);
    
    if (g_verbose) std::cerr << "Done.\n";
    return 0;
}

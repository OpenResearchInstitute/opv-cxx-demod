#pragma once
// =============================================================================
// opv_codec.hpp -- THE single Opulent Voice frame codec (transmit side).
//
// One definition of the encoder, LFSR randomizer, and interleaver, included by
// BOTH opv-mod.cpp and opv-modem.cpp. There is no second copy to drift from --
// which is the entire point: the 174/155 tap bug and the tail-biting station-ID
// bug both came from hand-copied encoders getting out of sync.
//
// Convolutional code: K=7, rate 1/2, TRUE Voyager 171/133 (d_free=10).
//   Parity masks 0x67 / 0x76 for the state=(in<<6)|sr layout.  GROUND TRUTH:
//   a single 1 encodes to g1=1111001, g2=1011011.  (NOT 0x79/0x5B.)
// Framing: TAIL-BITING (start state = frame's own end state -> ring, no tail
//   floor), 67x32 block interleaver, CCSDS randomizer (x^8+x^7+x^5+x^3+1).
// =============================================================================
#include <array>
#include <cstdint>
#include <cstddef>
#include <cassert>

constexpr size_t  FRAME_BYTES  = 134;
constexpr size_t  FRAME_BITS   = FRAME_BYTES * 8;   // 1072
constexpr size_t  ENCODED_BITS = FRAME_BITS  * 2;   // 2144
constexpr uint8_t G1_MASK = 0x67;   // 171 octal, delays {0,1,2,3,6}
constexpr uint8_t G2_MASK = 0x76;   // 133 octal, delays {0,2,3,5,6}

using frame_t        = std::array<uint8_t, FRAME_BYTES>;
using encoded_bits_t = std::array<uint8_t, ENCODED_BITS>;

// CCSDS pseudo-randomizer -----------------------------------------------------
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

// K=7 rate-1/2 convolutional encoder (171/133) --------------------------------
class ConvEncoder {
public:
    void reset() { sr = 0; }
    uint8_t get_state() const { return sr; }
    void encode_bit(uint8_t in, uint8_t& g1, uint8_t& g2) {
        uint8_t state = (in << 6) | sr;
        g1 = __builtin_parity(state & G1_MASK);
        g2 = __builtin_parity(state & G2_MASK);
        sr = ((sr << 1) | in) & 0x3F;
    }
private:
    uint8_t sr = 0;
};

// 67x32 block interleaver (MSB-first byte correction to match HDL) ------------
inline void interleave(encoded_bits_t& bits) {
    encoded_bits_t temp = {};
    for (size_t i = 0; i < ENCODED_BITS; ++i) {
        size_t interleaved_pos = (i % 32) * 67 + (i / 32);
        size_t byte_num = interleaved_pos / 8;
        size_t bit_in_byte = interleaved_pos % 8;
        size_t corrected_pos = byte_num * 8 + (7 - bit_in_byte);
        temp[corrected_pos] = bits[i];
    }
    bits = temp;
}

// Frame encoder: randomize -> TAIL-BITING conv encode -> interleave -----------
inline encoded_bits_t encode_frame(const frame_t& payload) {
    LFSR lfsr; lfsr.reset();
    ConvEncoder conv;
    encoded_bits_t encoded = {};

    std::array<uint8_t, FRAME_BYTES> randomized;
    for (size_t i = 0; i < FRAME_BYTES; ++i)
        randomized[i] = payload[i] ^ lfsr.next_byte();

    // tail-biting pass 1: encode from zero to discover the end state (discard)
    conv.reset();
    for (int byte_idx = FRAME_BYTES - 1; byte_idx >= 0; --byte_idx)
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
            uint8_t g1, g2;
            conv.encode_bit((randomized[byte_idx] >> bit_pos) & 1, g1, g2);
        }
    uint8_t seed = conv.get_state();

    // pass 2: real encode, continuing from the seeded state (NO reset)
    size_t out_idx = 0;
    for (int byte_idx = FRAME_BYTES - 1; byte_idx >= 0; --byte_idx)
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
            uint8_t g1, g2;
            conv.encode_bit((randomized[byte_idx] >> bit_pos) & 1, g1, g2);
            encoded[out_idx++] = g1;
            encoded[out_idx++] = g2;
        }
    assert(conv.get_state() == seed);   // ring-closure invariant

    interleave(encoded);
    return encoded;
}

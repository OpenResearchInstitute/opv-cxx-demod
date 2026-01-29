// Copyright 2020 Mobilinkd LLC.
// Copyright 2022-2026 Open Research Institute, Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <cstdint>
#include <cstddef>

namespace mobilinkd
{

/**
 * CCSDS LFSR Randomizer for Opulent Voice Protocol
 * 
 * This implements the standard CCSDS randomizer used for spectral whitening
 * before FEC encoding (and derandomizing after FEC decoding).
 * 
 * Polynomial: x^8 + x^7 + x^5 + x^3 + 1
 * Seed: 0xFF (all ones)
 * Period: 255 bits
 * 
 * The LFSR is reset to 0xFF at the start of each frame.
 * 
 * Reference: CCSDS 131.0-B-3 (TM Synchronization and Channel Coding)
 */
class CcsdsLfsr
{
public:
    static constexpr uint8_t SEED = 0xFF;
    
    CcsdsLfsr() : state_(SEED) {}
    
    /// Reset LFSR to initial seed (call at start of each frame)
    void reset() { state_ = SEED; }
    
    /// Clock the LFSR once and return the output bit
    uint8_t clock()
    {
        // Output is MSB
        uint8_t output = (state_ >> 7) & 1;
        
        // Feedback taps: x^8 + x^7 + x^5 + x^3 + 1
        // In 0-indexed terms: bits 7, 6, 4, 2
        uint8_t feedback = ((state_ >> 7) ^ (state_ >> 6) ^ (state_ >> 4) ^ (state_ >> 2)) & 1;
        
        // Shift left and insert feedback at LSB
        state_ = ((state_ << 1) | feedback) & 0xFF;
        
        return output;
    }
    
    /// Generate 8 output bits (one byte), MSB first
    uint8_t output_byte()
    {
        uint8_t result = 0;
        for (int i = 7; i >= 0; --i)
        {
            result |= (clock() << i);
        }
        return result;
    }
    
    /// Get current state (for debugging)
    uint8_t state() const { return state_; }
    
private:
    uint8_t state_;
};


/**
 * Frame-level randomizer for Opulent Voice
 * 
 * Randomizes/derandomizes a 134-byte frame using CCSDS LFSR.
 * The operation is symmetric (XOR), so the same function is used
 * for both randomization and derandomization.
 * 
 * Usage:
 *   OPVFrameRandomizer randomizer;
 *   randomizer(frame_bytes);  // Randomize before FEC encode
 *   ...
 *   randomizer(decoded_bytes); // Derandomize after FEC decode
 */
template <size_t N = 134>  // 134 bytes = input frame size before FEC
class OPVFrameRandomizer
{
public:
    using frame_t = std::array<uint8_t, N>;
    
    /// Randomize or derandomize a byte array (same operation)
    void operator()(frame_t& frame)
    {
        lfsr_.reset();
        for (size_t i = 0; i < N; ++i)
        {
            frame[i] ^= lfsr_.output_byte();
        }
    }
    
    /// Randomize/derandomize with explicit reset
    void randomize(frame_t& frame)
    {
        operator()(frame);
    }
    
    /// Alias for clarity in receive path
    void derandomize(frame_t& frame)
    {
        operator()(frame);
    }
    
private:
    CcsdsLfsr lfsr_;
};


/**
 * Generate the first N bytes of LFSR output for testing
 * 
 * Test vector (first 10 bytes from seed 0xFF):
 *   0xFF, 0x1A, 0xAF, 0x66, 0x52, 0x23, 0x1E, 0x10, 0xA0, 0xF9
 */
template <size_t N>
std::array<uint8_t, N> generate_lfsr_sequence()
{
    std::array<uint8_t, N> result;
    CcsdsLfsr lfsr;
    for (size_t i = 0; i < N; ++i)
    {
        result[i] = lfsr.output_byte();
    }
    return result;
}


// =============================================================================
// BACKWARD COMPATIBILITY ALIAS
// =============================================================================
// The old code used OPVRandomizer<N> where N was the bit count.
// This alias allows old code to compile while we migrate.
// TODO: Remove this once all code is migrated to OPVFrameRandomizer.

template <size_t N_BITS>
using OPVRandomizer = OPVFrameRandomizer<N_BITS / 8>;



} // namespace mobilinkd

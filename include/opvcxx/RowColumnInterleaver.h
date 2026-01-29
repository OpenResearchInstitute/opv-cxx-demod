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
 * Row-Column Block Interleaver for Opulent Voice Protocol
 * 
 * This implements a 67×32 bit-level block interleaver for burst error
 * protection. Consecutive bits are spread 67 positions apart, allowing
 * the Viterbi decoder to correct burst errors that would otherwise
 * overwhelm the convolutional code.
 * 
 * Dimensions: 67 rows × 32 columns = 2144 bits (268 bytes)
 * 
 * Write order: Row-major (fill rows sequentially)
 * Read order:  Column-major (read columns sequentially)
 * 
 * Address mapping:
 *   For input bit position p (0 to 2143):
 *     row = p / 32
 *     col = p % 32
 *     output_position = col * 67 + row
 * 
 * Effect: Input bits 0,1,2,3... appear at positions 0,67,134,201...
 * 
 * This matches the HDL implementation in ov_frame_encoder.vhd
 */
template <size_t ROWS = 67, size_t COLS = 32>
class RowColumnInterleaver
{
public:
    static constexpr size_t N_BITS = ROWS * COLS;      // 2144 bits
    static constexpr size_t N_BYTES = N_BITS / 8;      // 268 bytes
    
    using soft_buffer_t = std::array<int8_t, N_BITS>;  // Soft decision values (one per bit)
    using byte_buffer_t = std::array<uint8_t, N_BYTES>; // Packed bytes
    
    /**
     * Calculate interleaved output position for a given input position.
     * Write row-major, read column-major.
     */
    static constexpr size_t interleave_index(size_t p)
    {
        size_t row = p / COLS;
        size_t col = p % COLS;
        return col * ROWS + row;
    }
    
    /**
     * Calculate deinterleaved output position for a given input position.
     * This is the inverse mapping.
     */
    static constexpr size_t deinterleave_index(size_t p)
    {
        size_t col = p / ROWS;
        size_t row = p % ROWS;
        return row * COLS + col;
    }
    
    /**
     * Interleave soft decision buffer (one int8_t per bit)
     * Used in transmit path after FEC encoding.
     */
    void interleave(soft_buffer_t& data) const
    {
        soft_buffer_t temp;
        for (size_t i = 0; i < N_BITS; ++i)
        {
            temp[interleave_index(i)] = data[i];
        }
        data = temp;
    }
    
    /**
     * Deinterleave soft decision buffer (one int8_t per bit)
     * Used in receive path before Viterbi decoding.
     */
    void deinterleave(soft_buffer_t& data) const
    {
        soft_buffer_t temp;
        for (size_t i = 0; i < N_BITS; ++i)
        {
            temp[i] = data[interleave_index(i)];
        }
        data = temp;
    }
    
    /**
     * Interleave packed byte buffer at bit level.
     * Each byte is unpacked, bits are interleaved, then repacked.
     */
    void interleave(byte_buffer_t& data) const
    {
        // Unpack to bits
        soft_buffer_t bits;
        for (size_t i = 0; i < N_BYTES; ++i)
        {
            for (size_t j = 0; j < 8; ++j)
            {
                bits[i * 8 + j] = (data[i] >> (7 - j)) & 1;
            }
        }
        
        // Interleave
        soft_buffer_t interleaved;
        for (size_t i = 0; i < N_BITS; ++i)
        {
            interleaved[interleave_index(i)] = bits[i];
        }
        
        // Repack to bytes
        for (size_t i = 0; i < N_BYTES; ++i)
        {
            uint8_t byte = 0;
            for (size_t j = 0; j < 8; ++j)
            {
                byte |= (interleaved[i * 8 + j] << (7 - j));
            }
            data[i] = byte;
        }
    }
    
    /**
     * Deinterleave packed byte buffer at bit level.
     */
    void deinterleave(byte_buffer_t& data) const
    {
        // Unpack to bits
        soft_buffer_t bits;
        for (size_t i = 0; i < N_BYTES; ++i)
        {
            for (size_t j = 0; j < 8; ++j)
            {
                bits[i * 8 + j] = (data[i] >> (7 - j)) & 1;
            }
        }
        
        // Deinterleave
        soft_buffer_t deinterleaved;
        for (size_t i = 0; i < N_BITS; ++i)
        {
            deinterleaved[i] = bits[interleave_index(i)];
        }
        
        // Repack to bytes
        for (size_t i = 0; i < N_BYTES; ++i)
        {
            uint8_t byte = 0;
            for (size_t j = 0; j < 8; ++j)
            {
                byte |= (deinterleaved[i * 8 + j] << (7 - j));
            }
            data[i] = byte;
        }
    }
};

// Default interleaver type for OPV (67×32 = 2144 bits)
using OPVInterleaver = RowColumnInterleaver<67, 32>;

} // namespace mobilinkd

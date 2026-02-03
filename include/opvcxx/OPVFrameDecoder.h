// Copyright 2021 Mobilinkd LLC.
// Copyright 2022-2026 Open Research Institute, Inc.
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// OPV Frame Decoder - HDL-aligned version
// 
// Pipeline: deinterleave → Viterbi → derandomize → parse

#pragma once

#include "OPVRandomizer.h"
#include "RowColumnInterleaver.h"
#include "Trellis.h"
#include "Viterbi.h"
#include "Numerology.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <iostream>

namespace mobilinkd
{

struct OPVFrameDecoder
{
    // HDL-aligned components
    OPVInterleaver interleaver_;                                              // 67x32 row-column
    Trellis<6,2> trellis_{makeTrellis<6, 2>({opv_conv_G1, opv_conv_G2})};     // K=7 NASA/Voyager
    Viterbi<decltype(trellis_), 4> viterbi_{trellis_};
    OPVFrameRandomizer<opv_frame_bytes> derandomizer_;                        // CCSDS LFSR on 134 bytes

    enum class State { ACQ, STREAM };
    enum class DecodeResult { FAIL, OK, EOS };
    enum class FrameType { OPV_COBS, OPV_BERT };

    State state_ = State::ACQ;

    // Buffer types for new pipeline
    using encoded_frame_t = std::array<int8_t, opv_encoded_bits>;     // 2144 soft bits (input)
    using decoded_frame_t = std::array<uint8_t, opv_frame_bytes>;     // 134 bytes (after Viterbi)
    
    // Backward-compatible type aliases (used by OPVDemodulator.h and opv-demod.cpp)
    using frame_type4_buffer_t = encoded_frame_t;
    using stream_type1_bytes_t = std::array<uint8_t, opv_payload_bytes>;  // 122 bytes

    // Output structure
    using output_buffer_t = struct {
        FrameType type;
        std::array<uint8_t, opv_header_bytes> header;      // 12 bytes: callsign, token, flags
        std::array<uint8_t, opv_payload_bytes> data;       // 122 bytes: Opus, BERT, etc.
    };

    /**
     * Callback function for decoded frames.
     * Returns true if data was good, false if known bad.
     */
    using callback_t = std::function<bool(const output_buffer_t&, int viterbi_cost)>;

    callback_t callback_;
    output_buffer_t output_buffer_;

    OPVFrameDecoder(callback_t callback)
    : callback_(callback)
    {}

    void reset()
    {
        state_ = State::ACQ;
    }

    /**
     * Parse the 134-byte decoded frame into header and payload.
     * 
     * Frame structure:
     *   Bytes 0-11:   Header (callsign, token, flags)
     *   Bytes 12-133: Payload (Opus audio, BERT data, etc.)
     */
    void parse_frame(const decoded_frame_t& frame)
    {
        // Copy header (first 12 bytes)
        std::copy(frame.begin(), frame.begin() + opv_header_bytes, 
                  output_buffer_.header.begin());
        
        // Copy payload (remaining 122 bytes)
        std::copy(frame.begin() + opv_header_bytes, frame.end(),
                  output_buffer_.data.begin());
        
        // Determine frame type from flags byte
        // Header byte 6 contains flags: bit 7 = LAST_FRAME, bit 6 = BERT_MODE
        uint8_t flags = output_buffer_.header[6];
        output_buffer_.type = (flags & 0x40) ? FrameType::OPV_BERT : FrameType::OPV_COBS;
    }

    /**
     * Decode OPV frame.
     * 
     * Input: 2144 soft decision bits (after sync detection, before deinterleave)
     * 
     * Pipeline:
     *   1. Deinterleave (67x32 row-column)
     *   2. Viterbi decode (K=7 NASA, entire frame)
     *   3. Derandomize (CCSDS LFSR)
     *   4. Parse header + payload
     */
    DecodeResult operator()(encoded_frame_t& buffer, size_t& viterbi_cost)
    {
        // 1. Deinterleave
        interleaver_.deinterleave(buffer);

        // 2. Viterbi decode entire frame (2144 soft bits → 134 bytes)
        //    Note: Viterbi outputs unpacked bits, need to pack to bytes
        std::array<uint8_t, opv_frame_bits> decoded_bits;
        viterbi_cost = viterbi_.decode(buffer, decoded_bits);
        
        // Pack bits to bytes
        decoded_frame_t decoded_bytes;
        for (size_t i = 0; i < opv_frame_bytes; ++i)
        {
            uint8_t byte = 0;
            for (size_t j = 0; j < 8; ++j)
            {
                byte = (byte << 1) | (decoded_bits[i * 8 + j] & 1);
            }
            decoded_bytes[i] = byte;
        }

        // 3. Derandomize (CCSDS LFSR on 134 bytes)
        derandomizer_.derandomize(decoded_bytes);

        // 4. Parse into header and payload
        parse_frame(decoded_bytes);

        // Check for end-of-stream flag
        uint8_t flags = output_buffer_.header[6];
        bool is_last = (flags & 0x80) != 0;
        
        if (is_last)
        {
            state_ = State::ACQ;
        }

        // Invoke callback
        callback_(output_buffer_, viterbi_cost);

        return is_last ? DecodeResult::EOS : DecodeResult::OK;
    }
};

} // namespace mobilinkd

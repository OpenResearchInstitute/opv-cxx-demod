#pragma once

#include <array>
#include <cassert>

namespace mobilinkd
{
    // =========================================================================
    // AUDIO PARAMETERS (unchanged)
    // =========================================================================
    
    const int opus_bitrate = 16000;         // target output bit rate from voice encoder
    const int audio_sample_rate = 48000;    // 16-bit PCM samples per second for audio signals
    const int audio_samples_per_opv_frame = audio_sample_rate * 0.04;  // PCM samples per 40ms frame
    const int audio_bytes_per_opv_frame = audio_samples_per_opv_frame * 2;  // 2 bytes per PCM sample

    // =========================================================================
    // NEW OPV CONSTANTS (HDL-aligned, pluto_msk compatible)
    // =========================================================================
    // These constants match the FPGA reference implementation.
    // Use these for new code; legacy constants below are for migration only.
    
    // --- Frame Structure ---
    const int opv_header_bytes = 12;                // OPV frame header (callsign, token, flags)
    const int opv_payload_bytes = 122;              // OPV payload (Opus audio, BERT, text, data)
    const int opv_frame_bytes = 134;                // Total frame before FEC (header + payload)
    const int opv_frame_bits = opv_frame_bytes * 8; // 1072 bits
    
    // --- After FEC (K=7, rate 1/2 convolutional) ---
    const int opv_encoded_bytes = 268;              // 134 * 2 after rate-1/2 encoding
    const int opv_encoded_bits = 2144;              // 268 * 8 = 67 * 32 (matches interleaver)
    
    // --- Sync Word ---
    const int opv_sync_word = 0x02B8DB;             // 24-bit sync word (PSLR optimized)
    const int opv_sync_bits = 24;
    const int opv_sync_bytes = 3;
    
    // --- Row-Column Interleaver (67 x 32) ---
    const int opv_interleaver_rows = 67;
    const int opv_interleaver_cols = 32;
    // Note: 67 * 32 = 2144 bits = opv_encoded_bits (must match!)
    
    // --- NASA/Voyager K=7 Convolutional Code ---
    const int opv_conv_K = 7;                       // Constraint length (64 states)
    const int opv_conv_G1 = 0171;                   // Generator 1: 171 octal = 0x79
    const int opv_conv_G2 = 0133;                   // Generator 2: 133 octal = 0x5B
    
    // --- Derived timing constants ---
    const int opv_total_frame_bits = opv_sync_bits + opv_encoded_bits;  // 24 + 2144 = 2168 bits
    const int opv_frame_period_ms = 40;             // 40ms per frame
    
    // --- BERT (Bit Error Rate Test) ---
    const int opv_bert_payload_bits = opv_payload_bytes * 8;  // 976 bits
    const int opv_bert_prime_size = 971;            // Largest prime < 976 for PRBS cycling
    
    // --- Static assertions for new constants ---
    static_assert(opv_interleaver_rows * opv_interleaver_cols == opv_encoded_bits,
                  "Interleaver dimensions must match encoded frame size");
    static_assert(opv_frame_bytes == opv_header_bytes + opv_payload_bytes,
                  "Frame size must equal header + payload");
    static_assert(opv_encoded_bytes == opv_frame_bytes * 2,
                  "Encoded size must be 2x frame size (rate 1/2)");
    static_assert(opv_bert_prime_size < opv_bert_payload_bits,
                  "BERT prime must be smaller than payload");
    
    // =========================================================================
    // LEGACY CONSTANTS (M17 heritage - for migration, will be removed)
    // =========================================================================
    // WARNING: These constants are from the old architecture which used:
    //   - Golay encoding for header (separate from payload)
    //   - K=5 convolutional code (not K=7)
    //   - Polynomial interleaver (not row-column)
    //   - Randomizer applied AFTER interleaving (should be BEFORE FEC)
    // Do not use these for new code!
    
    // (converting to) COBS/RDP/UDP/IP Frame Format for OPUlent Voice
    const int fheader_size_bytes = 12;        // bytes in a frame header (multiple of 3 for Golay encoding)
    const int encoded_fheader_size = fheader_size_bytes * 8 * 2;    // bits in an encoded frame header

    const int opus_frame_size_bytes = 80;   // bytes in an encoded 40ms Opus frame (including a TOC byte)
    const int opus_packet_size_bytes = opus_frame_size_bytes;   // exactly one frame per packet
    const int rtp_header_bytes = 12;    // per RFC3550
    const int udp_header_bytes = 8;     // per RFC768
    const int ip_v4_header_bytes = 20;  // per RFC791 (IPv4 only)
    const int cobs_overhead_bytes_for_opus = 2; // max of 1 byte COBS (since Opus packet < 254 byte) plus a 0 separator
    const int total_protocol_bytes = rtp_header_bytes + udp_header_bytes + ip_v4_header_bytes + cobs_overhead_bytes_for_opus;

    const int stream_frame_payload_bytes = total_protocol_bytes + opus_packet_size_bytes;   // All frames carry this much type1 payload data
    const int stream_frame_payload_size = 8 * stream_frame_payload_bytes;
    const int stream_frame_type1_size = stream_frame_payload_size + 4;  // add encoder tail bits
    const int stream_type2_payload_size = 2 * stream_frame_type1_size;  // Encoding type1 to type2 doubles the size, plus encoder tail
    const int stream_type3_payload_size = stream_type2_payload_size;    // no puncturing in OPUlent Voice
    const int stream_type4_size = encoded_fheader_size + stream_type3_payload_size;
    const int stream_type4_bytes = stream_type4_size / 8;
    
    const int baseband_frame_symbols = 16 / 2 + stream_type4_size / 2;   // dibits or symbols in sync+payload in a frame
    const int baseband_frame_packed_bytes = baseband_frame_symbols / 4;  // packed bytes in sync_payload in a frame
    
    const int bert_frame_total_size = stream_frame_payload_size;
    const int bert_frame_prime_size = 971;      // largest prime smaller than bert_frame_total_size

    const int symbol_rate = baseband_frame_symbols / 0.04;  // symbols per second
    const int sample_rate = symbol_rate * 10;               // sample rate
    const int samples_per_frame = sample_rate * 0.04;       // samples per 40ms frame

    static_assert((stream_type3_payload_size % 8) == 0, "Type3 payload size not an integer number of bytes");
    static_assert(bert_frame_prime_size < bert_frame_total_size, "BERT prime size not less than BERT total size");
    static_assert(bert_frame_prime_size % 2 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 3 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 5 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 7 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 11 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 13 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 17 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 19 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 23 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 29 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 31 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 37 != 0, "BERT prime size not prime");
    static_assert(bert_frame_prime_size % 41 != 0, "BERT prime size not prime");

    // LEGACY: Polynomial interleaver coefficients (replaced by row-column interleaver)
    const int PolynomialInterleaverX = 59;
    const int PolynomialInterleaverX2 = 1076;

    // LEGACY: M17 convolutional code (replaced by NASA/Voyager K=7)
    const int ConvolutionPolyA = 031;   // octal representation of taps
    const int ConvolutionPolyB = 027;   // octal representation of taps

    // Parameters for data communication
    const int ip_mtu = 1500;    // common Maximum Transmission Unit (MTU) value for Ethernet, in bytes
}

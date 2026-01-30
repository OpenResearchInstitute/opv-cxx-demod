// Copyright 2026 Open Research Institute, Inc.
// SPDX-License-Identifier: MIT
//
// MSK Modulator for OPV
//
// Minimum Shift Keying (MSK) is continuous-phase FSK with h=0.5
// Each bit shifts phase by ±90° using half-sinusoid shaping
//
// Output: Complex I/Q samples for SDR transmission

#pragma once

#include <array>
#include <vector>
#include <cmath>
#include <cstdint>

namespace mobilinkd {

/**
 * MSK Modulator
 * 
 * Converts bits to complex I/Q baseband samples.
 * Uses half-sinusoid frequency pulse for smooth phase transitions.
 *
 * Template parameters:
 *   SamplesPerSymbol - oversampling factor (default 10)
 */
template <size_t SamplesPerSymbol = 10>
class MSKModulator
{
public:
    static constexpr size_t SAMPLES_PER_SYMBOL = SamplesPerSymbol;
    static constexpr double PI = 3.14159265358979323846;
    
    // Output sample type (interleaved I/Q)
    struct IQSample {
        int16_t I;
        int16_t Q;
    };

private:
    double phase_ = 0.0;           // Current phase accumulator
    double amplitude_ = 16383.0;   // Output amplitude (for 16-bit signed: max 32767)
    int8_t prev_symbol_ = 1;       // Previous symbol for differential encoding (+1 or -1)
    
    // Precomputed half-sinusoid lookup table for phase progression
    std::array<double, SamplesPerSymbol> phase_lut_;
    
public:
    MSKModulator(bool use_linear_phase = true)
    {
        // Precompute phase progression lookup table
        for (size_t i = 0; i < SamplesPerSymbol; ++i)
        {
            double t = static_cast<double>(i) / SamplesPerSymbol;
            if (use_linear_phase)
            {
                // Linear phase ramp = constant frequency (true MSK, matches HDL)
                // This gives rectangular frequency pulse -> 81.3 kHz null-to-null BW
                phase_lut_[i] = t;
            }
            else
            {
                // Half-sinusoid phase = smooth frequency transition (filtered MSK)
                // This gives raised-cosine frequency pulse
                phase_lut_[i] = (1.0 - std::cos(PI * t)) / 2.0;
            }
        }
    }
    
    /**
     * Reset the modulator state
     */
    void reset()
    {
        phase_ = 0.0;
        prev_symbol_ = 1;  // Match HDL init state
    }
    
    /**
     * Set output amplitude (default 16383 for ~50% of 16-bit range)
     */
    void set_amplitude(double amp)
    {
        amplitude_ = amp;
    }
    
    /**
     * Modulate a single bit with HDL-compatible differential encoding
     * 
     * HDL logic (from msk_modulator.vhd):
     *   d_val = +1 when bit=0, -1 when bit=1
     *   output_symbol = d_val * prev_symbol (XOR on signs)
     * 
     * @param bit - 0 or 1
     * @param output - array to receive SamplesPerSymbol I/Q pairs
     */
    void modulate_bit(uint8_t bit, std::array<IQSample, SamplesPerSymbol>& output)
    {
        // Map bit to d_val: 0 → +1, 1 → -1 (matches HDL)
        int8_t d_val = (bit & 1) ? -1 : +1;
        
        // Differential encoding: multiply signs (equivalent to XOR)
        int8_t symbol = d_val * prev_symbol_;
        
        // Update state for next bit
        prev_symbol_ = symbol;
        
        // Phase change: +1 → +π/2, -1 → -π/2
        double delta_phase = symbol * (PI / 2.0);
        
        double start_phase = phase_;
        
        // Generate samples with smooth phase transition
        for (size_t i = 0; i < SamplesPerSymbol; ++i)
        {
            // Instantaneous phase using half-sinusoid shaping
            double inst_phase = start_phase + delta_phase * phase_lut_[i];
            
            // Generate I/Q
            output[i].I = static_cast<int16_t>(amplitude_ * std::cos(inst_phase));
            output[i].Q = static_cast<int16_t>(amplitude_ * std::sin(inst_phase));
        }
        
        // Update phase for next symbol
        phase_ = start_phase + delta_phase;
        
        // Keep phase bounded to avoid floating point issues
        while (phase_ > PI) phase_ -= 2.0 * PI;
        while (phase_ < -PI) phase_ += 2.0 * PI;
    }
    
    /**
     * Modulate an array of bits
     * 
     * @param bits - input bits (unpacked, one bit per byte, LSB used)
     * @param num_bits - number of bits to process
     * @return vector of I/Q samples
     */
    template <typename InputIt>
    std::vector<IQSample> modulate(InputIt bits_begin, size_t num_bits)
    {
        std::vector<IQSample> output;
        output.reserve(num_bits * SamplesPerSymbol);
        
        std::array<IQSample, SamplesPerSymbol> symbol_samples;
        
        auto it = bits_begin;
        for (size_t i = 0; i < num_bits; ++i, ++it)
        {
            modulate_bit(*it & 1, symbol_samples);
            output.insert(output.end(), symbol_samples.begin(), symbol_samples.end());
        }
        
        return output;
    }
    
    /**
     * Modulate a byte array (packed bits, MSB first)
     * 
     * @param bytes - input bytes
     * @param num_bytes - number of bytes
     * @return vector of I/Q samples
     */
    std::vector<IQSample> modulate_bytes(const uint8_t* bytes, size_t num_bytes)
    {
        std::vector<IQSample> output;
        output.reserve(num_bytes * 8 * SamplesPerSymbol);
        
        std::array<IQSample, SamplesPerSymbol> symbol_samples;
        
        for (size_t i = 0; i < num_bytes; ++i)
        {
            uint8_t byte = bytes[i];
            // MSB first
            for (int bit_idx = 7; bit_idx >= 0; --bit_idx)
            {
                uint8_t bit = (byte >> bit_idx) & 1;
                modulate_bit(bit, symbol_samples);
                output.insert(output.end(), symbol_samples.begin(), symbol_samples.end());
            }
        }
        
        return output;
    }
    
    /**
     * Generate preamble (alternating 1/0 pattern)
     * 
     * @param num_bits - number of preamble bits
     * @return vector of I/Q samples
     */
    std::vector<IQSample> generate_preamble(size_t num_bits)
    {
        std::vector<IQSample> output;
        output.reserve(num_bits * SamplesPerSymbol);
        
        std::array<IQSample, SamplesPerSymbol> symbol_samples;
        
        for (size_t i = 0; i < num_bits; ++i)
        {
            modulate_bit(i & 1, symbol_samples);  // Alternating 0, 1, 0, 1...
            output.insert(output.end(), symbol_samples.begin(), symbol_samples.end());
        }
        
        return output;
    }
    
    /**
     * Generate unmodulated carrier (for dead carrier periods)
     * 
     * @param num_samples - number of I/Q samples to generate
     * @return vector of I/Q samples at current phase
     */
    std::vector<IQSample> generate_carrier(size_t num_samples)
    {
        std::vector<IQSample> output(num_samples);
        
        // Constant phase = constant I/Q
        int16_t I = static_cast<int16_t>(amplitude_ * std::cos(phase_));
        int16_t Q = static_cast<int16_t>(amplitude_ * std::sin(phase_));
        
        for (auto& sample : output)
        {
            sample.I = I;
            sample.Q = Q;
        }
        
        return output;
    }
};

// 40 samples/bit gives 2.168 MSPS at 54200 bps
// PlutoSDR minimum is ~521 kSPS, so this is safe
// Pluto will interpolate up to 61.44 MSPS DAC rate
// Default uses linear phase (true MSK) to match HDL
using OPVMSKModulator = MSKModulator<40>;

} // namespace mobilinkd

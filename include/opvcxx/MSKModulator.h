#pragma once
// MSKModulator.h - HDL-compatible MSK modulator with differential encoding
// Matches msk_modulator.vhd exactly

#include <array>
#include <cstdint>
#include <cmath>
#include <vector>

namespace opvcxx {

/**
 * MSK Modulator matching HDL implementation in msk_modulator.vhd
 * 
 * Key parameters (from HDL):
 *   - Bit rate: 54,200 bps
 *   - Frame duration: 40 ms (2168 bits per frame)
 *   - MSK bandwidth: 1.5 × bit_rate = 81.3 kHz
 * 
 * Sample rate: 40 samples/bit = 2.168 MSPS (PlutoSDR interpolates to DAC rate)
 * 
 * Differential encoding (matches HDL msk_modulator.vhd):
 *   - bit 0 → d_val = +1
 *   - bit 1 → d_val = -1
 *   - symbol = d_val * prev_symbol
 * 
 * Phase progression:
 *   - symbol +1 → phase increases by π/2 (linear ramp)
 *   - symbol -1 → phase decreases by π/2 (linear ramp)
 */
template<size_t SamplesPerSymbol = 40>
class OPVMSKModulator {
public:
    static constexpr size_t SAMPLES_PER_SYMBOL = SamplesPerSymbol;
    static constexpr double PI = 3.14159265358979323846;
    
    struct IQSample {
        int16_t I;
        int16_t Q;
    };
    
    OPVMSKModulator()
        : phase_(0.0)
        , prev_symbol_(1)  // Initial state matches HDL
        , amplitude_(16383.0)  // ~50% of 16-bit range
    {
        // Pre-compute phase lookup table for linear ramp (true MSK)
        for (size_t i = 0; i < SamplesPerSymbol; ++i) {
            double t = static_cast<double>(i + 1) / SamplesPerSymbol;
            phase_lut_[i] = t;  // Linear phase ramp
        }
    }
    
    /**
     * Reset modulator state
     */
    void reset()
    {
        phase_ = 0.0;
        prev_symbol_ = 1;
    }
    
    /**
     * Get current differential encoder state
     */
    int8_t get_prev_symbol() const
    {
        return prev_symbol_;
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
        
        // Generate samples with linear phase transition (true MSK)
        for (size_t i = 0; i < SamplesPerSymbol; ++i)
        {
            // Instantaneous phase using linear ramp
            double inst_phase = start_phase + delta_phase * phase_lut_[i];
            
            // Generate I/Q
            output[i].I = static_cast<int16_t>(amplitude_ * std::cos(inst_phase));
            output[i].Q = static_cast<int16_t>(amplitude_ * std::sin(inst_phase));
        }
        
        // Update accumulated phase (wrapped to ±2π for numerical stability)
        phase_ = std::fmod(start_phase + delta_phase, 2.0 * PI);
    }
    
    /**
     * Generate continuous carrier (single symbol duration, output to array)
     */
    void generate_carrier(std::array<IQSample, SamplesPerSymbol>& output)
    {
        for (size_t i = 0; i < SamplesPerSymbol; ++i)
        {
            output[i].I = static_cast<int16_t>(amplitude_ * std::cos(phase_));
            output[i].Q = static_cast<int16_t>(amplitude_ * std::sin(phase_));
        }
    }
    
    /**
     * Generate continuous carrier (arbitrary number of samples)
     */
    std::vector<IQSample> generate_carrier(size_t num_samples)
    {
        std::vector<IQSample> samples(num_samples);
        for (size_t i = 0; i < num_samples; ++i)
        {
            samples[i].I = static_cast<int16_t>(amplitude_ * std::cos(phase_));
            samples[i].Q = static_cast<int16_t>(amplitude_ * std::sin(phase_));
        }
        return samples;
    }
    
    /**
     * Generate preamble (alternating 0/1 bits)
     */
    std::vector<IQSample> generate_preamble(size_t num_bits)
    {
        std::vector<IQSample> samples;
        samples.reserve(num_bits * SamplesPerSymbol);
        std::array<IQSample, SamplesPerSymbol> symbol_samples;
        
        for (size_t i = 0; i < num_bits; ++i)
        {
            modulate_bit(i & 1, symbol_samples);
            for (const auto& s : symbol_samples)
            {
                samples.push_back(s);
            }
        }
        return samples;
    }
    
    /**
     * Modulate a byte (8 bits, MSB first)
     */
    void modulate_byte(uint8_t byte, 
                       std::array<std::array<IQSample, SamplesPerSymbol>, 8>& output)
    {
        for (int i = 7; i >= 0; --i)
        {
            modulate_bit((byte >> i) & 1, output[7 - i]);
        }
    }

private:
    double phase_;              // Current carrier phase
    int8_t prev_symbol_;        // Previous symbol for differential encoding
    double amplitude_;          // Output amplitude
    std::array<double, SamplesPerSymbol> phase_lut_;  // Phase shaping LUT
};

// Default modulator type
using OPVMSKModulator40 = OPVMSKModulator<40>;

} // namespace opvcxx

// Global namespace alias for backward compatibility with opv-mod.cpp
using OPVMSKModulator = opvcxx::OPVMSKModulator<40>;

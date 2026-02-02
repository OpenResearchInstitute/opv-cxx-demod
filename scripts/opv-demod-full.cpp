/**
 * opv-demod-full.cpp - OPV demodulator with Viterbi matched to HDL encoder
 * 
 * Reads int16 IQ samples from stdin, outputs decoded frames to stdout.
 */
#include <iostream>
#include <vector>
#include <array>
#include <cstdint>
#include <cmath>
#include <iomanip>

constexpr size_t SAMPLES_PER_SYMBOL = 40;
constexpr double SAMPLE_RATE = 2168000.0;
constexpr double FREQ_DEV = 54200.0 / 4.0;
constexpr double F1_FREQ = -FREQ_DEV;
constexpr double F2_FREQ = +FREQ_DEV;
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr uint32_t SYNC_WORD = 0x02B8DB;
constexpr size_t FRAME_BYTES = 134;
constexpr size_t ENCODED_BITS = 2144;

struct IQSample { int16_t I, Q; };

void deinterleave(const std::vector<uint8_t>& in, std::vector<uint8_t>& out) {
    out.resize(ENCODED_BITS);
    for (size_t p = 0; p < ENCODED_BITS; ++p) {
        size_t row = p / 32;
        size_t col = p % 32;
        size_t src = col * 67 + row;
        out[p] = (src < in.size()) ? in[src] : 0;
    }
}

// Viterbi decoder matched to HDL encoder
class ViterbiDecoder {
public:
    static constexpr int NUM_STATES = 64;
    static constexpr uint8_t G1 = 0x79;
    static constexpr uint8_t G2 = 0x5B;
    
    ViterbiDecoder() {
        for (int sr = 0; sr < NUM_STATES; ++sr) {
            for (int inp = 0; inp < 2; ++inp) {
                uint8_t state = (inp << 6) | sr;
                uint8_t g1 = __builtin_parity(state & G1);
                uint8_t g2 = __builtin_parity(state & G2);
                branch_output_[sr][inp] = (g1 << 1) | g2;
                // MUST match encoder: sr = ((sr << 1) | in) & 0x3F
                next_state_[sr][inp] = ((sr << 1) | inp) & 0x3F;
            }
        }
    }
    
    std::vector<uint8_t> decode(const std::vector<std::pair<uint8_t, uint8_t>>& symbols) {
        const size_t n = symbols.size();
        if (n == 0) return {};
        
        constexpr int INF = 100000;
        std::array<int, NUM_STATES> pm, npm;
        std::vector<std::array<uint8_t, NUM_STATES>> surv(n);
        
        pm.fill(INF);
        pm[0] = 0;
        
        for (size_t t = 0; t < n; ++t) {
            int rx = (symbols[t].first << 1) | symbols[t].second;
            npm.fill(INF);
            
            for (int s = 0; s < NUM_STATES; ++s) {
                if (pm[s] >= INF) continue;
                for (int inp = 0; inp < 2; ++inp) {
                    int ns = next_state_[s][inp];
                    int bm = __builtin_popcount(rx ^ branch_output_[s][inp]);
                    int newpm = pm[s] + bm;
                    if (newpm < npm[ns]) {
                        npm[ns] = newpm;
                        surv[t][ns] = s;
                    }
                }
            }
            std::swap(pm, npm);
        }
        
        int best_s = 0;
        for (int s = 1; s < NUM_STATES; ++s) {
            if (pm[s] < pm[best_s]) best_s = s;
        }
        
        std::vector<uint8_t> out(n);
        int s = best_s;
        for (int t = n - 1; t >= 0; --t) {
            int ps = surv[t][s];
            out[t] = (next_state_[ps][1] == s) ? 1 : 0;
            s = ps;
        }
        return out;
    }
    
private:
    uint8_t branch_output_[NUM_STATES][2];
    uint8_t next_state_[NUM_STATES][2];
};

void derandomize(std::vector<uint8_t>& data) {
    uint8_t lfsr = 0xFF;
    for (size_t i = 0; i < data.size(); ++i) {
        uint8_t rand_byte = 0;
        for (int b = 7; b >= 0; --b) {
            rand_byte |= ((lfsr >> 7) & 1) << b;
            uint8_t fb = ((lfsr >> 7) ^ (lfsr >> 6) ^ (lfsr >> 4) ^ (lfsr >> 2)) & 1;
            lfsr = (lfsr << 1) | fb;
        }
        data[i] ^= rand_byte;
    }
}

std::vector<uint8_t> decode_frame(const std::vector<uint8_t>& payload_bits) {
    std::vector<uint8_t> deinterleaved;
    deinterleave(payload_bits, deinterleaved);
    
    std::vector<std::pair<uint8_t, uint8_t>> symbols;
    for (size_t i = 0; i < deinterleaved.size(); i += 2) {
        symbols.push_back({deinterleaved[i], deinterleaved[i+1]});
    }
    
    ViterbiDecoder vit;
    auto decoded_bits = vit.decode(symbols);
    
    std::vector<uint8_t> decoded_randomized(FRAME_BYTES, 0);
    for (int byte_idx = FRAME_BYTES - 1; byte_idx >= 0; --byte_idx) {
        uint8_t byte_val = 0;
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
            size_t bit_idx = (FRAME_BYTES - 1 - byte_idx) * 8 + (7 - bit_pos);
            if (bit_idx < decoded_bits.size()) {
                byte_val |= (decoded_bits[bit_idx] & 1) << bit_pos;
            }
        }
        decoded_randomized[byte_idx] = byte_val;
    }
    
    derandomize(decoded_randomized);
    return decoded_randomized;
}

void print_frame(int frame_num, const std::vector<uint8_t>& frame) {
    std::cout << "┌─────────────────────────────────────────────────────────\n";
    std::cout << "│ FRAME " << frame_num << "\n";
    std::cout << "├─────────────────────────────────────────────────────────\n";
    
    // Callsign (bytes 0-5)
    std::cout << "│ Callsign: ";
    bool valid_call = true;
    for (int i = 0; i < 6; ++i) {
        if (frame[i] != 0 && (frame[i] < 0x20 || frame[i] > 0x7E)) {
            valid_call = false;
            break;
        }
    }
    if (valid_call) {
        for (int i = 0; i < 6 && frame[i] != 0; ++i) {
            std::cout << (char)frame[i];
        }
        std::cout << "\n";
    } else {
        std::cout << "(invalid)\n";
    }
    
    // AAAAA Token (bytes 6-8)
    uint32_t aaaaa_token = (frame[6] << 16) | (frame[7] << 8) | frame[8];
    std::cout << "|    Token: " <<aaaaa_token<< "\n";
    
    // Hex dump
    std::cout << "│\n│ Hex dump:\n";
    for (size_t i = 0; i < frame.size(); i += 16) {
        std::cout << "│   " << std::hex << std::setfill('0') << std::setw(3) << i << ": ";
        
        // Hex
        for (size_t j = i; j < i + 16 && j < frame.size(); ++j) {
            std::cout << std::setw(2) << (int)frame[j] << " ";
        }
        
        // Padding if short line
        for (size_t j = frame.size(); j < i + 16; ++j) {
            std::cout << "   ";
        }
        
        // ASCII
        std::cout << " │";
        for (size_t j = i; j < i + 16 && j < frame.size(); ++j) {
            char c = (frame[j] >= 0x20 && frame[j] < 0x7F) ? frame[j] : '.';
            std::cout << c;
        }
        std::cout << "│\n";
    }
    std::cout << std::dec;
    
    std::cout << "└─────────────────────────────────────────────────────────\n\n";
}

int main() {
    std::cerr << "OPV Demodulator v1.0\n";
    std::cerr << "Waiting for IQ data on stdin...\n\n";
    
    double phase_f1 = 0.0, phase_f2 = 0.0;
    double f1_accum = 0.0, f2_accum = 0.0;
    size_t sample_count = 0;
    int prev_encoded = 0;
    int cclk = 0;
    
    uint32_t shift_reg = 0;
    std::vector<uint8_t> payload_bits;
    bool collecting = false;
    
    size_t total_samples = 0;
    size_t total_bits = 0;
    int syncs_found = 0;
    int frames_decoded = 0;
    
    IQSample s;
    while (std::cin.read(reinterpret_cast<char*>(&s), sizeof(s))) {
        total_samples++;
        
        double I = s.I / 32768.0;
        double Q = s.Q / 32768.0;
        
        f1_accum += I * std::sin(phase_f1) + Q * std::cos(phase_f1);
        f2_accum += I * std::sin(phase_f2) + Q * std::cos(phase_f2);
        
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
            
            int enc = (data_sum < 0) ? 1 : 0;
            int decoded_bit = enc ^ prev_encoded;
            prev_encoded = enc;
            
            if (collecting) {
                payload_bits.push_back(decoded_bit);
                
                if (payload_bits.size() == ENCODED_BITS) {
                    auto frame = decode_frame(payload_bits);
                    syncs_found++;
                    
                    // Check if valid
                    bool valid = true;
                    for (int i = 0; i < 6; ++i) {
                        if (frame[i] != 0 && (frame[i] < 0x20 || frame[i] > 0x7E)) {
                            valid = false;
                            break;
                        }
                    }
                    
                    if (valid) {
                        frames_decoded++;
                        print_frame(frames_decoded, frame);
                    } else {
                        std::cerr << "Frame " << syncs_found << ": decode failed (invalid callsign)\n";
                    }
                    
                    collecting = false;
                    payload_bits.clear();
                }
            } else {
                shift_reg = ((shift_reg << 1) | decoded_bit) & 0xFFFFFF;
                if (shift_reg == SYNC_WORD) {
                    std::cerr << "SYNC detected at bit " << total_bits << "\n";
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
    
    std::cerr << "\n────────────────────────────────────────────────────────────\n";
    std::cerr << "Summary: " << total_samples << " samples, " << total_bits << " bits\n";
    std::cerr << "         " << syncs_found << " sync(s), " << frames_decoded << " frame(s) decoded\n";
    std::cerr << "────────────────────────────────────────────────────────────\n";
    
    return (frames_decoded > 0) ? 0 : 1;
}

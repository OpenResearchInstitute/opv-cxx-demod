/**
 * opv-sync-test.cpp - Send sync words with differential encoding
 */

#include <iostream>
#include <cstdint>
#include <cmath>
#include <array>
#include <getopt.h>

constexpr uint32_t SYNC_WORD = 0x02B8DB;
constexpr size_t SAMPLES_PER_SYMBOL = 40;
constexpr double SAMPLE_RATE = 2168000.0;
constexpr double SYMBOL_RATE = 54200.0;
constexpr double FREQ_DEV = SYMBOL_RATE / 4.0;
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

struct IQSample { int16_t I, Q; };

class DifferentialMSK {
public:
    void reset() {
        phase = 0.0;
        prev_encoded = 0;
    }
    
    void modulate_bit(uint8_t tx_bit, std::array<IQSample, SAMPLES_PER_SYMBOL>& output) {
        uint8_t encoded = tx_bit ^ prev_encoded;
        prev_encoded = encoded;
        
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

DifferentialMSK g_mod;

void output(const std::array<IQSample, SAMPLES_PER_SYMBOL>& s) {
    for (const auto& x : s) {
        std::cout.write(reinterpret_cast<const char*>(&x.I), 2);
        std::cout.write(reinterpret_cast<const char*>(&x.Q), 2);
    }
}

void send_sync_word() {
    std::array<IQSample, SAMPLES_PER_SYMBOL> samples;
    for (int i = 23; i >= 0; --i) {
        g_mod.modulate_bit((SYNC_WORD >> i) & 1, samples);
        output(samples);
    }
}

void send_dummy_payload() {
    std::array<IQSample, SAMPLES_PER_SYMBOL> samples;
    for (int i = 0; i < 2144; ++i) {
        g_mod.modulate_bit(0, samples);
        output(samples);
    }
}

int main(int argc, char* argv[]) {
    int count = 100;
    bool with_payload = true;
    
    int opt;
    while ((opt = getopt(argc, argv, "c:nh")) != -1) {
        switch (opt) {
            case 'c': count = std::atoi(optarg); break;
            case 'n': with_payload = false; break;
            default:
                std::cerr << "Usage: " << argv[0] << " [-c COUNT] [-n]\n";
                return 1;
        }
    }
    
    std::cerr << "Sending " << count << " sync words (differential MSK)";
    if (with_payload) std::cerr << " with dummy payload";
    std::cerr << "...\n";
    
    g_mod.reset();
    
    for (int i = 0; i < count; ++i) {
        send_sync_word();
        if (with_payload) send_dummy_payload();
    }
    
    std::array<IQSample, SAMPLES_PER_SYMBOL> zeros = {};
    for (int i = 0; i < 100; ++i) output(zeros);
    
    std::cerr << "Done.\n";
    return 0;
}

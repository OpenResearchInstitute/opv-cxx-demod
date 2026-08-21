#include "opv_demod.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace {

std::array<uint8_t, ENCODED_BITS> make_valid_codeword() {
    std::array<uint8_t, FRAME_BITS> bits{};
    uint32_t state = 0x6d2b79f5u;
    for (uint8_t& bit : bits) {
        state = state * 1664525u + 1013904223u;
        bit = static_cast<uint8_t>(state >> 31);
    }

    auto advance = [](uint8_t& shift, uint8_t bit, uint8_t& g1, uint8_t& g2) {
        const uint8_t full = static_cast<uint8_t>((bit << 6) | shift);
        g1 = static_cast<uint8_t>(__builtin_parity(full & G1_MASK));
        g2 = static_cast<uint8_t>(__builtin_parity(full & G2_MASK));
        shift = static_cast<uint8_t>(((shift << 1) | bit) & 0x3f);
    };

    uint8_t shift = 0;
    uint8_t unused_g1 = 0, unused_g2 = 0;
    for (uint8_t bit : bits) advance(shift, bit, unused_g1, unused_g2);

    std::array<uint8_t, ENCODED_BITS> convolutional{};
    size_t index = 0;
    for (uint8_t bit : bits) {
        uint8_t g1 = 0, g2 = 0;
        advance(shift, bit, g1, g2);
        convolutional[index++] = g1;
        convolutional[index++] = g2;
    }

    std::array<uint8_t, ENCODED_BITS> on_air{};
    for (size_t i = 0; i < ENCODED_BITS; ++i)
        on_air[deinterleave_addr(i)] = convolutional[i];
    return on_air;
}

void put_sync(std::vector<double>& stream, size_t pos, double amplitude) {
    for (size_t i = 0; i < SYNC_BITS; ++i) {
        const bool bit = ((SYNC_WORD >> (SYNC_BITS - 1 - i)) & 1u) != 0;
        stream[pos + i] = bit ? -amplitude : amplitude;
    }
}

double sync_correlation(const std::vector<double>& stream, size_t pos) {
    double sum = 0.0;
    double energy = 1e-9;
    for (size_t i = 0; i < SYNC_BITS; ++i) {
        const bool bit = ((SYNC_WORD >> (SYNC_BITS - 1 - i)) & 1u) != 0;
        const double expected = bit ? -1.0 : 1.0;
        sum += expected * stream[pos + i];
        energy += std::fabs(stream[pos + i]);
    }
    return std::fabs(sum / energy);
}

bool same_stream(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    if (lhs.size() != rhs.size()) return false;
    for (size_t i = 0; i < lhs.size(); ++i)
        if (std::fabs(lhs[i] - rhs[i]) > 1e-12) return false;
    return true;
}

}  // namespace

int main() {
    constexpr size_t correct_sync_pos = 7;
    constexpr size_t wrong_sync_pos = 6;
    const size_t stream_size = correct_sync_pos + SYNC_BITS + ENCODED_BITS;

    std::vector<double> correct(stream_size, 0.25);
    std::vector<double> wrong(stream_size, 0.25);
    put_sync(correct, correct_sync_pos, 1.0);
    put_sync(wrong, wrong_sync_pos, 2.0);

    const auto codeword = make_valid_codeword();
    for (size_t i = 0; i < ENCODED_BITS; ++i)
        correct[correct_sync_pos + SYNC_BITS + i] = codeword[i] ? -1.0 : 1.0;

    uint32_t noise = 0x94d049bbu;
    for (size_t i = 0; i < ENCODED_BITS; ++i) {
        noise = noise * 1103515245u + 12345u;
        wrong[wrong_sync_pos + SYNC_BITS + i] = (noise >> 31) ? -1.0 : 1.0;
    }

    // The wrong sync has twice the energy, so the old correlation-only rule
    // rated its nominally perfect peak a few ULPs higher. Decode verification
    // must still select the valid parity stream.
    if (!(sync_correlation(wrong, wrong_sync_pos) >
          sync_correlation(correct, correct_sync_pos))) {
        std::fprintf(stderr, "test fixture does not reproduce the correlation tie-break\n");
        return 1;
    }
    const std::vector<double> resolved = SyncTracker::resolve(correct, wrong);
    if (!same_stream(resolved, correct)) {
        std::fprintf(stderr, "decode-verified resolver selected the wrong parity\n");
        return 1;
    }

    // A short capture cannot contain a frame. It must retain the legacy
    // correlation-only behavior, including polarity correction.
    std::vector<double> short_dec0(SYNC_BITS, 0.1);
    std::vector<double> short_dec1(SYNC_BITS, 0.1);
    put_sync(short_dec1, 0, -3.0);
    const std::vector<double> short_resolved = SyncTracker::resolve(short_dec0, short_dec1);
    for (size_t i = 0; i < SYNC_BITS; ++i) {
        const bool bit = ((SYNC_WORD >> (SYNC_BITS - 1 - i)) & 1u) != 0;
        const double expected = bit ? -3.0 : 3.0;
        if (std::fabs(short_resolved[i] - expected) > 1e-12) {
            std::fprintf(stderr, "short-capture fallback changed behavior\n");
            return 1;
        }
    }

    std::puts("PASS: decode-verified sync resolution");
    return 0;
}

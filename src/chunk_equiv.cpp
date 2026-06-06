// chunk_equiv.cpp — validate demodulate_stream (chunked, stateful) against the
// proven batch track_correlations+combine on a real channel-rate signal.
// Usage:  opv-mod -P -B 5 | opv-resample 2168000 625000 | ./chunk_equiv 625000 <chunk>
#include "opv_demod.hpp"
#include <cstdio>
#include <vector>
#include <cmath>
#include <cstdlib>


int main(int argc, char** argv) {
    double chan_rate = (argc > 1) ? atof(argv[1]) : 625000.0;
    size_t chunk     = (argc > 2) ? (size_t)atol(argv[2]) : 4096;

    // read int16 I/Q from stdin
    std::vector<sample_t> sig;
    int16_t iq[2];
    while (fread(iq, sizeof(int16_t), 2, stdin) == 2)
        sig.emplace_back((double)iq[0] / 32767.0, (double)iq[1] / 32767.0);
    fprintf(stderr, "read %zu samples (%.2f sps nominal)\n",
            sig.size(), chan_rate / SYMBOL_RATE);

    // ---- batch path (proven) ----
    CoherentMSKDemodulator db;
    db.set_nominal_sps(chan_rate / SYMBOL_RATE);
    std::vector<std::complex<double>> Y1, Y2;
    db.track_correlations(sig.data(), sig.size(), Y1, Y2);
    std::vector<double> d0b, d1b;
    db.combine(Y1, Y2, d0b, d1b);

    // ---- streaming path (chunked, leftover-buffered) ----
    CoherentMSKDemodulator ds;
    ds.set_nominal_sps(chan_rate / SYMBOL_RATE);
    ds.reset_stream();
    std::vector<double> d0s, d1s;
    std::vector<std::complex<double>> Y1s, Y2s;
    std::vector<sample_t> leftover;
    size_t off = 0;
    while (off < sig.size()) {
        size_t take = std::min(chunk, sig.size() - off);
        std::vector<sample_t> buf;
        buf.reserve(leftover.size() + take);
        buf.insert(buf.end(), leftover.begin(), leftover.end());
        buf.insert(buf.end(), sig.begin() + off, sig.begin() + off + take);
        off += take;
        size_t consumed = 0;
        ds.demodulate_stream(buf.data(), buf.size(), d0s, d1s, consumed, &Y1s, &Y2s);
        if (consumed > buf.size()) consumed = buf.size();
        leftover.assign(buf.begin() + consumed, buf.end());
    }

    // Y front-end equivalence (timing): does streaming corr == batch corr?
    {
        size_t NY = std::min(Y1.size(), Y1s.size());
        double sb = 0, ss = 0, dot = 0; size_t firstdiv = NY;
        for (size_t i = 0; i < NY; ++i) {
            double mb = std::abs(Y1[i]), ms = std::abs(Y1s[i]);
            sb += mb*mb; ss += ms*ms; dot += mb*ms;
            if (firstdiv == NY && std::fabs(mb - ms) > 0.02 * (mb + 1e-9)) firstdiv = i;
        }
        fprintf(stderr, "  |Y1| front-end NCC=%.6f  first |Y1| diverge@sym %zu/%zu\n",
                dot / (std::sqrt(sb*ss) + 1e-12), firstdiv, NY);
    }

    // ---- compare on the overlap ----
    size_t N = std::min(d0b.size(), d0s.size());
    fprintf(stderr, "batch nsym=%zu  stream nsym=%zu  compare N=%zu\n",
            d0b.size(), d0s.size(), N);

    auto report = [&](const char* nm, const std::vector<double>& B,
                                       const std::vector<double>& S) {
        double sb = 0, ss = 0, dot = 0; size_t sign_ok = 0, big = 0;
        for (size_t i = 0; i < N; ++i) {
            sb += B[i]*B[i]; ss += S[i]*S[i]; dot += B[i]*S[i];
            if ((B[i] < 0) == (S[i] < 0)) sign_ok++;
            if (std::fabs(B[i] - S[i]) > 0.05 * (std::fabs(B[i]) + 1e-9)) big++;
        }
        double ncc = dot / (std::sqrt(sb*ss) + 1e-12);
        fprintf(stderr, "  %s: NCC=%.6f  sign-match=%.4f  (>5%% rel diff: %zu/%zu)\n",
                nm, ncc, (double)sign_ok / N, big, N);
    };
    report("dec0", d0b, d0s);
    report("dec1", d1b, d1s);

    // localize first divergences on the clean stream (dec1 here)
    double sps = chan_rate / SYMBOL_RATE;
    int shown = 0;
    for (size_t i = 0; i < N && shown < 8; ++i) {
        if (std::fabs(d1b[i] - d1s[i]) > 0.1 * (std::fabs(d1b[i]) + 1e-9)) {
            double approx_sym_in_chunk = std::fmod((double)i, (double)chunk / sps);
            fprintf(stderr, "  diverge@sym %zu: batch=%+.4f stream=%+.4f  (%.1f sym into a chunk of ~%.0f)\n",
                    i, d1b[i], d1s[i], approx_sym_in_chunk, (double)chunk / sps);
            shown++;
        }
    }
    return 0;
}

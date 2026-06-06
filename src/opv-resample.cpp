// opv-resample.cpp — fractional-rate resampler for OPV I/Q.
//
// Reads 16-bit interleaved little-endian I/Q from stdin at Fin, writes the same
// format at Fout. Windowed-sinc (Blackman) interpolation with the anti-alias
// cutoff at min(Fin,Fout)/2, kernel normalized to unity DC gain. Default
// 2168000 -> 625000 (OPV native -> one 625 ksps FunCube+ channel; ratio 2168/625,
// the prime-271 factor that makes a clean integer SPS impossible — handled here
// as an arbitrary ratio).
//
// Dual use: (1) channel-rate loopback source for the coherent fractional demod,
// (2) SIC reference — reconstruct an OPV signal at high rate, bring it to the
// channel rate to subtract.
//
// Usage:  opv-resample [Fin Fout]   (defaults 2168000 625000)
//   opv-mod -S W5NYV -P -B 10 | opv-resample 2168000 625000 | opv-demod -c -R 625000
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <complex>
#include <algorithm>

struct IQ16 { int16_t I, Q; };
static const double PI = 3.14159265358979323846;

static inline double sinc(double x) { return (std::fabs(x) < 1e-12) ? 1.0 : std::sin(PI*x)/(PI*x); }

int main(int argc, char** argv) {
    double Fin  = (argc > 1) ? atof(argv[1]) : 2168000.0;
    double Fout = (argc > 2) ? atof(argv[2]) : 625000.0;
    const int K = (argc > 3) ? atoi(argv[3]) : 32;
    const double fc = std::min(1.0, Fout/Fin);   // normalized anti-alias cutoff (input-rate)

    // Read all input I/Q
    std::vector<std::complex<double>> x;
    x.reserve(1u << 20);
    IQ16 s;
    while (std::fread(&s, sizeof(s), 1, stdin) == 1)
        x.push_back(std::complex<double>((double)s.I, (double)s.Q));
    if (x.size() < (size_t)(2*K + 2)) { fprintf(stderr, "opv-resample: too few samples\n"); return 1; }

    const double step = Fin / Fout;          // input samples advanced per output sample
    size_t Nout = (size_t)((double)(x.size() - 2*K - 1) / step);
    fprintf(stderr, "opv-resample: %zu in @ %.0f -> %zu out @ %.0f  (ratio %.4f, fc=%.4f, %d taps)\n",
            x.size(), Fin, Nout, Fout, Fout/Fin, fc, 2*K);

    for (size_t m = 0; m < Nout; ++m) {
        double t = (double)K + (double)m * step;   // input position (offset by K for kernel margin)
        long c = (long)std::floor(t);
        std::complex<double> acc(0,0);
        double wsum = 0.0;
        for (long k = c - K + 1; k <= c + K; ++k) {
            if (k < 0 || k >= (long)x.size()) continue;
            double d = t - (double)k;
            double win = 0.42 - 0.5*std::cos(2*PI*(d + K)/(2*K)) + 0.08*std::cos(4*PI*(d + K)/(2*K)); // Blackman
            double w = fc * sinc(fc*d) * win;
            acc += x[(size_t)k] * w;
            wsum += w;
        }
        if (wsum != 0.0) acc /= wsum;          // unity DC gain
        IQ16 o;
        o.I = (int16_t)std::clamp((long)std::lround(acc.real()), -32768L, 32767L);
        o.Q = (int16_t)std::clamp((long)std::lround(acc.imag()), -32768L, 32767L);
        std::fwrite(&o, sizeof(o), 1, stdout);
    }
    return 0;
}

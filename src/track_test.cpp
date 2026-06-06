// Validation harness for the fractional-timing coherent front-end.
// Ports frac2.py to C++: synthesize OPV at the channelized rate (~11.53 sps)
// with an initial timing offset + slow symbol-clock drift (perfect carrier),
// run CoherentMSKDemodulator::track_correlations(), and score raw pre-FEC BER
// against a known-timing baseline. Confirms the loop holds the coherent gain.
#include "opv_demod.hpp"
#include <random>
#include <cstdio>
#include <vector>
#include <complex>
#include <cmath>

// OPV parallel-tone precoder: per-symbol tone selections ds1,ds2.
static void precode(const std::vector<int>& bits,
                    std::vector<int>& D1, std::vector<int>& D2) {
    int dx = 1, bn = 1;
    for (int b : bits) {
        int dv  = (b == 0) ? 1 : -1;
        int dpe = (dv + 1) >> 1;
        int dne = (bn == 0) ? ((dv - 1) >> 1) : -((dv - 1) >> 1);
        int ds1 = (dpe==1 && dx==1)? 1 : (dpe==1 && dx==-1)? -1 : 0;
        int ds2 = (dne==-1&&dx==1)?-1:(dne==-1&&dx==-1)?1:(dne==1&&dx==1)?1:(dne==1&&dx==-1)?-1:0;
        int dxn = (dv==1&&dx==1)?1:(dv==1&&dx==-1)?-1:(dv==-1&&dx==1)?-1:1;
        D1.push_back(ds1); D2.push_back(ds2); dx = dxn; bn = 1 - bn;
    }
}

// frac2-style hard-decision scorer: best over parity, polarity, integer shift.
static double best_ber(const std::vector<std::complex<double>>& Y1,
                       const std::vector<std::complex<double>>& Y2,
                       const std::vector<int>& src) {
    size_t N = Y1.size();
    std::vector<double> X(N), Yv(N);
    for (size_t i = 0; i < N; ++i) { X[i] = Y1[i].imag(); Yv[i] = Y2[i].imag(); }
    double best = 1.0;
    for (int parity = 0; parity < 2; ++parity) {
        std::vector<int> enc(N, 0), dec(N, 0);
        for (size_t i = 0; i + 1 < N; ++i) {
            double A = X[i] + X[i+1], B = Yv[i] + Yv[i+1];
            double sgn = (((i + parity) & 1) == 0) ? 1.0 : -1.0;
            enc[i] = ((A - sgn * B) < 0) ? 1 : 0;
        }
        for (int inv = 0; inv < 2; ++inv) {
            dec[0] = 0;
            for (size_t i = 1; i < N; ++i) dec[i] = (enc[i] ^ enc[i-1]) ^ inv;
            for (int sh = -4; sh <= 4; ++sh) {
                size_t ds = (sh > 0) ? (size_t)sh : 0;
                size_t ss = (sh < 0) ? (size_t)(-sh) : 0;
                size_t L  = std::min(N - ds, src.size() - ss);
                if (L < 2000) continue;
                size_t errs = 0, tot = 0;
                for (size_t i = 600; i + 50 < L; ++i) { tot++; if (dec[ds+i] != src[ss+i]) errs++; }
                if (tot) best = std::min(best, (double)errs / (double)tot);
            }
        }
    }
    return best;
}

// Score a single dec-domain soft stream (try polarity + integer shift).
static double ber_dec(const std::vector<double>& dec, const std::vector<int>& src) {
    size_t N = dec.size(); double best = 1.0;
    for (int pol = 0; pol < 2; ++pol)
        for (int sh = -4; sh <= 4; ++sh) {
            size_t ds = (sh>0)?(size_t)sh:0, ss = (sh<0)?(size_t)(-sh):0;
            size_t L = std::min(N - ds, src.size() - ss);
            if (L < 2000) continue;
            size_t errs = 0, tot = 0;
            for (size_t i = 600; i + 50 < L; ++i) {
                double v = pol ? -dec[ds+i] : dec[ds+i];
                int b = (v < 0) ? 1 : 0;
                tot++; if (b != src[ss+i]) errs++;
            }
            if (tot) best = std::min(best, (double)errs / (double)tot);
        }
    return best;
}

int main(int argc, char** argv) {
    const double sps   = (argc > 1) ? atof(argv[1]) : 625000.0 / SYMBOL_RATE; // ~11.5314
    const double DEV   = FREQ_DEV;
    const double Fs    = sps * SYMBOL_RATE;
    const int    Nsym  = 42000;
    const double off0  = 0.37;     // initial timing offset, symbols
    printf("sps_nom = %.4f  (Fs = %.1f Hz)\n", sps, Fs);

    std::mt19937 rng(7);
    std::uniform_int_distribution<int> bit(0, 1);
    std::vector<int> src(Nsym);
    for (int i = 0; i < Nsym; ++i) src[i] = bit(rng);
    std::vector<int> D1, D2; precode(src, D1, D2);

    auto run = [&](double ebn0, double ppm, unsigned seed, bool loop,
                   std::vector<std::complex<double>>& Y1,
                   std::vector<std::complex<double>>& Y2,
                   std::vector<double>* mu_true_out) {
        size_t Ntot = (size_t)(Nsym * sps) - (size_t)(2 * sps);
        std::vector<sample_t> sig(Ntot);
        std::vector<double> symphase(Ntot);
        for (size_t n = 0; n < Ntot; ++n) {
            double t = (double)n / Fs;
            double sp = off0 + SYMBOL_RATE * t * (1.0 + 0.5 * ppm * ((double)n / Ntot));
            symphase[n] = sp;
            int k = (int)std::floor(sp); if (k < 0) k = 0; if (k >= Nsym) k = Nsym - 1;
            double a1 = TWO_PI * (-DEV) * t, a2 = TWO_PI * (+DEV) * t;
            sig[n] = (double)D1[k] * sample_t(std::sin(a1), std::cos(a1))
                   + (double)D2[k] * sample_t(std::sin(a2), std::cos(a2));
        }
        // AWGN at the requested Eb/N0
        std::mt19937 ng(seed);
        double v = sps / std::pow(10.0, ebn0 / 10.0);
        std::normal_distribution<double> nd(0.0, std::sqrt(v / 2.0));
        for (size_t n = 0; n < Ntot; ++n) sig[n] += sample_t(nd(ng), nd(ng));

        CoherentMSKDemodulator d;
        d.set_nominal_sps(sps);
        if (loop) {
            d.track_correlations(sig.data(), Ntot, Y1, Y2);
        } else {
            // baseline: matched filter at KNOWN fractional symbol positions
            std::vector<double> mt(Nsym);
            size_t n = 0;
            for (int k = 0; k < Nsym; ++k) {
                while (n + 1 < Ntot && symphase[n] < k) n++;
                mt[k] = (n == 0) ? 0.0
                      : (double)(n-1) + (k - symphase[n-1]) / (symphase[n] - symphase[n-1] + 1e-12);
            }
            Y1.clear(); Y2.clear();
            for (int k = 0; k < Nsym; ++k) {
                double b = mt[k];
                if (b + sps + 2 >= (double)Ntot - 3) break;
                if (b < 1) b = 1;
                std::complex<double> y1, y2; d.corr_at(sig.data(), Ntot, b, y1, y2);
                Y1.push_back(y1); Y2.push_back(y2);
            }
            if (mu_true_out) *mu_true_out = mt;
        }
    };

    printf("\n-- baseline: MF at KNOWN timing --\n");
    for (double e : {6.0, 8.0, 10.0}) {
        std::vector<std::complex<double>> Y1, Y2;
        run(e, 25e-6, 100 + (unsigned)e, false, Y1, Y2, nullptr);
        printf("  EbN0=%.0f : baseline BER = %.3e\n", e, best_ber(Y1, Y2, src));
    }

    struct G { double a, b; };
    std::vector<G> gains = {{0.06,0.0025},{0.04,0.0015},{0.10,0.004},{0.02,0.0008}};
    printf("\n-- fractional PI loop: 0.37-sym offset + 25 ppm drift, cubic interp --\n");
    printf("   (per-config: BER over seeds 201,202,203,204 at EbN0=8)\n");
    for (auto g : gains) {
        printf("  a=%.3f b=%.4f : ", g.a, g.b);
        for (unsigned sd : {201u,202u,203u,204u}) {
            size_t Ntot = (size_t)(Nsym * sps) - (size_t)(2 * sps);
            std::vector<sample_t> sig(Ntot); std::vector<double> sph(Ntot);
            for (size_t n = 0; n < Ntot; ++n) {
                double t=(double)n/Fs; double sp=off0+SYMBOL_RATE*t*(1.0+0.5*25e-6*((double)n/Ntot));
                sph[n]=sp; int k=(int)std::floor(sp); if(k<0)k=0; if(k>=Nsym)k=Nsym-1;
                double a1=TWO_PI*(-DEV)*t,a2=TWO_PI*(+DEV)*t;
                sig[n]=(double)D1[k]*sample_t(std::sin(a1),std::cos(a1))+(double)D2[k]*sample_t(std::sin(a2),std::cos(a2));
            }
            std::mt19937 ng(sd); double v=sps/std::pow(10.0,8.0/10.0);
            std::normal_distribution<double> nd(0.0,std::sqrt(v/2.0));
            for (size_t n=0;n<Ntot;++n) sig[n]+=sample_t(nd(ng),nd(ng));
            CoherentMSKDemodulator d; d.set_nominal_sps(sps); d.set_timing_gains(g.a,g.b);
            std::vector<std::complex<double>> Y1,Y2; d.track_correlations(sig.data(),Ntot,Y1,Y2);
            printf("%.2e  ", best_ber(Y1,Y2,src));
        }
        printf("\n");
    }
    // ---- validate the relocated contract: demod emits BOTH parity streams,
    //      one decodes cleanly (the sync correlator would pick it), the other
    //      is scrambled. No sync word inside the demod. ----
    printf("\n-- combine(): demod emits both parity streams (seed 201, EbN0=8) --\n");
    {
        size_t Ntot = (size_t)(Nsym * sps) - (size_t)(2 * sps);
        std::vector<sample_t> sig(Ntot);
        for (size_t n = 0; n < Ntot; ++n) {
            double t=(double)n/Fs; double sp=off0+SYMBOL_RATE*t*(1.0+0.5*25e-6*((double)n/Ntot));
            int k=(int)std::floor(sp); if(k<0)k=0; if(k>=Nsym)k=Nsym-1;
            double a1=TWO_PI*(-DEV)*t,a2=TWO_PI*(+DEV)*t;
            sig[n]=(double)D1[k]*sample_t(std::sin(a1),std::cos(a1))+(double)D2[k]*sample_t(std::sin(a2),std::cos(a2));
        }
        std::mt19937 ng(201); double v=sps/std::pow(10.0,8.0/10.0);
        std::normal_distribution<double> nd(0.0,std::sqrt(v/2.0));
        for (size_t n=0;n<Ntot;++n) sig[n]+=sample_t(nd(ng),nd(ng));
        CoherentMSKDemodulator d; d.set_nominal_sps(sps);
        std::vector<std::complex<double>> Y1,Y2; d.track_correlations(sig.data(),Ntot,Y1,Y2);
        std::vector<double> dec0, dec1; d.combine(Y1, Y2, dec0, dec1);
        printf("  parity-0 stream BER = %.3e\n  parity-1 stream BER = %.3e   (one clean, one scrambled)\n",
               ber_dec(dec0, src), ber_dec(dec1, src));
    }
    return 0;
}

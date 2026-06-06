//------------------------------------------------------------------------------
// opv-demod.cpp - OPV MSK Demodulator v2.0
//------------------------------------------------------------------------------
// MSK demodulator with automatic frequency control, symbol timing recovery
// (early-late gate TED with 2nd order loop), symbol lock detection, and
// proper two-sync frame acquisition matching the HDL implementation.
//
// Signal parameters:
//   MSK modulation: F1=-13550 Hz (bit '1'), F2=+13550 Hz (bit '0')
//   Symbol rate: 54.2 kbaud (40 samples/symbol at 2.168 MSPS)
//   Sync word: 0x02B8DB (24 bits)
//   Frame: 24-bit sync + 2144 encoded bits = 2168 symbols
//
// Architecture:
//   - Dual-tone correlation with integrate-and-dump
//   - AFC: estimates frequency offset from tone phase rotation
//   - Symbol lock detector: gates frame sync search on TED convergence
//   - State machine: HUNTING → VERIFYING → LOCKED (flywheel)
//     • HUNTING: only searches when symbol timing is locked (TED gate)
//     • VERIFYING: requires TWO consecutive sync hits to declare lock
//     • LOCKED: flywheel tolerates up to 3 missed syncs before release
//
// Author: ORI/Abraxas3d collaboration
// License: CERN-OHL-S v2
//------------------------------------------------------------------------------

#include "opv_demod.hpp"

//------------------------------------------------------------------------------
// Frame Display
//------------------------------------------------------------------------------
void print_frame(int num, const std::array<uint8_t, FRAME_BYTES>& f, int metric, double sync_corr) {
    fprintf(stderr, "┌─────────────────────────────────────────────────────────────────┐\n");
    fprintf(stderr, "│ FRAME %4d  │  Sync: %.3f  │  Metric: %5d", num, sync_corr, metric);
    if (metric == 0) fprintf(stderr, " (perfect)");
    fprintf(stderr, "\n├─────────────────────────────────────────────────────────────────┤\n");
    
    fprintf(stderr, "│ Station ID:  %-12s (Base-40)\n", decode_base40(&f[0]).c_str());
    
    uint32_t tok = (f[6] << 16) | (f[7] << 8) | f[8];
    fprintf(stderr, "│ Token:       0x%06X%s\n", tok, (tok == 0xBBAADD) ? " (default)" : "");
    
    uint32_t res = (f[9] << 16) | (f[10] << 8) | f[11];
    fprintf(stderr, "│ Reserved:    0x%06X\n", res);
    
    fprintf(stderr, "├─────────────────────────────────────────────────────────────────┤\n");
    fprintf(stderr, "│ Hex Dump:                                                       │\n");
    
    for (size_t i = 0; i < FRAME_BYTES; i += 16) {
        fprintf(stderr, "│ %02zx: ", i);
        for (size_t j = i; j < i + 16 && j < FRAME_BYTES; ++j)
            fprintf(stderr, "%02X ", f[j]);
        for (size_t j = FRAME_BYTES; j < i + 16; ++j)
            fprintf(stderr, "   ");
        fprintf(stderr, " │");
        for (size_t j = i; j < i + 16 && j < FRAME_BYTES; ++j) {
            char c = (f[j] >= 0x20 && f[j] < 0x7F) ? f[j] : '.';
            fprintf(stderr, "%c", c);
        }
        fprintf(stderr, "│\n");
    }
    fprintf(stderr, "└─────────────────────────────────────────────────────────────────┘\n\n");
}

//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    bool quiet = false, raw = false, coherent = false, streaming = false;
    double afc_bw = 0.001;
    double pll_bw = 50.0;  // PLL bandwidth in Hz
    double init_offset = 0.0;  // Initial frequency offset for streaming mode
    bool have_init_offset = false;
    double chan_rate = 0.0;    // >0: channelized sample rate -> fractional-timing coherent front-end
    
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-q")) quiet = true;
        else if (!strcmp(argv[i], "-r")) raw = true;
        else if (!strcmp(argv[i], "-c")) coherent = true;
        else if (!strcmp(argv[i], "-s")) streaming = true;
        else if (!strcmp(argv[i], "-R") && i + 1 < argc) chan_rate = atof(argv[++i]);
        else if (!strcmp(argv[i], "-a") && i + 1 < argc) afc_bw = atof(argv[++i]);
        else if (!strcmp(argv[i], "-p") && i + 1 < argc) pll_bw = atof(argv[++i]);
        else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
            init_offset = atof(argv[++i]);
            have_init_offset = true;
        }
        else if (!strcmp(argv[i], "-h")) {
            fprintf(stderr, "Usage: %s [options] < input.iq\n\n", argv[0]);
            fprintf(stderr, "Options:\n");
            fprintf(stderr, "  -q          Quiet mode\n");
            fprintf(stderr, "  -r          Raw output to stdout\n");
            fprintf(stderr, "  -s          Streaming mode (for live PlutoSDR input)\n");
            fprintf(stderr, "  -c          Coherent mode (Costas loop, ~3dB better)\n");
            fprintf(stderr, "  -a <bw>     AFC bandwidth (default: 0.001)\n");
            fprintf(stderr, "  -o <hz>     Initial frequency offset (streaming mode)\n");
            fprintf(stderr, "  -p <hz>     PLL bandwidth in Hz (default: 50, coherent only)\n");
            fprintf(stderr, "  -h          Help\n");
            return 0;
        }
    }
    
    // Set up frame-sized stdout buffer for efficient raw output
    // This ensures each frame is written with a single syscall
    static char stdout_buffer[FRAME_BYTES];
    std::setvbuf(stdout, stdout_buffer, _IOFBF, FRAME_BYTES);
    
    if (!quiet) {
        fprintf(stderr, "╔═══════════════════════════════════════════════════════════════════╗\n");
        if (coherent)
            fprintf(stderr, "║    OPV MSK Demodulator v2.0 — Costas Loop (coherent)              ║\n");
        else if (streaming)
            fprintf(stderr, "║    OPV MSK Demodulator v2.0 — SymLock + 2-Sync (streaming)        ║\n");
        else
            fprintf(stderr, "║    OPV MSK Demodulator v2.0 — SymLock + 2-Sync                    ║\n");
        fprintf(stderr, "╚═══════════════════════════════════════════════════════════════════╝\n\n");
    }
    
    // =========================================================================
    // STREAMING MODE - process data as it arrives
    // =========================================================================
    if (streaming) {
        if (!quiet)
            fprintf(stderr, "Streaming mode: processing data as it arrives...\n\n");

        // One ChannelReceiver = one channel's full receive chain. A multi-channel
        // host design (dogu) holds many of these; the program drives exactly one.
        ChannelReceiver rx;

        // Set initial offset if provided, otherwise start at 0
        if (have_init_offset) {
            rx.set_freq_offset(init_offset);
            if (!quiet)
                fprintf(stderr, "Initial frequency offset: %.1f Hz\n", init_offset);
        }
        rx.set_afc_bandwidth(afc_bw);

        // Process in chunks of one frame worth of samples
        const size_t CHUNK_SAMPLES = FRAME_SYMBOLS * SAMPLES_PER_SYMBOL;  // ~86720 samples
        std::vector<sample_t> chunk_buf;
        chunk_buf.reserve(CHUNK_SAMPLES);

        int decoded = 0, perfect = 0;
        size_t total_samples = 0;
        bool first_chunk = true;

        // Called the instant each frame completes (inline), so diagnostic output
        // stays in the same order it streamed in.
        auto emit = [&](const ChannelReceiver::Frame& f) {
            decoded++;
            if (f.metric == 0) perfect++;
            if (!quiet)
                print_frame(decoded, f.bytes, f.metric, f.sync_quality);
            if (raw) {
                std::cout.write(reinterpret_cast<const char*>(f.bytes.data()), FRAME_BYTES);
                std::cout.flush();
            }
        };

        IQSample iq;
        while (std::cin.read(reinterpret_cast<char*>(&iq), sizeof(iq))) {
            chunk_buf.push_back(sample_t(iq.I, iq.Q));

            // Process when we have a full chunk
            if (chunk_buf.size() >= CHUNK_SAMPLES) {
                total_samples += chunk_buf.size();

                // On first chunk, estimate frequency offset
                if (first_chunk) {
                    if (!have_init_offset) {
                        double est_offset = rx.estimate_offset(chunk_buf.data(), chunk_buf.size());
                        rx.set_freq_offset(est_offset);
                        if (!quiet)
                            fprintf(stderr, "Estimated carrier offset: %.1f Hz\n\n", est_offset);
                    }
                    first_chunk = false;
                }

                // Demodulate + sync-track + decode this chunk (gated path)
                rx.process(chunk_buf.data(), chunk_buf.size(), emit);

                // Keep leftover samples for next chunk (for timing recovery continuity)
                size_t leftover = rx.get_leftover();
                if (leftover > 0 && leftover < chunk_buf.size()) {
                    std::vector<sample_t> keep(chunk_buf.end() - leftover, chunk_buf.end());
                    chunk_buf = std::move(keep);
                } else {
                    chunk_buf.clear();
                }

                // Periodic status update
                if (!quiet && (total_samples % (size_t)(SAMPLE_RATE * 5) < CHUNK_SAMPLES)) {
                    fprintf(stderr, "[%.1fs] %zu symbols, %d frames (%d perfect), AFC: %.1f Hz, TFreq: %.4f, SymLock: %s\n",
                            total_samples / SAMPLE_RATE, rx.total_symbols(), decoded, perfect,
                            rx.get_freq_offset(), rx.get_timing_freq(),
                            rx.symbol_locked() ? "YES" : "no");
                }
            }
        }

        // Symbol count for the summary is the pre-flush value (the original tail
        // flush never updated the running total — preserved here deliberately).
        size_t summary_symbols = rx.total_symbols();

        // Process any remaining samples (un-gated tail flush)
        if (!chunk_buf.empty()) {
            rx.process(chunk_buf.data(), chunk_buf.size(), emit, /*gate=*/false);
        }

        if (!quiet) {
            fprintf(stderr, "\n════════════════════════════════════════════════════════════════════\n");
            fprintf(stderr, "Summary: %d frames (%d perfect, %d errors)\n", decoded, perfect, decoded - perfect);
            fprintf(stderr, "Total: %.3f sec, %zu symbols\n", total_samples / SAMPLE_RATE, summary_symbols);
            fprintf(stderr, "Final state: %s, AFC: %.1f Hz\n",
                    state_name(rx.sync_state()), rx.get_freq_offset());
            fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
        }

        return decoded > 0 ? 0 : 1;
    }
    
    // =========================================================================
    // BATCH MODE - load all samples then process (original behavior)
    // =========================================================================
    
    // Load samples
    std::vector<sample_t> samples;
    IQSample iq;
    while (std::cin.read(reinterpret_cast<char*>(&iq), sizeof(iq)))
        samples.push_back(sample_t(iq.I, iq.Q));
    
    if (!quiet)
        fprintf(stderr, "Loaded %zu samples (%.3f sec)\n", samples.size(), samples.size() / SAMPLE_RATE);
    
    // Demodulate using selected mode
    std::vector<double> soft;
    double final_offset;
    
    if (coherent) {
        // Coherent demodulation with Costas loop
        CoherentMSKDemodulator demod;
        bool ch = (chan_rate > 0.0);
        if (ch) demod.set_nominal_sps(chan_rate / SYMBOL_RATE);

        // de Buda coarse acquisition is scaled to the native 2.168 Msps capture;
        // for a channelized-rate loopback we run at baseband (offset 0).
        double est_offset = ch ? 0.0 : demod.estimate_offset(samples.data(), samples.size());
        demod.set_freq_offset(est_offset);

        if (!quiet) {
            if (ch) fprintf(stderr, "Channel rate: %.0f Hz (%.4f sps), offset assumed 0\n",
                            chan_rate, chan_rate / SYMBOL_RATE);
            else    fprintf(stderr, "Estimated carrier offset: %.1f Hz\n", est_offset);
        }

        demod.set_pll_bandwidth(pll_bw);

        if (!quiet && !ch)
            fprintf(stderr, "PLL bandwidth: %.1f Hz\n", pll_bw);

        // Protocol-agnostic demod: it emits BOTH parity streams; the sync
        // correlator (resolve) picks parity/polarity using the sync word.
        std::vector<std::complex<double>> Y1, Y2;
        if (ch) demod.track_correlations(samples.data(), samples.size(), Y1, Y2);
        else    demod.batch_correlations(samples.data(), samples.size(), Y1, Y2);
        std::vector<double> dec0, dec1;
        demod.combine(Y1, Y2, dec0, dec1);
        soft = SyncTracker::resolve(dec0, dec1);
        final_offset = demod.get_freq_offset();
    } else {
        // Non-coherent energy detection (original)
        MSKDemodulatorAFC demod;
        
        double est_offset = demod.estimate_offset(samples.data(), samples.size());
        demod.set_freq_offset(est_offset);
        
        if (!quiet)
            fprintf(stderr, "Estimated carrier offset: %.1f Hz\n", est_offset);
        
        demod.set_afc_bandwidth(afc_bw);
        demod.demodulate(samples.data(), samples.size(), soft);
        final_offset = demod.get_freq_offset();
    }
    
    if (!quiet)
        fprintf(stderr, "Demodulated %zu symbols, final AFC offset: %.1f Hz\n\n", 
                soft.size(), final_offset);
    
    // Process through state machine
    SyncTracker tracker;
    FrameDecoder fdec;
    int decoded = 0, perfect = 0;
    
    // Batch mode: signal is present in the file, bypass symbol lock gate
    tracker.set_symbol_lock(true);
    
    for (size_t i = 0; i < soft.size(); ++i) {
        auto res = tracker.process(soft[i], i);
        
        if (res.frame_ready && !res.payload.empty()) {
            std::array<uint8_t, FRAME_BYTES> frame;
            int metric = fdec.decode(res.payload.data(), frame);
            
            if (metric >= 0) {
                decoded++;
                if (metric == 0) perfect++;
                
                if (!quiet)
                    print_frame(decoded, frame, metric, res.sync_quality);
                
                if (raw) {
                    std::cout.write(reinterpret_cast<char*>(frame.data()), FRAME_BYTES);
                    std::cout.flush();
                }
            }
        }
    }
    
    if (!quiet) {
        fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
        fprintf(stderr, "Summary: %d frames (%d perfect, %d errors)\n", decoded, perfect, decoded - perfect);
        fprintf(stderr, "Final state: %s, AFC: %.1f Hz\n", 
                state_name(tracker.get_state()), final_offset);
        fprintf(stderr, "════════════════════════════════════════════════════════════════════\n");
    }
    
    return decoded > 0 ? 0 : 1;
}

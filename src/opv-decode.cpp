// opv-decode.cpp — A53 decode-only harness for the fabric-demod split.
//
// In the ZCU102 deployment the PL does channelizer -> MSK demod -> one
// differentially-decoded int16 soft metric per symbol (rx_data_soft), and the
// A53 decodes. This tool is the A53 side: it reads int16 soft metrics and runs
// SingleStreamDecodeReceiver (sync + deinterleave + Viterbi + derandomize). No
// demod, no parity/polarity hypotheses -- the fabric resolves those upstream.
//
// Input formats:
//   default   : raw little-endian int16 soft stream, one channel
//   -m N       : multiplexed records {uint8 channel, int16 soft}, N channels
//                (DecodeBank demux) -- the shape the shared/multiplexed fabric
//                demod will emit; record layout is provisional, see interface note
//
// Usage:  opv-decode [-q] [-m N] [-r] < soft.s16
//   -q  quiet (summary only)   -r  emit decoded frame bytes to stdout

#include "opv_demod.hpp"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>
#include <chrono>

int main(int argc, char** argv) {
    bool quiet = false, raw = false;
    int  nchan = 0;                    // 0 = single-channel raw int16 stream
    bool seamB = false;                // -3: framed 3-bit input (fabric m_axis_soft_bit)
    for (int i = 1; i < argc; ++i) {
        if      (!strcmp(argv[i], "-q")) quiet = true;
        else if (!strcmp(argv[i], "-r")) raw = true;
        else if (!strcmp(argv[i], "-3")) seamB = true;
        else if (!strcmp(argv[i], "-m") && i + 1 < argc) nchan = atoi(argv[++i]);
    }

    int frames = 0, perfect = 0;
    auto on_frame = [&](const SingleStreamDecodeReceiver::Frame& f) {
        ++frames;
        if (f.metric == 0) ++perfect;
        if (!quiet)
            fprintf(stderr, "FRAME %4d  ch %3d  sync %.3f  metric %4d %s\n",
                    frames, f.channel, f.sync_quality, f.metric,
                    f.metric == 0 ? "(perfect)" : "");
        if (raw) std::fwrite(f.bytes.data(), 1, FRAME_BYTES, stdout);
    };

    const size_t BLK = 65536;
    size_t total_syms = 0;
    auto t0 = std::chrono::steady_clock::now();

    if (seamB) {
        // Seam B: fabric already framed + 3-bit-quantized (frame_sync_detector_soft
        // m_axis_soft_bit). Input is back-to-back frames of ENCODED_BITS bytes
        // (0..7); each complete frame -> decode_soft3. No sync, no SyncTracker --
        // the deployment path. (Single channel here; multiplexed adds a per-frame
        // channel tag from TID/TUSER.)
        FrameDecoder fdec;
        std::vector<uint8_t> frame(ENCODED_BITS);
        size_t fill = 0;
        std::vector<uint8_t> buf(BLK);
        std::array<uint8_t, FRAME_BYTES> out;
        while (true) {
            size_t got = std::fread(buf.data(), 1, BLK, stdin);
            for (size_t k = 0; k < got; ++k) {
                frame[fill++] = buf[k];
                if (fill == ENCODED_BITS) {
                    int metric = fdec.decode_soft3(frame.data(), out);
                    total_syms += ENCODED_BITS;
                    ++frames;
                    if (metric == 0) ++perfect;
                    if (!quiet)
                        fprintf(stderr, "FRAME %4d  metric %4d %s\n",
                                frames, metric, metric == 0 ? "(perfect)" : "");
                    if (raw) std::fwrite(out.data(), 1, FRAME_BYTES, stdout);
                    fill = 0;
                }
            }
            if (got < BLK) break;
        }
    } else if (nchan <= 0) {
        // single-channel raw int16 soft stream
        SingleStreamDecodeReceiver rx;
        std::vector<int16_t> buf(BLK);
        while (true) {
            size_t got = std::fread(buf.data(), sizeof(int16_t), BLK, stdin);
            for (size_t k = 0; k < got; ++k) rx.push((double)buf[k], on_frame);
            if (got < BLK) break;
        }
        total_syms = rx.total_symbols();
    } else {
        // multiplexed channel-tagged records {uint8 channel, int16 soft}
        #pragma pack(push, 1)
        struct Rec { uint8_t ch; int16_t soft; };
        #pragma pack(pop)
        DecodeBank bank(nchan);
        std::vector<Rec> buf(BLK);
        while (true) {
            size_t got = std::fread(buf.data(), sizeof(Rec), BLK, stdin);
            for (size_t k = 0; k < got; ++k)
                bank.push(buf[k].ch, (double)buf[k].soft, on_frame);
            if (got < BLK) break;
        }
        for (size_t c = 0; c < bank.channels(); ++c) total_syms += bank.channel(c).total_symbols();
    }

    auto t1 = std::chrono::steady_clock::now();
    double sec = std::chrono::duration<double>(t1 - t0).count();

    if (!quiet) {
        fprintf(stderr, "\nSummary (decode-only): %d frames (%d perfect, %d errors)\n",
                frames, perfect, frames - perfect);
        fprintf(stderr, "decoded %zu symbols in %.3f s\n", total_syms, sec);
    }
    return frames > 0 ? 0 : 1;
}

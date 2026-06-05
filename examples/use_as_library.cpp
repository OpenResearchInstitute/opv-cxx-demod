//------------------------------------------------------------------------------
// use_as_library.cpp - minimal example of consuming the OPV demod as a library.
//
// This file has its OWN main() and only #includes the header. It demonstrates
// the pattern a host design (e.g. the Haifuraiya "dogu" payload) uses: hold one
// ChannelReceiver per channel and feed it blocks of complex baseband, receiving
// decoded frames through a callback the moment each one completes.
//
// Build (from repo root):
//   g++ -std=c++17 -O3 -Isrc examples/use_as_library.cpp -o /tmp/use_as_library
//------------------------------------------------------------------------------
#include "opv_demod.hpp"     // the whole receive DSP, header-only
#include <vector>
#include <iostream>

int main() {
    // ---- High-level API: one ChannelReceiver == one channel's full receive
    // chain (demod + symbol-lock + sync tracker + FEC frame decoder). A host
    // like dogu holds N of these (one per channelizer output) and round-robins
    // blocks of IQ through them.
    ChannelReceiver rx;
    rx.set_freq_offset(0.0);       // or rx.estimate_offset(block,n) on first block

    int frames = 0;
    auto on_frame = [&](const ChannelReceiver::Frame& f) {
        // Delivered the instant a frame completes. A real host would hand f.bytes
        // (134 bytes) to its downlink / SIC stage; here we just count.
        ++frames;
        std::cout << "  frame: metric=" << f.metric
                  << " sync=" << f.sync_quality << "\n";
    };

    std::vector<sample_t> block(4096, sample_t(0.0, 0.0));  // one block of IQ
    rx.process(block.data(), block.size(), on_frame);       // callback form

    std::cout << "ChannelReceiver OK: fed " << block.size() << " samples, "
              << rx.total_symbols() << " soft symbols, " << frames
              << " frames, AFC offset = " << rx.get_freq_offset() << " Hz\n";

    // ---- The underlying primitives remain available if a host needs to wire a
    // custom orchestration (the ChannelReceiver above is just a thin wrapper).
    MSKDemodulatorAFC demod;
    std::vector<double> soft;
    demod.demodulate(block.data(), block.size(), soft);
    std::cout << "low-level OK: " << soft.size() << " soft symbols direct from demod\n";
    return 0;
}

// Copyright 2020 Mobilinkd LLC.
// Copyright 2022-2026 Open Research Institute, Inc.
// 
// Modified to use HDL-aligned pipeline:
//   frame (134 bytes) → randomize → conv encode (K=7) → interleave (67×32) → output

#include "Util.h"
#include "queue.h"
#include "FirFilter.h"
#include "Trellis.h"
#include "Convolution.h"
#include "RowColumnInterleaver.h"
#include "OPVRandomizer.h"
#include "Util.h"
#include "Golay24.h"
#include "OPVFrameHeader.h"
#include "UDPNetwork.h"
#include "cobs.h"

#include "Numerology.h"
#include <opus/opus.h>

#include <boost/program_options.hpp>
#include <boost/optional.hpp>

#include <thread>

#include <array>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <optional>
#include <mutex>

#include <cstdlib>

#include <signal.h>

// Generated using scikit-commpy
const auto rrc_taps = std::array<double, 150>{
    0.0029364388513841593, 0.0031468394550958484, 0.002699564567597445, 0.001661182944400927, 0.00023319405581230247, -0.0012851320781224025, -0.0025577136087664687, -0.0032843366522956313, -0.0032697038088887226, -0.0024733964729590865, -0.0010285696910973807, 0.0007766690889758685, 0.002553421969211845, 0.0038920145144327816, 0.004451886520053017, 0.00404219185231544, 0.002674727068399207, 0.0005756567993179152, -0.0018493784971116507, -0.004092346891623224, -0.005648131453822014, -0.006126925416243605, -0.005349511529163396, -0.003403189203405097, -0.0006430502751187517, 0.002365929161655135, 0.004957956568090113, 0.006506845894531803, 0.006569574194782443, 0.0050017573119839134, 0.002017321931508163, -0.0018256054303579805, -0.00571615173291049, -0.008746639552588416, -0.010105075751866371, -0.009265784007800534, -0.006136551625729697, -0.001125978562075172, 0.004891777252042491, 0.01071805138282269, 0.01505751553351295, 0.01679337935001369, 0.015256245142156299, 0.01042830577908502, 0.003031522725559901, -0.0055333532968188165, -0.013403099825723372, -0.018598682349642525, -0.01944761739590459, -0.015005271935951746, -0.0053887880354343935, 0.008056525910253532, 0.022816244158307273, 0.035513467692208076, 0.04244131815783876, 0.04025481153629372, 0.02671818654865632, 0.0013810216516704976, -0.03394615682795165, -0.07502635967975885, -0.11540977897637611, -0.14703962203941534, -0.16119995609538576, -0.14969512896336504, -0.10610329539459686, -0.026921412469634916, 0.08757875030779196, 0.23293327870303457, 0.4006012210123992, 0.5786324696325503, 0.7528286479934068, 0.908262741447522, 1.0309661131633199, 1.1095611856548013, 1.1366197723675815, 1.1095611856548013, 1.0309661131633199, 0.908262741447522, 0.7528286479934068, 0.5786324696325503, 0.4006012210123992, 0.23293327870303457, 0.08757875030779196, -0.026921412469634916, -0.10610329539459686, -0.14969512896336504, -0.16119995609538576, -0.14703962203941534, -0.11540977897637611, -0.07502635967975885, -0.03394615682795165, 0.0013810216516704976, 0.02671818654865632, 0.04025481153629372, 0.04244131815783876, 0.035513467692208076, 0.022816244158307273, 0.008056525910253532, -0.0053887880354343935, -0.015005271935951746, -0.01944761739590459, -0.018598682349642525, -0.013403099825723372, -0.0055333532968188165, 0.003031522725559901, 0.01042830577908502, 0.015256245142156299, 0.01679337935001369, 0.01505751553351295, 0.01071805138282269, 0.004891777252042491, -0.001125978562075172, -0.006136551625729697, -0.009265784007800534, -0.010105075751866371, -0.008746639552588416, -0.00571615173291049, -0.0018256054303579805, 0.002017321931508163, 0.0050017573119839134, 0.006569574194782443, 0.006506845894531803, 0.004957956568090113, 0.002365929161655135, -0.0006430502751187517, -0.003403189203405097, -0.005349511529163396, -0.006126925416243605, -0.005648131453822014, -0.004092346891623224, -0.0018493784971116507, 0.0005756567993179152, 0.002674727068399207, 0.00404219185231544, 0.004451886520053017, 0.0038920145144327816, 0.002553421969211845, 0.0007766690889758685, -0.0010285696910973807, -0.0024733964729590865, -0.0032697038088887226, -0.0032843366522956313, -0.0025577136087664687, -0.0012851320781224025, 0.00023319405581230247, 0.001661182944400927, 0.002699564567597445, 0.0031468394550958484, 0.0029364388513841593, 0.0
};

const char VERSION[] = "0.2-hdl";

using namespace mobilinkd;

struct Config
{
    std::string source_address;
    bool verbose = false;
    bool debug = false;
    bool quiet = false;
    bool bitstream = false; // default is baseband audio
    bool output_to_network = false; // default is output to stdout
    std::string network_ip;
    uint16_t network_port;
    uint32_t bert = 0; // Frames of Bit error rate testing.
    uint64_t token = 0; // authentication token for frame header
    bool invert = false;
    bool preamble_only = false;

    static std::optional<Config> parse(int argc, char* argv[])
    {
        namespace po = boost::program_options;

        Config result;

        // Declare the supported options.
        po::options_description desc(
            "Program options");
        desc.add_options()
            ("help,h", "Print this help message and exit.")
            ("version,V", "Print the application verion and exit.")
            ("src,S", po::value<std::string>(&result.source_address)->required(),
                "transmitter identifier (your callsign).")
            ("token,T", po::value<uint64_t>(&result.token)->default_value(0x8765432112345678),
                "authentication token")
            ("bitstream,b", po::bool_switch(&result.bitstream),
                "output bitstream (default is baseband).")
            ("network,n", po::bool_switch(&result.output_to_network),
                "output to network (implies --bitstream)")
            ("ip", po::value<std::string>(&result.network_ip)->default_value("127.0.0.1"),
                "IP address (used with --network)")
            ("port", po::value<uint16_t>(&result.network_port)->default_value(7373),
                "output to port (used with --network)")
            ("bert,B", po::value<uint32_t>(&result.bert)->default_value(0),
                "number of BERT frames to output (default or 0 to read audio from STDIN instead).")
            ("invert,i", po::bool_switch(&result.invert), "invert the output baseband (ignored for bitstream)")
            ("preamble,P", po::bool_switch(&result.preamble_only), "preamble-only output")
            ("verbose,v", po::bool_switch(&result.verbose), "verbose output")
            ("debug,d", po::bool_switch(&result.debug), "debug-level output")
            ("quiet,q", po::bool_switch(&result.quiet), "silence all output")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << "Read audio from STDIN and write baseband OPV to STDOUT\n"
                << desc << std::endl;

            return std::nullopt;
        }

        if (vm.count("version"))
        {
            std::cout << argv[0] << ": " << VERSION << std::endl;
            std::cout << opus_get_version_string() << std::endl;
            return std::nullopt;
        }

        try {
            po::notify(vm);
        } catch (std::exception& ex)
        {
            std::cerr << ex.what() << std::endl;
            std::cout << desc << std::endl;
            return std::nullopt;
        }

        if (result.debug + result.verbose + result.quiet > 1)
        {
            std::cerr << "Only one of quiet, verbose or debug may be chosen." << std::endl;
            return std::nullopt;
        }

        if (result.source_address.size() > 9)
        {
            std::cerr << "Source identifier too long." << std::endl;
            return std::nullopt;
        }

        return result;
    }
};

std::optional<Config> config;

std::atomic<bool> running{false};
UDPNetwork udp;

bool invert = false;


// Intercept ^C and just tell the transmit thread to end, which ends the program
void signal_handler(int)
{
    running = false;
    std::cerr << "quitting" << std::endl;
}


// Convert a dibit into a modulation symbol
int8_t bits_to_symbol(uint8_t bits)
{
    switch (bits)
    {
    case 0: return 1;
    case 1: return 3;
    case 2: return -1;
    case 3: return -3;
    }
    abort();
}


// Convert an unpacked array of bits into an unpacked array of modulation symbols
template <typename T, size_t N>
std::array<int8_t, N / 2> bits_to_symbols(const std::array<T, N>& bits)
{
    std::array<int8_t, N / 2> result;
    size_t index = 0;
    for (size_t i = 0; i != N; i += 2)
    {
        result[index++] = bits_to_symbol((bits[i] << 1) | bits[i + 1]);
    }
    return result;
}


// Convert a packed array of bits into an unpacked array of modulation symbols
template <typename T, size_t N>
std::array<int8_t, N * 4> bytes_to_symbols(const std::array<T, N>& bytes)
{
    std::array<int8_t, N * 4> result;
    size_t index = 0;
    for (auto b : bytes)
    {
        for (size_t i = 0; i != 4; ++i)
        {
            result[index++] = bits_to_symbol(b >> 6);
            b <<= 2;
        }
    }
    return result;
}


// Convert an unpacked array of modulation symbols into an array of modulation samples.
// This includes the 10x interpolation using the RRC filter.
template <size_t N>
std::array<int16_t, N*10> symbols_to_baseband(std::array<int8_t, N> symbols)
{
    using namespace mobilinkd;

    static BaseFirFilter<double, std::tuple_size<decltype(rrc_taps)>::value> rrc = makeFirFilter(rrc_taps);

    std::array<int16_t, N*10> baseband;
    baseband.fill(0);
    for (size_t i = 0; i != symbols.size(); ++i)
    {
        baseband[i * 10] = symbols[i];
    }

    for (auto& b : baseband)
    {
        b = rrc(b) * 7168.0 * (invert ? -1.0 : 1.0);
    }

    return baseband;
}


// =============================================================================
// NEW OPV TYPE DEFINITIONS (HDL-aligned)
// =============================================================================

// Frame types for HDL-aligned pipeline
using opv_frame_t = std::array<uint8_t, opv_frame_bytes>;           // 134 bytes (header + payload)
using opv_encoded_t = std::array<int8_t, opv_encoded_bits>;         // 2144 bits after FEC

// Old frame types (still needed for some functions)
using fheader_t = std::array<uint8_t, opv_header_bytes>;            // Frame Header: 12 bytes
using stream_frame_t = std::array<uint8_t, opv_payload_bytes>;      // Payload: 122 bytes

using queue_t = queue<int16_t, audio_samples_per_opv_frame>;
using audio_frame_t = std::array<int16_t, audio_samples_per_opv_frame>;

// New sync word (24 bits = 3 bytes) - PSLR optimized
constexpr std::array<uint8_t, 3> OPV_SYNC_BYTES = {0x02, 0xB8, 0xDB};

// Legacy sync words (16 bits = 2 bytes) - for preamble compatibility
constexpr std::array<uint8_t, 2> STREAM_SYNC_WORD = {0xFF, 0x5D};
constexpr std::array<uint8_t, 2> EOT_SYNC = { 0x55, 0x5D };

// Number of symbols in a complete frame (sync + encoded data)
// 24 bits sync / 2 = 12 symbols, 2144 bits data / 2 = 1072 symbols
constexpr size_t OPV_FRAME_SYMBOLS = 12 + opv_encoded_bits / 2;  // 1084 symbols


// =============================================================================
// NEW OPV ENCODER FUNCTIONS (HDL-aligned)
// =============================================================================

/**
 * Convolutional encode with K=7 NASA/Voyager code
 * 
 * Input: 134 bytes (1072 bits)
 * Output: 2144 bits (as unpacked int8_t, values 0 or 1)
 * 
 * Generator polynomials: G1=171o (0x79), G2=133o (0x5B)
 * This is an unterminated trellis (no flush bits), matching HDL.
 */
opv_encoded_t conv_encode_k7(const opv_frame_t& frame)
{
    opv_encoded_t encoded;
    size_t out_idx = 0;
    uint8_t memory = 0;  // 6 bits of shift register (K-1 = 6)
    
    for (auto byte : frame)
    {
        for (int bit_idx = 7; bit_idx >= 0; --bit_idx)
        {
            uint8_t input_bit = (byte >> bit_idx) & 1;
            
            // State = [input_bit | memory], 7 bits total
            uint8_t state = (input_bit << 6) | memory;
            
            // G1 = 171 octal = 1111001 binary = 0x79
            uint8_t g1_out = __builtin_parity(state & 0x79);
            
            // G2 = 133 octal = 1011011 binary = 0x5B
            uint8_t g2_out = __builtin_parity(state & 0x5B);
            
            encoded[out_idx++] = g1_out;
            encoded[out_idx++] = g2_out;
            
            // Shift memory
            memory = ((memory << 1) | input_bit) & 0x3F;
        }
    }
    
    return encoded;
}


/**
 * Encode a complete OPV frame using HDL-aligned pipeline
 * 
 * Pipeline:
 *   1. Combine header (12 bytes) + payload (122 bytes) = 134 bytes
 *   2. Randomize (CCSDS LFSR)
 *   3. Convolutional encode (K=7, rate 1/2) → 2144 bits
 *   4. Interleave (67×32 row-column)
 */
opv_encoded_t encode_opv_frame(const fheader_t& header, const stream_frame_t& payload)
{
    // 1. Combine header + payload
    opv_frame_t frame;
    std::copy(header.begin(), header.end(), frame.begin());
    std::copy(payload.begin(), payload.end(), frame.begin() + opv_header_bytes);
    
    // 2. Randomize (CCSDS LFSR on bytes, BEFORE FEC)
    OPVFrameRandomizer<opv_frame_bytes> randomizer;
    randomizer.randomize(frame);
    
    // 3. Convolutional encode (K=7)
    opv_encoded_t encoded = conv_encode_k7(frame);
    
    // 4. Interleave (67×32 row-column)
    OPVInterleaver interleaver;
    interleaver.interleave(encoded);
    
    return encoded;
}


/**
 * Output encoded OPV frame as bitstream (packed bytes)
 */
void output_opv_bitstream(const opv_encoded_t& frame)
{
    // Output sync word (3 bytes)
    for (auto b : OPV_SYNC_BYTES)
    {
        std::cout << b;
    }
    
    // Pack and output frame data (2144 bits = 268 bytes)
    for (size_t i = 0; i < opv_encoded_bits; i += 8)
    {
        uint8_t byte = 0;
        for (size_t j = 0; j < 8; ++j)
        {
            byte = (byte << 1) | (frame[i + j] & 1);
        }
        std::cout << byte;
    }
}


/**
 * Output encoded OPV frame as baseband samples (4-FSK modulated)
 */
void output_opv_baseband(const opv_encoded_t& frame)
{
    // Convert sync word bytes to symbols (3 bytes = 12 symbols)
    auto sync_symbols = bytes_to_symbols(OPV_SYNC_BYTES);
    
    // Convert frame bits to symbols (2144 bits = 1072 symbols)
    auto frame_symbols = bits_to_symbols(frame);
    
    // Combine: 12 + 1072 = 1084 symbols
    std::array<int8_t, OPV_FRAME_SYMBOLS> all_symbols;
    std::copy(sync_symbols.begin(), sync_symbols.end(), all_symbols.begin());
    std::copy(frame_symbols.begin(), frame_symbols.end(), all_symbols.begin() + sync_symbols.size());
    
    // RRC filter and output
    auto baseband = symbols_to_baseband(all_symbols);
    for (auto b : baseband)
    {
        std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
    }
}


/**
 * Output encoded OPV frame in configured format
 */
void output_opv_frame(const opv_encoded_t& frame)
{
    if (config->bitstream)
    {
        output_opv_bitstream(frame);
    }
    else
    {
        output_opv_baseband(frame);
    }
}


/**
 * Send a complete OPV stream frame (new HDL-aligned pipeline)
 */
void send_stream_frame(const fheader_t& header, const stream_frame_t& payload)
{
    opv_encoded_t encoded = encode_opv_frame(header, payload);
    output_opv_frame(encoded);
}


// =============================================================================
// PREAMBLE AND EOT FUNCTIONS (use old frame size for compatibility)
// =============================================================================

// Preamble frame size: use new encoded size + sync bits for consistency
constexpr size_t PREAMBLE_BYTES = (opv_sync_bits + opv_encoded_bits) / 8;  // 271 bytes

void send_preamble()
{
    if (config->verbose) std::cerr << "Sending preamble: " << PREAMBLE_BYTES * 8 << " bits." << std::endl;

    std::array<uint8_t, PREAMBLE_BYTES> preamble_bytes;
    preamble_bytes.fill(0x77);  // +3, -3, +3, -3 pattern
    
    if (config->bitstream)
    {
        for (auto c : preamble_bytes) std::cout << c;
    }
    else
    {
        auto preamble_symbols = bytes_to_symbols(preamble_bytes);
        auto preamble_baseband = symbols_to_baseband(preamble_symbols);
        for (auto b : preamble_baseband) std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
    }
}


void send_dead_carrier()
{
    if (config->output_to_network) return;

    if (config->verbose) std::cerr << "Sending dead carrier: " << PREAMBLE_BYTES * 8 << " bits." << std::endl;

    std::array<uint8_t, PREAMBLE_BYTES> carrier_bytes;
    carrier_bytes.fill(0x00);  // +1, +1, +1, +1 pattern
    
    if (config->bitstream)
    {
        for (auto c : carrier_bytes) std::cout << c;
    }
    else
    {
        auto carrier_symbols = bytes_to_symbols(carrier_bytes);
        auto carrier_baseband = symbols_to_baseband(carrier_symbols);
        for (auto b : carrier_baseband) std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
    }
}


void output_eot()
{
    if (config->bitstream)
    {
        for (auto c : EOT_SYNC) std::cout << c;
        for (size_t i = 0; i != 10; ++i) std::cout << '\0';
    }
    else
    {
        std::array<int8_t, 48> out_symbols;
        out_symbols.fill(0);
        auto symbols = bytes_to_symbols(EOT_SYNC);
        for (size_t i = 0; i != symbols.size(); ++i)
        {
            out_symbols[i] = symbols[i];
        }
        auto baseband = symbols_to_baseband(out_symbols);
        for (auto b : baseband) std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
    }
}


// =============================================================================
// FRAME BUILDING FUNCTIONS
// =============================================================================

// Fill in the minimal 12-byte RTP header
void build_rtp_header(uint8_t* frame_buffer)
{
    memcpy(frame_buffer, "RTP_RTP_RTP_", 12);
}


// Fill in the 8-byte UDP header
void build_udp_header(uint8_t* frame_buffer, int udp_length)
{
    const uint16_t src_port = 54321;
    const uint16_t dst_port = 1234;
    uint8_t udp_header[8] =
    {
        (uint8_t)(src_port/256), (uint8_t)(src_port%256),
        (uint8_t)(dst_port/256), (uint8_t)(dst_port%256),
        (uint8_t)(udp_length/256), (uint8_t)(udp_length%256),
        0x00, 0x00
    };
    memcpy(frame_buffer, udp_header, 8);
}


// Fill in the 20-byte IPv4 header
void build_ip_header(uint8_t* frame_buffer, int packet_len)
{
    uint8_t ip_header[20] = { 0x45, 0x00, (uint8_t)(packet_len/256), (uint8_t)(packet_len%256),
                              0x00, 0x00, 0x00, 0x00,
                              64,   17,   0x00, 0x00,
                              192,  168,  0,    1,
                              192,  168,  0,    2
                            };
    memcpy(frame_buffer, ip_header, 20);
}


// COBS-encode the Opus+RTP+UDP+IP packet
void cobs_encode_voice_frame(uint8_t* frame, uint8_t* cobs_frame)
{
    cobs_encode_result result;
    result = cobs_encode(cobs_frame, opv_payload_bytes, frame, opv_payload_bytes - cobs_overhead_bytes_for_opus);
    if (result.out_len >= opv_payload_bytes || result.status != COBS_ENCODE_OK)
    {
        fprintf(stderr, "Failure COBS encoding voice frame.\n");
    }
    else
    {
        for (size_t i = result.out_len; i < opv_payload_bytes; i++)
        {
            cobs_frame[i] = 0;
        }
    }
}


// Create the payload for an OPV voice frame
stream_frame_t fill_voice_frame(OpusEncoder *opus_encoder, const audio_frame_t& audio)
{
    stream_frame_t frame;
    stream_frame_t cobs_frame;
    opus_int32 count;

    memset(&frame[0], 0, opv_payload_bytes);

    count = opus_encode(opus_encoder,
                        const_cast<int16_t*>(&audio[0]),
                        audio_samples_per_opv_frame,
                        &frame[ip_v4_header_bytes+udp_header_bytes+rtp_header_bytes],
                        opus_packet_size_bytes
                        );

    if (count != opus_packet_size_bytes)
    {
        std::cerr << "Got unexpected encoded voice size " << count << std::endl;
    }

    build_rtp_header(&frame[ip_v4_header_bytes+udp_header_bytes]);
    build_udp_header(&frame[ip_v4_header_bytes], udp_header_bytes+rtp_header_bytes+opus_packet_size_bytes);
    build_ip_header(&frame[0], ip_v4_header_bytes+udp_header_bytes+rtp_header_bytes+opus_packet_size_bytes);

    cobs_encode_voice_frame(&frame[0], &cobs_frame[0]);

    return cobs_frame;
}


// Create the payload for a BERT frame
template <typename PRBS>
stream_frame_t fill_bert_frame(PRBS& prbs)
{
    stream_frame_t bert_bytes;
    std::array<uint8_t, opv_payload_bytes * 8> bert_bits;
    size_t index = 0;

    for (size_t i = 0; i < bert_bits.size(); i++)
    {
        bool bit;
        if (index < opv_bert_prime_size)
        {
            bit = prbs.generate();
        }
        else
        {
            bit = bert_bits[index - opv_bert_prime_size];
        }
        bert_bits[index] = bit;
        index++;
    }

    to_byte_array(bert_bits, bert_bytes);
    if (config->verbose) std::cerr << "BERT frame" << std::endl;

    return bert_bytes;
}


// =============================================================================
// FRAME HEADER FUNCTIONS
// =============================================================================

void dump_fheader(const fheader_t header)
{
    std::cerr << "Frame Header: "
            << std::hex
            << std::setfill('0');

    for (auto hbyte: header)
    {
        std::cerr << std::setw(2) << int(hbyte) << " ";
    }

    if (header[6] & 0x80) std::cerr << "last ";
    if (header[6] & 0x40) std::cerr << "BERT";

    std::cerr << std::endl << std::dec;
}


fheader_t fill_fheader(const std::string& source_callsign, OPVFrameHeader::token_t& access_token, bool is_bert)
{
    fheader_t header;
    header.fill(0);

    OPVFrameHeader::call_t callsign;
    callsign.fill(0);
    std::copy(source_callsign.begin(), source_callsign.end(), callsign.begin());
    auto encoded_callsign = OPVFrameHeader::encode_callsign(callsign);
    uint8_t flags = 0;
    if (is_bert) flags |= 0x40;

    std::copy(encoded_callsign.begin(), encoded_callsign.end(), header.begin());
    std::copy(access_token.begin(), access_token.end(), header.begin() + 9);
    header[6] = flags;

    if (config->verbose) dump_fheader(header);

    return header;
}


void set_last_frame_bit(fheader_t& fh)
{
    fh[6] |= 0x80;
}


// =============================================================================
// TRANSMIT FUNCTION
// =============================================================================

void transmit(queue_t& queue, fheader_t& fh)
{
    int encoder_err;

    assert(running);

    OpusEncoder* opus_encoder = ::opus_encoder_create(audio_sample_rate, 1, OPUS_APPLICATION_VOIP, &encoder_err);

    if (encoder_err < 0)
    {
        std::cerr << "Failed to create an Opus encoder!";
        abort();
    }

    encoder_err = opus_encoder_ctl(opus_encoder, OPUS_SET_BITRATE(opus_bitrate));
    if (encoder_err < 0)
    {
        std::cerr << "Failed to set Opus bitrate!";
        abort();
    }

    encoder_err = opus_encoder_ctl(opus_encoder, OPUS_SET_VBR(0));
    if (encoder_err < 0)
    {
        std::cerr << "Failed to set Opus to constant bit rate mode!";
        abort();
    }

    audio_frame_t audio;
    size_t index = 0;

    while (!queue.is_closed() && queue.empty()) std::this_thread::yield();
    while (!queue.is_closed())
    {
        int16_t sample;
        if (!queue.get(sample, std::chrono::milliseconds(3000))) break;
        audio[index++] = sample;
        if (index == audio.size())
        {
            index = 0;
            auto payload = fill_voice_frame(opus_encoder, audio);
            send_stream_frame(fh, payload);
            audio.fill(0);
        } 
    }

    if (index > 0)
    {
        auto payload = fill_voice_frame(opus_encoder, audio);
        send_stream_frame(fh, payload);
    }

    // Last frame is an extra frame of silence with EOS flag
    audio.fill(0);
    auto payload = fill_voice_frame(opus_encoder, audio);
    set_last_frame_bit(fh);
    if (config->verbose) dump_fheader(fh);
    send_stream_frame(fh, payload);
    output_eot();

    opus_encoder_destroy(opus_encoder);
}


// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char* argv[])
{
    using namespace mobilinkd;

    try
    {
        config = Config::parse(argc, argv);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    if (!config) return 0;

    invert = config->invert;

    if (config->output_to_network)
    {
        config->bitstream = true;
        udp.network_setup(config->network_ip, config->network_port);
    }

    OPVFrameHeader::token_t access_token;
    std::cerr << "Access token: 0x" << std::hex << std::setw(6) << config->token << std::dec << std::endl;
    access_token[0] = (config->token & 0xff0000) >> 16;
    access_token[1] = (config->token & 0x00ff00) >> 8;
    access_token[2] = (config->token & 0x0000ff);

    auto fh = fill_fheader(config->source_address, access_token, config->bert != 0);

    // Debug output
    dump_fheader(fh);
    std::cerr << "Using HDL-aligned pipeline: randomize → K=7 conv → 67×32 interleave" << std::endl;
    std::cerr << "Frame: " << opv_frame_bytes << " bytes → " << opv_encoded_bits << " bits" << std::endl;
    std::cerr << "Sync word: 0x" << std::hex << opv_sync_word << std::dec << std::endl;

    signal(SIGINT, &signal_handler);

    send_dead_carrier();
    send_dead_carrier();
    send_preamble();

    if (config->preamble_only)
    {
        running = true;
        std::cerr << "opv-mod sending only preambles" << std::endl;

        while (running)
        {
            send_preamble();
        }
    }
    else if (config->bert)
    {
        // BERT mode
        PRBS9 prbs;
        running = true;

        uint32_t frame_count;
        for (frame_count = 0; frame_count < config->bert; frame_count++)
        {
            if (!running) break;

            auto payload = fill_bert_frame(prbs);

            if (frame_count + 1 == config->bert)
            {
                set_last_frame_bit(fh);
                if (config->verbose) dump_fheader(fh);
            }

            send_stream_frame(fh, payload);
        }

        std::cerr << "Output " << frame_count << " frames of BERT data." << std::endl;
        
        output_eot();
        send_dead_carrier();
    }
    else
    {
        // Normal mode (voice, data)
        running = true;
        queue_t queue;
        std::thread thd([&queue, &fh](){transmit(queue, fh);});

        std::cerr << "opv-mod running. ctrl-D to break." << std::endl;

        while (running)
        {
            int16_t sample;
            if (!std::cin.read(reinterpret_cast<char*>(&sample), 2)) break;
            if (!queue.put(sample, std::chrono::seconds(300))) break;
        }

        running = false;

        queue.close();
        thd.join();
    }
    
    return EXIT_SUCCESS;
}

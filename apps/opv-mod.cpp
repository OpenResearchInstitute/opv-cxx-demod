// Copyright 2020 Mobilinkd LLC.
// Copyright 2022-2026 Open Research Institute, Inc.
// 
// OPV Modulator with MSK output for SDR transmission
//
// Pipeline: frame (134 bytes) → randomize → conv encode (K=7) → interleave (67×32) → diff encode → MSK modulate → I/Q output

#include "Util.h"
#include "queue.h"
#include "FirFilter.h"
#include "Trellis.h"
#include "Convolution.h"
#include "RowColumnInterleaver.h"
#include "OPVRandomizer.h"
#include "MSKModulator.h"
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
#include <vector>

#include <cstdlib>
#include <cmath>

#include <signal.h>

const char VERSION[] = "0.3-msk";

using namespace mobilinkd;

struct Config
{
    std::string source_address;
    bool verbose = false;
    bool debug = false;
    bool quiet = false;
    bool bitstream = false;     // Output packed bits (no modulation)
    bool msk = true;            // MSK modulation (default) vs legacy 4-FSK
    bool output_to_network = false;
    std::string network_ip;
    uint16_t network_port;
    uint32_t bert = 0;
    uint64_t token = 0;
    bool invert = false;
    bool preamble_only = false;
    bool continuous = false;  // Continuous BERT transmission

    static std::optional<Config> parse(int argc, char* argv[])
    {
        namespace po = boost::program_options;

        Config result;

        po::options_description desc("Program options");
        desc.add_options()
            ("help,h", "Print this help message and exit.")
            ("version,V", "Print the application version and exit.")
            ("src,S", po::value<std::string>(&result.source_address)->required(),
                "transmitter identifier (your callsign).")
            ("token,T", po::value<uint64_t>(&result.token)->default_value(0xC0FFEE),
                "authentication token (default 0xC0FFEE for C++ modem, Interlocutor uses 0xBBAADD)")
            ("bitstream,b", po::bool_switch(&result.bitstream),
                "output bitstream (packed bits, no modulation).")
            ("4fsk", po::bool_switch(),
                "use legacy 4-FSK modulation instead of MSK")
            ("network,n", po::bool_switch(&result.output_to_network),
                "output to network (implies --bitstream)")
            ("ip", po::value<std::string>(&result.network_ip)->default_value("127.0.0.1"),
                "IP address (used with --network)")
            ("port", po::value<uint16_t>(&result.network_port)->default_value(7373),
                "output to port (used with --network)")
            ("bert,B", po::value<uint32_t>(&result.bert)->default_value(0),
                "number of BERT frames to output.")
            ("invert,i", po::bool_switch(&result.invert), "invert the output")
            ("preamble,P", po::bool_switch(&result.preamble_only), "preamble-only output")
            ("continuous,c", po::bool_switch(&result.continuous), "continuous BERT transmission (use with -B)")
            ("verbose,v", po::bool_switch(&result.verbose), "verbose output")
            ("debug,d", po::bool_switch(&result.debug), "debug-level output")
            ("quiet,q", po::bool_switch(&result.quiet), "silence all output")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << "Read audio from STDIN and write MSK I/Q baseband to STDOUT\n"
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

        // Handle --4fsk flag
        if (vm["4fsk"].as<bool>())
        {
            result.msk = false;
        }

        return result;
    }
};

std::optional<Config> config;

std::atomic<bool> running{false};
UDPNetwork udp;

// MSK Modulator instance
OPVMSKModulator msk_modulator;

// =============================================================================
// TYPE DEFINITIONS
// =============================================================================

using opv_frame_t = std::array<uint8_t, opv_frame_bytes>;           // 134 bytes
using opv_encoded_t = std::array<int8_t, opv_encoded_bits>;         // 2144 bits
using fheader_t = std::array<uint8_t, opv_header_bytes>;            // 12 bytes
using stream_frame_t = std::array<uint8_t, opv_payload_bytes>;      // 122 bytes

using queue_t = queue<int16_t, audio_samples_per_opv_frame>;
using audio_frame_t = std::array<int16_t, audio_samples_per_opv_frame>;

// Sync word as bytes (24 bits = 3 bytes)
constexpr std::array<uint8_t, 3> OPV_SYNC_BYTES = {0x02, 0xB8, 0xDB};

// Sync word as unpacked bits (for MSK modulation)
constexpr std::array<uint8_t, 24> OPV_SYNC_BITS = {
    0,0,0,0,0,0,1,0,  // 0x02
    1,0,1,1,1,0,0,0,  // 0xB8
    1,1,0,1,1,0,1,1   // 0xDB
};

// Legacy sync words for EOT
constexpr std::array<uint8_t, 2> EOT_SYNC = { 0x55, 0x5D };


// =============================================================================
// SIGNAL HANDLER
// =============================================================================

void signal_handler(int)
{
    running = false;
    std::cerr << "quitting" << std::endl;
}


// =============================================================================
// MSK OUTPUT FUNCTIONS
// =============================================================================

/**
 * Write I/Q samples to stdout
 * Format: interleaved 16-bit signed integers (I0, Q0, I1, Q1, ...)
 */
void output_iq(const std::vector<OPVMSKModulator::IQSample>& samples)
{
    for (const auto& s : samples)
    {
        // Output as little-endian 16-bit signed integers
        std::cout.write(reinterpret_cast<const char*>(&s.I), sizeof(int16_t));
        std::cout.write(reinterpret_cast<const char*>(&s.Q), sizeof(int16_t));
    }
}

/**
 * Output I/Q samples from array
 */
template <size_t N>
void output_iq(const std::array<OPVMSKModulator::IQSample, N>& samples)
{
    for (const auto& s : samples)
    {
        std::cout.write(reinterpret_cast<const char*>(&s.I), sizeof(int16_t));
        std::cout.write(reinterpret_cast<const char*>(&s.Q), sizeof(int16_t));
    }
}


// =============================================================================
// BITSTREAM OUTPUT FUNCTIONS
// =============================================================================

/**
 * Output encoded frame as packed bytes (bitstream mode)
 */
void output_bitstream_frame(const opv_encoded_t& frame)
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


// =============================================================================
// ENCODING FUNCTIONS
// =============================================================================




opv_encoded_t conv_encode_k7(const opv_frame_t& frame)
{
    opv_encoded_t encoded;
    size_t out_idx = 0;
    uint8_t memory = 0;
    
    // FIXED: Iterate bytes in REVERSE order to match HDL
    // HDL encoder reads from MSB of input buffer, which contains last byte
    for (int i = frame.size() - 1; i >= 0; --i)
    {
        auto byte = frame[i];
        for (int bit_idx = 7; bit_idx >= 0; --bit_idx)
        {
            uint8_t input_bit = (byte >> bit_idx) & 1;
            uint8_t state = (input_bit << 6) | memory;
            
            uint8_t g1_out = __builtin_parity(state & 0x4F);
            uint8_t g2_out = __builtin_parity(state & 0x6D);
            
            encoded[out_idx++] = g1_out;
            encoded[out_idx++] = g2_out;
            
            memory = ((memory << 1) | input_bit) & 0x3F;
        }
    }
    
    return encoded;
}




/**
 * Convolutional encode with K=7 NASA/Voyager code
 */

/**
 * Encode a complete OPV frame
 * Pipeline: combine → randomize → conv encode → interleave
 */
opv_encoded_t encode_opv_frame(const fheader_t& header, const stream_frame_t& payload)
{
    // 1. Combine header + payload
    opv_frame_t frame;
    std::copy(header.begin(), header.end(), frame.begin());
    std::copy(payload.begin(), payload.end(), frame.begin() + opv_header_bytes);


    // DEBUG: Show frame BEFORE randomize
    std::cerr << "Frame before randomize (first 20 bytes): ";
    for (int i = 0; i < 20; i++) 
        std::cerr << std::hex << std::setw(2) << std::setfill('0') << (int)frame[i] << " ";
    std::cerr << std::dec << std::endl;

    // 2. Randomize
    OPVFrameRandomizer<opv_frame_bytes> randomizer;
    randomizer.randomize(frame);

    // DEBUG: Show frame AFTER randomize  
    std::cerr << "Frame after randomize (first 20 bytes):  ";
    for (int i = 0; i < 20; i++) 
        std::cerr << std::hex << std::setw(2) << std::setfill('0') << (int)frame[i] << " ";
    std::cerr << std::dec << std::endl;
    
    // 3. Convolutional encode
    opv_encoded_t encoded = conv_encode_k7(frame);


    // DEBUG: Show first 40 bits after FEC (as 0/1)
    std::cerr << "[3] After conv encode (first 40 of " << opv_encoded_bits << " bits): ";
    for (int i = 0; i < 40; i++) 
        std::cerr << (int)(encoded[i] & 1);
    std::cerr << std::endl;
    
    // 4. Interleave
    OPVInterleaver interleaver;
    interleaver.interleave(encoded);

    // DEBUG: Show first 40 bits after interleave
    std::cerr << "[4] After interleave (first 40 bits): ";
    for (int i = 0; i < 40; i++) 
        std::cerr << (int)(encoded[i] & 1);
    std::cerr << std::endl;
    
    return encoded;
}


// =============================================================================
// MSK FRAME OUTPUT
// =============================================================================

/**
 * Send encoded frame with MSK modulation
 */
void send_msk_frame(const opv_encoded_t& encoded)
{
    static uint64_t frame_num = 0;
    frame_num++;
    
    size_t samples_this_frame = 0;
    
    // Debug: dump sync word info
    std::cerr << "Frame " << frame_num << ": sync=";
    for (size_t i = 0; i < OPV_SYNC_BITS.size(); ++i)
    {
        std::cerr << (int)OPV_SYNC_BITS[i];
    }
    std::cerr << " (0x" << std::hex << std::setfill('0') 
              << std::setw(2) << (int)OPV_SYNC_BYTES[0]
              << std::setw(2) << (int)OPV_SYNC_BYTES[1]
              << std::setw(2) << (int)OPV_SYNC_BYTES[2]
              << std::dec << ")";
    
    // Modulate sync word (24 bits)
    std::array<OPVMSKModulator::IQSample, OPVMSKModulator::SAMPLES_PER_SYMBOL> symbol_samples;
    for (size_t i = 0; i < OPV_SYNC_BITS.size(); ++i)
    {
        msk_modulator.modulate_bit(OPV_SYNC_BITS[i], symbol_samples);
        output_iq(symbol_samples);
        samples_this_frame += OPVMSKModulator::SAMPLES_PER_SYMBOL;
    }


    // Debug: Show first 24 data bits being modulated
    std::cerr << "[TX] Frame data first 24 bits: ";
    for (int i = 0; i < 24; i++) {
        std::cerr << (int)(encoded[i] & 1);
    }
    std::cerr << std::endl;



    // Old way of doing this
    // Modulate frame data (2144 bits = 268 bytes)
    //for (size_t i = 0; i < opv_encoded_bits; ++i)
    //{
    //    msk_modulator.modulate_bit(encoded[i] & 1, symbol_samples);
    //    output_iq(symbol_samples);
    //    samples_this_frame += OPVMSKModulator::SAMPLES_PER_SYMBOL;
    //}



    // Modulate frame data (2144 bits = 268 bytes)
    // Send bits MSB-first within each byte to match HDL byte_to_bit_deserializer
    for (size_t byte_idx = 0; byte_idx < opv_encoded_bits / 8; ++byte_idx)
    {
        // Send bits 7,6,5,4,3,2,1,0 of each 8-bit group
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos)
        {
            size_t i = byte_idx * 8 + bit_pos;
            msk_modulator.modulate_bit(encoded[i] & 1, symbol_samples);
            output_iq(symbol_samples);
            samples_this_frame += OPVMSKModulator::SAMPLES_PER_SYMBOL;
        }
    }



    // Debug: Show last 24 data bits modulated
    std::cerr << "[TX] Frame data last 24 bits:  ";
    for (int i = opv_encoded_bits - 24; i < (int)opv_encoded_bits; i++) {
        std::cerr << (int)(encoded[i] & 1);
    }
    std::cerr << std::endl;


    // Count 1-bits in the data we just transmitted
    int ones_in_frame = 0;
    for (size_t i = 0; i < opv_encoded_bits; ++i) {
        if (encoded[i] & 1) ones_in_frame++;
    }
    std::cerr << "[TX] Frame " << frame_num << ": " << ones_in_frame 
              << " ones in data (" << (ones_in_frame % 2 ? "ODD" : "EVEN") << ")" << std::endl;


    
    // Expected: (24 + 2144) * 40 = 86720 samples
    std::cerr << " samples=" << samples_this_frame << std::endl;
}

/**
 * Send stream frame with optional inter-frame preamble
 */
void send_stream_frame(const fheader_t& header, const stream_frame_t& payload)
{
    opv_encoded_t encoded = encode_opv_frame(header, payload);
    
    if (config->bitstream)
    {
        output_bitstream_frame(encoded);
    }
    else
    {
        send_msk_frame(encoded);
    }
}


// =============================================================================
// PREAMBLE AND CONTROL FRAMES
// =============================================================================

// Number of preamble bits (approximately one frame worth)
constexpr size_t PREAMBLE_BITS = opv_sync_bits + opv_encoded_bits;  // 2168 bits

/**
 * Send preamble (alternating bits for clock recovery)
 */
void send_preamble()
{
    if (config->verbose) std::cerr << "Sending preamble: " << PREAMBLE_BITS << " bits." << std::endl;
    
    if (config->bitstream)
    {
        // Alternating bits packed as 0x55 bytes
        for (size_t i = 0; i < PREAMBLE_BITS / 8; ++i)
        {
            std::cout << static_cast<char>(0x55);
        }
    }
    else
    {
        // MSK modulate alternating bits
        auto samples = msk_modulator.generate_preamble(PREAMBLE_BITS);
        output_iq(samples);
    }
}

/**
 * Send dead carrier (constant phase)
 */
void send_dead_carrier()
{
    if (config->output_to_network) return;
    
    if (config->verbose) std::cerr << "Sending dead carrier." << std::endl;
    
    if (config->bitstream)
    {
        // All zeros
        for (size_t i = 0; i < PREAMBLE_BITS / 8; ++i)
        {
            std::cout << static_cast<char>(0x00);
        }
    }
    else
    {
        // Constant carrier (no modulation)
        constexpr size_t CARRIER_SAMPLES = PREAMBLE_BITS * OPVMSKModulator::SAMPLES_PER_SYMBOL;
        auto samples = msk_modulator.generate_carrier(CARRIER_SAMPLES);
        output_iq(samples);
    }
}

/**
 * Send end-of-transmission marker
 */
void output_eot()
{
    if (config->verbose) std::cerr << "Sending EOT." << std::endl;
    
    if (config->bitstream)
    {
        for (auto c : EOT_SYNC) std::cout << c;
        for (size_t i = 0; i < 10; ++i) std::cout << '\0';
    }
    else
    {
        // Modulate EOT sync word
        std::array<OPVMSKModulator::IQSample, OPVMSKModulator::SAMPLES_PER_SYMBOL> symbol_samples;
        for (auto byte : EOT_SYNC)
        {
            for (int bit_idx = 7; bit_idx >= 0; --bit_idx)
            {
                uint8_t bit = (byte >> bit_idx) & 1;
                msk_modulator.modulate_bit(bit, symbol_samples);
                output_iq(symbol_samples);
            }
        }
        
        // Flush with carrier
        auto flush = msk_modulator.generate_carrier(OPVMSKModulator::SAMPLES_PER_SYMBOL * 10);
        output_iq(flush);
    }
}


// =============================================================================
// PAYLOAD BUILDING FUNCTIONS
// =============================================================================

void build_rtp_header(uint8_t* frame_buffer)
{
    memcpy(frame_buffer, "RTP_RTP_RTP_", 12);
}

void build_udp_header(uint8_t* frame_buffer, int udp_length)
{
    // OPV Port Assignments (from protocol spec):
    // 57372 = Network Transmitter (Data/BERT)
    // 57373 = Audio (Voice)
    // 57374 = Text
    // 57375 = Control
    const uint16_t src_port = 57373;  // Audio source
    const uint16_t dst_port = 57373;  // Audio destination
    uint8_t udp_header[8] = {
        (uint8_t)(src_port/256), (uint8_t)(src_port%256),
        (uint8_t)(dst_port/256), (uint8_t)(dst_port%256),
        (uint8_t)(udp_length/256), (uint8_t)(udp_length%256),
        0x00, 0x00
    };
    memcpy(frame_buffer, udp_header, 8);
}

void build_ip_header(uint8_t* frame_buffer, int packet_len)
{
    uint8_t ip_header[20] = {
        0x45, 0x00, (uint8_t)(packet_len/256), (uint8_t)(packet_len%256),
        0x00, 0x00, 0x00, 0x00,
        64, 17, 0x00, 0x00,
        192, 168, 0, 1,
        192, 168, 0, 2
    };
    memcpy(frame_buffer, ip_header, 20);
}

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
                        opus_packet_size_bytes);

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
    std::cerr << "Frame Header: " << std::hex << std::setfill('0');
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

    // Status output
    dump_fheader(fh);
    std::cerr << "Pipeline: randomize → K=7 conv → 67×32 interleave → diff encode → "
              << (config->msk ? "MSK" : "bitstream") << std::endl;
    std::cerr << "Frame: " << opv_frame_bytes << " bytes → " << opv_encoded_bits << " bits" << std::endl;
    std::cerr << "Sync word: 0x" << std::hex << opv_sync_word << std::dec << std::endl;
    if (!config->bitstream)
    {
        std::cerr << "Output: I/Q samples, " << OPVMSKModulator::SAMPLES_PER_SYMBOL 
                  << " samples/bit, 16-bit signed, " 
                  << (54200 * OPVMSKModulator::SAMPLES_PER_SYMBOL) << " SPS" << std::endl;
    }

    signal(SIGINT, &signal_handler);

    // Reset modulator state
    msk_modulator.reset();

    send_dead_carrier();
    send_dead_carrier();
    send_preamble();

    if (config->preamble_only)
    {
        running = true;
        std::cerr << "opv-mod sending only preambles (Ctrl+C to stop)" << std::endl;

        while (running)
        {
            send_preamble();
        }
    }
    else if (config->bert)
    {
        PRBS9 prbs;
        running = true;

        if (config->continuous)
        {
            std::cerr << "opv-mod sending continuous BERT frames (Ctrl+C to stop)" << std::endl;
            uint64_t total_frames = 0;
            
            while (running)
            {
                for (uint32_t i = 0; i < config->bert && running; i++)
                {
                    auto payload = fill_bert_frame(prbs);
                    send_stream_frame(fh, payload);
                    total_frames++;
                }
                
                // Brief status every 250 frames (~10 seconds)
                if (total_frames % 250 == 0)
                {
                    std::cerr << "Transmitted " << total_frames << " frames..." << std::endl;
                }
            }
            
            std::cerr << "Output " << total_frames << " frames of BERT data." << std::endl;
        }
        else
        {
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
        }
        
        output_eot();
        send_dead_carrier();
    }
    else
    {
        running = true;
        queue_t queue;
        std::thread thd([&queue, &fh](){transmit(queue, fh);});

        std::cerr << "opv-mod running. Ctrl+D to end." << std::endl;

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

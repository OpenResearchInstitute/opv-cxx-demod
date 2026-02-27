/**
 * opv-modem.cpp - OPV Modem Server
 * 
 * Listens on UDP port for 134-byte OPV frames from Interlocutor,
 * modulates them, and in loopback mode demodulates and returns
 * frames back to Interlocutor.
 * 
 * Key features:
 *   - Transmission session management (preamble, dummy frames, postamble)
 *   - Persistent demodulator subprocess (maintains lock across frames)
 *   - Optional callsign rewrite for loopback repeater testing
 * 
 * Session management matches Dialogus behavior:
 *   - Preamble (40ms, 0xCC pattern) before first frame of each session
 *   - Dummy frames (all-zero payload) to fill gaps and keep carrier alive
 *   - Postamble (Barker-11 pattern) after hang timer expires
 *   - Clean session start/end with continuous IQ output throughout
 * 
 * Usage:
 *   opv-modem [OPTIONS]
 *   
 * Options:
 *   -p PORT     UDP port to listen on (default: 57372)
 *   -l          Loopback mode: mod → demod → return to sender
 *   -t          TX mode: output IQ samples to stdout (for piping to PlutoSDR)
 *   -c CALL     Rewrite station ID on returned frames (loopback repeater)
 *   -d PATH     Path to opv-demod binary (default: ./bin/opv-demod)
 *   -o FILE     Also save IQ samples to file
 *   -n FRAMES   Hang timer frames before postamble (default: 25)
 *   -P FRAMES   Preamble frame periods (default: 1, each 40ms)
 *   -v          Verbose output
 *   -q          Quiet mode
 * 
 * Examples:
 *   opv-modem -l -c REPEAT -r 57373 -v    # Loopback with callsign rewrite
 *   opv-modem -t | iio_writedev ...       # TX to PlutoSDR
 *   opv-modem -t -o capture.iq            # TX to Pluto and save IQ
 * 
 * Copyright 2026 Open Research Institute, Inc.
 * SPDX-License-Identifier: CERN-OHL-S-2.0
 */

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <complex>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <algorithm>
#include <getopt.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/select.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>

// Networking
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// =============================================================================
// PARAMETERS
// =============================================================================

constexpr size_t FRAME_BYTES = 134;
constexpr size_t FRAME_BITS = FRAME_BYTES * 8;
constexpr size_t ENCODED_BITS = FRAME_BITS * 2;

constexpr uint32_t SYNC_WORD = 0x02B8DB;
constexpr size_t SYNC_BITS = 24;
constexpr size_t FRAME_SYMBOLS = SYNC_BITS + ENCODED_BITS;

constexpr size_t SAMPLES_PER_SYMBOL = 40;
constexpr double SAMPLE_RATE = 2168000.0;
constexpr double FREQ_DEV = 13550.0;

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

constexpr double F1_FREQ = -FREQ_DEV;
constexpr double F2_FREQ = +FREQ_DEV;

constexpr uint8_t G1_MASK = 0x4F;
constexpr uint8_t G2_MASK = 0x6D;

// Session management (matches Dialogus behavior)
constexpr size_t OVP_HEADER_SIZE = 12;        // station ID(6) + token(3) + reserved(3)
constexpr int    HANG_TIMER_FRAMES_DEFAULT = 25; // dummy frames before postamble (1000ms)
constexpr int    PREAMBLE_FRAMES_DEFAULT = 1;    // preamble frame periods (40ms each)
constexpr int    FRAME_PERIOD_US = 40000;     // 40ms per frame
constexpr int    IDLE_POLL_TIMEOUT_MS = 100;  // poll timeout when idle

// =============================================================================
// TYPES
// =============================================================================

using frame_t = std::array<uint8_t, FRAME_BYTES>;
using encoded_bits_t = std::array<uint8_t, ENCODED_BITS>;

struct IQSample { int16_t I, Q; };

// =============================================================================
// GLOBAL STATE
// =============================================================================

volatile sig_atomic_t g_running = 1;
std::atomic<uint64_t> g_frames_tx{0};
std::atomic<uint64_t> g_frames_rx{0};

// =============================================================================
// SIGNAL HANDLER
// =============================================================================

void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

// =============================================================================
// BASE-40 ENCODER/DECODER
// =============================================================================

// Decode Base-40 bytes to callsign string
std::string decode_base40(const uint8_t* bytes, size_t len = 6) {
    uint64_t value = 0;
    for (size_t i = 0; i < len; ++i) value = (value << 8) | bytes[i];
    if (value == 0) return "(empty)";
    
    std::string result;
    while (value > 0) {
        int d = value % 40;
        value /= 40;
        char c = (d == 0) ? '\0' : 
                 (d <= 26) ? ('A' + d - 1) :
                 (d <= 36) ? ('0' + d - 27) :
                 (d == 37) ? '-' : (d == 38) ? '/' : '.';
        if (c) result += c;
    }
    return result.empty() ? "(empty)" : result;
}

// Encode callsign string to Base-40 bytes (6 bytes, big-endian)
bool encode_base40(const std::string& callsign, uint8_t* bytes) {
    uint64_t value = 0;
    
    // Process in reverse order (LSB first encoding)
    for (auto it = callsign.rbegin(); it != callsign.rend(); ++it) {
        char c = std::toupper(*it);
        int d;
        
        if (c >= 'A' && c <= 'Z') d = c - 'A' + 1;
        else if (c >= '0' && c <= '9') d = c - '0' + 27;
        else if (c == '-') d = 37;
        else if (c == '/') d = 38;
        else if (c == '.') d = 39;
        else return false;  // Invalid character
        
        value = value * 40 + d;
    }
    
    // Write as big-endian 6 bytes
    for (int i = 5; i >= 0; --i) {
        bytes[i] = value & 0xFF;
        value >>= 8;
    }
    
    return true;
}

// =============================================================================
// CCSDS LFSR RANDOMIZER
// =============================================================================

class LFSR {
public:
    void reset() { state = 0xFF; }
    
    uint8_t next_byte() {
        uint8_t out = 0;
        for (int i = 7; i >= 0; --i) {
            out |= ((state >> 7) & 1) << i;
            uint8_t fb = ((state >> 7) ^ (state >> 6) ^ (state >> 4) ^ (state >> 2)) & 1;
            state = (state << 1) | fb;
        }
        return out;
    }
    
private:
    uint8_t state = 0xFF;
};

// =============================================================================
// CONVOLUTIONAL ENCODER
// =============================================================================

class ConvEncoder {
public:
    void reset() { sr = 0; }
    
    void encode_bit(uint8_t in, uint8_t& g1, uint8_t& g2) {
        uint8_t state = (in << 6) | sr;
        g1 = __builtin_parity(state & G1_MASK);
        g2 = __builtin_parity(state & G2_MASK);
        sr = ((sr << 1) | in) & 0x3F;
    }
    
private:
    uint8_t sr = 0;
};

// =============================================================================
// INTERLEAVER
// =============================================================================

void interleave(encoded_bits_t& bits) {
    encoded_bits_t temp = {};
    for (size_t i = 0; i < ENCODED_BITS; ++i) {
        size_t interleaved_pos = (i % 32) * 67 + (i / 32);
        size_t byte_num = interleaved_pos / 8;
        size_t bit_in_byte = interleaved_pos % 8;
        size_t corrected_pos = byte_num * 8 + (7 - bit_in_byte);
        temp[corrected_pos] = bits[i];
    }
    bits = temp;
}

// =============================================================================
// FRAME ENCODER
// =============================================================================

encoded_bits_t encode_frame(const frame_t& payload) {
    LFSR lfsr; lfsr.reset();
    ConvEncoder conv; conv.reset();
    encoded_bits_t encoded = {};
    size_t out_idx = 0;
    
    std::array<uint8_t, FRAME_BYTES> randomized;
    for (size_t i = 0; i < FRAME_BYTES; ++i) {
        randomized[i] = payload[i] ^ lfsr.next_byte();
    }
    
    for (int byte_idx = FRAME_BYTES - 1; byte_idx >= 0; --byte_idx) {
        uint8_t byte = randomized[byte_idx];
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
            uint8_t g1, g2;
            uint8_t in_bit = (byte >> bit_pos) & 1;
            conv.encode_bit(in_bit, g1, g2);
            encoded[out_idx++] = g1;
            encoded[out_idx++] = g2;
        }
    }
    
    interleave(encoded);
    return encoded;
}

// =============================================================================
// HDL-ACCURATE MSK MODULATOR
// =============================================================================

class HDLModulator {
public:
    void reset() {
        phase_f1 = 0.0;
        phase_f2 = 0.0;
        d_val_xor_T = 0;
        b_n = 1;
    }
    
    void modulate_bit(uint8_t tx_bit, std::array<IQSample, SAMPLES_PER_SYMBOL>& output) {
        int8_t d_val = (tx_bit == 0) ? 1 : -1;
        
        int8_t d_val_xor;
        if (d_val == 1 && d_val_xor_T == 1) d_val_xor = 1;
        else if (d_val == 1 && d_val_xor_T == -1) d_val_xor = -1;
        else if (d_val == -1 && d_val_xor_T == 1) d_val_xor = -1;
        else if (d_val == -1 && d_val_xor_T == -1) d_val_xor = 1;
        else d_val_xor = 1;
        
        int8_t d_pos = (d_val + 1) >> 1;
        int8_t d_neg = (d_val - 1) >> 1;
        int8_t d_pos_enc = d_pos;
        int8_t d_neg_enc = (b_n == 0) ? d_neg : -d_neg;
        
        int8_t d_s1, d_s2;
        if (d_pos_enc == 1 && d_val_xor_T == 1) d_s1 = 1;
        else if (d_pos_enc == 1 && d_val_xor_T == -1) d_s1 = -1;
        else d_s1 = 0;
        
        if (d_neg_enc == -1 && d_val_xor_T == 1) d_s2 = -1;
        else if (d_neg_enc == -1 && d_val_xor_T == -1) d_s2 = 1;
        else if (d_neg_enc == 1 && d_val_xor_T == 1) d_s2 = 1;
        else if (d_neg_enc == 1 && d_val_xor_T == -1) d_s2 = -1;
        else d_s2 = 0;
        
        double phase_inc_f1 = TWO_PI * F1_FREQ / SAMPLE_RATE;
        double phase_inc_f2 = TWO_PI * F2_FREQ / SAMPLE_RATE;
        
        for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
            double sin_f1 = std::sin(phase_f1);
            double cos_f1 = std::cos(phase_f1);
            double sin_f2 = std::sin(phase_f2);
            double cos_f2 = std::cos(phase_f2);
            
            double I = d_s1 * sin_f1 + d_s2 * sin_f2;
            double Q = d_s1 * cos_f1 + d_s2 * cos_f2;
            
            output[i].I = static_cast<int16_t>(16383.0 * I);
            output[i].Q = static_cast<int16_t>(16383.0 * Q);
            
            phase_f1 += phase_inc_f1;
            phase_f2 += phase_inc_f2;
            while (phase_f1 > PI) phase_f1 -= TWO_PI;
            while (phase_f1 < -PI) phase_f1 += TWO_PI;
            while (phase_f2 > PI) phase_f2 -= TWO_PI;
            while (phase_f2 < -PI) phase_f2 += TWO_PI;
        }
        
        d_val_xor_T = d_val_xor;
        b_n = 1 - b_n;
    }
    
private:
    double phase_f1 = 0.0;
    double phase_f2 = 0.0;
    int8_t d_val_xor_T = 0;
    int8_t b_n = 1;
};

// =============================================================================
// MODULATE FRAME TO IQ SAMPLES
// =============================================================================

void modulate_frame(const frame_t& frame, HDLModulator& mod, std::vector<IQSample>& iq_out) {
    encoded_bits_t encoded = encode_frame(frame);
    std::array<IQSample, SAMPLES_PER_SYMBOL> sym_samples;
    
    // Sync word
    for (int i = SYNC_BITS - 1; i >= 0; --i) {
        mod.modulate_bit((SYNC_WORD >> i) & 1, sym_samples);
        for (const auto& s : sym_samples) iq_out.push_back(s);
    }
    
    // Encoded data
    for (size_t i = 0; i < ENCODED_BITS; ++i) {
        mod.modulate_bit(encoded[i], sym_samples);
        for (const auto& s : sym_samples) iq_out.push_back(s);
    }
}

// =============================================================================
// WALL-CLOCK TIMING
// =============================================================================

// Forward declaration for IQOutputContext
class PersistentDemodulator;

static int64_t get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

// =============================================================================
// PREAMBLE GENERATION
// =============================================================================
//
// The preamble is a raw 1100 repeating pattern (0xCC) fed directly to the
// modulator for one full frame period (40ms). No sync word, no FEC, no
// randomization. This gives the receiving demodulator a clean signal to
// acquire frequency, phase, and symbol timing before the first real frame.
// Matches Dialogus start_transmission_session() behavior.

void emit_preamble(HDLModulator& mod, std::vector<IQSample>& iq_out) {
    static const uint8_t pattern[] = {1, 1, 0, 0};
    std::array<IQSample, SAMPLES_PER_SYMBOL> sym_samples;
    for (size_t i = 0; i < FRAME_SYMBOLS; ++i) {
        mod.modulate_bit(pattern[i % 4], sym_samples);
        for (const auto& s : sym_samples) iq_out.push_back(s);
    }
}

// =============================================================================
// DUMMY FRAME GENERATION
// =============================================================================
//
// A dummy frame preserves the last real frame's header (station ID, token,
// reserved fields) but replaces the payload with all zeros. This goes through
// the normal encode pipeline (randomize → FEC → interleave → sync word) so
// the receiver sees a valid frame. The all-zero payload acts as a COBS
// delimiter on the receive side — it terminates any in-progress reassembly
// and produces zero-length packets that are silently dropped.
//
// The purpose is to keep the RF carrier alive during pauses, maintaining
// the demodulator's tracking loops. Matches Dialogus create_dummy_frame().

frame_t make_dummy_frame(const frame_t& last_real) {
    frame_t dummy = last_real;
    std::fill(dummy.begin() + OVP_HEADER_SIZE, dummy.end(), 0x00);
    return dummy;
}

// =============================================================================
// POSTAMBLE FRAME GENERATION
// =============================================================================
//
// The postamble signals the end of a transmission session. Its payload is
// filled with a repeating 11-bit Barker code (11100010010), which is a
// distinctive pattern the receiver can use to identify end-of-transmission.
// The last byte is forced to 0x00 as a COBS delimiter, flushing any partial
// packet data from the receiver's reassembly buffer.
//
// Like dummy frames, the postamble preserves the last real frame's header
// and goes through the normal encode pipeline.
// Matches Dialogus create_postamble_frame() / create_postamble_logical_frame().

// Precomputed from Dialogus: Barker-11 (11100010010) packed into bytes,
// repeating across the 11-byte period. See dialogus.c line 321-325.
static const uint8_t barker_11_pattern[] = {
    0xE2, 0x5C, 0x4B, 0x89, 0x71, 0x2E,
    0x25, 0xC4, 0xB8, 0x97, 0x12
};

frame_t make_postamble_frame(const frame_t& last_real) {
    frame_t post = last_real;
    for (size_t i = OVP_HEADER_SIZE; i < FRAME_BYTES - 1; ++i) {
        post[i] = barker_11_pattern[(i - OVP_HEADER_SIZE) % sizeof(barker_11_pattern)];
    }
    post[FRAME_BYTES - 1] = 0x00;  // COBS delimiter
    return post;
}

// =============================================================================
// TX SESSION STATE MACHINE
// =============================================================================
//
// Manages the lifecycle of a transmission session:
//   IDLE → (first frame) → PREAMBLE + ACTIVE → (gap) → HANGING → POSTAMBLE → IDLE
//
// This mirrors the Dialogus architecture where the modem layer autonomously
// handles preamble/dummy/postamble framing. Interlocutor just sends real OV
// frames over UDP and doesn't need to know about session management.

struct TxSession {
    enum State { IDLE, ACTIVE, HANGING };

    State state = IDLE;
    frame_t last_real_frame = {};     // most recent real frame (for dummy/postamble header)
    bool have_real_frame = false;     // at least one real frame seen in this session
    int dummy_count = 0;              // consecutive dummy frames sent in current hang
    int hang_timer_frames = HANG_TIMER_FRAMES_DEFAULT;
    int64_t next_deadline_us = 0;     // wall-clock time when next frame slot arrives

    // Statistics for this session
    uint64_t session_real_frames = 0;
    uint64_t session_dummy_frames = 0;
    int64_t session_start_us = 0;

    bool is_idle() const { return state == IDLE; }

    void start(int64_t now_us) {
        state = ACTIVE;
        dummy_count = 0;
        session_real_frames = 0;
        session_dummy_frames = 0;
        session_start_us = now_us;
        // First real frame was just emitted (preceded by preamble).
        // Next frame slot is 40ms from now.
        next_deadline_us = now_us + FRAME_PERIOD_US;
    }

    void real_frame_sent(const frame_t& frame) {
        last_real_frame = frame;
        have_real_frame = true;
        session_real_frames++;
        // If we were hanging (sending dummies), a real frame cancels it
        dummy_count = 0;
        state = ACTIVE;
    }

    void advance_deadline() {
        next_deadline_us += FRAME_PERIOD_US;
    }

    // Returns poll timeout in milliseconds, clamped to [0, IDLE_POLL_TIMEOUT_MS]
    int poll_timeout_ms(int64_t now_us) const {
        if (state == IDLE) return IDLE_POLL_TIMEOUT_MS;
        int64_t remaining_us = next_deadline_us - now_us;
        if (remaining_us <= 0) return 0;
        int ms = (int)(remaining_us / 1000);
        return std::min(ms, IDLE_POLL_TIMEOUT_MS);
    }

    bool deadline_expired(int64_t now_us) const {
        return state != IDLE && now_us >= next_deadline_us;
    }

    void reset() {
        state = IDLE;
        dummy_count = 0;
        have_real_frame = false;
        next_deadline_us = 0;
        session_real_frames = 0;
        session_dummy_frames = 0;
        session_start_us = 0;
    }
};

// =============================================================================
// IQ OUTPUT HELPER
// =============================================================================
//
// Common output path for all frame types (preamble, real, dummy, postamble).
// Writes to stdout (PlutoSDR), IQ file, and/or loopback demod as configured.

struct IQOutputContext {
    bool tx_mode;
    std::ofstream& iq_out;
    PersistentDemodulator* demod;
    bool loopback;
};

// Defined after PersistentDemodulator (needs full class definition for write_iq)
void output_iq(const std::vector<IQSample>& iq, const IQOutputContext& ctx);
void modulate_and_output(const frame_t& frame, HDLModulator& mod, const IQOutputContext& ctx);

// =============================================================================
// PERSISTENT DEMODULATOR SUBPROCESS
// =============================================================================

class PersistentDemodulator {
public:
    PersistentDemodulator(const std::string& demod_path, bool verbose)
        : demod_path_(demod_path), verbose_(verbose), pid_(-1),
          pipe_to_demod_{-1, -1}, pipe_from_demod_{-1, -1} {}
    
    ~PersistentDemodulator() {
        stop();
    }
    
    bool start() {
        if (pipe(pipe_to_demod_) < 0 || pipe(pipe_from_demod_) < 0) {
            std::cerr << "Failed to create pipes\n";
            return false;
        }
        
        pid_ = fork();
        
        if (pid_ < 0) {
            std::cerr << "Fork failed\n";
            return false;
        }
        
        if (pid_ == 0) {
            // Child process - run opv-demod
            close(pipe_to_demod_[1]);   // Close write end of input pipe
            close(pipe_from_demod_[0]); // Close read end of output pipe
            
            dup2(pipe_to_demod_[0], STDIN_FILENO);
            close(pipe_to_demod_[0]);
            
            dup2(pipe_from_demod_[1], STDOUT_FILENO);
            close(pipe_from_demod_[1]);
            
            // Keep stderr for status messages, redirect if not verbose
            if (!verbose_) {
                int devnull = open("/dev/null", O_WRONLY);
                dup2(devnull, STDERR_FILENO);
                close(devnull);
            }
            
            // Execute opv-demod with streaming and raw output
            execlp(demod_path_.c_str(), demod_path_.c_str(), "-s", "-r", nullptr);
            
            std::cerr << "Failed to exec " << demod_path_ << "\n";
            _exit(1);
        }
        
        // Parent process
        close(pipe_to_demod_[0]);   // Close read end
        close(pipe_from_demod_[1]); // Close write end
        
        // Make the read end non-blocking
        int flags = fcntl(pipe_from_demod_[0], F_GETFL, 0);
        fcntl(pipe_from_demod_[0], F_SETFL, flags | O_NONBLOCK);
        
        return true;
    }
    
    void stop() {
        if (pid_ > 0) {
            // Close write pipe to signal EOF
            if (pipe_to_demod_[1] >= 0) {
                close(pipe_to_demod_[1]);
                pipe_to_demod_[1] = -1;
            }
            
            // Wait briefly for child to exit
            int status;
            waitpid(pid_, &status, WNOHANG);
            
            // Kill if still running
            kill(pid_, SIGTERM);
            waitpid(pid_, &status, 0);
            pid_ = -1;
        }
        
        if (pipe_from_demod_[0] >= 0) {
            close(pipe_from_demod_[0]);
            pipe_from_demod_[0] = -1;
        }
    }
    
    // Write IQ samples to demodulator (non-blocking)
    bool write_iq(const std::vector<IQSample>& samples) {
        if (pipe_to_demod_[1] < 0) return false;
        
        size_t total = samples.size() * sizeof(IQSample);
        ssize_t written = write(pipe_to_demod_[1], samples.data(), total);
        
        return written == static_cast<ssize_t>(total);
    }
    
    // Try to read a frame from demodulator (non-blocking)
    // Returns true if a complete frame was read
    bool try_read_frame(frame_t& frame) {
        if (pipe_from_demod_[0] < 0) return false;
        
        // Try to read into buffer
        uint8_t buf[256];
        ssize_t n = read(pipe_from_demod_[0], buf, sizeof(buf));
        
        if (n > 0) {
            // Add to internal buffer
            for (ssize_t i = 0; i < n; ++i) {
                frame_buffer_.push_back(buf[i]);
            }
        }
        
        // Check if we have a complete frame
        if (frame_buffer_.size() >= FRAME_BYTES) {
            std::copy(frame_buffer_.begin(), frame_buffer_.begin() + FRAME_BYTES, frame.begin());
            frame_buffer_.erase(frame_buffer_.begin(), frame_buffer_.begin() + FRAME_BYTES);
            return true;
        }
        
        return false;
    }
    
    int read_fd() const { return pipe_from_demod_[0]; }
    
private:
    std::string demod_path_;
    bool verbose_;
    pid_t pid_;
    int pipe_to_demod_[2];
    int pipe_from_demod_[2];
    std::vector<uint8_t> frame_buffer_;
};

// =============================================================================
// IQ OUTPUT FUNCTIONS (deferred - need full PersistentDemodulator definition)
// =============================================================================

void output_iq(const std::vector<IQSample>& iq, const IQOutputContext& ctx) {
    if (ctx.iq_out.is_open()) {
        ctx.iq_out.write(reinterpret_cast<const char*>(iq.data()),
                         iq.size() * sizeof(IQSample));
    }
    if (ctx.tx_mode) {
        std::cout.write(reinterpret_cast<const char*>(iq.data()),
                        iq.size() * sizeof(IQSample));
        std::cout.flush();
    }
    if (ctx.loopback && ctx.demod) {
        ctx.demod->write_iq(iq);
    }
}

void modulate_and_output(const frame_t& frame, HDLModulator& mod, const IQOutputContext& ctx) {
    std::vector<IQSample> iq;
    iq.reserve(FRAME_SYMBOLS * SAMPLES_PER_SYMBOL);
    modulate_frame(frame, mod, iq);
    output_iq(iq, ctx);
}

// =============================================================================
// UDP SERVER
// =============================================================================

class UDPServer {
public:
    UDPServer(int port) : port_(port), sockfd_(-1) {}
    
    ~UDPServer() {
        if (sockfd_ >= 0) close(sockfd_);
    }
    
    bool start() {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            std::cerr << "Error creating socket\n";
            return false;
        }
        
        int opt = 1;
        setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);
        
        if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Error binding to port " << port_ << "\n";
            return false;
        }
        
        // Make non-blocking
        int flags = fcntl(sockfd_, F_GETFL, 0);
        fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);
        
        return true;
    }
    
    bool receive(frame_t& frame, struct sockaddr_in& sender) {
        socklen_t sender_len = sizeof(sender);
        ssize_t n = recvfrom(sockfd_, frame.data(), FRAME_BYTES, 0,
                             (struct sockaddr*)&sender, &sender_len);
        return n == FRAME_BYTES;
    }
    
    bool send(const frame_t& frame, const struct sockaddr_in& dest) {
        ssize_t n = sendto(sockfd_, frame.data(), FRAME_BYTES, 0,
                           (const struct sockaddr*)&dest, sizeof(dest));
        return n == FRAME_BYTES;
    }
    
    int fd() const { return sockfd_; }
    
private:
    int port_;
    int sockfd_;
};

// =============================================================================
// MAIN
// =============================================================================

void usage(const char* prog) {
    std::cerr << "Usage: " << prog << " [OPTIONS]\n\n";
    std::cerr << "OPV Modem Server - Interlocutor integration\n\n";
    std::cerr << "Options:\n";
    std::cerr << "  -p PORT     UDP port to listen on (default: 57372)\n";
    std::cerr << "  -r PORT     UDP port to send to (RX mode) or respond to (loopback)\n";
    std::cerr << "  -l          Loopback mode: mod → demod → return to sender\n";
    std::cerr << "  -t          TX mode: output IQ samples to stdout (for PlutoSDR)\n";
    std::cerr << "  -R          RX mode: read IQ from stdin, send frames to UDP\n";
    std::cerr << "  -c CALL     Rewrite callsign on returned frames (loopback repeater)\n";
    std::cerr << "  -d PATH     Path to opv-demod binary (default: ./bin/opv-demod)\n";
    std::cerr << "  -o FILE     Save IQ to file\n";
    std::cerr << "  -n FRAMES   Hang timer frames before postamble (default: 25)\n";
    std::cerr << "  -P FRAMES   Preamble frame periods (default: 1, each 40ms)\n";
    std::cerr << "  -v          Verbose\n";
    std::cerr << "  -q          Quiet\n";
    std::cerr << "  -h          Help\n";
    std::cerr << "\n";
    std::cerr << "Examples:\n";
    std::cerr << "  " << prog << " -l                        # Loopback, pass-through\n";
    std::cerr << "  " << prog << " -l -c REPEAT              # Loopback repeater\n";
    std::cerr << "  " << prog << " -t | iio_writedev ...     # TX to PlutoSDR\n";
    std::cerr << "  iio_readdev ... | " << prog << " -R -r 57373  # RX from PlutoSDR\n";
    exit(1);
}

int main(int argc, char* argv[]) {
    int port = 57372;
    int response_port = 0;  // 0 means use sender's port
    bool loopback = false;
    bool tx_mode = false;   // Output IQ to stdout for PlutoSDR
    bool rx_mode = false;   // Read IQ from stdin, send frames via UDP
    bool verbose = false;
    bool quiet = false;
    std::string iq_file;
    std::string demod_path = "./bin/opv-demod";
    std::string rewrite_callsign;
    uint8_t rewrite_callsign_bytes[6] = {};
    bool do_rewrite = false;
    int hang_timer_frames = HANG_TIMER_FRAMES_DEFAULT;
    int preamble_frames = PREAMBLE_FRAMES_DEFAULT;
    
    int opt;
    while ((opt = getopt(argc, argv, "p:r:ltRc:d:o:n:P:vqh")) != -1) {
        switch (opt) {
            case 'p': port = std::atoi(optarg); break;
            case 'r': response_port = std::atoi(optarg); break;
            case 'l': loopback = true; break;
            case 't': tx_mode = true; break;
            case 'R': rx_mode = true; break;
            case 'c': rewrite_callsign = optarg; break;
            case 'd': demod_path = optarg; break;
            case 'o': iq_file = optarg; break;
            case 'n': hang_timer_frames = std::atoi(optarg); break;
            case 'P': preamble_frames = std::atoi(optarg); break;
            case 'v': verbose = true; break;
            case 'q': quiet = true; break;
            default: usage(argv[0]);
        }
    }
    
    // Validate mode combinations
    int mode_count = (loopback ? 1 : 0) + (tx_mode ? 1 : 0) + (rx_mode ? 1 : 0);
    if (mode_count > 1) {
        std::cerr << "Error: Cannot combine -l, -t, and -R modes\n";
        return 1;
    }
    
    // RX mode requires response port
    if (rx_mode && response_port == 0) {
        response_port = 57373;  // Default for Interlocutor
    }
    
    // Set stdout to binary mode for TX (important for IQ data)
    if (tx_mode) {
        std::ios_base::sync_with_stdio(false);
        // Set a large buffer for stdout to improve throughput
        static char stdout_buffer[FRAME_SYMBOLS * SAMPLES_PER_SYMBOL * sizeof(IQSample)];
        std::setvbuf(stdout, stdout_buffer, _IOFBF, sizeof(stdout_buffer));
    }
    
    // Validate and encode rewrite callsign
    if (!rewrite_callsign.empty()) {
        if (!encode_base40(rewrite_callsign, rewrite_callsign_bytes)) {
            std::cerr << "Error: Invalid callsign '" << rewrite_callsign << "'\n";
            std::cerr << "Use A-Z, 0-9, -, /, . only\n";
            return 1;
        }
        do_rewrite = true;
    }
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);  // Ignore broken pipe
    
    if (!quiet) {
        std::cerr << "╔═══════════════════════════════════════════════════════════════════╗\n";
        std::cerr << "║                    OPV Modem Server v2.0                          ║\n";
        std::cerr << "╚═══════════════════════════════════════════════════════════════════╝\n\n";
        if (rx_mode) {
            std::cerr << "  Mode:      RX (stdin → demod → UDP)\n";
            std::cerr << "  Demod:     " << demod_path << "\n";
            std::cerr << "  Send to:   127.0.0.1:" << response_port << "\n";
        } else {
            std::cerr << "  Port:      " << port << "\n";
            if (loopback) {
                std::cerr << "  Mode:      Loopback (mod→demod→return)\n";
                std::cerr << "  Demod:     " << demod_path << "\n";
                if (do_rewrite) {
                    std::cerr << "  Repeater:  " << rewrite_callsign << " (callsign rewrite)\n";
                }
                if (response_port > 0) {
                    std::cerr << "  Reply to:  port " << response_port << "\n";
                }
            } else if (tx_mode) {
                std::cerr << "  Mode:      TX (IQ → stdout for PlutoSDR)\n";
            } else {
                std::cerr << "  Mode:      Monitor only\n";
            }
        }
        if (!iq_file.empty())
            std::cerr << "  IQ File:   " << iq_file << "\n";
        std::cerr << "  Session:   preamble(" << preamble_frames << "×40ms) + hang(" << hang_timer_frames << " frames) + postamble\n";
        std::cerr << "\n";
    }
    
    // Verify demod exists if needed
    if (loopback || rx_mode) {
        if (access(demod_path.c_str(), X_OK) != 0) {
            std::cerr << "Error: Cannot execute " << demod_path << "\n";
            std::cerr << "Use -d to specify path to opv-demod\n";
            return 1;
        }
    }
    
    // =========================================================================
    // RX MODE - separate path (stdin → demod → UDP)
    // =========================================================================
    if (rx_mode) {
        // Create UDP socket for sending
        int tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (tx_sock < 0) {
            std::cerr << "Error creating UDP socket\n";
            return 1;
        }
        
        struct sockaddr_in dest_addr = {};
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(response_port);
        dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        
        // Create pipes: stdin→demod and demod→us
        int pipe_to_demod[2];    // We write IQ, demod reads
        int pipe_from_demod[2];  // Demod writes frames, we read
        
        if (pipe(pipe_to_demod) < 0 || pipe(pipe_from_demod) < 0) {
            std::cerr << "Error creating pipes\n";
            close(tx_sock);
            return 1;
        }
        
        pid_t pid = fork();
        if (pid < 0) {
            std::cerr << "Error forking\n";
            close(tx_sock);
            return 1;
        }
        
        if (pid == 0) {
            // Child: run opv-demod
            close(pipe_to_demod[1]);   // Close write end
            close(pipe_from_demod[0]); // Close read end
            
            dup2(pipe_to_demod[0], STDIN_FILENO);
            dup2(pipe_from_demod[1], STDOUT_FILENO);
            
            close(pipe_to_demod[0]);
            close(pipe_from_demod[1]);
            
            execlp(demod_path.c_str(), demod_path.c_str(), "-s", "-r", nullptr);
            std::cerr << "Error executing " << demod_path << "\n";
            _exit(1);
        }
        
        // Parent
        close(pipe_to_demod[0]);   // Close read end
        close(pipe_from_demod[1]); // Close write end
        
        int write_fd = pipe_to_demod[1];
        int read_fd = pipe_from_demod[0];
        
        // Make read_fd non-blocking for polling, but we'll use select
        fcntl(read_fd, F_SETFL, O_NONBLOCK);
        
        if (!quiet) {
            std::cerr << "✓ Receiving from stdin...\n\n";
        }
        
        // Buffer for reading stdin
        std::array<char, 16384> iq_buffer;
        frame_t frame;
        size_t frame_offset = 0;  // Partial frame buffer
        
        while (g_running) {
            // Check if stdin has data or EOF
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(STDIN_FILENO, &read_fds);
            FD_SET(read_fd, &read_fds);
            int max_fd = std::max(STDIN_FILENO, read_fd);
            
            struct timeval tv = {0, 10000};  // 10ms timeout
            int sel = select(max_fd + 1, &read_fds, nullptr, nullptr, &tv);
            
            if (sel < 0) break;
            
            // Read IQ from stdin and forward to demod
            if (FD_ISSET(STDIN_FILENO, &read_fds)) {
                ssize_t n_in = read(STDIN_FILENO, iq_buffer.data(), iq_buffer.size());
                if (n_in > 0) {
                    ssize_t written = write(write_fd, iq_buffer.data(), n_in);
                    (void)written;
                } else if (n_in == 0) {
                    // EOF on stdin - close write pipe to signal demod
                    close(write_fd);
                    write_fd = -1;
                }
            }
            
            // Check for decoded frames from demod
            if (FD_ISSET(read_fd, &read_fds)) {
                // Read into partial frame buffer
                ssize_t n = read(read_fd, frame.data() + frame_offset, FRAME_BYTES - frame_offset);
                if (n > 0) {
                    frame_offset += n;
                    if (frame_offset == FRAME_BYTES) {
                        g_frames_rx++;
                        
                        std::string station_id = decode_base40(&frame[0], 6);
                        uint32_t token = (frame[6] << 16) | (frame[7] << 8) | frame[8];
                        
                        if (verbose) {
                            std::cerr << "RX " << g_frames_rx << ": " << station_id 
                                      << " [0x" << std::hex << token << std::dec << "]\n";
                        }
                        
                        // Send to Interlocutor
                        sendto(tx_sock, frame.data(), FRAME_BYTES, 0,
                               (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                        
                        frame_offset = 0;
                    }
                } else if (n == 0) {
                    // EOF from demod
                    break;
                }
            }
            
            // If stdin is closed and no more data from demod, exit
            if (write_fd < 0 && !FD_ISSET(read_fd, &read_fds) && sel == 0) {
                // Give demod a bit more time to finish
                usleep(100000);  // 100ms
                // Try one more read
                while (true) {
                    ssize_t n = read(read_fd, frame.data() + frame_offset, FRAME_BYTES - frame_offset);
                    if (n > 0) {
                        frame_offset += n;
                        if (frame_offset == FRAME_BYTES) {
                            g_frames_rx++;
                            std::string station_id = decode_base40(&frame[0], 6);
                            uint32_t token = (frame[6] << 16) | (frame[7] << 8) | frame[8];
                            if (verbose) {
                                std::cerr << "RX " << g_frames_rx << ": " << station_id 
                                          << " [0x" << std::hex << token << std::dec << "]\n";
                            }
                            sendto(tx_sock, frame.data(), FRAME_BYTES, 0,
                                   (struct sockaddr*)&dest_addr, sizeof(dest_addr));
                            frame_offset = 0;
                        }
                    } else {
                        break;
                    }
                }
                break;
            }
        }
        
        // Clean up
        close(write_fd);
        close(read_fd);
        close(tx_sock);
        
        // Wait for child
        int status;
        waitpid(pid, &status, 0);
        
        if (!quiet) {
            std::cerr << "\n═══════════════════════════════════════════════════════════════════\n";
            std::cerr << "Summary:\n";
            std::cerr << "  RX:  " << g_frames_rx << " frames\n";
            std::cerr << "═══════════════════════════════════════════════════════════════════\n";
        }
        return 0;
    }
    
    // =========================================================================
    // TX / LOOPBACK / MONITOR MODES (UDP server)
    // =========================================================================
    
    UDPServer server(port);
    if (!server.start()) {
        return 1;
    }
    
    // Start persistent demodulator if loopback mode
    std::unique_ptr<PersistentDemodulator> demod;
    if (loopback) {
        demod = std::make_unique<PersistentDemodulator>(demod_path, verbose);
        if (!demod->start()) {
            std::cerr << "Failed to start demodulator\n";
            return 1;
        }
    }
    
    if (!quiet) {
        std::cerr << "✓ Listening on UDP port " << port << "...\n\n";
    }
    
    std::ofstream iq_out;
    if (!iq_file.empty()) {
        iq_out.open(iq_file, std::ios::binary);
    }
    
    HDLModulator modulator;
    modulator.reset();
    
    // Track sender for return path
    struct sockaddr_in last_sender = {};
    bool have_sender = false;
    
    // Session management
    TxSession session;
    session.hang_timer_frames = hang_timer_frames;
    uint64_t total_sessions = 0;
    uint64_t total_dummy_frames = 0;
    
    // IQ output context (shared by preamble, real, dummy, and postamble)
    IQOutputContext iq_ctx{tx_mode, iq_out, demod.get(), loopback};
    
    while (g_running) {
        int64_t now = get_time_us();
        
        // Set up poll for both UDP and demod output
        struct pollfd fds[2];
        int nfds = 1;
        
        fds[0].fd = server.fd();
        fds[0].events = POLLIN;
        
        if (loopback && demod) {
            fds[1].fd = demod->read_fd();
            fds[1].events = POLLIN;
            nfds = 2;
        }
        
        // Adaptive timeout: tight during active session, relaxed when idle
        int timeout_ms = session.poll_timeout_ms(now);
        int ret = poll(fds, nfds, timeout_ms);
        
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        
        now = get_time_us();  // refresh after poll
        
        // =================================================================
        // HANDLE INCOMING UDP FRAMES (real content from Interlocutor)
        // =================================================================
        if (fds[0].revents & POLLIN) {
            frame_t frame;
            struct sockaddr_in sender;
            
            if (server.receive(frame, sender)) {
                g_frames_tx++;
                last_sender = sender;
                have_sender = true;
                
                std::string station_id = decode_base40(&frame[0], 6);
                uint32_t token = (frame[6] << 16) | (frame[7] << 8) | frame[8];
                
                if (verbose) {
                    char sender_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &sender.sin_addr, sender_ip, sizeof(sender_ip));
                    std::cerr << "TX " << g_frames_tx << ": " << station_id 
                              << " [0x" << std::hex << token << std::dec << "] from "
                              << sender_ip << ":" << ntohs(sender.sin_port) << "\n";
                }
                
                // --- Session start: emit preamble before first frame ---
                if (session.is_idle()) {
                    if (verbose) {
                        std::cerr << "SESSION: Starting transmission for " << station_id << "\n";
                    }
                    
                    // Emit preamble: raw 1100 pattern, no sync word or FEC
                    // Each frame period is 40ms of carrier for demod acquisition
                    for (int p = 0; p < preamble_frames; ++p) {
                        std::vector<IQSample> preamble_iq;
                        preamble_iq.reserve(FRAME_SYMBOLS * SAMPLES_PER_SYMBOL);
                        emit_preamble(modulator, preamble_iq);
                        output_iq(preamble_iq, iq_ctx);
                    }
                    
                    if (verbose) {
                        std::cerr << "SESSION: Preamble sent (" << preamble_frames
                                  << "×" << FRAME_SYMBOLS << " symbols, "
                                  << preamble_frames * 40 << "ms)\n";
                    }
                    
                    session.start(get_time_us());
                    total_sessions++;
                }
                
                // --- Modulate and output the real frame ---
                modulate_and_output(frame, modulator, iq_ctx);
                session.real_frame_sent(frame);
                
                // Advance the deadline for the next frame slot.
                // If we were hanging and just got a real frame, we take over
                // the current slot and schedule the next one.
                session.next_deadline_us = get_time_us() + FRAME_PERIOD_US;
            }
        }
        
        // =================================================================
        // HANDLE FRAME DEADLINE EXPIRY (no real frame arrived in time)
        // =================================================================
        if (session.deadline_expired(now)) {
            
            if (!session.have_real_frame) {
                // Edge case: session started but we lost the frame somehow.
                // This shouldn't happen, but recover gracefully.
                session.reset();
            
            } else if (session.dummy_count >= session.hang_timer_frames) {
                // Hang time exhausted — send postamble and end session
                frame_t postamble = make_postamble_frame(session.last_real_frame);
                modulate_and_output(postamble, modulator, iq_ctx);
                
                if (verbose) {
                    double duration_s = (double)(get_time_us() - session.session_start_us) / 1e6;
                    std::cerr << "SESSION: Postamble sent — ending transmission\n";
                    std::cerr << "SESSION: Session ended after " 
                              << std::fixed << std::setprecision(1) << duration_s << "s ("
                              << session.session_real_frames << " real + "
                              << session.session_dummy_frames << " dummy + 1 postamble)\n";
                }
                
                total_dummy_frames += session.session_dummy_frames;
                session.reset();
            
            } else {
                // Still hanging — send a dummy frame to keep the carrier alive
                frame_t dummy = make_dummy_frame(session.last_real_frame);
                modulate_and_output(dummy, modulator, iq_ctx);
                
                session.dummy_count++;
                session.session_dummy_frames++;
                session.state = TxSession::HANGING;
                session.advance_deadline();
                
                if (verbose) {
                    std::cerr << "SESSION: Dummy frame " << session.dummy_count
                              << "/" << session.hang_timer_frames << "\n";
                }
            }
        }
        
        // =================================================================
        // HANDLE DECODED FRAMES FROM DEMODULATOR (loopback / repeater)
        // =================================================================
        if (loopback && demod && nfds > 1 && (fds[1].revents & POLLIN)) {
            frame_t decoded;
            while (demod->try_read_frame(decoded)) {
                g_frames_rx++;
                
                std::string orig_station = decode_base40(&decoded[0], 6);
                
                // Skip frames that already have our callsign (prevents feedback loops)
                if (do_rewrite && std::memcmp(&decoded[0], rewrite_callsign_bytes, 6) == 0) {
                    if (verbose) {
                        std::cerr << "SKIP " << g_frames_rx << ": already " << rewrite_callsign << "\n";
                    }
                    continue;
                }
                
                // Rewrite callsign if requested
                if (do_rewrite) {
                    std::memcpy(&decoded[0], rewrite_callsign_bytes, 6);
                }
                
                std::string new_station = decode_base40(&decoded[0], 6);
                
                if (verbose) {
                    if (do_rewrite) {
                        std::cerr << "RX " << g_frames_rx << ": " << orig_station 
                                  << " → " << new_station << "\n";
                    } else {
                        std::cerr << "RX " << g_frames_rx << ": " << new_station << "\n";
                    }
                }
                
                // Send back to Interlocutor
                if (have_sender) {
                    struct sockaddr_in dest = last_sender;
                    if (response_port > 0) {
                        dest.sin_port = htons(response_port);
                    }
                    server.send(decoded, dest);
                }
            }
        }
    }
    
    // If we're in a session when shutting down, send postamble
    if (!session.is_idle() && session.have_real_frame) {
        if (verbose) {
            std::cerr << "SESSION: Sending postamble on shutdown\n";
        }
        frame_t postamble = make_postamble_frame(session.last_real_frame);
        modulate_and_output(postamble, modulator, iq_ctx);
    }
    
    // Trailing zeros for file
    if (iq_out.is_open()) {
        std::array<IQSample, SAMPLES_PER_SYMBOL> zeros = {};
        for (int i = 0; i < 100; ++i) {
            iq_out.write(reinterpret_cast<char*>(zeros.data()),
                        zeros.size() * sizeof(IQSample));
        }
        iq_out.close();
    }
    
    if (!quiet) {
        std::cerr << "\n═══════════════════════════════════════════════════════════════════\n";
        std::cerr << "Summary:\n";
        std::cerr << "  TX:       " << g_frames_tx << " real frames\n";
        std::cerr << "  Dummy:    " << total_dummy_frames << " frames\n";
        std::cerr << "  Sessions: " << total_sessions << "\n";
        if (loopback)
            std::cerr << "  RX:       " << g_frames_rx << " frames\n";
        std::cerr << "═══════════════════════════════════════════════════════════════════\n";
    }
    
    return 0;
}

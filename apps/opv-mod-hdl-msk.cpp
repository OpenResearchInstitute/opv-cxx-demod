// opv-mod-hdl-msk.cpp - OPV modulator with HDL-matched MSK
// Uses dual-NCO staggered QPSK approach matching msk_modulator.vhd
//
// Build: g++ -O2 -o opv-mod-hdl-msk opv-mod-hdl-msk.cpp -lm
// Usage: ./opv-mod-hdl-msk -S CALLSIGN -B FRAMES

#include <iostream>
#include <array>
#include <vector>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <getopt.h>

// =============================================================================
// CONSTANTS
// =============================================================================

static constexpr size_t OPV_FRAME_BYTES = 134;
static constexpr size_t OPV_HEADER_BYTES = 12;
static constexpr size_t OPV_PAYLOAD_BYTES = 122;
static constexpr size_t OPV_ENCODED_BITS = 2144;
static constexpr size_t SAMPLES_PER_SYMBOL = 40;

// Sync word: 0x02B8DB
static constexpr std::array<uint8_t, 24> SYNC_BITS = {
    0,0,0,0,0,0,1,0,  // 0x02
    1,0,1,1,1,0,0,0,  // 0xB8
    1,1,0,1,1,0,1,1   // 0xDB
};

// LFSR table (first 255 bytes, then repeats)
static constexpr std::array<uint8_t, 255> LFSR_TABLE = {{
    0xff, 0x1a, 0xaf, 0x66, 0x52, 0x23, 0x1e, 0x10, 0xa0, 0xf9, 0xfa, 0x8a, 0x98, 0x67, 0x7d, 0xd2,
    0xb4, 0xe6, 0xc5, 0xdb, 0xcb, 0x6b, 0x92, 0x68, 0xe2, 0x7a, 0x1d, 0x60, 0xb2, 0x06, 0xe0, 0x25,
    0xfe, 0x35, 0x5e, 0xcc, 0xa4, 0x46, 0x3c, 0x21, 0x41, 0xf3, 0xf5, 0x15, 0x30, 0xce, 0xfb, 0xa5,
    0x69, 0xcd, 0x8b, 0xb7, 0x96, 0xd7, 0x24, 0xd1, 0xc4, 0xf4, 0x3a, 0xc1, 0x64, 0x0d, 0xc0, 0x4b,
    0xfc, 0x6a, 0xbd, 0x99, 0x48, 0x8c, 0x78, 0x42, 0x83, 0xe7, 0xea, 0x2a, 0x61, 0x9d, 0xf7, 0x4a,
    0xd3, 0x9b, 0x17, 0x6f, 0x2d, 0xae, 0x49, 0xa3, 0x89, 0xe8, 0x75, 0x82, 0xc8, 0x1b, 0x80, 0x97,
    0xf8, 0xd5, 0x7b, 0x32, 0x91, 0x18, 0xf0, 0x85, 0x07, 0xcf, 0xd4, 0x54, 0xc3, 0x3b, 0xee, 0x95,
    0xa7, 0x36, 0x2e, 0xde, 0x5b, 0x5c, 0x93, 0x47, 0x13, 0xd0, 0xeb, 0x05, 0x90, 0x37, 0x01, 0x2f,
    0xf1, 0xaa, 0xf6, 0x65, 0x22, 0x31, 0x0f, 0x08, 0x50, 0x7c, 0xfd, 0x45, 0x4c, 0xb3, 0xbe, 0x69,
    0x5a, 0x73, 0xe2, 0xed, 0xe5, 0x35, 0x49, 0x34, 0x71, 0xbd, 0x0e, 0xb0, 0x59, 0x03, 0x70, 0x12,
    0xff, 0x1a, 0xaf, 0x66, 0x52, 0x23, 0x1e, 0x10, 0xa0, 0xf9, 0xfa, 0x8a, 0x98, 0x67, 0x7d, 0xd2,
    0xb4, 0xe6, 0xc5, 0xdb, 0xcb, 0x6b, 0x92, 0x68, 0xe2, 0x7a, 0x1d, 0x60, 0xb2, 0x06, 0xe0, 0x25,
    0xfe, 0x35, 0x5e, 0xcc, 0xa4, 0x46, 0x3c, 0x21, 0x41, 0xf3, 0xf5, 0x15, 0x30, 0xce, 0xfb, 0xa5,
    0x69, 0xcd, 0x8b, 0xb7, 0x96, 0xd7, 0x24, 0xd1, 0xc4, 0xf4, 0x3a, 0xc1, 0x64, 0x0d, 0xc0, 0x4b,
    0xfc, 0x6a, 0xbd, 0x99, 0x48, 0x8c, 0x78, 0x42, 0x83, 0xe7, 0xea, 0x2a, 0x61, 0x9d, 0xf7, 0x4a,
    0xd3, 0x9b, 0x17, 0x6f, 0x2d, 0xae, 0x49, 0xa3, 0x89, 0xe8, 0x75, 0x82, 0xc8, 0x1b, 0x80
}};

using frame_t = std::array<uint8_t, OPV_FRAME_BYTES>;
using encoded_t = std::array<uint8_t, OPV_ENCODED_BITS>;

// =============================================================================
// HDL-MATCHED MSK MODULATOR
// =============================================================================
// Uses dual-NCO staggered QPSK approach exactly matching msk_modulator.vhd
// I = sin(F1)*d_s1 + sin(F2)*d_s2
// Q = cos(F1)*d_s1 + cos(F2)*d_s2

class MSKModulatorHDL {
public:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double TWO_PI = 2.0 * PI;
    static constexpr double SAMPLE_RATE = 2168000.0;
    static constexpr double BIT_RATE = 54200.0;
    static constexpr double FREQ_DEV = BIT_RATE / 4.0;  // 13550 Hz

    static constexpr double PHASE_INC_F1 = -TWO_PI * FREQ_DEV / SAMPLE_RATE;
    static constexpr double PHASE_INC_F2 = +TWO_PI * FREQ_DEV / SAMPLE_RATE;

    struct IQSample {
        int16_t I;
        int16_t Q;
    };
    
    MSKModulatorHDL() { reset(); }
    
    void reset() {
        phase_f1_ = 0.0;
        phase_f2_ = 0.0;
        // Initialize to +1 so we get valid output from the first symbol
        // (HDL initializes to "000" which gives zero output for first symbol,
        // but after one symbol it becomes +1 or -1)
        d_val_xor_T_ = 1;
        b_n_ = true;  // HDL initializes b_n to '1'
    }
    



void simple_test_for_modulate_bit(uint8_t bit, std::array<IQSample, SAMPLES_PER_SYMBOL>& output) {
    double freq = (bit & 1) ? PHASE_INC_F2 : PHASE_INC_F1;
    
    for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
        output[i].I = static_cast<int16_t>(16383.0 * std::cos(phase_f1_));
        output[i].Q = static_cast<int16_t>(16383.0 * std::sin(phase_f1_));
        phase_f1_ += freq;
        while (phase_f1_ > TWO_PI) phase_f1_ -= TWO_PI;
        while (phase_f1_ < 0) phase_f1_ += TWO_PI;  // ADD THIS LINE
    }
}



    void modulate_bit(uint8_t bit, std::array<IQSample, SAMPLES_PER_SYMBOL>& output) {
        // Toggle b_n FIRST (like HDL does at tclk_dly(0)) - this corrupted spectrum
        //b_n_ = !b_n_;

        // d_val: 0 -> +1, 1 -> -1 (HDL line 223)
        //int d_val = (bit & 1) ? -1 : +1;   // original
        int d_val = (bit & 1) ? +1 : -1;  // INVERTED: was -1 : +1 
        
        // Differential encoding (HDL lines 225-228)
        // d_val_xor = d_val * d_val_xor_T (XOR on sign bits)
        int d_val_xor;
        if (d_val == 1 && d_val_xor_T_ == 1) {
            d_val_xor = 1;
        } else if (d_val == 1 && d_val_xor_T_ == -1) {
            d_val_xor = -1;
        } else if (d_val == -1 && d_val_xor_T_ == 1) {
            d_val_xor = -1;
        } else {  // d_val == -1 && d_val_xor_T_ == -1
            d_val_xor = 1;
        }
        
        // d_pos = (d_val + 1) / 2: +1->1, -1->0
        // d_neg = (d_val - 1) / 2: +1->0, -1->-1
        int d_pos = (d_val + 1) / 2;
        int d_neg = (d_val - 1) / 2;
        
        // d_neg_enc depends on b_n (HDL line 234)
        // d_neg_enc = d_neg when b_n=0, else -d_neg
        int d_neg_enc = b_n_ ? -d_neg : d_neg;
        
        // d_s1 from d_pos_xor truth table (HDL lines 236-238)
        // d_pos_xor = +1 when d_pos_enc=1 and d_val_xor_T=+1
        //           = -1 when d_pos_enc=1 and d_val_xor_T=-1
        //           = 0 otherwise
        int d_s1;
        if (d_pos == 1 && d_val_xor_T_ == 1) {
            d_s1 = 1;
        } else if (d_pos == 1 && d_val_xor_T_ == -1) {
            d_s1 = -1;
        } else {
            d_s1 = 0;
        }
        
        // d_s2 from d_neg_xor truth table (HDL lines 239-243)
        int d_s2;
        if (d_neg_enc == -1 && d_val_xor_T_ == 1) {
            d_s2 = -1;
        } else if (d_neg_enc == -1 && d_val_xor_T_ == -1) {
            d_s2 = 1;
        } else if (d_neg_enc == 1 && d_val_xor_T_ == 1) {
            d_s2 = 1;
        } else if (d_neg_enc == 1 && d_val_xor_T_ == -1) {
            d_s2 = -1;
        } else {
            d_s2 = 0;
        }
        
        // Generate samples
        for (size_t i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
            double sin_f1 = std::sin(phase_f1_);
            double cos_f1 = std::cos(phase_f1_);
            double sin_f2 = std::sin(phase_f2_);
            double cos_f2 = std::cos(phase_f2_);
            
            // I = sin(F1)*d_s1 + sin(F2)*d_s2
            // Q = cos(F1)*d_s1 + cos(F2)*d_s2
            double I = sin_f1 * d_s1 + sin_f2 * d_s2;
            double Q = cos_f1 * d_s1 + cos_f2 * d_s2;
            
            // original I and Q assignment
            output[i].I = static_cast<int16_t>(16383.0 * I);
            output[i].Q = static_cast<int16_t>(16383.0 * Q);

            // swap I and Q to see if that's the problem - made no difference
            //output[i].I = static_cast<int16_t>(16383.0 * Q);  // swapped
            //output[i].Q = static_cast<int16_t>(16383.0 * I);  // swapped
            
            // Advance NCO phases
            phase_f1_ += PHASE_INC_F1;
            phase_f2_ += PHASE_INC_F2;
            
            // Wrap phases
            while (phase_f1_ > TWO_PI) phase_f1_ -= TWO_PI;
            while (phase_f1_ < 0) phase_f1_ += TWO_PI;
            while (phase_f2_ > TWO_PI) phase_f2_ -= TWO_PI;
            while (phase_f2_ < 0) phase_f2_ += TWO_PI;
        }
        
        // Update state for next symbol
        d_val_xor_T_ = d_val_xor;
        b_n_ = !b_n_;   //moved to beginning of function to match HDL and spectrum was corrupted
    }
    
private:
    double phase_f1_;
    double phase_f2_;
    int d_val_xor_T_;
    bool b_n_;
};

// Global modulator instance
static MSKModulatorHDL g_modulator;

// =============================================================================
// ENCODING FUNCTIONS
// =============================================================================

void randomize(frame_t& frame) {
    for (size_t i = 0; i < OPV_FRAME_BYTES; ++i) {
        frame[i] ^= LFSR_TABLE[i % 255];
    }
}




encoded_t conv_encode(const frame_t& frame) {
    encoded_t encoded;
    size_t out_idx = 0;
    uint8_t shift_reg = 0;
    
    // FIXED: Process bytes FORWARD (0→133) to match HDL
    // HDL encoder reads bytes starting from byte 0
    for (size_t byte_idx = 0; byte_idx < OPV_FRAME_BYTES; ++byte_idx) {
        uint8_t byte = frame[byte_idx];
        
        // MSB first within each byte
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
            uint8_t input_bit = (byte >> bit_pos) & 1;
            uint8_t state = (input_bit << 6) | shift_reg;
            
            uint8_t g1 = __builtin_parity(state & 0x4F);
            uint8_t g2 = __builtin_parity(state & 0x6D);
            
            encoded[out_idx++] = g1;
            encoded[out_idx++] = g2;
            
            shift_reg = ((shift_reg << 1) | input_bit) & 0x3F;
        }
    }
    
    // NOTE: reverse_fec() is called separately after this function
    // to match HDL line 589: fec_buffer(i) <= encoder_output_buf(ENCODED_BITS - 1 - i)
    return encoded;
}





void interleave(encoded_t& data) {
    encoded_t temp;
    for (size_t i = 0; i < OPV_ENCODED_BITS; ++i) {
        size_t row = i / 32;
        size_t col = i % 32;
        size_t out_pos = col * 67 + row;
        temp[out_pos] = data[i];
    }
    data = temp;
}

void reverse_fec(encoded_t& data) {
    for (size_t i = 0; i < OPV_ENCODED_BITS / 2; ++i) {
        std::swap(data[i], data[OPV_ENCODED_BITS - 1 - i]);
    }
}



// =============================================================================
// FRAME BUILDING
// =============================================================================



// In build_bert_frame, temporarily replace with:
frame_t all_zeros_build_bert_frame(const char* callsign, uint32_t frame_num) {
    frame_t frame = {};
    // All zeros - this is a valid test pattern
    return frame;
}


frame_t build_bert_frame(const char* callsign, uint32_t frame_num) {
    frame_t frame = {};
    
    // Header: 6-byte callsign (Base-40), 3-byte token, 3-byte reserved
    // Simplified: just put callsign bytes and frame number
    size_t len = strlen(callsign);
    for (size_t i = 0; i < 6 && i < len; ++i) {
        frame[i] = callsign[i];
    }
    
    // Token = frame number
    frame[6] = (frame_num >> 16) & 0xFF;
    frame[7] = (frame_num >> 8) & 0xFF;
    frame[8] = frame_num & 0xFF;
    
    // Reserved = 0
    frame[9] = 0;
    frame[10] = 0;
    frame[11] = 0;
    
    // Payload: BERT pattern (incrementing bytes)
    for (size_t i = 0; i < OPV_PAYLOAD_BYTES; ++i) {
        frame[OPV_HEADER_BYTES + i] = (frame_num + i) & 0xFF;
    }
    
    return frame;
}

// =============================================================================
// TRANSMISSION
// =============================================================================

void output_sample(int16_t I, int16_t Q) {
    std::cout.write(reinterpret_cast<const char*>(&I), sizeof(int16_t));
    std::cout.write(reinterpret_cast<const char*>(&Q), sizeof(int16_t));
}





void send_frame(const encoded_t& encoded) {
    std::array<MSKModulatorHDL::IQSample, SAMPLES_PER_SYMBOL> samples;
    
    // Send sync word (24 bits)
    for (size_t i = 0; i < 24; ++i) {
        g_modulator.modulate_bit(SYNC_BITS[i], samples);
        for (const auto& s : samples) {
            output_sample(s.I, s.Q);
        }
    }
    
    // Send frame data - straight sequential order
    for (size_t i = 0; i < OPV_ENCODED_BITS; ++i) {
        g_modulator.modulate_bit(encoded[i], samples);
        for (const auto& s : samples) {
            output_sample(s.I, s.Q);
        }
    }
}






void temporarily_replaced_with_above_send_frame(const encoded_t& encoded) {
    std::array<MSKModulatorHDL::IQSample, SAMPLES_PER_SYMBOL> samples;
    
    // Send sync word (24 bits)
    for (size_t i = 0; i < 24; ++i) {
        g_modulator.modulate_bit(SYNC_BITS[i], samples);
        for (const auto& s : samples) {
            output_sample(s.I, s.Q);
        }
    }
    
    // Send frame data - MSB-first within each byte
    for (size_t byte_idx = 0; byte_idx < OPV_ENCODED_BITS / 8; ++byte_idx) {
        for (int bit_pos = 7; bit_pos >= 0; --bit_pos) {
            size_t i = byte_idx * 8 + bit_pos;
            g_modulator.modulate_bit(encoded[i], samples);
            for (const auto& s : samples) {
                output_sample(s.I, s.Q);
            }
        }
    }
}








void send_preamble(size_t num_bits) {
    std::array<MSKModulatorHDL::IQSample, SAMPLES_PER_SYMBOL> samples;
    
    for (size_t i = 0; i < num_bits; ++i) {
        g_modulator.modulate_bit(i & 1, samples);
        for (const auto& s : samples) {
            output_sample(s.I, s.Q);
        }
    }
}

// =============================================================================
// MAIN
// =============================================================================

int main(int argc, char* argv[]) {
    const char* callsign = "KB5MU";
    int bert_frames = 100;
    
    // Parse args
    int opt;
    while ((opt = getopt(argc, argv, "S:B:")) != -1) {
        switch (opt) {
            case 'S': callsign = optarg; break;
            case 'B': bert_frames = atoi(optarg); break;
            default:
                std::cerr << "Usage: " << argv[0] << " -S CALLSIGN -B FRAMES\n";
                return 1;
        }
    }
    
    std::cerr << "OPV Modulator (HDL-matched MSK)\n";
    std::cerr << "  Callsign: " << callsign << "\n";
    std::cerr << "  BERT frames: " << bert_frames << "\n";
    std::cerr << "  Modulation: Dual-NCO staggered QPSK (matching HDL)\n";
    
    // Reset modulator
    g_modulator.reset();
    
    // Send preamble (one frame worth of alternating bits)
    std::cerr << "Sending preamble (2168 bits)...\n";
    send_preamble(2168);
    
    // Send BERT frames
    for (int i = 0; i < bert_frames; ++i) {
        // Build frame
        frame_t frame = build_bert_frame(callsign, i + 1);
        
        // Debug: show first frame before/after randomize
        if (i == 0) {
            std::cerr << "Frame 1 before randomize: ";
            for (int j = 0; j < 12; ++j) std::cerr << std::hex << (int)frame[j] << " ";
            std::cerr << std::dec << "\n";
        }
        
        // Randomize
        randomize(frame);
        
        if (i == 0) {
            std::cerr << "Frame 1 after randomize:  ";
            for (int j = 0; j < 12; ++j) std::cerr << std::hex << (int)frame[j] << " ";
            std::cerr << std::dec << "\n";
        }
        
        // Convolutional encode
        encoded_t encoded = conv_encode(frame);


// Debug: dump bits BEFORE interleaving
if (i == 0) {
    FILE* dbg = fopen("/tmp/pre_interleave.bin", "wb");
    if (dbg) {
        fwrite(encoded.data(), 1, OPV_ENCODED_BITS, dbg);
        fclose(dbg);
        std::cerr << "Wrote pre-interleave bits to /tmp/pre_interleave.bin\n";
    }
}
        
        // CRITICAL FIX: Reverse FEC output to match HDL
        // HDL (ov_frame_encoder.vhd line 589): fec_buffer(i) <= encoder_output_buf(ENCODED_BITS - 1 - i)
        reverse_fec(encoded);
        
        // Interleave
        interleave(encoded);




// Debug: dump first frame's encoded bits to file
if (i == 0) {
    FILE* dbg = fopen("/tmp/encoded_bits.bin", "wb");
    if (dbg) {
        fwrite(encoded.data(), 1, OPV_ENCODED_BITS, dbg);
        fclose(dbg);
        std::cerr << "Wrote encoded bits to /tmp/encoded_bits.bin\n";
    }
}




        
        // Transmit
        send_frame(encoded);
        
        if ((i + 1) % 10 == 0) {
            std::cerr << "Sent frame " << (i + 1) << "/" << bert_frames << "\n";
        }
    }
    

size_t total_samples = 2168 * SAMPLES_PER_SYMBOL;  // preamble
total_samples += bert_frames * 2168 * SAMPLES_PER_SYMBOL;  // frames
std::cerr << "Total samples written: " << total_samples << "\n";
std::cerr << "Total bytes: " << (total_samples * 4) << "\n";


std::cout.flush();

    std::cerr << "Transmission complete.\n";
    return 0;
}

// Codec known-answer test: proves opv_codec.hpp is the true 171/133 tail-biting
// codec. Run by `make test`. Two independent checks:
//   (1) impulse response == 1111001/1011011  (the code's identity; no golden)
//   (2) full-frame KAT hex matches golden_frame.txt  (catches ANY drift)
#include "opv_codec.hpp"
#include <cstdio>
#include <string>
int main() {
    // (1) impulse -- independent oracle for the taps
    ConvEncoder c; std::string g1, g2; uint8_t a, b;
    for (int k = 0; k < 7; ++k) { c.encode_bit(k==0?1:0, a, b); g1 += char('0'+a); g2 += char('0'+b); }
    bool taps = (g1 == "1111001" && g2 == "1011011");
    fprintf(stderr, "  impulse g1=%s g2=%s  %s\n", g1.c_str(), g2.c_str(),
            taps ? "[171/133 OK]" : "[*** WRONG ***]");

    // (2) full-frame KAT: "73" x 67 (134 bytes). encode_frame() asserts ring closure.
    frame_t pay{}; { std::string m; for (int i=0;i<67;++i) m+="73";
                     for (size_t i=0;i<FRAME_BYTES;++i) pay[i]=(uint8_t)m[i]; }
    encoded_bits_t enc = encode_frame(pay);
    std::string hex;
    for (size_t i=0;i<ENCODED_BITS;i+=8){ uint8_t byte=0; for(int k=0;k<8;++k) byte=(byte<<1)|enc[i+k];
                                          char h[3]; snprintf(h,3,"%02x",byte); hex+=h; }
    printf("%s\n", hex.c_str());     // stdout = the KAT vector (diff against golden)
    return taps ? 0 : 1;
}

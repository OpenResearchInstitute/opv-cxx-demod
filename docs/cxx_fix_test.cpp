#include <string>
#include <cstdint>
#include <cstdio>
#include <vector>
// Minimal extract of opv-cxx-demod ConvEncoder with the CORRECTED masks.
constexpr uint8_t G1_MASK = 0x67;   // was 0x4F  -> now 171 octal
constexpr uint8_t G2_MASK = 0x76;   // was 0x6D  -> now 133 octal
struct ConvEncoder {
    uint8_t sr = 0;
    void reset(){ sr = 0; }
    void encode_bit(uint8_t in, uint8_t& g1, uint8_t& g2){
        uint8_t state = (in << 6) | sr;
        g1 = __builtin_parity(state & G1_MASK);
        g2 = __builtin_parity(state & G2_MASK);
        sr = ((sr << 1) | in) & 0x3F;
    }
};
int main(){
    ConvEncoder c; c.reset();
    // impulse: single 1 then six 0s
    uint8_t in[7] = {1,0,0,0,0,0,0};
    std::string g1s, g2s; uint8_t g1,g2;
    for(int i=0;i<7;i++){ c.encode_bit(in[i],g1,g2); g1s+=char('0'+g1); g2s+=char('0'+g2); }
    printf("impulse g1 = %s  (want 1111001)\n", g1s.c_str());
    printf("impulse g2 = %s  (want 1011011)\n", g2s.c_str());
    bool ok = (g1s=="1111001") && (g2s=="1011011");
    // convention-proof: input [1,1,0,0,0,0,0,0] must weigh 10
    c.reset(); uint8_t ev[8]={1,1,0,0,0,0,0,0}; int w=0;
    for(int i=0;i<8;i++){ c.encode_bit(ev[i],g1,g2); w+=g1+g2; }
    printf("weight of [1,1,0..] = %d  (want 10)\n", w);
    ok = ok && (w==10);
    printf("%s\n", ok ? "PASS: C++ encoder is now Voyager 171/133" : "FAIL");
    return ok?0:1;
}

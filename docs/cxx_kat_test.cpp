#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

constexpr uint8_t G1_MASK = 0x67;   // 171 octal
constexpr uint8_t G2_MASK = 0x76;   // 133 octal

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
    // payload = ASCII "73" x 67, MSB-first per byte  (matches golden_vectors.txt)
    std::string msg; for(int i=0;i<67;i++) msg += "73";
    std::vector<uint8_t> bits;
    for(unsigned char ch : msg) for(int k=7;k>=0;k--) bits.push_back((ch>>k)&1);

    ConvEncoder c; c.reset();
    std::vector<uint8_t> code;                 // g1,g2 interleaved
    uint8_t g1,g2;
    for(uint8_t b : bits){ c.encode_bit(b,g1,g2); code.push_back(g1); code.push_back(g2); }

    // pack MSB-first to bytes, print hex
    for(size_t i=0;i<code.size();i+=8){
        uint8_t byte=0; for(int k=0;k<8;k++) byte=(byte<<1)|code[i+k];
        printf("%02x", byte);
    }
    printf("\n");
    return 0;
}


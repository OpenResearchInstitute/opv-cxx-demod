#include "Viterbi.h"
#include "Trellis.h"
#include "Util.h"
#include "Numerology.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <chrono>

// make CXXFLAGS="$(pkg-config --cflags gtest) $(pkg-config --libs gtest) -I. -O3 -std=c++17" tests/ViterbiTest

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class ViterbiTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  // void TearDown() override {}
};

TEST_F(ViterbiTest, construct)
{
    mobilinkd::Trellis<4,2> trellis({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});
    mobilinkd::Viterbi<decltype(trellis)> viterbi(trellis);


}

TEST_F(ViterbiTest, makeNextState)
{
    mobilinkd::Trellis<4,2> trellis({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});
    auto nextState{mobilinkd::makeNextState(trellis)};
    for (size_t i = 0; i != 16; ++i)
    {
        std::cout << "[ " << int(nextState[i][0]) << ", " << int(nextState[i][1]) << " ]" << std::endl;
    }

    EXPECT_EQ(nextState[0][0], 0);
}

TEST_F(ViterbiTest, makePrevState)
{
    mobilinkd::Trellis<4,2> trellis({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});
    auto prevStates{mobilinkd::makePrevState(trellis)};
    for (size_t i = 0; i != 16; ++i)
    {
        std::cout << "[ " << int(prevStates[i][0]) << ", " << int(prevStates[i][1]) << " ]" << std::endl;
    }

    EXPECT_EQ(prevStates[0][0], 0);
    EXPECT_EQ(prevStates[0][1], 8);
}

TEST_F(ViterbiTest, makeCost)
{
    mobilinkd::Trellis<4,2> trellis({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});

    ASSERT_EQ(trellis.polynomials[0], mobilinkd::ConvolutionPolyA);
    ASSERT_EQ(trellis.polynomials[1], mobilinkd::ConvolutionPolyB);

    auto cost{mobilinkd::makeCost(trellis)};
    for (size_t i = 0; i != 16; ++i)
    {
        std::cout << "[ " << int(cost[i][0]) << ", " << int(cost[i][1]) << " ]" << std::endl;
    }

    EXPECT_EQ(cost[0][0], -1);
    EXPECT_EQ(cost[1][1], 1);
}

TEST_F(ViterbiTest, makeCostLLR)
{
    mobilinkd::Trellis<4,2> trellis({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});

    ASSERT_EQ(trellis.polynomials[0], mobilinkd::ConvolutionPolyA);
    ASSERT_EQ(trellis.polynomials[1], mobilinkd::ConvolutionPolyB);

    auto cost{mobilinkd::makeCost<decltype(trellis), 4>(trellis)};
    for (size_t i = 0; i != 16; ++i)
    {
        std::cout << "[ " << int(cost[i][0]) << ", " << int(cost[i][1]) << " ]" << std::endl;
    }

    EXPECT_EQ(cost[0][0], -7);
    EXPECT_EQ(cost[0][1], -7);
    EXPECT_EQ(cost[1][1], 7);
}

TEST_F(ViterbiTest, decode)
{
    std::array<uint8_t, 8> expected = {1,0,1,1,0,1,1,0};
    std::array<int8_t, 24> encoded = {1,1,0,1,1,0,0,0,1,1,0,0,1,1,1,1,1,1,0,1,1,1,0,0};
    std::array<uint8_t, 8> output;

    for (size_t i = 0; i != encoded.size(); ++i)
    {
        encoded[i] = encoded[i] * 2 - 1;
    }

    mobilinkd::Trellis<4,2> trellis({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});
    mobilinkd::Viterbi<decltype(trellis)> viterbi(trellis);
    viterbi.decode(encoded, output);

    for (size_t i = 0; i != output.size(); ++i)
    {
        std::cout << int(output[i]) << ", ";
        EXPECT_EQ(output[i], expected[i]);
    }
    std::cout << std::endl;

    output.fill(0);
    std::array<int8_t, 24> encoded2 = {1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1};
    for (size_t i = 0; i != encoded2.size(); ++i)
    {
        encoded2[i] = encoded2[i] * 2 - 1;
    }

    auto ber = viterbi.decode(encoded2, output);
    EXPECT_EQ(ber, 0);

    for (size_t i = 0; i != output.size(); ++i) std::cout << int(output[i]) << ", ";
    std::cout << std::endl;
}

TEST_F(ViterbiTest, decode_ber_1)
{
    std::array<uint8_t, 8> expected = {1,0,1,1,0,1,1,0};
    std::array<int8_t, 24> encoded = {1,1,0,1,1,0,0,0,1,1,0,0,1,1,1,1,1,1,0,1,1,1,0,0};
    std::array<uint8_t, 12> output;

    encoded[11] = 1;    // flip one bit.

    for (size_t i = 0; i != encoded.size(); ++i)
    {
        encoded[i] = encoded[i] * 2 - 1;
    }

    mobilinkd::Trellis<4,2> trellis({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});
    mobilinkd::Viterbi<decltype(trellis)> viterbi(trellis);
    auto ber = viterbi.decode(encoded,output);
    for (size_t i = 0; i != expected.size(); ++i) EXPECT_EQ(output[i], expected[i]);

    EXPECT_EQ(ber, 2);
}

TEST_F(ViterbiTest, decode_ber_llr)
{
    std::array<uint8_t, 8> expected = {1,0,1,1,0,1,1,0};
    std::array<int8_t, 24> encoded = {1,1,0,1,1,0,0,0,1,1,0,0,1,1,1,1,1,1,0,1,1,1,0,0};
    std::array<uint8_t, 12> output;

    encoded[11] = 1;

    for (size_t i = 0; i != encoded.size(); ++i)
    {
        encoded[i] = encoded[i] * 14 - 7;
    }

    mobilinkd::Trellis<4,2> trellis({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});
    mobilinkd::Viterbi<decltype(trellis), 4> viterbi(trellis);
    auto start = std::chrono::high_resolution_clock::now();
    auto ber = viterbi.decode(encoded, output);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Duration: " << (end - start).count() << "ns" << std::endl;
    EXPECT_EQ(ber, 2);
    for (size_t i = 0; i != expected.size(); ++i) EXPECT_EQ(output[i], expected[i]);

}

TEST_F(ViterbiTest, decode_ber_lsf)
{
    std::array<uint8_t, 240> expected = {1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0};
    std::array<int8_t, 488> encoded = {1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0};
    std::array<uint8_t, 244> output;

    encoded[11] = 1;

    for (size_t i = 0; i != encoded.size(); ++i)
    {
        encoded[i] = encoded[i] * 14 - 7;
    }

    mobilinkd::Trellis<4,2> trellis({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});
    mobilinkd::Viterbi<decltype(trellis), 4> viterbi(trellis);
    auto start = std::chrono::high_resolution_clock::now();
    auto ber = viterbi.decode(encoded, output);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Duration: " << (end - start).count() << "ns" << std::endl;
    EXPECT_EQ(ber, 0);
    for (size_t i = 0; i != expected.size(); ++i) EXPECT_EQ(output[i], expected[i]);

}


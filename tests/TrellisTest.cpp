#include "Trellis.h"
#include "Util.h"
#include "Numerology.h"

#include <gtest/gtest.h>

#include <cstdint>

// make CXXFLAGS="$(pkg-config --cflags gtest) $(pkg-config --libs gtest) -I. -O3 -std=c++17" tests/TrellisTest

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class TrellisTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  // void TearDown() override {}
};

TEST_F(TrellisTest, toBitArray)
{
    auto ba1 = mobilinkd::toBitArray<4>(7);
    ASSERT_EQ(ba1.size(), 4);
    EXPECT_EQ(ba1[0], 0);
    EXPECT_EQ(ba1[1], 1);
    EXPECT_EQ(ba1[2], 1);
    EXPECT_EQ(ba1[3], 1);

    auto ba2 = mobilinkd::toBitArray<4>(5);
    ASSERT_EQ(ba2.size(), 4);
    EXPECT_EQ(ba2[0], 0);
    EXPECT_EQ(ba2[1], 1);
    EXPECT_EQ(ba2[2], 0);
    EXPECT_EQ(ba2[3], 1);
}

TEST_F(TrellisTest, construct)
{
    auto trellis = mobilinkd::makeTrellis<4, 2>({mobilinkd::ConvolutionPolyA,mobilinkd::ConvolutionPolyB});

    ASSERT_EQ(trellis.polynomials.size(), 2);
    EXPECT_EQ(trellis.polynomials[1], mobilinkd::ConvolutionPolyB);
}

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "auv_test_vision/depth_conversion.hpp"

TEST(DepthConversion, ConvertsEnuDownwardPoseToPositiveDepth)
{
  EXPECT_DOUBLE_EQ(
    auv_test_vision::pose_z_to_positive_down_depth(-1.25, -1.0, 0.0),
    1.25);
}

TEST(DepthConversion, ClampsPoseAboveReferenceToSurface)
{
  EXPECT_DOUBLE_EQ(
    auv_test_vision::pose_z_to_positive_down_depth(0.34, -1.0, 0.0),
    0.0);
}

TEST(DepthConversion, AppliesScaleAndOffsetBeforeClamping)
{
  EXPECT_DOUBLE_EQ(
    auv_test_vision::pose_z_to_positive_down_depth(0.2, -1.0, 0.5),
    0.3);
  EXPECT_DOUBLE_EQ(
    auv_test_vision::pose_z_to_positive_down_depth(0.6, -1.0, 0.5),
    0.0);
}

TEST(DepthConversion, PreservesNonFiniteValuesForValidation)
{
  EXPECT_TRUE(std::isnan(
    auv_test_vision::pose_z_to_positive_down_depth(
      std::numeric_limits<double>::quiet_NaN(), -1.0, 0.0)));
  EXPECT_TRUE(std::isinf(
    auv_test_vision::pose_z_to_positive_down_depth(
      std::numeric_limits<double>::infinity(), -1.0, 0.0)));
}

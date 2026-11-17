#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "../src/lidar_ray_encoding.h"

namespace {
using lidar_ray_encoding::endpoint;
const double pi = 3.14159265358979323846;

TEST(LidarRayEncoding, FiniteReturnRetainsPhysicalRangeAndDirection) {
  const auto p = endpoint(17.0, 1.0, 120.0, pi / 3, -pi / 6);
  EXPECT_FLOAT_EQ(p.hit, 1.0f);
  EXPECT_NEAR(p.x, 17 * std::cos(pi/6) * .5, 1e-6);
  EXPECT_NEAR(p.y, 17 * .75, 1e-6);
  EXPECT_NEAR(p.z, -8.5, 1e-6);
}

TEST(LidarRayEncoding, NoReturnCarriesFreeRayEvidenceWithoutOccupiedEndpoint) {
  for (double range : {120.0, 120.1, std::numeric_limits<double>::infinity()}) {
    const auto p = endpoint(range, 1.0, 120.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(p.hit, 0.0f);
    EXPECT_FLOAT_EQ(p.x, 120.0f);
    EXPECT_FLOAT_EQ(p.y, 0.0f);
    EXPECT_FLOAT_EQ(p.z, 0.0f);
  }
}

TEST(LidarRayEncoding, InvalidAndTooCloseMeasurementsAreNotFreeRays) {
  for (double range : {std::numeric_limits<double>::quiet_NaN(),
                       -std::numeric_limits<double>::infinity(), -1.0, .9}) {
    const auto p = endpoint(range, 1.0, 120.0, 0.0, 0.0);
    EXPECT_FLOAT_EQ(p.hit, -1.0f);
    EXPECT_TRUE(std::isnan(p.x));
    EXPECT_TRUE(std::isnan(p.y));
    EXPECT_TRUE(std::isnan(p.z));
  }
}

TEST(LidarRayEncoding, SensorRangeBoundariesPreserveHitStreamFiltering) {
  EXPECT_FLOAT_EQ(endpoint(1.0,1,120,0,0).hit, 1.0f);
  EXPECT_FLOAT_EQ(endpoint(120 - 2e-5,1,120,0,0).hit, 1.0f);
  EXPECT_FLOAT_EQ(endpoint(120 - .5e-5,1,120,0,0).hit, 0.0f);
}

TEST(LidarRayEncoding, InvalidCalibrationCannotCreateFreeSpace) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FLOAT_EQ(endpoint(10,1,nan,0,0).hit,-1.0f);
  EXPECT_FLOAT_EQ(endpoint(10,1,1,0,0).hit,-1.0f);
  EXPECT_FLOAT_EQ(endpoint(10,1,120,nan,0).hit,-1.0f);
  EXPECT_FLOAT_EQ(endpoint(10,1,120,0,nan).hit,-1.0f);
}
}  // namespace

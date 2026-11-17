#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>
#include "../src/sensor_occupancy_map.h"

namespace {
using sensor_occupancy::Cell;
using sensor_occupancy::Key;
using sensor_occupancy::Map;
using sensor_occupancy::Point;
using sensor_occupancy::Ray;

void fill(Map& map, int lo, int hi, double stamp=1) {
    std::vector<Ray> rays;
    for (int x=lo; x<=hi; ++x) for (int y=lo; y<=hi; ++y) for (int z=lo; z<=hi; ++z)
        rays.push_back(Ray(Point(x+.5,y+.5,z+.5),0));
    // The origin voxel is traversed by all nonzero rays, despite the zero ray
    // to its own center being invalid by design.
    ASSERT_TRUE(map.integrate_scan(Point(.5,.5,.5),rays,stamp));
}

TEST(SensorOccupancy, SeparatesObservedFreeHitAndOccludedUnknown) {
    Map map(1);
    ASSERT_TRUE(map.integrate_scan(Point(.5,.5,.5),{Ray(Point(4.5,.5,.5),1)},1));
    EXPECT_EQ(map.state(Point(2.5,.5,.5)),Cell::FREE);
    EXPECT_EQ(map.state(Point(4.5,.5,.5)),Cell::OCCUPIED);
    EXPECT_EQ(map.state(Point(5.5,.5,.5)),Cell::UNKNOWN);
    EXPECT_TRUE(map.segment_is_free(Point(.6,.5,.5),Point(3.8,.5,.5)));
    EXPECT_FALSE(map.segment_is_free(Point(.6,.5,.5),Point(4.2,.5,.5)));
    EXPECT_FALSE(map.segment_is_free(Point(.6,.5,.5),Point(5.5,.5,.5)));
    EXPECT_FALSE(map.point_is_free(Point(2.5,1.5,.5)));
}

TEST(SensorOccupancy, NoReturnClearsRangeWithoutOccupiedShellOrBeyondRangeCell) {
    Map map(1);
    ASSERT_TRUE(map.integrate_scan(Point(.5,.5,.5),{Ray(Point(5,.5,.5),0)},1));
    EXPECT_EQ(map.stats().occupied_cells,0u);
    EXPECT_EQ(map.stats().no_return_rays,1u);
    EXPECT_EQ(map.state(Point(4.5,.5,.5)),Cell::FREE);
    EXPECT_EQ(map.state(Point(5.5,.5,.5)),Cell::UNKNOWN);
    EXPECT_FALSE(map.segment_is_free(Point(4.5,.5,.5),Point(5.5,.5,.5)));
}

TEST(SensorOccupancy, HitHasScanOrderPrecedenceAndPersistsInStaticMap) {
    Map a(1),b(1);
    const Ray hit(Point(4.5,.5,.5),1),clear(Point(8.5,.5,.5),0);
    ASSERT_TRUE(a.integrate_scan(Point(.5,.5,.5),{hit,clear},1));
    ASSERT_TRUE(b.integrate_scan(Point(.5,.5,.5),{clear,hit},1));
    for (int x=0;x<10;++x) EXPECT_EQ(a.state(Point(x+.5,.5,.5)),b.state(Point(x+.5,.5,.5)));
    EXPECT_EQ(a.state(Point(4.5,.5,.5)),Cell::OCCUPIED);
    ASSERT_TRUE(a.integrate_scan(Point(.5,.5,.5),{clear},2));
    EXPECT_EQ(a.state(Point(4.5,.5,.5)),Cell::OCCUPIED);
}

TEST(SensorOccupancy, NegativeCoordinatesUseFloorAndDoNotAliasPositiveVoxels) {
    Map map(1);
    ASSERT_TRUE(map.integrate_scan(Point(-.5,-.5,-.5),{Ray(Point(-4.5,-.5,-.5),1)},1));
    EXPECT_EQ(map.key(Point(-.01,0,.01)).x,-1);
    EXPECT_EQ(map.key(Point(-1.01,0,.01)).x,-2);
    EXPECT_EQ(map.state(Point(-3.5,-.5,-.5)),Cell::FREE);
    EXPECT_EQ(map.state(Point(-4.5,-.5,-.5)),Cell::OCCUPIED);
    EXPECT_EQ(map.state(Point(.5,.5,.5)),Cell::UNKNOWN);
}

TEST(SensorOccupancy, SegmentDetectsThinOccupiedVoxelBetweenFreeEndpoints) {
    Map map(1);fill(map,-4,4);
    ASSERT_TRUE(map.integrate_scan(Point(-2.5,.5,.5),{Ray(Point(.1,.5,.5),1)},2));
    EXPECT_EQ(map.state(Point(-2.5,.5,.5)),Cell::FREE);
    EXPECT_EQ(map.state(Point(2.5,.5,.5)),Cell::FREE);
    EXPECT_FALSE(map.segment_is_free(Point(-2.5,.5,.5),Point(2.5,.5,.5)));
    EXPECT_FALSE(map.segment_is_free(Point(2.5,.5,.5),Point(-2.5,.5,.5)));
}

TEST(SensorOccupancy, ClosedSupercoverRejectsUnobservedCornerTouches) {
    Map map(1);
    ASSERT_TRUE(map.integrate_scan(Point(.5,.5,.5),{Ray(Point(3.5,3.5,.5),0)},1));
    EXPECT_EQ(map.state(Point(1.5,1.5,.5)),Cell::FREE);
    EXPECT_EQ(map.state(Point(1.5,.5,.5)),Cell::UNKNOWN);
    EXPECT_FALSE(map.segment_is_free(Point(.5,.5,.5),Point(3.5,3.5,.5)));
    std::set<std::tuple<int,int,int>> cells;
    ASSERT_TRUE(map.traverse(Point(.5,.5,.5),Point(1.5,1.5,1.5),true,[&](const Key& k){cells.emplace(k.x,k.y,k.z);return true;}));
    EXPECT_EQ(cells.size(),8u);
}

TEST(SensorOccupancy, BoundaryAndZeroLengthQueriesIncludeBothSidesOfVoxelFaces) {
    Map map(1);fill(map,-3,3);
    EXPECT_TRUE(map.point_is_free(Point(0,0,0)));
    ASSERT_TRUE(map.integrate_scan(Point(-2.5,.5,.5),{Ray(Point(0,.5,.5),1)},2));
    EXPECT_EQ(map.state(Point(-.5,.5,.5)),Cell::OCCUPIED);
    EXPECT_EQ(map.state(Point(.5,.5,.5)),Cell::OCCUPIED);
    EXPECT_FALSE(map.point_is_free(Point(0,.5,.5)));
    EXPECT_FALSE(map.segment_is_free(Point(0,.5,.5),Point(0,2.5,.5)));
}

TEST(SensorOccupancy, PerFaceInflationRejectsTangencyWithoutRejectingRealGap) {
    Map map(1);fill(map,-4,4);
    ASSERT_TRUE(map.integrate_scan(Point(3.5,.5,.5),{Ray(Point(2.5,.5,.5),1)},2));
    const Point p(.5,.5,.5);
    EXPECT_TRUE(map.point_is_free(p,1.49));
    EXPECT_FALSE(map.point_is_free(p,1.5));
    EXPECT_TRUE(map.point_is_free(p,0));
    EXPECT_FALSE(map.point_is_free(Point(-3.8,.5,.5),.3)); // UNKNOWN outside mapped volume.
}

TEST(SensorOccupancy, MapMutationInvalidatesSafeCellCache) {
    Map map(1);fill(map,-4,4);
    const Point p(.5,.5,.5);
    ASSERT_TRUE(map.point_is_free(p,1));
    const auto generation=map.generation();
    ASSERT_TRUE(map.integrate_scan(Point(3.5,.5,.5),{Ray(Point(1.5,.5,.5),1)},2));
    EXPECT_GT(map.generation(),generation);
    EXPECT_FALSE(map.point_is_free(p,1));
}

TEST(SensorOccupancy, ClearanceBoundDistinguishesUnknownAndUsesOnlyConfirmedFreeCube) {
    Map map(1);
    EXPECT_LT(map.clearance_lower_bound(Point(.5,.5,.5)),0);
    fill(map,-3,3);
    EXPECT_DOUBLE_EQ(map.clearance_lower_bound(Point(.5,.5,.5),30),3.5);
    EXPECT_DOUBLE_EQ(map.clearance_lower_bound(Point(.5,.5,.5),2),2);
    ASSERT_TRUE(map.integrate_scan(Point(.5,.5,.5),{Ray(Point(2.5,.5,.5),1)},2));
    EXPECT_DOUBLE_EQ(map.clearance_lower_bound(Point(.5,.5,.5)),1.5);
}

TEST(SensorOccupancy, AlternatingRealMarginsPreserveThresholdsAndInvalidateEveryRadiusLayer) {
    Map map(1);fill(map,-6,6);
    const Point p(.5,.5,.5);
    for (int repeat=0;repeat<10;++repeat) {
        for (int step=0;step<=15;++step) {
            const double margin=2.0+step/15.0;
            EXPECT_TRUE(map.point_is_free(p,margin));
            EXPECT_TRUE(map.point_is_free(p,2.0));
        }
    }
    ASSERT_TRUE(map.integrate_scan(Point(4.5,.5,.5),{Ray(Point(2.5,.5,.5),1)},2));
    for (int step=0;step<=15;++step) EXPECT_FALSE(map.point_is_free(p,2.0+step/15.0));
    // These margins induce the same integer-radius cache layer. The actual
    // slab test still distinguishes a 1cm gap from exact tangency.
    for (int repeat=0;repeat<20;++repeat) {
        EXPECT_TRUE(map.point_is_free(p,1.49));
        EXPECT_FALSE(map.point_is_free(p,1.5));
        EXPECT_TRUE(map.point_is_free(p,.1));
        EXPECT_FALSE(map.point_is_free(p,3));
    }
    map.reset();
    for(double margin:{.1,1.49,1.5,2.0,3.0}) EXPECT_FALSE(map.point_is_free(p,margin));
    EXPECT_FALSE(map.point_is_free(p,std::numeric_limits<double>::max()));
}

TEST(SensorOccupancy, TightRadiusRetainsExactPositiveAndNegativeFaceTangencies) {
    Map map(2);std::vector<Ray> rays;
    for(int x=-5;x<=5;++x)for(int y=-5;y<=5;++y)for(int z=-5;z<=5;++z)
        rays.push_back(Ray(Point(2*x+1,2*y+1,2*z+1),0));
    ASSERT_TRUE(map.integrate_scan(Point(1,1,1),rays,1));
    ASSERT_TRUE(map.integrate_scan(Point(1,1,1),{Ray(Point(5,1,1),1),Ray(Point(-5,1,1),1)},2));
    const double eps=1e-7;
    for(int sign:{-1,1}) {
        const Point a(sign*2.,-3,1),b(sign*2.,3,1);
        EXPECT_TRUE(map.segment_is_free(a,b,2-eps));
        EXPECT_FALSE(map.segment_is_free(a,b,2));
        EXPECT_FALSE(map.segment_is_free(a,b,2+eps));
        EXPECT_FALSE(map.segment_is_free(b,a,2));
    }
    EXPECT_TRUE(map.segment_is_free(Point(1,-3,1),Point(1,3,1),3-eps));
    EXPECT_FALSE(map.segment_is_free(Point(1,-3,1),Point(1,3,1),3));
}

TEST(SensorOccupancy, ZeroRadiusIncludesFaceParallelEdgeAndCornerOnlyContacts) {
    Map map(2);std::vector<Ray> rays;
    for(int x=-5;x<=5;++x)for(int y=-5;y<=5;++y)for(int z=-5;z<=5;++z)
        rays.push_back(Ray(Point(2*x+1,2*y+1,2*z+1),0));
    ASSERT_TRUE(map.integrate_scan(Point(-3,1,1),rays,1));
    ASSERT_TRUE(map.integrate_scan(Point(-3,1,1),{Ray(Point(1,1,1),1)},2));
    const double eps=1e-7;
    EXPECT_FALSE(map.segment_is_free(Point(2,-1,1),Point(2,3,1),0));
    EXPECT_TRUE(map.segment_is_free(Point(2+eps,-1,1),Point(2+eps,3,1),0));
    EXPECT_FALSE(map.segment_is_free(Point(2,2,-1),Point(2,2,3),0));
    EXPECT_FALSE(map.segment_is_free(Point(2,2,2),Point(4,4,4),0));
    EXPECT_FALSE(map.segment_is_free(Point(4,4,4),Point(2,2,2),0));
    EXPECT_TRUE(map.segment_is_free(Point(2+eps,2+eps,2+eps),Point(4,4,4),0));
    EXPECT_FALSE(map.point_is_free(Point(2,2,2),0));
    EXPECT_TRUE(map.point_is_free(Point(2+eps,2+eps,2+eps),0));
}

TEST(SensorOccupancy, InvalidStaleAndCapacityDataCannotCreateAssumedFreeSpace) {
    EXPECT_THROW(Map(0),std::invalid_argument);
    Map map(1,2);
    const double nan=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(map.integrate_scan(Point(nan,0,0),{},1));
    ASSERT_TRUE(map.integrate_scan(Point(.5,.5,.5),{Ray(Point(10.5,.5,.5),0),Ray(Point(2,2,2),-1),Ray(Point(nan,2,2),1)},2));
    EXPECT_EQ(map.size(),2u);EXPECT_GT(map.stats().dropped_capacity,0u);
    EXPECT_EQ(map.stats().invalid_rays,2u);
    EXPECT_FALSE(map.integrate_scan(Point(.5,.5,.5),{},1));
    EXPECT_EQ(map.stats().stale_scans,1u);
    EXPECT_FALSE(map.point_is_free(Point(nan,0,0)));
    EXPECT_FALSE(map.segment_is_free(Point(.5,.5,.5),Point(1.5,.5,.5),-1));
    map.reset();EXPECT_EQ(map.size(),0u);EXPECT_EQ(map.stats().last_stamp,-1);
    EXPECT_EQ(map.state(Point(.5,.5,.5)),Cell::UNKNOWN);
}
} // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

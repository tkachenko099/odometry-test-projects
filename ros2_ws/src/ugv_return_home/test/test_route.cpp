// Unit tests: bounded storage + breadcrumb recorder.
#include <gtest/gtest.h>

#include <cmath>

import ugv.rth;

using namespace ugv::rth;

namespace {
/// Build a point `metres` east of the equatorial origin.
GeoPoint east(Scalar metres) noexcept {
    return {0.0, metres / geo::kEarthRadius, 0.0};
}
}  // namespace

TEST(StaticVector, PushFullAndClear) {
    StaticVector<int, 4> v;
    EXPECT_TRUE(v.empty());
    for (int i = 0; i < 4; ++i) EXPECT_TRUE(v.push_back(i));
    EXPECT_TRUE(v.full());
    EXPECT_FALSE(v.push_back(99));  // rejected, not overflowed
    EXPECT_EQ(v.size(), 4U);
    EXPECT_EQ(v.back(), 3);
    v.clear();
    EXPECT_TRUE(v.empty());
}

TEST(StaticVector, DecimateByHalfKeepsFirst) {
    StaticVector<int, 8> v;
    for (int i = 0; i < 6; ++i) (void)v.push_back(i);  // 0..5
    v.decimateByHalf();                                // keep 0,2,4
    ASSERT_EQ(v.size(), 3U);
    EXPECT_EQ(v[0], 0);
    EXPECT_EQ(v[1], 2);
    EXPECT_EQ(v[2], 4);
}

TEST(RouteRecorder, DecimatesByMinDistance) {
    RouteRecorder rec{RouteConfig{.min_record_distance = 10.0, .arrival_radius = 2.0}};
    for (int m = 0; m <= 100; ++m) (void)rec.record(east(static_cast<Scalar>(m)));
    // 101 dense fixes decimate to roughly one per 10 m (boundary rounding aside).
    EXPECT_GE(rec.size(), 9U);
    EXPECT_LE(rec.size(), 11U);
    EXPECT_NEAR(rec.home().lon, 0.0, 1e-12);
    EXPECT_GT(rec.travelledDistance(), 85.0);
}

TEST(RouteRecorder, ClearResets) {
    RouteRecorder rec{RouteConfig{.min_record_distance = 1.0, .arrival_radius = 2.0}};
    (void)rec.record(east(0.0));
    (void)rec.record(east(10.0));
    EXPECT_EQ(rec.size(), 2U);
    rec.clear();
    EXPECT_TRUE(rec.empty());
}

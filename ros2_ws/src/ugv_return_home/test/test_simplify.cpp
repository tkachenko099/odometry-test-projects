// Unit tests: Douglas-Peucker simplification + return-path construction.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

import ugv.rth;

using namespace ugv::rth;

namespace {
GeoPoint east(Scalar metres) noexcept { return {0.0, metres / geo::kEarthRadius, 0.0}; }
GeoPoint at(Scalar north_m, Scalar east_m) noexcept {
    return {north_m / geo::kEarthRadius, east_m / geo::kEarthRadius, 0.0};
}
}  // namespace

TEST(Simplify, CollinearCollapsesToEndpoints) {
    std::vector<GeoPoint> line;
    for (int m = 0; m <= 10; ++m) line.push_back(east(static_cast<Scalar>(m)));
    StaticVector<GeoPoint, kMaxWaypoints> out;
    const std::size_t k = douglasPeucker(line, 0.5, out);
    EXPECT_EQ(k, 2U);  // only the two endpoints survive
}

TEST(Simplify, KeepsSignificantOutlier) {
    // Straight east run with a single 50 m northward spike in the middle.
    std::vector<GeoPoint> path{at(0, 0), at(0, 10), at(50, 20), at(0, 30), at(0, 40)};
    StaticVector<GeoPoint, kMaxWaypoints> out;
    const std::size_t k = douglasPeucker(path, 1.0, out);
    ASSERT_GE(k, 3U);
    ASSERT_LE(k, path.size());
    // Endpoints are always preserved.
    EXPECT_NEAR(out.front().lon, path.front().lon, 1e-12);
    EXPECT_NEAR(out.back().lon, path.back().lon, 1e-12);
    // The dominant spike must be retained.
    bool has_spike = false;
    for (std::size_t i = 0; i < out.size(); ++i) {
        if (std::abs(out[i].lat - at(50, 20).lat) < 1e-12) has_spike = true;
    }
    EXPECT_TRUE(has_spike);
}

TEST(Simplify, ReturnPathIsReversedWithSequence) {
    std::vector<GeoPoint> trail{east(0.0), east(10.0), east(20.0), east(30.0)};
    MissionConfig cfg;
    cfg.simplify = false;
    StaticVector<Waypoint, kMaxWaypoints> out;
    buildReturnPath(trail, cfg, out);
    ASSERT_EQ(out.size(), 4U);
    EXPECT_EQ(out[0].seq, 0U);
    EXPECT_EQ(out[3].seq, 3U);
    // First waypoint = current (last trail point); last = home (first trail point).
    EXPECT_NEAR(out.front().pos.lon, trail.back().lon, 1e-12);
    EXPECT_NEAR(out.back().pos.lon, trail.front().lon, 1e-12);
}

TEST(Simplify, EmptyTrailYieldsEmptyMission) {
    std::vector<GeoPoint> trail;
    MissionConfig cfg;
    StaticVector<Waypoint, kMaxWaypoints> out;
    buildReturnPath(trail, cfg, out);
    EXPECT_TRUE(out.empty());
}

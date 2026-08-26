// Unit tests: dependency-free geodesy helpers.
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

import ugv.rth;

using namespace ugv::rth;

namespace {
constexpr Scalar kDeg = std::numbers::pi_v<Scalar> / 180.0;
}

TEST(Geo, HaversineOneDegreeLatitude) {
    const GeoPoint a{0.0, 0.0, 0.0};
    const GeoPoint b{1.0 * kDeg, 0.0, 0.0};
    // 1 deg of arc on the mean-radius sphere.
    EXPECT_NEAR(geo::haversine(a, b), kDeg * geo::kEarthRadius, 1.0);
}

TEST(Geo, HaversineSymmetricAndZero) {
    const GeoPoint a{0.5, 0.3, 10.0};
    const GeoPoint b{0.51, 0.31, 5.0};
    EXPECT_NEAR(geo::haversine(a, b), geo::haversine(b, a), 1e-9);
    EXPECT_NEAR(geo::haversine(a, a), 0.0, 1e-9);
}

TEST(Geo, PerpendicularDistanceToEastSegment) {
    const GeoPoint a{0.0, 0.0, 0.0};
    const GeoPoint b{0.0, 0.01 * kDeg, 0.0};       // due east
    const GeoPoint p{0.001 * kDeg, 0.005 * kDeg, 0.0};  // ~111 m north of the line
    EXPECT_NEAR(geo::perpendicularDistance(p, a, b), 0.001 * kDeg * geo::kEarthRadius, 1.0);
}

TEST(Geo, PerpendicularDistanceDegenerateSegment) {
    const GeoPoint a{0.0, 0.0, 0.0};
    const GeoPoint p{0.0, 0.001 * kDeg, 0.0};
    EXPECT_NEAR(geo::perpendicularDistance(p, a, a), geo::haversine(a, p), 1e-6);
}

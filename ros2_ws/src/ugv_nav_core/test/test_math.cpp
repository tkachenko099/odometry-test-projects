// Unit tests: SO3/quaternion algebra and WGS-84 geodesy.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

import ugv.nav.core;

using namespace ugv::nav;
namespace m = ugv::nav::math;

namespace {
constexpr double kTol = 1e-9;
}

TEST(Math, SkewIsAntisymmetric) {
    const Vec3 v(1.0, -2.0, 3.0);
    const Mat3 s = m::skew(v);
    EXPECT_TRUE(s.isApprox(-s.transpose(), kTol));
    // skew(v) * w == v x w
    const Vec3 w(0.5, 0.25, -1.0);
    EXPECT_TRUE((s * w).isApprox(v.cross(w), kTol));
}

TEST(Math, ExpLogRoundTrip) {
    const Vec3 phi(0.3, -0.15, 0.05);
    const Quat q = m::expq(phi);
    EXPECT_NEAR(q.norm(), 1.0, kTol);
    EXPECT_TRUE(m::logq(q).isApprox(phi, 1e-8));
}

TEST(Math, ExpSmallAngleStable) {
    const Vec3 phi(1e-10, -2e-10, 0.0);
    const Quat q = m::expq(phi);
    EXPECT_NEAR(q.w(), 1.0, 1e-9);
    EXPECT_TRUE(m::logq(q).isApprox(phi, 1e-9));
}

TEST(Math, BoxplusComposesRotations) {
    const Quat q = Quat(Eigen::AngleAxisd(0.4, Vec3::UnitZ()));
    const Vec3 dphi(0.0, 0.0, 0.1);
    const Quat q2 = m::boxplus(q, dphi);
    const double yaw = m::quat2euler(q2).z();
    EXPECT_NEAR(yaw, 0.5, 1e-6);
}

TEST(Math, EulerFromIdentityIsZero) {
    const Vec3 e = m::quat2euler(Quat::Identity());
    EXPECT_TRUE(e.isApprox(Vec3::Zero(), kTol));
}

TEST(Math, EulerYawPitchRoll) {
    const double roll = 0.2, pitch = -0.1, yaw = 0.9;
    const Quat q = Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ()) *
                        Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
                        Eigen::AngleAxisd(roll, Vec3::UnitX()));
    const Vec3 e = m::quat2euler(q);
    EXPECT_NEAR(e.x(), roll, 1e-6);
    EXPECT_NEAR(e.y(), pitch, 1e-6);
    EXPECT_NEAR(e.z(), yaw, 1e-6);
}

TEST(Geodesy, NedRoundTrip) {
    const Geodetic ref{30.5 * m::kDeg2Rad, 114.3 * m::kDeg2Rad, 20.0};
    const Vec3 ned(123.4, -56.7, 8.9);
    const Geodetic g = m::ned2llh(ned, ref);
    const Vec3 back = m::llh2ned(g, ref);
    EXPECT_TRUE(back.isApprox(ned, 1e-3));  // mm-level for ~100 m offsets
}

TEST(Geodesy, OriginMapsToZero) {
    const Geodetic ref{0.7, 0.2, 100.0};
    EXPECT_TRUE(m::llh2ned(ref, ref).isApprox(Vec3::Zero(), 1e-6));
}

TEST(Geodesy, NorthingIncreasesWithLatitude) {
    const Geodetic ref{0.5, 0.5, 0.0};
    Geodetic north = ref;
    north.lat += 1e-4;  // move north
    const Vec3 ned = m::llh2ned(north, ref);
    EXPECT_GT(ned.x(), 0.0);                 // +N
    EXPECT_NEAR(ned.y(), 0.0, 1e-3);
}

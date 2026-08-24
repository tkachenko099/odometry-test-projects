// Unit tests: differential-drive wheel odometry and GNSS conversion.
#include <gtest/gtest.h>

#include <Eigen/Dense>

import ugv.nav.core;

using namespace ugv::nav;
namespace s = ugv::nav::sensors;
namespace m = ugv::nav::math;

TEST(WheelOdometry, StraightLine) {
    const WheelSample w{.t = 0.0, .omega_left = 10.0, .omega_right = 10.0};
    const auto tw = s::wheelOdometry(w, /*radius=*/0.1, /*track=*/0.5);
    EXPECT_NEAR(tw.v, 1.0, 1e-12);       // r * omega
    EXPECT_NEAR(tw.omega_z, 0.0, 1e-12);
}

TEST(WheelOdometry, PureRotation) {
    const WheelSample w{.t = 0.0, .omega_left = -5.0, .omega_right = 5.0};
    const auto tw = s::wheelOdometry(w, 0.1, 0.5);
    EXPECT_NEAR(tw.v, 0.0, 1e-12);
    EXPECT_NEAR(tw.omega_z, 0.1 / 0.5 * 10.0, 1e-12);  // r/L * (wr - wl)
}

TEST(WheelOdometry, ZeroTrackNoDivideByZero) {
    const WheelSample w{.t = 0.0, .omega_left = 1.0, .omega_right = 2.0};
    const auto tw = s::wheelOdometry(w, 0.1, 0.0);
    EXPECT_NEAR(tw.omega_z, 0.0, 1e-12);
}

TEST(GnssConversion, MatchesGeodesyHelper) {
    const Geodetic ref{0.53, 1.99, 30.0};
    Geodetic fixll = ref;
    fixll.lat += 2e-5;
    fixll.lon -= 1e-5;
    const GnssSample fix{.t = 1.0, .llh = fixll, .std_ned = Vec3::Constant(1.0)};
    EXPECT_TRUE(s::gnssToNed(fix, ref).isApprox(m::llh2ned(fixll, ref), 1e-9));
}

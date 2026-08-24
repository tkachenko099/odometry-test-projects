// Unit tests: trajectory metrics (RMSE / ATE / RPE / drift).
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

import ugv.nav.core;

using namespace ugv::nav;
namespace mt = ugv::nav::metrics;

namespace {

std::vector<mt::PoseError> makeConstantOffset(int n, const Vec3& offset) {
    std::vector<mt::PoseError> out;
    out.reserve(n);
    for (int i = 0; i < n; ++i) {
        mt::PoseError e;
        e.t = static_cast<double>(i);
        e.p_gt = Vec3(static_cast<double>(i), 0.0, 0.0);
        e.p_est = e.p_gt + offset;
        e.v_gt = Vec3(1.0, 0.0, 0.0);
        e.v_est = Vec3(1.0, 0.0, 0.0);
        out.push_back(e);
    }
    return out;
}

}  // namespace

TEST(Metrics, ZeroErrorForPerfectEstimate) {
    const auto s = makeConstantOffset(10, Vec3::Zero());
    const auto r = mt::evaluate(s);
    EXPECT_NEAR(r.position_rmse, 0.0, 1e-12);
    EXPECT_NEAR(r.max_error, 0.0, 1e-12);
    EXPECT_NEAR(r.velocity_rmse, 0.0, 1e-12);
}

TEST(Metrics, ConstantOffsetRmseEqualsOffsetNorm) {
    const Vec3 off(3.0, 4.0, 0.0);  // norm 5
    const auto s = makeConstantOffset(100, off);
    const auto r = mt::evaluate(s);
    EXPECT_NEAR(r.position_rmse, 5.0, 1e-9);
    EXPECT_NEAR(r.ate, 5.0, 1e-9);
    EXPECT_NEAR(r.max_error, 5.0, 1e-9);
}

TEST(Metrics, RpeZeroForConstantOffset) {
    // A constant offset cancels in frame-to-frame differences.
    const auto s = makeConstantOffset(50, Vec3(2.0, 0.0, 0.0));
    EXPECT_NEAR(mt::rpe(s), 0.0, 1e-9);
}

TEST(Metrics, TravelledDistanceAndDrift) {
    const auto s = makeConstantOffset(11, Vec3(1.0, 0.0, 0.0));  // 10 m path, 1 m error
    const auto r = mt::evaluate(s);
    EXPECT_NEAR(r.travelled_distance, 10.0, 1e-9);
    EXPECT_NEAR(r.duration, 10.0, 1e-9);
    EXPECT_NEAR(r.relative_drift_pct, 10.0, 1e-9);  // 1 m / 10 m * 100
}

TEST(Metrics, YawWrapping) {
    std::vector<mt::PoseError> s(1);
    s[0].yaw_gt = 3.13;
    s[0].yaw_est = -3.13;  // ~2π apart -> tiny wrapped error
    const double y = mt::yawRmse(s);
    EXPECT_LT(y, 0.03);
}

TEST(Metrics, EmptyInputIsSafe) {
    const auto r = mt::evaluate({});
    EXPECT_EQ(r.samples, 0u);
    EXPECT_NEAR(r.position_rmse, 0.0, 1e-12);
}

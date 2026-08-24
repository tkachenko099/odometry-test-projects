// Unit tests: ESKF IMU propagation and measurement updates.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

import ugv.nav.core;

using namespace ugv::nav;

namespace {

/// Build a stationary IMU sample that exactly cancels gravity for a level,
/// north-aligned body (specific force = -g_nav, expressed in body frame).
ImuSample stationaryImu(double t) {
    return ImuSample{.t = t, .accel = Vec3(0.0, 0.0, -math::kGravity), .gyro = Vec3::Zero()};
}

}  // namespace

TEST(Eskf, StationaryStaysAtRest) {
    ErrorStateKf kf;
    NavState x0;
    kf.reset(x0);
    const double dt = 0.01;
    for (int i = 1; i <= 500; ++i) kf.predict(stationaryImu(i * dt), dt);
    const auto& x = kf.state();
    EXPECT_LT(x.p.norm(), 1e-6);
    EXPECT_LT(x.v.norm(), 1e-6);
}

TEST(Eskf, ConstantAccelerationIntegratesKinematically) {
    ErrorStateKf kf;
    kf.reset(NavState{});
    const double a = 2.0;  // forward (north) accel
    const double dt = 0.001;
    const int n = 1000;    // 1 second
    for (int i = 1; i <= n; ++i) {
        // Specific force = desired nav accel minus gravity, in body frame.
        ImuSample s{.t = i * dt, .accel = Vec3(a, 0.0, -math::kGravity), .gyro = Vec3::Zero()};
        kf.predict(s, dt);
    }
    const auto& x = kf.state();
    EXPECT_NEAR(x.v.x(), a * 1.0, 1e-2);            // v = a t
    EXPECT_NEAR(x.p.x(), 0.5 * a * 1.0 * 1.0, 1e-2);  // p = ½ a t²
}

TEST(Eskf, GyroIntegratesYaw) {
    ErrorStateKf kf;
    kf.reset(NavState{});
    const double wz = 0.5;  // rad/s
    const double dt = 0.001;
    for (int i = 1; i <= 1000; ++i)
        kf.predict({i * dt, Vec3(0, 0, -math::kGravity), Vec3(0, 0, wz)}, dt);
    const double yaw = math::quat2euler(kf.state().q).z();
    EXPECT_NEAR(yaw, wz * 1.0, 1e-3);
}

TEST(Eskf, GnssUpdatePullsPositionAndReducesCovariance) {
    EskfConfig cfg;
    cfg.init_pos_std = 10.0;
    ErrorStateKf kf(cfg);
    kf.reset(NavState{});
    const double p0 = kf.covariance()(0, 0);
    const Vec3 meas(5.0, -3.0, 0.0);
    for (int i = 0; i < 20; ++i) kf.updateGnssNed(meas, Vec3::Constant(0.5));
    const auto& x = kf.state();
    EXPECT_NEAR(x.p.x(), meas.x(), 0.5);
    EXPECT_NEAR(x.p.y(), meas.y(), 0.5);
    EXPECT_LT(kf.covariance()(0, 0), p0);  // covariance shrank
}

TEST(Eskf, WheelSpeedUpdateConstrainsForwardVelocity) {
    ErrorStateKf kf;
    NavState x0;
    x0.v = Vec3(0.0, 0.0, 0.0);
    kf.reset(x0);
    const double v_meas = 1.5;
    for (int i = 0; i < 50; ++i) kf.updateWheelSpeed(v_meas, 0.05);
    // Body is north-aligned, so forward velocity maps to +x (north).
    EXPECT_NEAR(kf.state().v.x(), v_meas, 0.1);
}

TEST(Eskf, CovarianceStaysSymmetricPositiveDefinite) {
    ErrorStateKf kf;
    kf.reset(NavState{});
    for (int i = 1; i <= 100; ++i) {
        kf.predict(stationaryImu(i * 0.01), 0.01);
        if (i % 10 == 0) kf.updateGnssNed(Vec3(0.1 * i, 0.0, 0.0), Vec3::Constant(1.0));
    }
    const auto P = kf.covariance();
    EXPECT_TRUE(P.isApprox(P.transpose(), 1e-9));
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(P);
    EXPECT_GT(es.eigenvalues().minCoeff(), -1e-9);  // PSD
}

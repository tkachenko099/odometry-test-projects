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

// --- Regression tests for the attitude-error convention --------------------
// The first six tests keep the body level and north-aligned, where a nav-frame
// and a body-frame attitude correction coincide. These two do not: they fail
// if `inject` applies the correction in the wrong frame or if a measurement
// Jacobian has the wrong sign.

TEST(Eskf, HeadingPriorIsIndependentOfLevellingPrior) {
    EskfConfig cfg;
    cfg.init_att_std = 2.0 * math::kDeg2Rad;
    cfg.init_yaw_std = 90.0 * math::kDeg2Rad;
    ErrorStateKf kf(cfg);
    kf.reset(NavState{});
    const int a = static_cast<int>(ErrIdx::kAtt);
    const auto P = kf.covariance();
    EXPECT_NEAR(P(a, a), cfg.init_att_std * cfg.init_att_std, 1e-12);
    EXPECT_NEAR(P(a + 1, a + 1), cfg.init_att_std * cfg.init_att_std, 1e-12);
    EXPECT_NEAR(P(a + 2, a + 2), cfg.init_yaw_std * cfg.init_yaw_std, 1e-12);
}

TEST(Eskf, TiltErrorConvergesAtArbitraryHeading) {
    // Truth: stationary, level, heading 120 deg. The filter starts with the
    // right heading but 3 deg roll / -2 deg pitch of levelling error, which
    // only GNSS position (via the mis-projected gravity) can reveal.
    EskfConfig cfg;
    cfg.init_ba_std = 1.0e-3;  // keep accel bias from absorbing the tilt
    ErrorStateKf kf(cfg);
    const double yaw = 120.0 * math::kDeg2Rad;
    NavState x0;
    x0.q = Quat(Eigen::AngleAxisd(yaw, Vec3::UnitZ()) *
                Eigen::AngleAxisd(-2.0 * math::kDeg2Rad, Vec3::UnitY()) *
                Eigen::AngleAxisd(3.0 * math::kDeg2Rad, Vec3::UnitX()));
    kf.reset(x0);

    const double dt = 0.01;
    for (int i = 1; i <= 3000; ++i) {  // 30 s
        kf.predict(stationaryImu(i * dt), dt);
        if (i % 20 == 0) kf.updateGnssNed(Vec3::Zero(), Vec3::Constant(0.05));
    }
    const Vec3 rpy = math::quat2euler(kf.state().q);
    EXPECT_NEAR(rpy.x(), 0.0, 0.3 * math::kDeg2Rad);
    EXPECT_NEAR(rpy.y(), 0.0, 0.3 * math::kDeg2Rad);
    EXPECT_LT(kf.state().p.norm(), 0.2);
    EXPECT_LT(kf.state().v.norm(), 0.1);
}

TEST(Eskf, UnknownHeadingIsRecoveredFromMotion) {
    // Truth: driving due East at 1 m/s. The filter believes it points North
    // and is told (open prior) that it does not really know. GNSS position
    // plus the wheel-speed and non-holonomic constraints must swing the
    // heading round to East.
    EskfConfig cfg;
    cfg.init_yaw_std = 180.0 * math::kDeg2Rad;
    ErrorStateKf kf(cfg);
    kf.reset(NavState{});  // identity attitude = heading North

    const double dt = 0.01;
    const double speed = 1.0;
    for (int i = 1; i <= 3000; ++i) {  // 30 s
        const double t = i * dt;
        kf.predict(stationaryImu(t), dt);  // constant velocity: no specific force but gravity
        if (i % 2 == 0) {
            kf.updateWheelSpeed(speed, 0.05);
            kf.updateNonHolonomic(0.1, 0.1);
        }
        if (i % 20 == 0) kf.updateGnssNed(Vec3(0.0, speed * t, 0.0), Vec3::Constant(0.3));
    }
    const double heading = math::quat2euler(kf.state().q).z();
    EXPECT_NEAR(heading, 90.0 * math::kDeg2Rad, 5.0 * math::kDeg2Rad);
    EXPECT_NEAR(kf.state().v.y(), speed, 0.1);
    EXPECT_NEAR(kf.state().p.y(), speed * 30.0, 0.5);
}

TEST(Eskf, CoarseAlignmentRescuesGrosslyWrongHeading) {
    // Same drive as above but starting 180 deg out - far beyond what the
    // linearised updates can pull in. A course-over-ground alignment (as the
    // ROS node performs after a few metres of GNSS baseline) fixes it.
    EskfConfig cfg;
    cfg.init_yaw_std = 180.0 * math::kDeg2Rad;
    ErrorStateKf kf(cfg);
    NavState x0;
    x0.q = Quat(Eigen::AngleAxisd(-90.0 * math::kDeg2Rad, Vec3::UnitZ()));  // facing West
    kf.reset(x0);

    const double dt = 0.01;
    const double speed = 1.0;
    bool aligned = false;
    for (int i = 1; i <= 2000; ++i) {  // 20 s
        const double t = i * dt;
        kf.predict(stationaryImu(t), dt);
        if (i % 20 == 0) kf.updateGnssNed(Vec3(0.0, speed * t, 0.0), Vec3::Constant(0.3));
        if (!aligned && t >= 3.0) {
            // Course from the 3 m GNSS baseline, 12 deg off to mimic its noise.
            kf.alignHeading(78.0 * math::kDeg2Rad, speed, 15.0 * math::kDeg2Rad);
            EXPECT_NEAR(kf.headingStd(), 15.0 * math::kDeg2Rad, 1e-12);
            aligned = true;
        }
        if (aligned && i % 2 == 0) {
            kf.updateWheelSpeed(speed, 0.05);
            kf.updateNonHolonomic(0.1, 0.1);
        }
    }
    EXPECT_NEAR(math::quat2euler(kf.state().q).z(), 90.0 * math::kDeg2Rad, 3.0 * math::kDeg2Rad);
    EXPECT_LT(kf.headingStd(), 15.0 * math::kDeg2Rad);  // refined by the updates
    EXPECT_NEAR(kf.state().p.y(), speed * 20.0, 0.3);
}

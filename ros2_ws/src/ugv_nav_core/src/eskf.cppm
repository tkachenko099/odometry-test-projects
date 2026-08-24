// C++23 module partition: 15-state Error-State Kalman Filter (GNSS/INS + ODO).
module;

#include <Eigen/Dense>
#include <algorithm>
#include <optional>

export module ugv.nav.core:eskf;

import :types;
import :math;

export namespace ugv::nav {

/// Continuous-time process-noise / initial-uncertainty configuration.
struct EskfConfig {
    // Continuous IMU noise densities.
    Scalar accel_noise{1.0e-2};       ///< [m/s^2/√Hz]
    Scalar gyro_noise{1.0e-3};        ///< [rad/s/√Hz]
    Scalar accel_bias_walk{1.0e-4};   ///< [m/s^3/√Hz]
    Scalar gyro_bias_walk{1.0e-5};    ///< [rad/s^2/√Hz]

    // Initial standard deviations.
    Scalar init_pos_std{1.0};
    Scalar init_vel_std{0.5};
    Scalar init_att_std{5.0 * math::kDeg2Rad};
    Scalar init_ba_std{0.1};
    Scalar init_bg_std{0.01};

    Vec3 gravity{0.0, 0.0, math::kGravity};  ///< NED gravity vector
};

/// Loosely-coupled ESKF with a Phi-angle error model (KF-GINS style, reduced
/// to 15 states). Nominal state is integrated deterministically while the
/// error state is tracked through the covariance `P_`.
class ErrorStateKf {
public:
    explicit ErrorStateKf(EskfConfig cfg = {}) : cfg_(cfg) { reset(NavState{}); }

    /// (Re)initialise the nominal state and diagonal covariance.
    void reset(const NavState& x0) noexcept {
        x_ = x0;
        P_.setZero();
        auto blk = [&](ErrIdx i, Scalar s) {
            P_.block<3, 3>(static_cast<int>(i), static_cast<int>(i)) =
                Mat3::Identity() * (s * s);
        };
        blk(ErrIdx::kPos, cfg_.init_pos_std);
        blk(ErrIdx::kVel, cfg_.init_vel_std);
        blk(ErrIdx::kAtt, cfg_.init_att_std);
        blk(ErrIdx::kBAcc, cfg_.init_ba_std);
        blk(ErrIdx::kBGyro, cfg_.init_bg_std);
        initialised_ = true;
    }

    [[nodiscard]] const NavState& state() const noexcept { return x_; }
    [[nodiscard]] const Mat<kStateDim, kStateDim>& covariance() const noexcept { return P_; }
    [[nodiscard]] bool initialised() const noexcept { return initialised_; }

    /// IMU-driven strapdown propagation of nominal state + covariance over `dt`.
    void predict(const ImuSample& imu, Scalar dt) {
        if (dt <= 0.0) return;
        const Vec3 acc = imu.accel - x_.accel_bias;
        const Vec3 gyr = imu.gyro - x_.gyro_bias;
        const Mat3 R = x_.q.toRotationMatrix();

        // Nominal integration (first-order strapdown).
        const Vec3 acc_n = R * acc + cfg_.gravity;
        x_.p += x_.v * dt + 0.5 * acc_n * dt * dt;
        x_.v += acc_n * dt;
        x_.q = math::boxplus(x_.q, gyr * dt);
        x_.t = imu.t;

        // Error-state transition F (continuous) -> discrete Phi = I + F dt.
        Mat<kStateDim, kStateDim> F = Mat<kStateDim, kStateDim>::Zero();
        constexpr int P = static_cast<int>(ErrIdx::kPos);
        constexpr int V = static_cast<int>(ErrIdx::kVel);
        constexpr int A = static_cast<int>(ErrIdx::kAtt);
        constexpr int BA = static_cast<int>(ErrIdx::kBAcc);
        constexpr int BG = static_cast<int>(ErrIdx::kBGyro);
        F.block<3, 3>(P, V) = Mat3::Identity();
        F.block<3, 3>(V, A) = -math::skew(R * acc);
        F.block<3, 3>(V, BA) = -R;
        F.block<3, 3>(A, BG) = -R;

        const Mat<kStateDim, kStateDim> Phi =
            Mat<kStateDim, kStateDim>::Identity() + F * dt;

        // Discrete process noise (diagonal densities * dt).
        Mat<kStateDim, kStateDim> Q = Mat<kStateDim, kStateDim>::Zero();
        Q.block<3, 3>(V, V) = Mat3::Identity() * (cfg_.accel_noise * cfg_.accel_noise * dt);
        Q.block<3, 3>(A, A) = Mat3::Identity() * (cfg_.gyro_noise * cfg_.gyro_noise * dt);
        Q.block<3, 3>(BA, BA) =
            Mat3::Identity() * (cfg_.accel_bias_walk * cfg_.accel_bias_walk * dt);
        Q.block<3, 3>(BG, BG) =
            Mat3::Identity() * (cfg_.gyro_bias_walk * cfg_.gyro_bias_walk * dt);

        P_ = Phi * P_ * Phi.transpose() + Q;
        symmetrise();
    }

    /// GNSS position update in local NED (measurement = position + noise).
    void updateGnssNed(const Vec3& p_meas, const Vec3& std_ned) {
        Mat<3, kStateDim> H = Mat<3, kStateDim>::Zero();
        H.block<3, 3>(0, static_cast<int>(ErrIdx::kPos)) = Mat3::Identity();
        const Mat3 Rm = std_ned.cwiseProduct(std_ned).asDiagonal();
        applyUpdate<3>(H, p_meas - x_.p, Rm);
    }

    /// Wheel-odometry forward-velocity update: projects NED velocity onto the
    /// body x-axis and compares against the measured longitudinal speed.
    void updateWheelSpeed(Scalar v_forward, Scalar std_v) {
        const Mat3 R = x_.q.toRotationMatrix();
        const Vec3 fwd = R.col(0);                 // body x-axis in nav frame
        Mat<1, kStateDim> H = Mat<1, kStateDim>::Zero();
        H.block<1, 3>(0, static_cast<int>(ErrIdx::kVel)) = fwd.transpose();
        H.block<1, 3>(0, static_cast<int>(ErrIdx::kAtt)) =
            (-fwd.transpose() * math::skew(x_.v));  // d(R e0)·v w.r.t. dphi
        Mat<1, 1> Rm;
        Rm(0, 0) = std_v * std_v;
        Mat<1, 1> innov;
        innov(0, 0) = v_forward - fwd.dot(x_.v);
        applyUpdate<1>(H, innov, Rm);
    }

private:
    /// Joseph-form measurement update followed by error injection.
    template <int M>
    void applyUpdate(const Mat<M, kStateDim>& H, const Mat<M, 1>& innov,
                     const Mat<M, M>& R) {
        const Mat<M, M> S = H * P_ * H.transpose() + R;
        const Mat<kStateDim, M> K = P_ * H.transpose() * S.inverse();
        const Mat<kStateDim, 1> dx = K * innov;
        inject(dx);
        const Mat<kStateDim, kStateDim> I_KH =
            Mat<kStateDim, kStateDim>::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
        symmetrise();
    }

    /// Inject an error-state correction into the nominal state.
    void inject(const Mat<kStateDim, 1>& dx) noexcept {
        x_.p += dx.segment<3>(static_cast<int>(ErrIdx::kPos));
        x_.v += dx.segment<3>(static_cast<int>(ErrIdx::kVel));
        x_.q = math::boxplus(x_.q, dx.segment<3>(static_cast<int>(ErrIdx::kAtt)));
        x_.accel_bias += dx.segment<3>(static_cast<int>(ErrIdx::kBAcc));
        x_.gyro_bias += dx.segment<3>(static_cast<int>(ErrIdx::kBGyro));
    }

    void symmetrise() noexcept { P_ = 0.5 * (P_ + P_.transpose()).eval(); }

    EskfConfig cfg_{};
    NavState x_{};
    Mat<kStateDim, kStateDim> P_{Mat<kStateDim, kStateDim>::Identity()};
    bool initialised_{false};
};

}  // namespace ugv::nav

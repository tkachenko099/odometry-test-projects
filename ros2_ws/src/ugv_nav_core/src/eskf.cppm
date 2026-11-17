// C++23 module partition: 15-state Error-State Kalman Filter (GNSS/INS + ODO).
module;

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
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
    Scalar init_att_std{5.0 * math::kDeg2Rad};  ///< Roll/pitch (levelling) [rad]
    /// Heading (rotation about nav-frame Down) [rad]. Position-only GNSS cannot
    /// observe yaw at rest, so an unknown initial heading must be given an open
    /// prior here instead of sharing the tight levelling value.
    Scalar init_yaw_std{5.0 * math::kDeg2Rad};
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
        // The attitude error is expressed in the nav frame (see `inject`), so
        // its z component is exactly the heading error.
        const int yaw = static_cast<int>(ErrIdx::kAtt) + 2;
        P_(yaw, yaw) = cfg_.init_yaw_std * cfg_.init_yaw_std;
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
        // h = (R e0)·v with R_true = Exp(dphi) R  =>  dh = dphi·(fwd x v),
        // i.e. the row (fwd x v)^T = fwd^T [v]x. (The opposite sign steers the
        // heading *away* from the direction of travel.)
        H.block<1, 3>(0, static_cast<int>(ErrIdx::kAtt)) = fwd.transpose() * math::skew(x_.v);
        Mat<1, 1> Rm;
        Rm(0, 0) = std_v * std_v;
        Mat<1, 1> innov;
        innov(0, 0) = v_forward - fwd.dot(x_.v);
        applyUpdate<1>(H, innov, Rm);
    }

    /// Non-holonomic constraint for a wheeled vehicle: the body-frame lateral
    /// and vertical velocities are (pseudo-)measured as zero.
    ///
    /// Forward speed alone constrains heading only to second order (cos of the
    /// error); the lateral component is first-order (sin), which is what makes
    /// yaw observable on a straight run with position-only GNSS.
    void updateNonHolonomic(Scalar std_lateral, Scalar std_vertical) {
        const Mat3 R = x_.q.toRotationMatrix();
        Mat<2, kStateDim> H = Mat<2, kStateDim>::Zero();
        Mat<2, 1> innov;
        for (int row = 0; row < 2; ++row) {
            const Vec3 axis = R.col(row + 1);  // body y / z in the nav frame
            H.block<1, 3>(row, static_cast<int>(ErrIdx::kVel)) = axis.transpose();
            H.block<1, 3>(row, static_cast<int>(ErrIdx::kAtt)) =
                axis.transpose() * math::skew(x_.v);
            innov(row, 0) = -axis.dot(x_.v);
        }
        Mat<2, 2> Rm = Mat<2, 2>::Zero();
        Rm(0, 0) = std_lateral * std_lateral;
        Rm(1, 1) = std_vertical * std_vertical;
        applyUpdate<2>(H, innov, Rm);
    }

    /// Coarse heading alignment from an external source (e.g. GNSS course over
    /// ground): rotate the attitude about nav-frame Down so that the heading
    /// becomes `heading`, point the horizontal velocity along it at `speed`,
    /// and replace the heading uncertainty by `std_heading`.
    ///
    /// An EKF cannot pull in a large heading error on its own - the wheel and
    /// non-holonomic Jacobians are only valid for small errors - so an unknown
    /// initial heading has to be set this way before those updates are used.
    void alignHeading(Scalar heading, Scalar speed, Scalar std_heading) noexcept {
        const Scalar delta = heading - math::quat2euler(x_.q).z();
        x_.q = (math::expq(Vec3(0.0, 0.0, delta)) * x_.q).normalized();
        x_.v.x() = speed * std::cos(heading);
        x_.v.y() = speed * std::sin(heading);
        const int yaw = static_cast<int>(ErrIdx::kAtt) + 2;
        P_.row(yaw).setZero();
        P_.col(yaw).setZero();
        P_(yaw, yaw) = std_heading * std_heading;
    }

    /// 1-sigma heading uncertainty [rad].
    [[nodiscard]] Scalar headingStd() const noexcept {
        const int yaw = static_cast<int>(ErrIdx::kAtt) + 2;
        return std::sqrt(P_(yaw, yaw));
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
    ///
    /// `F` and `H` above are derived for a *global* (nav-frame) attitude error,
    /// R_true = Exp(dphi) * R_est, so the correction must be applied on the
    /// left. (`boxplus` is the body-frame map and is only right for the gyro
    /// increment in `predict`.)
    void inject(const Mat<kStateDim, 1>& dx) noexcept {
        x_.p += dx.segment<3>(static_cast<int>(ErrIdx::kPos));
        x_.v += dx.segment<3>(static_cast<int>(ErrIdx::kVel));
        const Vec3 dphi = dx.segment<3>(static_cast<int>(ErrIdx::kAtt));
        x_.q = (math::expq(dphi) * x_.q).normalized();
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
